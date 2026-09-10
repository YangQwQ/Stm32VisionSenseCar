#include "direct_exec.h"
#include "nezha_direct.h"
#include <math.h>
#include <string.h>

// ================= 二连杆 IK 标定（挖掘机式机械臂） =================
// 几何：第一节(A)长 L1，第二节(B)长 L2，右舵机控 A 相对水平角 α，左舵机(推杆)控两臂夹角 β。
// 轴(肩)高于地面 AXIS_H cm；末端夹爪位置 = f(α,β)。L1=L2=7.5cm（转轴到转轴）。
#define IK_L1       7.5f
#define IK_L2       7.5f
#define AXIS_H      9.5f   // 肩轴相对地面的高度（3.5 平台 + 0.5 转轴 + 5.5 底盘）
#define RMIN        4.0f   // 可达最近（约 3.5，留余量）
#define RMAX        15.0f  // 理论最远 L1+L2

// 标定映射（实测）：表按横轴升序排列。纵轴出界自动夹到首尾。
// α(第一节与水平夹角, 度) → 右舵机 pwm
const float ALPHA_X[] = { 0.f, 50.f, 55.f, 90.f };
const float ALPHA_PWM[] = { 220.f, 200.f, 150.f, 130.f };
// β(两臂夹角, 度) → 左舵机 pwm
const float BETA_X[] = { 25.f, 45.f, 80.f, 140.f };
const float BETA_PWM[] = { 130.f, 150.f, 180.f, 230.f };

// 分段线性插值（查表），横轴需升序
static float plerp(const float* xs, const float* ys, int n, float x) {
  if (x <= xs[0]) return ys[0];
  for (int i = 1; i < n; i++) {
    if (x <= xs[i]) {
      float t = (x - xs[i-1]) / (xs[i] - xs[i-1]);
      return ys[i-1] + t * (ys[i] - ys[i-1]);
    }
  }
  return ys[n-1];
}

// ================= 舵机限位 / 回中（移植自执行板官方机械臂驱动，勿随意改） =================
// 大圣机械臂(有方NeZha)三舵机：Servo2=左舵机(前后移爪) Servo3=前舵机(夹爪) Servo4=右舵机(抬落)。
// 每个舵机为一个独立自由度，非左右联动；正负方向见各舵机注释。
// Servo1 转向：150 正前 / 120 左死 / 180 右死
#define STEER_CENTER  150
#define STEER_LO      120
#define STEER_HI      180
// Servo2 前后移爪（reach/左舵机·推杆）：有效行程按实测 β 表 130..230，留余量 120..250
#define REACH_CENTER  200
#define REACH_LO      120
#define REACH_HI      250
// Servo3 夹爪（grip）：夹紧 50 / 松 140 / 回正初始 140
#define GRIP_CENTER   140
#define GRIP_CLOSE    50
#define GRIP_LO       50
#define GRIP_HI       140
// Servo4 抬落（lift/右舵机·第一节）：有效行程按实测 α 表 130..220，留余量 115..250
#define LIFT_CENTER   180
#define LIFT_LO       115
#define LIFT_HI       250
// 连续动作每节拍步进；dist_cm 仅近似（无里程计），每 cm 约 8 拍
#define ARM_STEP      2
#define ARM_CNT_PER_CM 8

// ================= 内部状态 =================
// 当前各舵机 pwm（写板前跟踪，随指令/步进更新）
static int16_t s_steer = STEER_CENTER;
static int16_t s_reach = REACH_CENTER;
static int16_t s_lift  = LIFT_CENTER;
static int16_t s_grip  = GRIP_CENTER;

// 小车行驶状态（本地合成用）：0 停 / 1 前 / 2 后
static int s_car_motion = 0;
static int s_steer_dir  = 0;  // 0 正前 / 1 左 / 2 右
static int s_spin = 0;        // 原地旋转：0 无 / 1 左进右退(右转/顺时针) / -1 左退右进(左转/逆时针)

// 灯状态记忆（供 get_state 查询同步手机按钮）：仅记录是否开，开关动作由 nezha::led 落地。
static bool s_light_front = false;
static bool s_light_vibe  = false;
static bool s_light_back  = false;

// 连续机械臂动作：servo=0 表示无活跃动作
static struct {
  uint8_t servo;     // 2 移爪 / 4 抬落
  int16_t dir;       // 每拍伺服增量（+/-）
  int16_t budget;    // 剩余拍数，0=持续
} s_active = { 0, 0, 0 };

static void drive_motors(int spd) {
  int16_t u = spd < 0 ? (uint16_t)-spd : (uint16_t)spd;
  bool rev = spd < 0;
  // 轮map：M1左后 M2右后 M3右前 M4左前；左轮 a 正前，右轮 b 正前。
  nezha::set_motor(1, rev ? 0u : u, rev ? u : 0u);
  nezha::set_motor(2, rev ? u : 0u, rev ? 0u : u);
  nezha::set_motor(3, rev ? u : 0u, rev ? 0u : u);
  nezha::set_motor(4, rev ? 0u : u, rev ? u : 0u);
}

// 原地旋转（普通四轮滑移式，无需特殊轮子）：左/右侧轮反向拖胎绕中心旋转。
// dir=+1 左进右退=右转(顺时针) / -1 左退右进=左转(逆时针) / 0 停；speed=单车轮 pwm 0..1000。
// 车轮映射与 drive_motors 一致：左轮 a 正前、右轮 b 正前，故 dir>0 统一写 (a,0)、dir<0 统一写 (0,b)。
static void send_spin(const JsonObjectConst& p) {
  int dir = p["dir"] | 0;
  int spd = p["speed"] | 500;
  if (dir == 0) spd = 0;  // dir=0 = 停车，speed 必须连同归零，否则默认 500 会让轮子继续转
  if (spd < 0) spd = 0;
  if (spd > 1000) spd = 1000;
  s_spin = dir == 0 ? 0 : (dir > 0 ? 1 : -1);
  uint16_t u = (uint16_t)spd;
  if (dir > 0) {
    nezha::set_motor(1, u, 0); nezha::set_motor(2, u, 0);
    nezha::set_motor(3, u, 0); nezha::set_motor(4, u, 0);
  } else {
    nezha::set_motor(1, 0, u); nezha::set_motor(2, 0, u);
    nezha::set_motor(3, 0, u); nezha::set_motor(4, 0, u);
  }
  s_car_motion = 0;  // 原地旋转不算前进/后退
}

static void clear_active(void) { s_active.servo = 0; s_active.budget = 0; }

static void write_steer(void) { nezha::set_servo(1, (uint16_t)s_steer); }
static void write_reach(void) { nezha::set_servo(2, (uint16_t)s_reach); }
static void write_grip(void)  { nezha::set_servo(3, (uint16_t)s_grip); }
static void write_lift(void)  { nezha::set_servo(4, (uint16_t)s_lift); }

static void set_steer_pwm(int16_t p) {
  if (p < STEER_LO) p = STEER_LO;
  if (p > STEER_HI) p = STEER_HI;
  s_steer = p; write_steer();
  s_steer_dir = p == STEER_CENTER ? 0 : (p < STEER_CENTER ? 1 : 2);
}

void exec::init(void) {
  // 惰性：nezha::init 幂等。回中四个舵机 + 电机 0。
  set_steer_pwm(STEER_CENTER);
  s_reach = REACH_CENTER;  write_reach();
  s_lift  = LIFT_CENTER;   write_lift();
  s_grip  = GRIP_CENTER;   write_grip();
  drive_motors(0);
}

void exec::reset(void) {
  s_car_motion = 0;
  s_spin = 0;
  clear_active();
  exec::init();
}

void exec::update_tick(void) {
  if (s_active.servo == 0) return;
  if (s_active.servo == 2) {
    s_reach += ARM_STEP * s_active.dir;
    if (s_reach < REACH_LO) s_reach = REACH_LO;
    if (s_reach > REACH_HI) s_reach = REACH_HI;
    write_reach();
  } else if (s_active.servo == 4) {
    s_lift += ARM_STEP * s_active.dir;
    if (s_lift < LIFT_LO) s_lift = LIFT_LO;
    if (s_lift > LIFT_HI) s_lift = LIFT_HI;
    write_lift();
  }
  if (s_active.budget > 0) {
    if (--s_active.budget <= 0) clear_active();
  }
}

static void setup_active(uint8_t servo, int16_t dir, bool has_dist, int16_t dist) {
  s_active.servo = servo;
  s_active.dir = dir;
  s_active.budget = has_dist ? (int16_t)(dist * ARM_CNT_PER_CM) : 0;
}

static void send_arm(const JsonObjectConst& p) {
  const char* act_ = p["act"] | "";
  bool has_dist = p["dist_cm"].is<int>();
  int16_t dist = (int16_t)((int)(p["dist_cm"] | 0));

  if (!strcmp(act_, "lift_up") || !strcmp(act_, "lift_down")) {
    // 抬落：lift_up = 抬（pwm 降向 LIFT_LO）；lift_down = 落（pwm 升）。
    int16_t dir = !strcmp(act_, "lift_up") ? -1 : +1;
    setup_active(4, dir, has_dist, dist);
  } else if (!strcmp(act_, "reach_forward") || !strcmp(act_, "reach_backward")) {
    // 移爪：forward = 伸（pwm 增）；backward = 缩（pwm 减）。
    int16_t dir = !strcmp(act_, "reach_forward") ? +1 : -1;
    setup_active(2, dir, has_dist, dist);
  } else if (!strcmp(act_, "clip")) {
    clear_active();
    s_grip = GRIP_CLOSE; write_grip();
  } else if (!strcmp(act_, "release")) {
    clear_active();
    s_grip = GRIP_HI; write_grip();
  }
}

static void send_move(const JsonObjectConst& p) {
  float th = p["throttle"] | 0.0f;
  float st = p["steering"] | 0.0f;

  if (fabsf(th) > 0.001f) {
    float a = fabsf(th); if (a > 1.0f) a = 1.0f;
    int16_t spd = (int16_t)(a * 1000.0f + 0.5f);
    if (th < 0) spd = -spd;
    drive_motors(spd);
    s_car_motion = th > 0 ? 1 : 2;
  } else {
    drive_motors(0);
    s_car_motion = 0;
  }
  if (fabsf(st) > 0.001f) {
    float s = st; if (s > 1.0f) s = 1.0f; if (s < -1.0f) s = -1.0f;
    set_steer_pwm((int16_t)(STEER_CENTER + s * 30.0f));
  }
  s_spin = 0;  // 常规行驶（move）接管后清除原地旋转
}

static void send_stop(const JsonObjectConst& p) {
  const char* scope = p["scope"] | "all";
  if (!strcmp(scope, "arm")) {
    clear_active();
  } else {
    drive_motors(0);
    s_car_motion = 0;
    s_spin = 0;
  }
}

bool exec::set_servo(uint8_t logical, uint16_t pwm) {
  // 调试直驱：直接写原始 pwm（50..250），不过标定限位——用于探机械极限/标定。
  if (pwm < 50 || pwm > 250) return false;
  switch (logical) {
    case 0:  // 转向
      s_steer = (int16_t)pwm; write_steer();
      s_steer_dir = pwm == STEER_CENTER ? 0 : (pwm < STEER_CENTER ? 1 : 2);
      return true;
    case 1:  // 左 = 前后移爪
      s_reach = (int16_t)pwm; write_reach();
      return true;
    case 2:  // 右 = 抬落
      s_lift = (int16_t)pwm; write_lift();
      return true;
    case 3:  // 前 = 夹爪
      s_grip = (int16_t)pwm; write_grip();
      return true;
    default:
      return false;
  }
}

bool exec::arm_pose(float x, float h) {
  // 给末端目标位姿：x=轴前方 cm，h=地面以上高度 cm。
  // 相对臂基坐标 z = h - AXIS_H；二连杆反解 → (α,β) → 查表得左右两舵机 pwm，联动下发。
  float z = h - AXIS_H;
  float r2 = x * x + z * z;
  if (r2 < RMIN * RMIN || r2 > RMAX * RMAX) return false;  // 目标不可达
  float r = sqrtf(r2);
  float d = (IK_L1*IK_L1 + IK_L2*IK_L2 - r2) / (2.f*IK_L1*IK_L2);
  d = d > 1.f ? 1.f : (d < -1.f ? -1.f : d);
  float beta = acosf(d) * 180.f / PI;
  float ph = (IK_L1*IK_L1 + r2 - IK_L2*IK_L2) / (2.f*IK_L1*r);
  ph = ph > 1.f ? 1.f : (ph < -1.f ? -1.f : ph);
  float phi0 = acosf(ph) * 180.f / PI;               // 原点处基线与第一节夹角
  float psi  = atan2f(z, x) * 180.f / PI;            // 原点指向目标的方位角
  float alpha = psi + phi0;

  int pr = (int)roundf(plerp(ALPHA_X, ALPHA_PWM, 4, alpha));
  int pl = (int)roundf(plerp(BETA_X,  BETA_PWM,  4, beta));
  pr = pr < 50 ? 50 : (pr > 250 ? 250 : pr);
  pl = pl < 50 ? 50 : (pl > 250 ? 250 : pl);
  // 联动：一次同时给左右两舵机目标。set_servo 内部会限 pwm 并同步状态。
  bool ok = set_servo(1, (uint16_t)pl) & set_servo(2, (uint16_t)pr);
  return ok;
}

bool exec::act(const char* type, const JsonObjectConst& params) {
  if (!strcmp(type, "move"))   { send_move(params); return true; }
  if (!strcmp(type, "spin"))   { send_spin(params); return true; }
  if (!strcmp(type, "stop"))   { send_stop(params); return true; }
  if (!strcmp(type, "arm"))    { send_arm(params);  return true; }
  if (!strcmp(type, "light")) {
    const char* kind = params["kind"] | "";
    bool on = params["on"] | false;
    bool ok = nezha::led(kind, on);
    if (ok) {
      if (!strcmp(kind, "front")) s_light_front = on;
      else if (!strcmp(kind, "vibe")) s_light_vibe = on;
      else if (!strcmp(kind, "back")) s_light_back = on;
    }
    return ok;
  }
  if (!strcmp(type, "reset"))  { exec::reset(); return true; }
  return false;
}

bool exec::is_continuous(const char* type, const JsonObjectConst& p) {
  if (!strcmp(type, "move")) {
    float th = p["throttle"] | 0.0f;
    float st = p["steering"] | 0.0f;
    return (fabsf(th) > 0.001f && !p["distance_cm"].is<int>()) ||
           (fabsf(st) > 0.001f && !p["angle_deg"].is<int>());
  }
  if (!strcmp(type, "spin")) {
    return (p["dir"] | 0) != 0;  // dir 非 0 = 持续原地旋转，需 stop 收尾
  }
  if (!strcmp(type, "arm")) {
    const char* act_ = p["act"] | "";
    bool cont = !strcmp(act_, "lift_up") || !strcmp(act_, "lift_down") ||
                !strcmp(act_, "reach_forward") || !strcmp(act_, "reach_backward");
    return cont && !p["dist_cm"].is<int>();
  }
  return false;
}

bool exec::grip_closing() { return s_grip == GRIP_CLOSE; }

bool exec::light_on(const char* kind) {
  if (!strcmp(kind, "front")) return s_light_front;
  if (!strcmp(kind, "vibe"))  return s_light_vibe;
  if (!strcmp(kind, "back"))  return s_light_back;
  return false;
}

bool exec::read_state(char* buf, size_t cap) {
  const char* car = s_spin != 0 ? (s_spin > 0 ? "原地右转" : "原地左转")
                                : (s_car_motion == 1 ? "前进" : (s_car_motion == 2 ? "后退" : "停止"));
  const char* steer = s_steer_dir == 1 ? "左" : (s_steer_dir == 2 ? "右" : "正");
  snprintf(buf, cap, "小车:%s %s | 机械臂:左:%d 右:%d 前:%d",
    car, steer, (int)s_reach, (int)s_lift, (int)s_grip);
  return buf[0] != '\0';
}