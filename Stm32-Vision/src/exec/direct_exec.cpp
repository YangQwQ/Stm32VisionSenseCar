#include "src/exec/direct_exec.h"
#include "src/exec/nezha_direct.h"
#include "src/exec/bivar.h"
#include "src/core/board_log.h"   // blog::logf（临时调试 armdbg 用）
#include "Calibration.h"   // 集中式校准数据（舵机限位/移动表/标定点）
#include <math.h>
#include <string.h>

// 机械臂夹心坐标（车头系）：末端夹心 = f(移爪pwm, 抬落pwm)，两连杆+曲柄非线性，
// 用实测散点反距离加权插值(IDW)（数据在 Calibration.h 的 kArmPts），FK/IK 由 bivar 查插。

// 分段线性插值（查表），横轴需升序。仍被 move/spin 时长表复用。
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

// 机械臂末夹心坐标（车头系：x=前方cm / h=离地cm）→ bivar 散点插值；失败时置 0，由调用方限位提示兜底。
static void arm_fk(int16_t reach_pwm, int16_t lift_pwm, float* x_out, float* h_out) {
  if (!bivar::arm_fk((float)reach_pwm, (float)lift_pwm, x_out, h_out)) { *x_out = 0; *h_out = 0; }
}

// ================= 内部状态 =================
// 当前各舵机 PWM（写板前跟踪，随指令/步进更新）
static int16_t s_steer = STEER_CENTER;
static int16_t s_reach = REACH_CENTER;
static int16_t s_lift  = LIFT_CENTER;
static int16_t s_grip  = GRIP_CENTER;
static bool s_folded = false;   // 臂是否处于"已折叠回平台"态: fold 置位, 任何移动臂位的动作(arm_pose/持续/reset)清除

// 缓动分离：s_reach/s_lift = 目标(逻辑/状态读取用)，s_reach_w/s_lift_w = 实际写入板的 PWM。
// 离散定位(fold/arm_pose/reset/init)在 update_tick 里用 S 形曲线把 s_*_w 追向 s_*；
// 持续动作仍逐拍小步、不经缓动。t0 为缓动曲线行程锚点（新指令跳远时重置）。夹爪不参与。
static int16_t s_reach_w = REACH_CENTER;
static int16_t s_lift_w  = LIFT_CENTER;
static int32_t s_reach_t0 = 0;
static int32_t s_lift_t0  = 0;

// 小车行驶状态（本地合成用）：0 停 / 1 前 / 2 后
static int s_car_motion = 0;
static int s_steer_dir  = 0;  // 0 正前 / 1 左 / 2 右
static int s_spin = 0;        // 原地旋转：0 无 / 1 左进右退(右转/顺时针) / -1 左退右进(左转/逆时针)

// 夹心逻辑位置（车头系：x 前方cm / h 离地cm）。持续按钮在它之上步进后经 arm_pose 单点定位，
// 不再依赖 FK→微步→反解的连续闭环（散点 IDW 下该闭环会卡住）。由 每次臂指令 同步重置。
static float s_claw_x = 0, s_claw_h = 0;

// 最近一次 arm_pose 撞边界/不可达诊断：记录被拒/被夹紧的目标位姿，供 read_state 反馈。
// 撞边界已被 arm_clamp 夹到盒内最近合法点并继续移动（reason=1），不再拒绝。
static struct {
  bool armed;     // 有未消费的诊断
  int  reason;    // 1=撞边界已夹紧到最近合法点 / 2=移爪越限 / 3=抬落越限
  float x, h;     // 请求的目标夹心坐标（未夹紧前）
  float cx, ch;   // reason=1 时夹紧后的实际落点
  int rr, ll;     // reason=2/3 时反解出的 target pwm
} s_arm_rej = { false, -1, 0, 0, 0, 0, 0 };

// 灯状态记忆（供 get_state 查询同步手机按钮）：仅记录是否开，开关动作由 nezha::led 落地。
static bool s_light_front = false;
static bool s_light_vibe  = false;
static bool s_light_back  = false;

// 连续机械臂动作（联动模式）：axis=-1 表示无活跃动作。
// 前伸/缩回、抬/落都按"保持另一维"的末端位姿轨迹联动双舵机，而非单舵机步进。
static struct {
  int8_t axis;     // -1 无 / 0 联动前伸(x) / 1 联动抬落(h)
  int16_t dir;     // 每拍目标位姿增量方向（+/-）
  int16_t budget;  // 剩余拍数，0=持续
} s_active = { -1, 0, 0 };

// AI 持续 move 的行驶截止时刻（ms），0=不限时。到期 update_tick 里自动停轮，
// 兜底 AI 两次决策间的长时间盲走；任何新轮动作/手动接管会刷新或关闭它。
// volatile：worker 任务 / loop(update_tick) / 命令回调跨核读写，防寄存器缓存读到旧值。
static volatile unsigned long s_move_cap_until = 0;

static void drive_motors(int spd) {
  int16_t u = spd < 0 ? (uint16_t)-spd : (uint16_t)spd;
  bool rev = spd < 0;
  // 轮map：M1左后 M2右后 M3右前 M4左前；左轮 a 正前，右轮 b 正前。
  nezha::set_motor(1, rev ? 0u : u, rev ? u : 0u);
  nezha::set_motor(2, rev ? u : 0u, rev ? 0u : u);
  nezha::set_motor(3, rev ? u : 0u, rev ? 0u : u);
  nezha::set_motor(4, rev ? 0u : u, rev ? u : 0u);
}

static void set_steer_pwm(int16_t p);  // 前向声明：send_spin 在回正转向轮时会调用

// 原地旋转（普通四轮滑移式，无需特殊轮子）：左/右侧轮反向拖胎绕中心旋转。
// dir=+1 左进右退=右转(顺时针) / -1 左退右进=左转(逆时针) / 0 停；speed=单车轮 pwm 0..1000。
// 车轮映射与 drive_motors 一致：左轮 a 正前、右轮 b 正前，故 dir>0 统一写 (a,0)、dir<0 统一写 (0,b)。
static void send_spin(const JsonObjectConst& p) {
  int dir = p["dir"] | 0;
  int spd = p["speed"] | 500;
  // 原地旋转前必须回正转向轮（前轮直行位），否则拖胎方向不纯、转不正（Bug6）。
  // dir=0（停车）不动舵机，避免每次停转都无谓地 reset 转向。
  if (dir != 0 && s_steer != STEER_CENTER) {
    set_steer_pwm(STEER_CENTER);
  }
  if (dir == 0) spd = 0;  // dir=0 = 停车，speed 必须连同归零，否则默认 500 会让轮子继续转
  if (spd < 0) spd = 0;
  if (spd > 1000) spd = 1000;
  if (dir != 0 && spd < SPIN_MIN_SPEED) spd = SPIN_MIN_SPEED;  // 低于拖得动线则抬升
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
  // 带 angle_deg = 定角微操：按时长近似自停（无里程计）；每度用时按转速插值标定表。
  int ang = p["angle_deg"] | 0;
  if (ang > 720) ang = 720;  // 粗钳防超长时限（手机/AI 已限 0..500，仅兜底裸前端）
  if (dir != 0 && ang > 0) {
    long ms = (long)(ang * plerp(SPIN_MSDEG_X, SPIN_MSDEG_Y, 4, (float)spd));
    exec::set_move_cap_ms(ms > 0 ? (int)ms : 1);
  }
}

static void clear_active(void) { s_active.axis = -1; s_active.budget = 0; }

// S 形缓动一步：把"已写 cur"平滑追向"目标 tgt"。速度按正弦分布（两端 0=缓出缓入、中段峰值），
// 目标显著跳远(新指令)会重新锚定行程并本拍不动作为缓入；到位直接落定，避免往返抖动。
static void advance_smooth(int16_t* cur, int16_t tgt, int32_t* t0) {
  int32_t gap = (int32_t)tgt - (int32_t)*cur;
  if (gap == 0) { *t0 = 0; return; }
  int32_t rem = gap < 0 ? -gap : gap;
  if (rem > *t0) { *t0 = rem; return; }             // 目标跳远 → 缓入起点，本拍不动
  float p = (float)(*t0 - rem) / (float)*t0;        // 已完成比例 0..1
  if (p > 1.0f) p = 1.0f;
  int16_t step = (int16_t)(ARM_SMOOTH_PEAK * sinf((float)PI * p) + 0.5f);
  if (step < 1) step = 1;                           // 保底，确保最终必到
  if (rem <= step) { *cur = tgt; *t0 = 0; return; }
  *cur += (gap > 0 ? step : -step);
}

static void write_steer(void) { nezha::set_servo(1, (uint16_t)s_steer); }
static void write_reach(void) { nezha::set_servo(2, (uint16_t)s_reach_w); }
static void write_grip(void)  { nezha::set_servo(3, (uint16_t)s_grip); }
static void write_lift(void)  { nezha::set_servo(4, (uint16_t)s_lift_w); }

static void set_steer_pwm(int16_t p) {
  if (p < STEER_LO) p = STEER_LO;
  if (p > STEER_HI) p = STEER_HI;
  s_steer = p; write_steer();
  s_steer_dir = p == STEER_CENTER ? 0 : (p < STEER_CENTER ? 1 : 2);
}

void exec::init(void) {
  // 惰性：nezha::init 幂等。回中四个舵机 + 电机 0。
  bivar::arm_init();   // 校验机械臂夹心标定散点并置 ready（setup 期调用一次，幂等）
  set_steer_pwm(STEER_CENTER);
  s_reach = REACH_CENTER;  write_reach();
  s_lift  = LIFT_CENTER;   write_lift();
  s_grip  = GRIP_CENTER;   write_grip();
  drive_motors(0);
}

void exec::reset(void) {
  s_car_motion = 0;
  s_spin = 0;
  s_move_cap_until = 0;
  clear_active();
  s_arm_rej.armed = false;  // 回正清掉旧的"目标不可达"诊断，避免状态残留误导
  exec::init();
}

void exec::update_tick(void) {
  // AI 持续/微操动作的行驶时限兜底：到期自动停轮（转向/机械臂不干预），状态复位便于 AI 看到"停止"。
  if (s_move_cap_until != 0 && (long)(millis() - s_move_cap_until) >= 0) {
    s_move_cap_until = 0;
    drive_motors(0);
    s_car_motion = 0;
    s_spin = 0;  // 原地旋转定角到点也一并复位，避免状态误报"仍在原地转"
  }
  if (s_active.axis < 0) {
    // 无活跃持续动作：离散定位（fold / arm_pose / reset / init）经 S 形缓动把已写值追向目标，
    // 避免一次性跳舵机把整车带震。夹爪不参与，仍即时到位。
    int16_t nr = s_reach_w, nl = s_lift_w;
    advance_smooth(&nr, s_reach, &s_reach_t0);
    advance_smooth(&nl, s_lift,  &s_lift_t0);
    if (nr != s_reach_w || nl != s_lift_w) { s_reach_w = nr; s_lift_w = nl; write_reach(); write_lift(); }
    return;
  }
  // 夹心持续平动：在逻辑位置 s_claw 上沿目标轴步进、保持另一维，再 arm_pose 单点定位。
  // 连续步进需节流帧率：伺服跟不上会产生滞后，FK 反馈读到滞后位置再回拽 → 极限环。
  // 约 50ms 落一步，伺服跟得上、反馈才稳。
  static uint32_t s_arm_step_ms = 0;
  uint32_t now_ms = millis();
  if (s_arm_step_ms != 0 && now_ms - s_arm_step_ms < 50) return;
  s_arm_step_ms = now_ms;
  // === 临时调试：定位移爪极限环（用毕删除）===
  static int s_arm_dbg = 0;
  if (s_claw_x < 1.f && s_claw_h < 1.f) {   // 逻辑位置未初始化（reset 后首动）：从当前舵机反算
    arm_fk(s_reach, s_lift, &s_claw_x, &s_claw_h);
    blog::logf(blog::EXEC, "armdbg REINIT pwm=(%d,%d) logic=(%.1f,%.1f)", s_reach, s_lift, s_claw_x, s_claw_h);
  }
  float tx = s_claw_x, th = s_claw_h;
  if (s_active.axis == 0) tx += ARM_STEP_CM * s_active.dir;   // 前伸/缩回：x 变，h 不变
  else                    th += ARM_STEP_CM * s_active.dir;   // 抬/落：h 变，x 不变
  // 撞边界先夹到盒内合法点（继续移动，不拒绝）
  bivar::arm_clamp(&tx, &th);
  if (!exec::arm_pose(tx, th)) { clear_active(); return; }    // 夹紧后仍不可达（物理限位）：到边即停
  s_reach_w = s_reach; s_lift_w = s_lift; write_reach(); write_lift();
  // 回授协调：只把"被步进的轴"锚到真实 FK，保持轴维持命令值——避免 fk 噪声把保持轴带入
  // 极限环 / 按住后反向漂移。臂到真实物理极限时 FK 停进，s_claw 随之停，命令不再空推。
  float rxx = 0, rhh = 0;
  if (bivar::arm_fk((float)s_reach, (float)s_lift, &rxx, &rhh)) {
    if (s_active.axis == 0) { s_claw_x = rxx; s_claw_h = th; }   // 移爪：x 跟回授，h 保持命令
    else                    { s_claw_h = rhh; s_claw_x = tx; }   // 抬落：h 跟回授，x 保持命令
  } else {
    s_claw_x = tx; s_claw_h = th;     // 反算越数据范围读不到：回退用目标（夹紧后仍 push）
  }
  if (s_arm_dbg++ < 600 && (s_arm_dbg & 3) == 0) {   // 每 4 拍打一拍，最多 600 行
    float fb_x = 0, fb_h = 0;
    arm_fk(s_reach, s_lift, &fb_x, &fb_h);
    blog::logf(blog::EXEC, "armdbg logic=(%.2f,%.2f) tgt=(%.2f,%.2f) pwm=(%d,%d) fk=(%.2f,%.2f)",
               s_claw_x, s_claw_h, tx, th, s_reach, s_lift, fb_x, fb_h);
  }
  if (s_active.budget > 0) {
    if (--s_active.budget <= 0) clear_active();
  }
}

static void setup_active(int8_t axis, int16_t dir, bool has_dist, int16_t dist) {
  s_active.axis = axis;
  s_active.dir = dir;
  s_active.budget = has_dist ? (int16_t)(dist * ARM_CNT_PER_CM) : 0;
  // 每次持续移动起点：把逻辑坐标重新锚到舵机真实当前位置（FK）。否则 reset/fold/离散
  // arm_pose 后残留旧逻辑值，首拍会从旧坐标起步、朝反方向先补一枪。
  float fx = 0, fh = 0;
  if (bivar::arm_fk((float)s_reach_w, (float)s_lift_w, &fx, &fh)) { s_claw_x = fx; s_claw_h = fh; }
}

static void send_arm(const JsonObjectConst& p) {
  // 每次手臂指令先清掉可能残留的旧联动步进（s_active）：防止上次持续动作因 button_up/限位
  // 等原因未清干净，导致本次新指令被上次残留干扰而"点了没反应"（回正=全清后即恢复）。
  clear_active();
  const char* act_ = p["act"] | "";
  bool has_dist = p["dist_cm"].is<int>();
  int16_t dist = (int16_t)((int)(p["dist_cm"] | 0));

  if (!strcmp(act_, "lift_up") || !strcmp(act_, "lift_down")) {
    // 抬落（联动 h 轴）：lift_up = 末端升高（h+）；lift_down = 降低（h-），保持 x 不变。
    int16_t dir = !strcmp(act_, "lift_up") ? +1 : -1;
    s_folded = false;   // 移动臂位即解除折叠态
    setup_active(1, dir, has_dist, dist);
  } else if (!strcmp(act_, "reach_forward") || !strcmp(act_, "reach_backward")) {
    // 移爪（联动 x 轴）：forward = 前伸（x+）；backward = 缩回（x-），保持高度 h 不变。
    int16_t dir = !strcmp(act_, "reach_forward") ? +1 : -1;
    s_folded = false;
    setup_active(0, dir, has_dist, dist);
  } else if (!strcmp(act_, "clip")) {
    clear_active();
    s_grip = GRIP_CLOSE; write_grip();
  } else if (!strcmp(act_, "release")) {
    clear_active();
    s_grip = GRIP_HI; write_grip();
  } else if (!strcmp(act_, "fold")) {
    // 收臂折叠回平台（一次性离散）：抬落 + 移爪都收到 130、夹爪回中；摄像头到最高位扩大视野、避开盲区。
    // 不动车轮（区别于 reset 的全停）。
    clear_active();
    s_lift  = FOLD_LIFT_PWM;  write_lift();
    s_reach = FOLD_REACH_PWM; write_reach();
    s_grip  = GRIP_CENTER;    write_grip();
    s_folded = true;
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
  // 带 distance_cm = 定距微操：本板无里程计，按时长近似自停。实测 actual≈v·t+c，
  // v/c 都随油门插值标定表（对油门不敏感），时长=(cm-c)/v。
  int cm = p["distance_cm"] | 0;
  float a = fabsf(th); if (a > 1.0f) a = 1.0f;
  if (cm > 0 && a > 0.001f) {
    float v = plerp(MV_SPEED_X, MV_SPEED_Y, 3, a);
    float c = plerp(MV_COAST_X, MV_COAST_Y, 3, a);
    float target = cm > c ? cm - c : 0.0f;
    long ms = (long)(target / v * 1000.0f);
    exec::set_move_cap_ms(ms > 0 ? (int)ms : 1);
  }
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

// 迭代反解：以 IDW 反查询为初值，用正演 FK 走牛顿迭代校正，使 FK(pwm)→目标精确，
// 消除 IDW FK∘IK 不互逆（否则持续相控时每步被 FK 打回 / 或边界空推）。到物理限位自动饱和停滞。
static bool ik_refine(float x, float h, float* r, float* l) {
  if (!bivar::arm_ik(x, h, r, l)) return false;   // 反查初值；越标定盒返回 false
  float rp = *r, lp = *l;
  for (int it = 0; it < 7; it++) {
    float fx, fh;
    if (!bivar::arm_fk(rp, lp, &fx, &fh)) break;             // 出标定数据区：没法校正，用当前值
    float ex = x - fx, eh = h - fh;
    if (fabsf(ex) < 0.02f && fabsf(eh) < 0.02f) break;        // 已收敛
    const float d = 1.0f;                                     // 数值雅可比：reach/lift 各偏 d
    float fxr, fhr, fxl, fhl;
    if (!bivar::arm_fk(rp + d, lp, &fxr, &fhr)) break;
    if (!bivar::arm_fk(rp, lp + d, &fxl, &fhl)) break;
    float J00 = (fxr - fx) / d, J01 = (fxl - fx) / d;
    float J10 = (fhr - fh) / d, J11 = (fhl - fh) / d;
    float det = J00 * J11 - J01 * J10;
    if (fabsf(det) < 1e-6f) break;                            // 奇异（饱和区雅可比退化）
    float dr = (ex * J11 - J01 * eh) / det;
    float dl = (-J10 * ex + J00 * eh) / det;
    dr = fmaxf(-6.f, fminf(6.f, dr));                         // 阻尼防发散
    dl = fmaxf(-6.f, fminf(6.f, dl));
    rp += dr; lp += dl;
    rp = fmaxf((float)REACH_LO, fminf((float)REACH_HI, rp));
    lp = fmaxf((float)LIFT_LO, fminf((float)LIFT_HI, lp));
  }
  *r = rp; *l = lp;
  return true;
}

bool exec::arm_pose(float x, float h) {
  // 给末端目标位姿：x=夹心车头前方 cm，h=夹心离地高度 cm。反解：IDW 反查初值 + FK 迭代校正
  // （使反解出的 PWM 的实际 FK 回读≈目标，收敛到物理饱和为止）。超出盒先夹到盒内最近的
  // 合法点再反解——不拒绝，移到最接近的合法位姿；反解落在物理舵机限位外也夹到限位继续。
  s_arm_rej.x = x; s_arm_rej.h = h;
  bool clamped = bivar::arm_clamp(&x, &h);
  // 夹到盒内后戳地(h<0)已不可达（盒 h 最小>0），这里仅兜底负高度竖直挤压情形。
  float reach = 0, lift = 0;
  if (!ik_refine(x, h, &reach, &lift)) return false;  // 反查初值即出盒→不可达
  int rr = (int)roundf(reach), ll = (int)roundf(lift);     // 左=移爪 reach / 右=抬落 lift
  // 反解在稀疏角区外插可能越物理限位：夹到舵机限位继续执行而非拒绝，让标定盒内的位姿总能
  // 尽力移到组件物理边界（否则 IDW 一外插超限就被当"不可达"卡住）。撞物理限位也算夹紧记进诊断。
  bool phy = false;
  if (rr < REACH_LO) { rr = REACH_LO; phy = true; }
  else if (rr > REACH_HI) { rr = REACH_HI; phy = true; }
  if (ll < LIFT_LO) { ll = LIFT_LO; phy = true; }
  else if (ll > LIFT_HI) { ll = LIFT_HI; phy = true; }
  s_arm_rej.armed = clamped || phy;
  if (clamped) { s_arm_rej.reason = 1; s_arm_rej.cx = x; s_arm_rej.ch = h; }
  // 联动：一次同时给左右两舵机目标。set_servo 内部会限 pwm 并同步状态。
  bool ok = set_servo(1, (uint16_t)rr) & set_servo(2, (uint16_t)ll);
  return ok;
}

bool exec::act(const char* type, const JsonObjectConst& params) {
  // 任何显式轮子/原地旋转指令都刷新或关闭 AI 的 move 时限，避免旧兜底在新指令后误停。
  if (!strcmp(type, "move") || !strcmp(type, "spin") || !strcmp(type, "stop")) s_move_cap_until = 0;
  if (!strcmp(type, "move"))   { send_move(params); return true; }
  if (!strcmp(type, "spin"))   { send_spin(params); return true; }
  if (!strcmp(type, "stop"))   { send_stop(params); return true; }
  if (!strcmp(type, "arm"))    { send_arm(params);  return true; }
  if (!strcmp(type, "arm_pose")) {
    // AI/手动指定位姿：x=夹心车头前方 cm，h=夹心离地高度 cm；不可达返回 false（不动）。
    s_folded = false;   // 指定位姿同样脱离折叠态
    return arm_pose(params["x"] | 0.0f, params["h"] | 0.0f);
  }
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
  if (!strcmp(type, "reset"))  { exec::reset(); s_folded = false; return true; }
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
    int d = p["dir"] | 0;
    return d != 0 && !p["angle_deg"].is<int>();  // 带 angle 的定角=有界，不需持续收尾
  }
  if (!strcmp(type, "arm")) {
    const char* act_ = p["act"] | "";
    bool cont = !strcmp(act_, "lift_up") || !strcmp(act_, "lift_down") ||
                !strcmp(act_, "reach_forward") || !strcmp(act_, "reach_backward");
    return cont && !p["dist_cm"].is<int>();
  }
  return false;
}

bool exec::wheels_moving() { return s_car_motion != 0 || s_spin != 0; }

bool exec::grip_closing() { return s_grip == GRIP_CLOSE; }

void exec::set_move_cap_ms(int ms) {
  s_move_cap_until = ms > 0 ? millis() + (unsigned long)ms : 0;
}

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
  // 夹爪状态文本化：合/开/中按当前 pwm 区间。注意"合"仅代表伺服闭合到位，不代表夹住物体
  // （AI 曾把"紧"误判为已夹住导致假成功）。
  const char* grip = s_grip <= GRIP_CLOSE + 5 ? "合" : (s_grip >= GRIP_HI - 5 ? "开" : "中");
  // 机械臂到限位提示：告诉 AI 继续同向动作不会再有变化（需反向或调整姿态）。
  char lim[32] = {0};
  if (s_reach >= REACH_HI - 2) snprintf(lim, sizeof(lim), " 移爪到顶");
  else if (s_reach <= REACH_LO + 2) snprintf(lim, sizeof(lim), " 移爪缩到底");
  if (s_lift >= LIFT_HI - 2) snprintf(lim + strlen(lim), sizeof(lim) - strlen(lim), " 抬落最低");
  else if (s_lift <= LIFT_LO + 2) snprintf(lim + strlen(lim), sizeof(lim) - strlen(lim), " 抬到顶");
  // 末端前端坐标（前向运动学）：让 AI 知道夹爪现在伸到多前、多高，判断还能往哪移/当前高度。
  // 括号内为左右舵机 PWM（Servo2=移爪 s_reach / Servo4=抬落 s_lift），供 exec_log 校准机械臂坐标。
  float fk_x = 0, fk_h = 0;
  arm_fk(s_reach, s_lift, &fk_x, &fk_h);
  if (fk_x < 0) fk_x = 0;
  // 臂态语义：是否已折叠回平台（fold 到位/未被打断）。给 AI 明确反馈，避免已折叠后仍反复 fold。
  const char* fold_stat = s_folded ? " 已折叠" : "";
  snprintf(buf, cap, "小车:%s %s | 抓手:前%.0fcm(%d) 高%.0fcm(%d) 爪:%s%s%s",
    car, steer, fk_x, (int)s_reach, fk_h, (int)s_lift, grip, lim, fold_stat);
  // 撞边界/不可达诊断：反馈"想去哪、实际落到哪/反解成多少"，帮用户/AI 判断机械臂边界
  // （exec_log 推给手机）。reason=1 表示撞边界但已夹到最近合法点继续移动，非错误。
  if (s_arm_rej.armed) {
    char d[96];
    if (s_arm_rej.reason == 1) {
      snprintf(d, sizeof(d), " | 撞边界[已移至最近点(%.1f,%.1f)]", s_arm_rej.cx, s_arm_rej.ch);
    } else {
      const char* rc = s_arm_rej.reason == 2 ? "移爪越限" : "抬落越限";
      snprintf(d, sizeof(d), " | 目标(%.1f,%.1f)不可达[%s] 反解reach=%d lift=%d",
        s_arm_rej.x, s_arm_rej.h, rc, s_arm_rej.rr, s_arm_rej.ll);
    }
    if (strlen(buf) + strlen(d) + 1 < cap) strcat(buf, d);
  }
  return buf[0] != '\0';
}
