#include "src/exec/direct_exec.h"
#include "src/exec/nezha_direct.h"
#include "src/exec/bivar.h"
#include "src/exec/motion_verify.h"   // 运动到位验证：受阻/旋转到位停轮兜底
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
static bool s_folded = false;   // 是否处于折叠位: fold 置位, 移动臂位的动作清除

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

// 最近一次 arm_pose 的位姿诊断：记录请求位姿与实际落点，供 read_state 反馈。
// 撞边界已被 arm_clamp 夹到盒内最近合法点并继续移动（reason=1），不再拒绝。
// 只在产生后 ARM_DIAG_FRESH_MS 内挂到状态行：诊断要跟着"刚下的那条指令"出现，过期就不再
// 重复挂（否则旧诊断会一轮轮粘在状态行上，让 AI 以为本轮的位姿又失败了）。
#define ARM_DIAG_FRESH_MS 8000
// 到位容差：正运动学回读的实际落点与请求相差超过它就认定"这个位姿做不到"。
// 可达域是条弯带而反查只按散点外接矩形判可达，弯带外的位姿会反解出"最接近"的 pwm、迭代
// 不收敛就停在半路（既不撞舵机限位也无报错），表现为"爪降不下去/卡住"——必须显式报出来。
#define ARM_POSE_TOL_CM 0.6f
static struct {
  bool armed;     // 本轮有诊断
  int  reason;    // 1=撞边界已夹紧到最近合法点 / 2=移爪越限 / 3=抬落越限 / 4=位姿没到位
  float x, h;     // 请求的目标夹心坐标（未夹紧前）
  float cx, ch;   // reason=1 时夹紧后的实际落点
  int rr, ll;     // 反解出的 target pwm
  float ax, ah;   // 正运动学回读的实际落点（reason=4 的判据与报数）
  unsigned long t_ms;   // 诊断产生时刻
} s_arm_rej = { false, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

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

// arm grasp 的第二步（合爪后自动抬臂）：时间到就发起定量的 lift_up。
// 拆成两拍是因为舵机合爪要时间 —— 同拍抬臂会在方块还没被夹住时就把它带飞。0=无待办。
static unsigned long s_grasp_lift_at = 0;
static int16_t s_grasp_lift_cm = (int16_t)GRASP_LIFT_CM;

// AI 持续 move 的行驶截止时刻（ms），0=不限时。到期 update_tick 里自动停轮，
// 兜底 AI 两次决策间的长时间盲走；任何新轮动作/手动接管会刷新或关闭它。
// volatile：worker 任务 / loop(update_tick) / 命令回调跨核读写，防寄存器缓存读到旧值。
static volatile unsigned long s_move_cap_until = 0;

// 定角原地旋转的到位补偿状态（视觉闭环：到点停后用 mvfy 实测转角差值补转）。
// mvfy 跨补偿段持续累积实测角；仅在测量可信（样本够、幅度够、方向与命令一致）时才按差值补转。
static float s_spin_goal = 0;      // 定角目标（abs，度），0=无目标（持续旋转/无定角）
static int   s_spin_dir = 0;       // 目标旋转方向（±1）
static int   s_spin_speed = 0;     // 下发转速（残余补转同速）
static int   s_spin_comp = 0;      // 补转次数（防反复震荡）
// 本次旋转的"实测等效 ms/度"（首段按 计划时长/实测角 算出，见 spin_after_settle）。0=未知，用表值。
// ⚠️ 补偿段必须用它而不是 SPIN_MSDEG 表：表只是先验，实车与表差一倍以上时，按表补转必然震荡
// （转过头 45° → 按表反向补又转过头 45° → 越补越偏，最终比不验证还歪）。
static float s_spin_rate = 0;
// 到点停轮后的沉降截止（ms），0=不在沉降。停轮瞬间还有滑行、实测值也最多滞后一拍采样，
// 立刻判残余会把这段算成"没转够"而多补一截 —— 故停轮后先等一段再判（见 spin_after_settle）。
// 该截止是**上限**：mvfy 测到画面静止（mvfy::spin_settled()）即提前收口，不白等满。
static unsigned long s_spin_settle_until = 0;

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
  s_spin_settle_until = 0;  // 新旋转指令作废上一轮待判的沉降（连同下方重置的补偿状态）
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
  // 带 angle_deg = 定角微操：按时长近似自停（无里程计）+ mvfy 视觉到位补偿。记录目标/方向/转速。
  int ang = p["angle_deg"] | 0;
  if (ang > 720) ang = 720;  // 粗钳防超长时限（手机/AI 已限 0..500，仅兜底裸前端）
  if (dir != 0 && ang > 0) {
    s_spin_goal = ang; s_spin_dir = dir; s_spin_speed = spd; s_spin_comp = 0;
    s_spin_rate = 0;   // 本次首段还没测过：等效速率待首段判残余时算出
    long ms = (long)(ang * plerp(SPIN_MSDEG_X, SPIN_MSDEG_Y, 4, (float)spd));
    exec::set_move_cap_ms(ms > 0 ? (int)ms : 1);
    mvfy::begin("spin", (float)ang);
  } else if (dir == 0) {
    // 停车：清掉定角目标与补偿状态
    s_spin_goal = 0; s_spin_dir = 0; s_spin_speed = 0; s_spin_comp = 0;
    mvfy::end(); mvfy::consume();
  } else {
    // 持续旋转（无定角）：只启动视觉测量，不做补偿。
    // ⚠️ 必须给时限兜底：此前这一支不设 s_move_cap_until，车轮会**一直转到下一条指令**才停，而
    // 下一条指令是 AI 的下一轮（秒级）—— 那段时间够转好几圈，且 car_update_pose 因 ang==0 不累积
    // 车向，于是"真转了几圈、程序以为没转"，此后所有物体坐标的朝向基准全错（实测现象：原地连转
    // 好几圈，转完再也找不到目标）。上限按"最多再转 SPIN_CONT_MAX_DEG 度"折算，到点自停，
    // 与定距 move 的时限同一条兜底通路（到期 drive_motors(0)）。
    long cms = (long)(SPIN_CONT_MAX_DEG * plerp(SPIN_MSDEG_X, SPIN_MSDEG_Y, 4,
                                                (float)(spd > 0 ? spd : SPIN_MIN_SPEED)));
    s_spin_goal = 0; s_spin_dir = dir; s_spin_speed = spd; s_spin_comp = 0;
    s_move_cap_until = millis() + (unsigned long)(cms > 0 ? cms : 500);
    mvfy::begin("spin", 0);
  }
}

static void clear_active(void) { s_active.axis = -1; s_active.budget = 0; }

// 撤销"合爪后待抬臂"（任何新的臂指令/停车/回正都作废它，避免隔了几拍突然自己抬一下）。
static void clear_grasp_pending(void) { s_grasp_lift_at = 0; }

// 前置声明：grasp 的第二步（合爪后自动抬臂）在 update_tick 里发起，而定义在它之后。
static void setup_active(int8_t axis, int16_t dir, bool has_dist, int16_t dist);

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

// 定角旋转到点后的收尾判定（沉降窗口到期时由 update_tick 调一次，轮子此时已停）。
// 用 mvfy 实测累计角算残余：需要且测得准就补转一段（返回 true=已重新驱动轮子），否则返回 false 收尾。
static bool spin_after_settle(void) {
  float got = mvfy::spin_delta_deg();          // mvfy 累计实测转角（有符号）
  int   n   = mvfy::spin_delta_n();            // 累计所用有效样本数（无纹理/乱匹配的拍不计）
  float want = s_spin_goal * (s_spin_dir > 0 ? 1.f : -1.f);  // 目标（带方向）
  // 可信度门槛：模式在位 + 样本够 + 幅度够。测量不可信就退回开环（等于没这套验证），
  // 比按噪声乱补安全——补偿是"锦上添花"，补错了反而比不补更差（会转过头且拖长）。
  if (mvfy::mode() != mvfy::VERIFY_SPIN || n < MVFY_SPIN_MIN_N || fabsf(got) < MVFY_SPIN_TRUST_DEG) {
    blog::logf(blog::EXEC, "mvfy 补偿跳过: 实测不足 got=%.1f n=%d tgt=%.0f", got, n, s_spin_goal);
    return false;
  }
  // 方向自检：实测累计必须与命令方向同号。反号说明这批匹配不可信（画面静止/纹理重复/位移超出
  // 搜索窗），此时宁可退回开环也不补转——否则会朝反方向补，越补越偏。
  if (got * want <= 0.f) {
    blog::logf(blog::EXEC, "mvfy 补偿跳过: 实测方向与命令不符 got=%.1f want=%.1f", got, want);
    return false;
  }
  // 由首段（计划时长 / 实测角）定出本次旋转的"实测等效 ms/度"，供补偿段换算——只用一次，
  // 后续补偿段沿用（同转速同地面）。实测不可信时退回表值，理由见 s_spin_rate 注释。
  float tbl = plerp(SPIN_MSDEG_X, SPIN_MSDEG_Y, 4,
                    (float)(s_spin_speed > 0 ? s_spin_speed : SPIN_MIN_SPEED));
  if (s_spin_comp == 0) {
    long plan_ms = (long)(s_spin_goal * tbl);  // 首段旋转的计划时长（与 send_spin 同式）
    float eq = (float)plan_ms / fabsf(got);
    bool sane = eq >= tbl * MVFY_RATE_LO_RATIO && eq <= tbl * MVFY_RATE_HI_RATIO;
    s_spin_rate = sane ? eq : tbl;
    // 标定读数：SPIN_MSDEG 表准不准就看这行（重标用 新值 = 旧值 × 目标角 / 实测角）。
    blog::logf(blog::EXEC, "mvfy 标定读数: %ldms 实测 %.1f° → 等效 %.1f ms/度（表值 %.1f，实车偏%s%s）",
               plan_ms, fabsf(got), eq, tbl, eq < tbl ? "快" : "慢",
               sane ? "" : "；超可信带,补偿改用表值");
  }
  // 补偿总开关（默认关，见 Calibration.h MVFY_SPIN_COMP_ENABLED）：只测不补。放在"标定读数"之后、
  // 残余计算之前 —— 读数照打（它是判断实测准不准的唯一依据），但绝不据它追加旋转：测量偏小时补偿会
  // 朝同方向越补越多，而旋转是车尾挂着充电线时最危险的动作（缠住就走不动）。
  if (!MVFY_SPIN_COMP_ENABLED) {
    blog::logf(blog::EXEC, "mvfy 补偿已关闭: 保持开环到点自停, got=%.1f want=%.1f", got, want);
    return false;
  }
  float residual = want - got;                 // 残余（正=没转够，负=转过头）
  if (fabsf(residual) <= MVFY_SPIN_TOL_DEG) {
    blog::logf(blog::EXEC, "mvfy 到位: got=%.1f want=%.1f 残差=%.1f 在容差内", got, want, residual);
    return false;
  }
  if (s_spin_comp >= MVFY_SPIN_MAX_COMP) {
    blog::logf(blog::EXEC, "mvfy 补偿达上限(%d 次): got=%.1f want=%.1f 残差=%.1f",
               s_spin_comp, got, want, residual);
    return false;
  }
  s_spin_comp++;
  // 补转方向 = 残余的符号本身：residual = want - got 就是"还差的带符号转角"，>0 还差右转、<0 还差左转。
  // ⚠️ 别写成"residual>0 沿原方向、否则反向"——那只对右转成立：左转时 want/got 均为负，
  //   没转够的 residual 也是负的，按原方向本该继续左转，这类写法会判成"走回头"而朝右补，越补越偏。
  int ndir = residual > 0 ? 1 : -1;
  uint16_t u = (uint16_t)s_spin_speed;
  if (s_spin_speed <= 0) u = SPIN_MIN_SPEED;
  if (ndir > 0) { nezha::set_motor(1,u,0); nezha::set_motor(2,u,0); nezha::set_motor(3,u,0); nezha::set_motor(4,u,0); }
  else          { nezha::set_motor(1,0,u); nezha::set_motor(2,0,u); nezha::set_motor(3,0,u); nezha::set_motor(4,0,u); }
  s_spin = ndir;
  s_car_motion = 0;
  mvfy::begin("spin", 0, true);   // 续测：同模式不重置角度累计，只把"卡死"判定恢复正常
  // 换算用首段实测等效值（s_spin_rate，见其注释），不是表值：表偏一倍时按表补会来回震荡。
  long ms = (long)(fabsf(residual) * (s_spin_rate > 0 ? s_spin_rate : tbl));
  s_move_cap_until = millis() + (unsigned long)(ms > 0 ? ms : 40);
  blog::logf(blog::EXEC, "mvfy 补偿: got=%.1f want=%.1f → 补 %.0f°(按 %.1f ms/度) n=%d",
             got, want, fabsf(residual), s_spin_rate > 0 ? s_spin_rate : tbl, s_spin_comp);
  return true;
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
  s_spin_settle_until = 0;
  clear_active();
  clear_grasp_pending();
  s_arm_rej.armed = false;  // 回正清掉旧的"目标不可达"诊断，避免状态残留误导
  mvfy::end(); mvfy::consume();  // 回正同样结束到位验证
  exec::init();
}

void exec::update_tick(void) {
  // 运动受阻心跳：mvfy 异步任务已测出「受阻/转不大」，轮询到位即停轮（mvfy 自身不下发，保持单一职责）。
  if (mvfy::should_stop()) {
    mvfy::consume();
    drive_motors(0);
    s_car_motion = 0;
    s_spin = 0;
    s_move_cap_until = 0;  // 同时清掉可能正在倒计时的定距/定角时限，避免误重写
    s_spin_settle_until = 0;
    s_spin_goal = 0; s_spin_dir = 0; s_spin_speed = 0; s_spin_comp = 0;
    mvfy::end();
    blog::logf(blog::EXEC, "mvfy: 视觉判定受阻/转不大, 已停轮");
  }
  // AI 持续/微操动作的行驶时限兜底：到期自动停轮（转向/机械臂不干预），状态复位便于 AI 看到"停止"。
  if (s_move_cap_until != 0 && (long)(millis() - s_move_cap_until) >= 0) {
    s_move_cap_until = 0;
    drive_motors(0);
    s_car_motion = 0;
    s_spin = 0;  // 原地旋转定角到点也一并复位，避免状态误报"仍在原地转"
    if (s_spin_goal > 0) {
      // 定角旋转到点：先停轮，再留沉降窗口，到期才判残余（两段式，见 s_spin_settle_until 注释）。
      // 窗口内 mvfy 继续采样累计；spin_stop 暂停"卡死"判定（轮子已停、画面本就不动，否则误报）。
      mvfy::spin_stop();
      s_spin_settle_until = millis() + (unsigned long)MVFY_SPIN_SETTLE_MS;
    } else {
      // 定距 move / AI 时限兜底：直接收尾，视觉测量一并结束。
      // （曾只在 spin 分支结束：定距 move 时 s_spin_goal/s_spin_dir 恒为 0 → mvfy 继续
      //   按采样周期软解打日志，直到"受阻"误判才停。）
      mvfy::end();
      s_spin_goal = 0; s_spin_dir = 0; s_spin_speed = 0; s_spin_comp = 0;
    }
  }
  // 沉降窗口收口：判"补转残余"还是"收尾"。spin_after_settle 返回 true 表示已发起补转
  // （s_move_cap_until 已续上，交给下一轮到点+沉降再判）。
  // 两个出口：mvfy 拍到"画面静止"（滑行结束、累积角已定）就提前收——再等只是白拖时间；
  // 或等到上限（迟迟测不到静止时的兜底，避免整条指令被拖住）。
  if (s_spin_settle_until != 0 &&
      (mvfy::spin_settled() || (long)(millis() - s_spin_settle_until) >= 0)) {
    bool by_still = mvfy::spin_settled();
    long left = (long)(s_spin_settle_until - millis());
    s_spin_settle_until = 0;
    // 这一行是沉降窗口够不够用的判据：总报"超时"说明窗口内压根没测到静止（尾部滑行可能漏测）。
    blog::logf(blog::EXEC, "mvfy 沉降收口: %s（等到 %ldms/%dms）",
               by_still ? "画面已静止" : "超时未静止",
               (long)MVFY_SPIN_SETTLE_MS - (left > 0 ? left : 0), MVFY_SPIN_SETTLE_MS);
    if (!spin_after_settle()) {
      mvfy::end();
      s_spin_goal = 0; s_spin_dir = 0; s_spin_speed = 0; s_spin_comp = 0;
    }
  }
  // arm grasp 第二步：合爪等够时间后自动定量抬臂（起点锚在舵机真实位置，见 setup_active）。
  if (s_grasp_lift_at != 0 && (long)(millis() - s_grasp_lift_at) >= 0) {
    s_grasp_lift_at = 0;
    setup_active(1, +1, true, s_grasp_lift_cm);
    blog::logf(blog::EXEC, "grasp: 合爪完成, 抬臂 %dcm", s_grasp_lift_cm);
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
  if (s_claw_x < 1.f && s_claw_h < 1.f) {   // 逻辑位置未初始化（reset 后首动）：从当前舵机反算
    arm_fk(s_reach, s_lift, &s_claw_x, &s_claw_h);
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
  // 到边界即停：连续 N 拍 FK 回读几乎不动，说明已顶到机械/可达域边界，再推只是让 arm_pose 反复
  // 重解同一个位姿 —— 最高位附近散点稀疏，IDW 会在两个解之间来回跳，舵机就嗡嗡抽搐（实测只在
  // 抬到最高位时出现）。这里主动停，和"收到 stop(scope=arm)/新离散动作"是同一条清场路径。
  {
    static float s_last_fk_x = -999, s_last_fk_h = -999;
    static int s_fk_stall = 0;
    if (fabsf(rxx - s_last_fk_x) < ARM_STALL_EPS_CM && fabsf(rhh - s_last_fk_h) < ARM_STALL_EPS_CM) {
      if (++s_fk_stall >= ARM_STEP_STALL_N) {
        blog::logf(blog::EXEC, "连续动作已到边界(FK 连续 %d 拍停在 (%.1f,%.1f)), 停止步进",
                   s_fk_stall, rxx, rhh);
        s_fk_stall = 0;
        clear_active();
        return;
      }
    } else {
      s_fk_stall = 0;
    }
    s_last_fk_x = rxx; s_last_fk_h = rhh;
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
  clear_grasp_pending();   // 新臂指令作废上一条 grasp 未执行的抬臂（grasp 分支自己会重新置上）
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
  } else if (!strcmp(act_, "low")) {
    // 低姿夹取准备位（一次性离散，落点见 Calibration.h ARM_LOW_*）：降到标定过的固定低姿，之后靠
    // 车的前后移动把目标送进两指之间。不夹爪、不动轮子。
    exec::arm_low();
  } else if (!strcmp(act_, "clip")) {
    clear_active();
    s_grip = GRIP_CLOSE; write_grip();
  } else if (!strcmp(act_, "grasp")) {
    // 夹取+抬臂组合（合爪 → 等舵机合到位 → 定量抬升）：一步到位省一轮云端往返。
    // 抬升量有界（GRASP_LIFT_CM 或调用方给的 dist_cm），不会像持续 lift_up 那样顶到机械止点抽搐。
    clear_active();
    s_grip = GRIP_CLOSE; write_grip();
    s_grasp_lift_cm = (int16_t)(has_dist && dist > 0 ? dist : (int)GRASP_LIFT_CM);
    s_grasp_lift_at = millis() + (unsigned long)GRASP_SETTLE_MS;
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
  s_spin_settle_until = 0;
  s_spin_goal = 0; s_spin_dir = 0; s_spin_speed = 0; s_spin_comp = 0;  // move 接管：清掉旋转补偿目标
  // 运动到位验证：车轮在动即开始检测（画面无变化判定受阻；手动摇杆同模式重发不重置）。
  if (fabsf(th) > 0.001f) mvfy::begin("move");
  else { mvfy::end(); mvfy::consume(); }
  // 带 distance_cm = 定距微操：本板无里程计，按时长近似自停。实测 actual≈v·t+c，
  // v/c 都随油门插值标定表（对油门不敏感），时长=(cm-c)/v。
  // ⚠️ 再补死区与脉冲下限（见 Calibration.h 的 MV_START_MS）：v/c 表拟合自长脉冲，短脉冲整段落在
  // 起步死区里 —— 车一动不动，而 car_update_pose 照样按 distance_cm 累加 → AI 记忆与实际分家。
  int cm = p["distance_cm"] | 0;
  float a = fabsf(th); if (a > 1.0f) a = 1.0f;
  if (cm > 0 && a > 0.001f) {
    float v = plerp(MV_SPEED_X, MV_SPEED_Y, 3, a);
    float c = plerp(MV_COAST_X, MV_COAST_Y, 3, a);
    float target = cm > c ? cm - c : 0.0f;
    long ms = MV_START_MS + (long)(target / v * 1000.0f);
    if (ms < MV_MIN_PULSE_MS) ms = MV_MIN_PULSE_MS;
    exec::set_move_cap_ms((int)ms);
  }
}

static void send_stop(const JsonObjectConst& p) {
  const char* scope = p["scope"] | "all";
  if (!strcmp(scope, "arm")) {
    clear_active();
    clear_grasp_pending();
  } else {
    drive_motors(0);
    s_car_motion = 0;
    s_spin = 0;
    s_spin_settle_until = 0;
    s_spin_goal = 0; s_spin_dir = 0; s_spin_speed = 0; s_spin_comp = 0;  // stop 轮子：不再做旋转补偿
    mvfy::end(); mvfy::consume();  // 停止轮子：结束到位验证，释放帧缓冲预算
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
  s_arm_rej.armed = false;   // 诊断只反映"最近这一次 arm_pose"：先清，下面按本轮结果重新置位
  s_arm_rej.reason = 0;      // 原因同清：否则下面判 armed 时会沿用上一轮的原因(如上次的 4)而误报
  bool clamped = bivar::arm_clamp(&x, &h);
  // 夹到盒内后戳地(h<0)已不可达（盒 h 最小>0），这里仅兜底负高度竖直挤压情形。
  float reach = 0, lift = 0;
  if (!ik_refine(x, h, &reach, &lift)) return false;  // 反查初值即出盒→不可达
  int rr = (int)roundf(reach), ll = (int)roundf(lift);     // 左=移爪 reach / 右=抬落 lift
  // 反解在稀疏角区外插可能越物理限位：夹到舵机限位继续执行而非拒绝，让标定盒内的位姿总能
  // 尽力移到组件物理边界（否则 IDW 一外插超限就被当"不可达"卡住）。撞物理限位也算夹紧记进诊断。
  bool phy_r = false, phy_l = false;
  if (rr < REACH_LO) { rr = REACH_LO; phy_r = true; }
  else if (rr > REACH_HI) { rr = REACH_HI; phy_r = true; }
  if (ll < LIFT_LO) { ll = LIFT_LO; phy_l = true; }
  else if (ll > LIFT_HI) { ll = LIFT_HI; phy_l = true; }
  // 联动：一次同时给左右两舵机目标。set_servo 内部会限 pwm 并同步状态。
  bool ok = set_servo(1, (uint16_t)rr) & set_servo(2, (uint16_t)ll);
  // 到位校验：反解只在物理限位处才会失败，弯带外的位姿则会"尽力挪一下"就停住（见 ARM_POSE_TOL_CM
  // 注释）。这里用正运动学回读实际落点，偏离请求即记为诊断，让 AI 知道"这条位姿做不到"而不是
  // 只看状态行的绝对位置、反复重发同一条够不着的指令。
  bivar::arm_fk((float)rr, (float)ll, &s_arm_rej.ax, &s_arm_rej.ah);
  s_arm_rej.rr = rr; s_arm_rej.ll = ll;
  s_arm_rej.t_ms = millis();
  // reason 必须与本轮 armed 一起判定（曾漏写：仅 phy 命中时不更新 reason，于是报告沿用上一次的
  // 原因甚至初值 -1，AI 拿到的是错误的解释）。
  if (clamped) { s_arm_rej.reason = 1; s_arm_rej.cx = x; s_arm_rej.ch = h; }
  else if (phy_r) s_arm_rej.reason = 2;
  else if (phy_l) s_arm_rej.reason = 3;
  else if (fabsf(s_arm_rej.ax - x) > ARM_POSE_TOL_CM || fabsf(s_arm_rej.ah - h) > ARM_POSE_TOL_CM)
    s_arm_rej.reason = 4;
  s_arm_rej.armed = clamped || phy_r || phy_l || s_arm_rej.reason == 4;
  return ok;
}

bool exec::arm_low(void) {
  // 低姿夹取准备位：固定落到标定过的低姿（Calibration.h ARM_LOW_*），不夹爪、不动轮子。
  // 存在意义：低处可达域很窄，让上层凭坐标猜 (x,h) 会猜到够不着的地方、空夹且无报错；改成一个
  // 保证够得着的固定位，再用车的前后移动把目标送进两指之间（机械臂不能左右移动）。
  clear_active();   // 与其它 arm 指令一致：先清残留持续步进，否则首拍会朝旧方向补一枪
  s_folded = false;
  bool ok = exec::arm_pose(ARM_LOW_X_CM, ARM_LOW_H_CM);
  // 逻辑坐标锚到实际落点（连续步进起点/后续 diagnose 的基准），别沿用上一动作的旧值。
  float rx = 0, rh = 0;
  if (exec::arm_pos(&rx, &rh)) { s_claw_x = rx; s_claw_h = rh; }
  else { s_claw_x = ARM_LOW_X_CM; s_claw_h = ARM_LOW_H_CM; }
  return ok;
}

bool exec::arm_pos(float* x, float* h) {
  // 与 read_state 同源：报"当前命令的 pwm"的 FK（不是缓动途中的中间值），上层据此判"爪是否真的
  // 停在目标身上"时，看到的和自己读状态行是同一个数。标定未就绪返回 false（调用方按未知处理）。
  float fx = 0, fh = 0;
  if (!bivar::arm_fk((float)s_reach, (float)s_lift, &fx, &fh)) return false;
  if (x) *x = fx < 0 ? 0.0f : fx;
  if (h) *h = fh;
  return true;
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
    clear_grasp_pending();
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
  // 只在诊断新鲜时挂（见 ARM_DIAG_FRESH_MS）：它是"刚下的那条指令的结果"，过期的别重复报。
  if (s_arm_rej.armed && millis() - s_arm_rej.t_ms <= ARM_DIAG_FRESH_MS) {
    char d[160];
    if (s_arm_rej.reason == 1) {
      snprintf(d, sizeof(d), " | 撞边界[已移至最近点(%.1f,%.1f)]", s_arm_rej.cx, s_arm_rej.ch);
    } else if (s_arm_rej.reason == 4) {
      snprintf(d, sizeof(d), " | 位姿没到位[请求(%.1f,%.1f)只到(%.1f,%.1f): 该位姿够不着, 换位姿或挪车]",
               s_arm_rej.x, s_arm_rej.h, s_arm_rej.ax, s_arm_rej.ah);
    } else {
      const char* rc = s_arm_rej.reason == 2 ? "移爪越限" : "抬落越限";
      snprintf(d, sizeof(d), " | 目标(%.1f,%.1f)不可达[%s] 反解reach=%d lift=%d",
        s_arm_rej.x, s_arm_rej.h, rc, s_arm_rej.rr, s_arm_rej.ll);
    }
    if (strlen(buf) + strlen(d) + 1 < cap) strcat(buf, d);
  }
  return buf[0] != '\0';
}
