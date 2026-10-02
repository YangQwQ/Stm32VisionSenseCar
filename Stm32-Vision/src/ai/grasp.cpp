#include "src/ai/grasp.h"

#include <ArduinoJson.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"

#include "src/ai/track.h"       // 本地追踪: 提供目标实时 (u,v)
#include "src/ai/ground_proj.h" // 单应：画面(u,v) ↔ 地面cm（用单应算"每厘米搬多少 v"的初值）
#include "src/ai/ai_client.h"   // ai::generation / ai::logf
#include "src/ai/ai_nav.h"      // ai::settle_wheels / settle_arm
#include "src/ai/ai_mem.h"      // ai::car_update_pose（保持会话位姿不被拉偏）
#include "src/ai/ai_alloc.h"    // g_js_alloc
#include "src/exec/direct_exec.h"
#include "src/core/board_log.h"
#include "Calibration.h"

namespace grasp {

static SemaphoreHandle_t s_mtx = nullptr;
static SemaphoreHandle_t s_notify = nullptr;
static TaskHandle_t s_task = nullptr;
static char s_last_result[64] = "尚未夹取过";   // 上次结果文本（AI 侧回执用，见 grasp.h）

struct Req { bool pending; float x, y, w, h; char name[40]; };
static Req s_req;
static volatile bool s_running = false;

enum class R : uint8_t { Grasped, Empty, Lost, Interrupted, TimedOut, NoFrame, NoProgress };

// 下一条轮子动作并**等它真正起转再等停稳**：exec::act 只置命令，实际起转在 update_tick 下一拍；
// 不等起转就 settle_wheels 会立刻返回，抓到的是动作前的旧画面（实测：连续几次"转了却没变化"）。
// 返回**动作停止的时刻**（millis）—— 调用方据此等"一帧动作之后的画面被追踪器处理过"。
static uint32_t act_wheels(unsigned long gen, const char* type, JsonDocument& p) {
  exec::act(type, p.as<JsonObjectConst>());
  ai::car_update_pose(type, p.as<JsonObjectConst>());
  for (int i = 0; i < GRASP_MOTION_START_MS / 20 && !exec::wheels_moving(); i++)
    vTaskDelay(pdMS_TO_TICKS(20));
  ai::settle_wheels(gen, 1500);
  return millis();
}

// 等"一帧**动作之后**拍的画面被追踪器处理过"，再读最新位置。位置不再由本循环自己抓帧
// （相机只有 ~4~6fps，"抓一帧"就要等 ~200ms；以前还要丢帧冲积压 ⇒ 每步光这一段 ~600ms）。
// 现在由 app_httpd 的 track_feed 连续喂帧，这里只等它更新到 t_motion_end 之后。
// ⚠️ 还必须等"一个**没读过的**新更新"（consumed 记录上次读到的时刻）：否则跟丢重试时会反复读同一帧，
//    12 次"连续跟丢"会在几十毫秒里数完、直接秒判中止。
static void wait_fresh_pos(uint32_t t_motion_end, uint32_t& consumed) {
  const uint32_t need = (uint32_t)(t_motion_end + GRASP_FRESH_MARGIN_MS);
  const uint32_t t0 = millis();
  for (;;) {
    const uint32_t lu = track::last_update_ms();
    if ((int32_t)(lu - need) >= 0 && lu != consumed) { consumed = lu; return; }
    if ((uint32_t)(millis() - t0) >= (uint32_t)GRASP_FRESH_WAIT_MS) return;   // 超时：就用当前值
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// （已删 grab_fresh：帧改由 app_httpd 的 track_feed 连续喂给追踪器，夹取循环不再自己抓帧 ——
//  原来这里"丢 2~3 帧再取"每步要等 ~600ms 相机出帧，是当时的最大开销。）

// 合爪后的**判定**（回给 AI 参考，不驱动任何动作）：抬臂后在"地面原位"与"夹持位邻域"各算外观分。
//   · **确定未夹住**：地面原位那块**确实还很像目标**（≥ GRASP_EMPTY_MIN）且**明显更像**夹持位 ⇒ 方块还在地上。
//   · 其它一律报"**可能已夹住**" —— 宁可漏报"没夹住"，也不误报（误报会让 AI 以为没夹住而白重试）。
// ⚠️ 曾经的教训：① 用"抬臂后位置变没变"判 —— 跟踪器抬臂途中常跟丢、又锁回地面，连续误报；
//   ② 用固定夹持位单点判 —— 夹持位随车姿俯仰漂，又误报。所以现在：位置改在邻域里取最大，且只在证据强时才下结论。
static R verify_grasp(unsigned long gen, float u0, float v0, uint32_t& consumed) {
  const uint32_t t_lift = millis();
  track::probe_set(u0, v0, GRASP_HOLD_U, GRASP_HOLD_V);
  int valid = 0;
  bool empty_evidence = false;
  for (int i = 0; i < 8 && valid < 3; i++) {
    if (gen != ai::generation()) { track::probe_off(); return R::Interrupted; }
    wait_fresh_pos(t_lift, consumed);
    if (!track::last_ok()) continue;              // 只取"本帧被采纳"的（被拒帧的画面是旧的）
    const float ag = track::probe_appear0();      // 地面原位
    const float ah = track::probe_appear1();      // 夹持位邻域取最大
    valid++;
    const bool empty_now = (ag >= GRASP_EMPTY_MIN && ag > ah + GRASP_HOLD_MARGIN);
    ai::logf("[grasp] 合爪判定: 地面原位%.2f  夹持位%.2f ⇒ %s (%d/3)",
             (double)ag, (double)ah, empty_now ? "确定未夹住" : "可能已夹住", valid);
    if (empty_now) empty_evidence = true;
    else empty_evidence = false;                  // 只要有一帧不支持，就不敢说"确定"
  }
  track::probe_off();
  if (valid == 0) ai::logf("[grasp] 合爪判定: 没拿到有效帧 ⇒ 按可能已夹取上报");
  return empty_evidence ? R::Empty : R::Grasped;
}

static R run(unsigned long gen) {
  // ①a 先**松爪**：上一次夹取（或手动点了合爪）可能把爪合着，不松开就套不住方块
  //（用户实测：直接 arm low 时爪子还夹着，导致后面合爪等于空夹）。
  { JsonDocument p(&g_js_alloc); p["act"] = "release"; exec::act("arm", p.as<JsonObjectConst>()); }
  ai::settle_arm(gen, 1200);
  if (gen != ai::generation()) return R::Interrupted;
  // ①b 再降到低姿准备位（机械臂不能左右移动，全靠车的前后/转向把目标送进两指之间）
  { JsonDocument p(&g_js_alloc); p["act"] = "low"; exec::act("arm", p.as<JsonObjectConst>()); }
  ai::settle_arm(gen, 1500);
  if (gen != ai::generation()) return R::Interrupted;

  // 本次夹取期间**禁用追踪器的宽搜重捕**：跟丢就是跟丢，别去瞎找回（见 track.h）。
  track::set_reacquire(false);

  const uint32_t t0 = millis();
  int lost = 0, no_prog = 0;
  int spins = 0;               // 自上次"已对准"以来累计转了几次（超 GRASP_SPIN_BUDGET = u 收不进来，别再瞎转）
  int aligned_n = 0;            // 连续"已对准"的帧数（见 GRASP_ALIGN_N）
  int last_dir = 0;             // 上次转向方向（用于反向减半的阻尼）
  float last_err = -1.0f;
  float last_eu_err = -1.0f;   // u 对准阶段的独立进度（旋转时 v 会因视角波动，不应混入总误差）
  bool was_aligning = true;     // 上一轮是否在对准阶段（切换阶段时重置进度计数）
  bool force_grasp = false;     // u 收不动但 v 已到位 ⇒ 放弃 u 直接合爪（见下面的 NoProgress 分支）
  uint32_t consumed_upd = 0;          // 已读过的追踪更新时刻（wait_fresh_pos 用来等"新的一帧"）
  uint32_t t_motion_end = millis();   // 最近一次动作"停稳"的时刻：等新位置时以它为界（见 wait_fresh_pos）
  int back_n = 0;                     // 已后退步数（上限 GRASP_BACK_MAX_N，防前后来回蹭）
  bool tried_back_for_stuck = false;  // "u 转不动"时是否已退过一步换视角
  float u_before_action = -1.0f;      // 上一步旋转前的 u（用于"动作后目标必须动"的判据）
  int   stuck_n = 0;                  // 连续"转了但 u 没动"的次数
  int   stuck_v_n = 0;                // 连续"走了但 v 没动"的次数（同一条物理判据的平移版）
  int motion_cmds = 0;              // 本次夹取里已发过的动作数（合爪前闸用：发过动作却零响应 ⇒ 锁错目标）
  bool  saw_resp = false;             // 目标是否**对某次动作做出过合理响应**（0=从没动过 ⇒ 多半锁在夹爪/影子上）
  bool  was_lost = false;             // 上一帧是否处于"不在 Tracking"（用来抓"丢失→重捕"那次跃迁）
  bool  req_verify = false;           // 当前这个锁是**重捕刚抓回来**的 ⇒ 合爪前必须先做一次平移验证
  // 控制量→画面位移的**实测增益**（自适应）：越近，同样的 cm 搬动的 v 越多（真机实测 4cm 的 Δv：
  // 0.051 → 0.074 → 0.125 递增），固定步长上限必然在最后一步冲过头（#10 v=0.562 越过 0.53 → 转而纠 u → 跟丢）。
  // 用实测值反推步长；同一个增益还当作"下一帧目标会挪到哪"的先验喂给追踪器（track::hint_motion）。
  float dv_per_cm = 0.012f;     // v/厘米（初值：约 30~40cm 处 4cm≈0.05）
  float du_per_cm = 0.0f;       // **前进的横向漂移** u/厘米（实测学习；0=还没测到、不补偿）
  float du_per_deg = 0.0055f;   // u/度（初值：真机实测）
  bool  dv_inited = false;      // dv_per_cm 是否已用单应算过初值（只算一次，之后交给实测）
  float pend_v = -1.0f, pend_cm = 0.0f;    // 上一步"前进"的在途测量（-1 = 无）
  float pend_um = -1.0f;                   // 上一步"前进"之前的 u（量横向漂移用）
  float pend_u = -1.0f, pend_deg = 0.0f;   // 上一步"原地转"的在途测量
  for (int it = 0; it < GRASP_MAX_ITERS; it++) {
    if (gen != ai::generation()) return R::Interrupted;
    if ((uint32_t)(millis() - t0) > GRASP_TIME_MS) {
      ai::logf("[grasp] 超时(%.0fs), 中止", (double)((millis() - t0) / 1000.0));
      return R::TimedOut;
    }
    // 位置由图传任务连续喂帧（见 wait_fresh_pos 注释）。这里只等"动作之后的新位置"就绪再读。
    const uint32_t t_it = millis();
    wait_fresh_pos(t_motion_end, consumed_upd);
    const uint32_t t_gf = millis();

    float lu = 0, lv = 0;
    track::last_raw_center(&lu, &lv);          // 无有效中心时用它当"上次位置"显示
    track::Result tr;
    tr.st = track::state();
    tr.conf = track::last_conf();
    tr.u = lu; tr.v = lv;
    // ⚠️ 必须同时要求**本帧被采纳**(last_ok)：被 PSR 拒掉的帧状态仍是 Tracking、位置还是上一帧旧值，
    // 只看 state 会照着旧位置决策（真机实测：conf=0.00 的帧也被当成"跟上了"）。
    tr.ok = track::last_ok() && (tr.st == track::State::Tracking) && track::last_center(&tr.u, &tr.v);
    // 每步耗时拆解：现在没有"抓帧"了，只剩"等新位置"（相机 ~5fps ⇒ 一帧周期 ~200ms）+ 追踪耗时。
    ai::logf("[grasp] #%d 耗时 等新位置%ums 追踪(取图%d/搜索%d)ms 目标%.0fpx 峰偏移%.2f 外观%.2f",
             it + 1, (unsigned)(t_gf - t_it), track::last_decode_ms(), track::last_track_ms(),
             (double)track::last_obj_px(), (double)track::last_off_frac(), (double)track::last_appear());
    if (!tr.ok) {
      if (++lost >= GRASP_LOST_MAX) {
        ai::logf("[grasp] 连续 %d 帧跟丢目标, 中止", lost);
        return R::Lost;
      }
      // conf 是判据：0.4~0.65 = "像但不够像"（多半是部分遮挡/视角变了，窗口没跑出去）；
      // ≈0/-1 = 画面里根本没有（全遮挡或出画）。两者修法完全不同，别再靠猜。
      // 试到哪(u,v)+倍率 进一步区分：位置还在附近而分低=外观/尺度退化；位置乱跳=搜索被骗；
      // 倍率贴着 2.4 上限=目标已大到超出尺度范围。
      ai::logf("[grasp] #%d 本帧未跟上(%s) conf=%.2f 试(%.3f,%.3f) 倍率%.2f, 重试", it + 1,
               track::state_name(tr.st), (double)tr.conf, (double)tr.u, (double)tr.v,
               (double)track::last_scale());
      continue;
    }
    lost = 0;

    // ★ 抓"丢失→重捕"那次跃迁：重捕是**宽搜**，抓到什么都可能（真机实测：常抓到"恰好停在合爪点
    //   附近"的夹爪/反光 ⇒ 看着到位却夹空），而且它没经过任何动作验证。所以标一下，合爪前补一次验证。
    {
      const bool in_track = (tr.st == track::State::Tracking);
      if (in_track && was_lost) {
        req_verify = true;
        ai::logf("[grasp] 本锁来自**重捕**, 合爪前会先做一次平移验证");
      }
      was_lost = !in_track;
    }

    // ★ "动作后目标必须动"：上一步若是旋转，这一帧的 u 应该明显变过；没变 ⇒ 目标不搭理我（多半锁到了
    // 随车一起动的夹爪）。连续 GRASP_STUCK_N 次就交给下面的对准分支去"退一步换视角 / 放弃 u"。
    // ★ 同平移判据：用**相对期望位移**（转 pend_deg 度、u 本该动 du_per_deg×pend_deg）而非绝对阈值，
    //   避免锁住时那点抖动/透视漂移把计数清零。pend_deg 此刻仍是上一步旋转的角度（下面 pend 块才清）。
    if (u_before_action >= 0.0f) {
      const float du = fabsf(tr.u - u_before_action);
      const float expect_u = du_per_deg * pend_deg;
      if (expect_u > 0.008f && du < 0.30f * expect_u) stuck_n++; else stuck_n = 0;
      u_before_action = -1.0f;
    }

    // 用**实测位移**刷新增益（指数平滑抗单帧噪声）。旋转时 v 会因视角波动、前进时 u 也会抖，
    // 故各自只取自己那根轴；上一条动作只有一条在途，所以两个 pend 不会同时有效。
    if (pend_cm > 0.5f && pend_v >= 0.0f && pend_um >= 0.0f) {      // ★ 平移版物理判据：走了 cm 之后 v 该变；没变 ⇒ 跟的物体不随平移动（多半是随车一起动的夹爪）。
      // 真机实测：连退 12cm（车位置 -3→-12）而 v 纹丝不动（0.753→0.741），旧逻辑却继续退、白退 4 步。
      // ★ 判据用**相对期望位移**而不是绝对阈值：走了 pend_cm 厘米，v 本该变 dv_per_cm×pend_cm。
      //   实测远小于期望（<35%）才算"没动"——绝对阈值（旧 <0.005）会被锁住时那点抖动/透视漂移骗到，
      //   真机实测就是这样一路把 stuck_v_n 清零、退满上限后掉进"合爪"。
      const float dvv = fabsf(tr.v - pend_v);
      const float expect_mag = fabsf(dv_per_cm * pend_cm);
      if (expect_mag > 0.03f && dvv < 0.35f * expect_mag) stuck_v_n++; else stuck_v_n = 0;
      // "响应"要求既达预期 35%、又超过绝对底噪 0.02（否则把小抖动误判成"目标动了"）。
      if (expect_mag > 0.03f && dvv > 0.35f * expect_mag && dvv > 0.02f) saw_resp = true;
      const float kv = (tr.v - pend_v) / pend_cm;
      if (kv > 1e-4f && kv < 0.5f) dv_per_cm = 0.5f * dv_per_cm + 0.5f * kv;
      const float ku = (tr.u - pend_um) / pend_cm;   // 前进的横向漂移（保号：走正了也要能学到负漂移）
      if (fabsf(ku) < 0.05f) du_per_cm = 0.5f * du_per_cm + 0.5f * ku;
      pend_cm = 0.0f; pend_v = -1.0f; pend_um = -1.0f;
    }
    if (pend_deg > 0.5f && pend_u >= 0.0f) {
      const float du = fabsf(tr.u - pend_u);
      const float k = du / pend_deg;
      if (k > 1e-4f && k < 0.2f) du_per_deg = 0.5f * du_per_deg + 0.5f * k;
      // 旋转响应：转了 pend_deg 度，u 至少该按 du_per_deg 动。实测达预期 30% 即算"搭理我了"
      //（真目标 ~0.0055/度、8° 就 ~0.044；夹爪≈0 → 判为从没响应过）。
      const float expect_u = du_per_deg * pend_deg;
      if (expect_u > 0.008f && du > 0.30f * expect_u && du > 0.01f) saw_resp = true;
      pend_deg = 0.0f; pend_u = -1.0f;
    }

    const float eu = tr.u - GRASP_U_TGT;   // >0 = 物体偏右, 需右转(dir=1)
    const float ev = tr.v - GRASP_V_TGT;   // >0 = 物体偏低(太近), 需后退
    ai::logf("[grasp] #%d u=%.3f v=%.3f conf=%.2f (差 %+.3f,%+.3f)", it + 1,
             (double)tr.u, (double)tr.v, (double)tr.conf, (double)eu, (double)ev);

    // 对准目标就是名义值本身。（曾试过"预补偿前进的横向漂移"：把对准目标先挪开 du_per_cm×cm，
    // 指望走完正好回到名义值 —— 已删。那个漂移的符号随"车头与目标方位夹角"变化，是**有号的量**，
    // 当成固定标量去学会把误差翻倍：真机实测设 0.475、结果被带到 0.430。）
    const float u_aim = GRASP_U_TGT;
    const float eu_aim = tr.u - u_aim;     // 与 eu 同（保留变量名便于后续再引入补偿）
    // u 容差**随距离自适应**：v 是距离代理（v 越大越近）。远处放宽(反正每走一步都会重对，精转是浪费)、
    // 进了可夹纵深(GRASP_V_TGT)收紧到最严。见 Calibration.h 的说明。
    float u_tol_now = GRASP_U_TOL;
    {
      const float v0 = GRASP_U_TOL_V_FAR, v1 = GRASP_V_TGT;
      float k = (v1 > v0) ? (tr.v - v0) / (v1 - v0) : 1.0f;
      if (k < 0.0f) k = 0.0f; else if (k > 1.0f) k = 1.0f;
      u_tol_now = GRASP_U_TOL_FAR + (GRASP_U_TOL_NEAR - GRASP_U_TOL_FAR) * k;
    }
    // 居中优先：先朝 容差×AIM 收；连续 2 次没进展再退到整容差接受（否则"一进带就停"会停在带边上）。
    float u_tol_use = (no_prog >= 2) ? u_tol_now : (u_tol_now * GRASP_U_TOL_AIM);
    // ★ 容差**不得小于车能迈出的最小半步**：最小转角搬 min_step_u 的 u，落点只可能落在间距 min_step_u
    //   的网格上。容差若低于半步，就没有任何网格点能进带 ⇒ 车一步跨过、停在带外侧（真机实测：终点 u
    //   恒定偏 +0.019 ≈ 半个最小步，从右侧接近就总偏右）。把容差垫到半步 = "接受最近的那个网格点"，
    //   落点误差收敛到 ≤半步，且不再出现"跨过去→反向→再跨过去"的空转。
    const float min_step_u = du_per_deg * (float)GRASP_SPIN_MIN_DEG;
    float u_floor = 0.5f * min_step_u;
    // ★ 已进可夹纵深(|ev|≤V_TOL)时放宽到**一整个最小步**：目标贴近旋转轴 ⇒ 转这一步几乎不改变 u
    //   （真机实测 #11~#13 连转三次、车向 46→51→56，u 恒 0.471）。此时再抠半步只会空转（抠也抠不动），
    //   还平白让夹爪贴着物体原地扭 —— 直接接受更划算。|eu| 大于一步时仍照常转/走换视角逻辑。
    if (fabsf(ev) <= GRASP_V_TOL) u_floor = min_step_u;
    if (u_tol_use < u_floor) u_tol_use = u_floor;

    // 进度闸：分阶段检查，避免"旋转对准时 v 视角波动"误杀。
    //   对准阶段：只看 |eu| 是否下降（v 波动是正常的）
    //   前进阶段：只看 |ev| 是否下降
    //   切换阶段时重置计数（新旧指标不可比）
    const bool aligning = !force_grasp && (fabsf(eu_aim) > u_tol_use);
    if (aligning) {
      // ★ 旋转预算：转这么多次 u 还收不进来 ⇒ 锁的不是能靠转身对准的东西（或目标贴在旋转轴上）。
      //   旧路径会一直转下去（真机实测转 38 次、车头 -54°→-114° 而 u 恒定 0.512），用户要的是"老实失败"。
      if (++spins > GRASP_SPIN_BUDGET) {
        ai::logf("[grasp] 已转 %d 次 u 仍对不上(%.3f) ⇒ 不再瞎转, 中止", spins, (double)fabsf(eu_aim));
        return R::NoProgress;
      }
      // 对准阶段：只跟踪 u 误差
      const float eu_err = fabsf(eu_aim);
      if (last_eu_err >= 0.0f && eu_err > last_eu_err - 0.003f) {
        if (++no_prog >= GRASP_NO_PROG_N || stuck_n >= GRASP_STUCK_N) {
          // u 一直收不进来：若 v 已经到位（物体已在可夹纵深），就别继续跟 u 死磕，直接去合爪。
          // 真机实测：DCF 漂到"随车一起动的夹爪"上时 u 恒定不变、永远对不上，而 v 明明已经到位
          //（用户看着"其实可以夹了"）—— 此时放弃 u 比空耗到超时/中止好。
          if (fabsf(ev) <= GRASP_V_TOL) {
            // u 怎么转都不变 ⇒ 多半锁到了"随车一起动的东西"（夹爪）。用户建议：先退一步换视角再试
            //（改视角 / 改目标在画面里的透视），退完重新判断；仍不动再按"v 已到位"直接合爪。
            if (!tried_back_for_stuck && back_n < GRASP_BACK_MAX_N) {
              tried_back_for_stuck = true;
              back_n++;
              JsonDocument sbp(&g_js_alloc);
              sbp["throttle"] = -GRASP_THROTTLE; sbp["steering"] = 0; sbp["distance_cm"] = GRASP_MOVE_MAX_BACK_CM;
              ai::logf("[grasp] u 连 %d 次不动(%.3f) → 先退 %dcm 换视角再试", no_prog, (double)eu_err,
                       GRASP_MOVE_MAX_BACK_CM);
              // 也登记在途测量：退完下一帧就能判"这一退 v 到底动没动"（若不动 ⇒ 锁错目标，合爪前闸兜住）
              pend_v = tr.v; pend_um = tr.u; pend_cm = (float)GRASP_MOVE_MAX_BACK_CM;
              motion_cmds++;
              t_motion_end = act_wheels(gen, "move", sbp);
              no_prog = 0;
              stuck_n = 0;
              continue;
            }
            // ★★ 物理闸（最后一道）：u 转不动有**两种**可能 ——
            //   ① 目标只是贴着旋转轴（真目标，放手去夹没问题）；② 锁到了随车固定的东西（夹爪/影子/反光）。
            //   两者靠画面分不开（真机实测：地面反光位把种子带偏后，锁到夹爪上时外观 0.54~0.58 一路过关），
            //   但**平移响应**能分开：真目标一前进/后退 v 就该变，随车固定的一动不动。
            //   上面那次"退一步换视角"就是一次平移测量（已登记 pend_cm）—— 若它没让 v 动，
            //   stuck_v_n 会 ≥1，那就说明是②，**不许放弃 u 硬合爪**。
            if (stuck_v_n >= 1) {
              ai::logf("[grasp] u 转不动且最近一次平移 v 也没动(%.3f) ⇒ 锁的是随车固定的东西, 不合爪, 中止",
                       (double)tr.v);
              return R::NoProgress;
            }
            ai::logf("[grasp] u 连续 %d 次对不上(%.3f) 但 v 已到位 → 放弃 u, 直接合爪", no_prog, (double)eu_err);
            force_grasp = true;
            no_prog = 0;
            stuck_n = 0;
          } else {
            ai::logf("[grasp] 连续 %d 次对准后 u 误差没变小(%.3f), 中止", no_prog, (double)eu_err);
            return R::NoProgress;
          }
        }
      } else {
        no_prog = 0;
      }
      last_eu_err = eu_err;
    } else {
      // 前进/到位阶段：跟踪总误差
      const float err = fabsf(eu) + fabsf(ev);
      if (last_err >= 0.0f && err > last_err - 0.005f) {
        if (++no_prog >= GRASP_NO_PROG_N) {
          ai::logf("[grasp] 连续 %d 次动作后误差没变小(%.3f) ⇒ 目标不随动作移动, 中止", no_prog, (double)err);
          return R::NoProgress;
        }
      } else {
        no_prog = 0;
      }
      last_err = err;
    }
    // 阶段切换时重置进度计数（避免用旧阶段的进度判新阶段）
    if (was_aligning != aligning) { no_prog = 0; was_aligning = aligning; }

    // ① 太近优先于对准：v 已冲过可夹点 ⇒ **先退开再说**，别在这里原地转。
    // 真机实测：一开局就贴着方块（v=0.848 ≈ 2cm）时先去做 u 对准，一转身夹爪就撞上方块，
    // 之后整段都在跟"随车一起动的夹爪"（u 恒定、conf 满格）。宁可先退几步把距离拉到正常区间。
    if (ev > GRASP_V_TOL) {
      // ★ 物理判据放最前、且**不受 back_n 上限限制**：退了 pend_cm 而 v 没动（相对判据见上），
      //   就说明跟的根本不是"能随车动的物体"（夹爪/影子/反光点）。
      //   旧代码把它关在 `back_n < GRASP_BACK_MAX_N` 里：退满 4 次后整段被跳过 → 直接掉到 ④"合爪"，
      //   真机实测就是这样贴着脸、锁着夹爪一路混到"已夹取"。
      if (stuck_v_n >= GRASP_STUCK_N) {
        ai::logf("[grasp] 太近且已退 %d 次但 v 纹丝不动(%.3f) ⇒ 锁的不是能随车动的物体(夹爪/影子/反光点?), 中止",
                 back_n, (double)tr.v);
        return R::NoProgress;
      }
      if (back_n < GRASP_BACK_MAX_N) {
        back_n++;
        int bcm = (int)(GRASP_MOVE_GAIN_CM * ev);
        if (bcm < GRASP_MOVE_MIN_CM) bcm = GRASP_MOVE_MIN_CM;
        if (bcm > GRASP_MOVE_MAX_BACK_CM) bcm = GRASP_MOVE_MAX_BACK_CM;
        JsonDocument bp(&g_js_alloc);
        bp["throttle"] = -GRASP_THROTTLE; bp["steering"] = 0; bp["distance_cm"] = bcm;   // 负油门 = 后退
        ai::logf("[grasp] 太近(v=%.3f 超 %.3f) → 后退 %dcm (%d/%d)", (double)tr.v, (double)ev, bcm,
                 back_n, GRASP_BACK_MAX_N);
        track::hint_motion(0.0f, -(dv_per_cm * (float)bcm));   // 后退 ⇒ 目标往 v 减小的方向挪
        pend_v = tr.v; pend_um = tr.u; pend_cm = (float)bcm;
        motion_cmds++;
        t_motion_end = act_wheels(gen, "move", bp);
        continue;
      }
      // 退到上限还是太近，而 v 并非"完全不动"（否则上面已中止）：说明确实贴得近却拉不开距离。
      // 不再静默掉进 ④"合爪"（那正是贴脸乱夹的来源），明确中止。
      ai::logf("[grasp] 已后退 %d 次仍太近(v=%.3f 超 %.3f), 中止", back_n, (double)tr.v, (double)ev);
      return R::NoProgress;
    }

    // ② 先对准 u（原地小角，分步收敛）
    if (!force_grasp && fabsf(eu_aim) > u_tol_use) {
      aligned_n = 0;
      int deg = (int)(GRASP_SPIN_GAIN * fabsf(eu_aim));
      if (deg < GRASP_SPIN_MIN_DEG) deg = GRASP_SPIN_MIN_DEG;
      if (deg > GRASP_SPIN_MAX_DEG) deg = GRASP_SPIN_MAX_DEG;
      // ★ 再按**追踪器当前搜索半径**收一层：这一步的预期位移 du/deg×角度 必须落在半径内，
      // 否则一步就把目标转出窗（这正是"半径该由预期位移定"的可落地做法）。
      float ru = 0, rv = 0;
      track::search_radius(&ru, &rv);
      if (du_per_deg > 1e-4f && ru > 0.0f) {
        int deg_cap = (int)(GRASP_RADIUS_FRAC * ru / du_per_deg);
        if (deg_cap < GRASP_SPIN_MIN_DEG) deg_cap = GRASP_SPIN_MIN_DEG;
        if (deg > deg_cap) deg = deg_cap;
      }
      const int dir = eu_aim > 0 ? 1 : -1;
      // 阻尼：本次方向与上次相反 ⇒ 上一脚转过头了，步长减半，避免在目标两侧来回蹭
      const bool flip = (last_dir != 0 && dir != last_dir);
      if (flip) { deg /= 2; if (deg < GRASP_SPIN_MIN_DEG) deg = GRASP_SPIN_MIN_DEG; }
      last_dir = dir;
      JsonDocument p(&g_js_alloc); p["dir"] = dir; p["speed"] = 800; p["angle_deg"] = deg;
      ai::logf("[grasp] 对准: 转%+d°%s (u容差±%.3f, 目标±%.3f)", dir * deg,
               flip ? "(反向, 步长减半)" : "", (double)u_tol_use, (double)u_tol_now);
      // 控制量先验：这次转完目标大约会横向挪 -du_per_deg*(dir*deg)，直接告诉追踪器下一帧往哪搜
      // （比它自己的图像预测可靠——那是按"转之前"那帧算的）。
      track::hint_motion(-du_per_deg * (float)(dir * deg), 0.0f);
      pend_u = tr.u; pend_deg = (float)deg;   // 记录在途测量：下一次成功帧用它刷新 du_per_deg
      u_before_action = tr.u;                 // 记录本次旋转前的 u：下一帧判"目标有没有搭理我"
      motion_cmds++;
      t_motion_end = act_wheels(gen, "spin", p);
      continue;
    }
    // 连续 GRASP_ALIGN_N 帧都对准才继续：单帧读数若是跳变/误匹配，会骗出"已对准"，导致在偏位合爪
    aligned_n++;
    spins = 0;                   // 已对准 ⇒ 旋转预算清零（下一个"对准→前进"循环重新计）
    if (aligned_n < GRASP_ALIGN_N) {
      ai::logf("[grasp] 对准待确认(%d/%d 帧)", aligned_n, GRASP_ALIGN_N);
      continue;
    }

    // ③ 再推进 v（边前进边保持 u 对准：下一轮会先纠 u）。
    //    **只前进、绝不后退**：用户实测到 v≈GRASP_V_TGT 时方块离夹爪还有十几厘米，继续前进才对；
    //    后退只会越退越远（且"v 略超目标"多半是跟偏，不该用后退去追）。
    if (ev < -GRASP_V_TOL) {                         // ev<0 = 太远 → 前进
      // 平移版物理判据（同后退那条）：走了几次 v 都不变 ⇒ 跟的物体不随平移动，再走也是白走。
      if (stuck_v_n >= GRASP_STUCK_N) {
        ai::logf("[grasp] 已前进多次但 v 纹丝不动 ⇒ 跟踪对象随车不动(夹爪/它的影子/反光点?), 中止");
        return R::NoProgress;
      }
      // 首次前进：先用**单应**（画面↔地面）把"每厘米搬动多少 v"算出来当初值，比拍脑袋的常数准。
      // 只用一次：之后就由实测 Δv 继续修正 —— 单应解的是**地面**点，而追踪给的是块的**中心**（块有高度）
      // 且有车姿俯仰误差，存在系统性偏差；实测值能把这些一并吸收掉。
      if (!dv_inited) {
        dv_inited = true;
        float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
        if (ground::ready() && tr.v + 0.02f <= 1.0f &&
            ground::screen_to_world(tr.u, tr.v, &x1, &y1) &&
            ground::screen_to_world(tr.u, tr.v + 0.02f, &x2, &y2)) {
          const float dy = y2 - y1;                       // v 增 0.02 对应的前向 cm（负：v 越大越近）
          if (dy < -1e-3f && dy > -50.0f) dv_per_cm = fabsf(0.02f / dy);
        }
        ai::logf("[grasp] dv/cm 初值 %.4f (%s)", (double)dv_per_cm, ground::ready() ? "单应" : "缺省");
      }
      // 步长：增益给一个，再用**实测的每厘米 Δv** 把它压到"不会冲过目标"——只搬剩余误差的 70%，
      // 这样越近自动越小步（用户提的：越近每步该越小），而远处仍受 GRASP_MOVE_MAX_CM 限制不慢。
      float cm_f = GRASP_MOVE_GAIN_CM * (-ev);
      const float cm_cap = (dv_per_cm > 1e-4f) ? (0.7f * (-ev) / dv_per_cm) : (float)GRASP_MOVE_MAX_CM;
      if (cm_f > cm_cap) cm_f = cm_cap;
      // ★ 再按追踪器当前搜索半径收一层（v 方向）：一步的 Δv = dv/cm×cm 必须落在半径内，否则一步出窗。
      {
        float rvu = 0, rvv = 0;
        track::search_radius(&rvu, &rvv);
        if (dv_per_cm > 1e-4f && rvv > 0.0f) {
          const float cm_cap2 = GRASP_RADIUS_FRAC * rvv / dv_per_cm;
          if (cm_f > cm_cap2) cm_f = cm_cap2;
        }
      }
      // ★ 再按**实测横向漂移**收一层：一步前进会带偏 du_per_cm×cm，这个偏移必须留在 u 容差里 ——
      //   否则一步就把目标带出容差，而**贴近时旋转对 u 几乎无效**（目标离旋转轴很近，真机实测转 3 次
      //   u 恒定不动），那个"偏右"就再也纠不回来。见 Calibration.h 的 GRASP_DRIFT_BUDGET。
      if (u_tol_now > 0.0f) {
        const float du_est = (fabsf(du_per_cm) > 1e-4f) ? fabsf(du_per_cm) : GRASP_DRIFT_ASSUMED;
        const float cm_cap3 = GRASP_DRIFT_BUDGET * u_tol_now / du_est;
        if (cm_f > cm_cap3) cm_f = cm_cap3;
      }
      if (cm_f > (float)GRASP_MOVE_MAX_CM) cm_f = (float)GRASP_MOVE_MAX_CM;
      int cm = (int)(cm_f + 0.5f);
      if (cm < GRASP_MOVE_MIN_CM) cm = GRASP_MOVE_MIN_CM;
      JsonDocument p(&g_js_alloc); p["throttle"] = GRASP_THROTTLE; p["steering"] = 0; p["distance_cm"] = cm;
      ai::logf("[grasp] 前进: %dcm (dv/cm≈%.3f u漂移/cm≈%+.3f, 对准预补偿到 u=%.3f)",
               cm, (double)dv_per_cm, (double)du_per_cm, (double)u_aim);
      track::hint_motion(0.0f, dv_per_cm * (float)cm);   // 先验：目标会往 v 增大的方向挪这么多
      pend_v = tr.v; pend_um = tr.u; pend_cm = (float)cm;   // 在途测量：下次成功帧刷新 dv_per_cm / du_per_cm
      motion_cmds++;
      t_motion_end = act_wheels(gen, "move", p);
      continue;
    }

    // ★ 合爪前闸：本次夹取**发过动作**，但目标从未对任何一次动作做出合理响应 ⇒ "到位"是假象，
    //   多半一直锁在随车一起动的夹爪/影子/反光点上（真机实测：贴脸+锁夹爪时一路混到"已夹取"）。
    //   没发过动作（开局就对准、v 也在带内）则直接放行——那种情况本来就该立刻合爪。
    if (motion_cmds > 0 && !saw_resp) {
      ai::logf("[grasp] 已发 %d 次动作但目标从未随动作移动 ⇒ 锁错目标(夹爪/影子/反光?), 不合爪, 中止", motion_cmds);
      return R::NoProgress;
    }

    // ★ 重捕锁的平移验证：退一步，真目标 v 必然变小（随车动）；一动不动 ⇒ 锁的是随车固定的东西。
    //   验证通过就"退开了再走回来"（下一轮会重新前进到可夹点）—— 宁可多花两秒，也不要夹空。
    if (req_verify) {
      const float dv_exp = dv_per_cm * (float)GRASP_VERIFY_BACK_CM;
      ai::logf("[grasp] 重捕锁验证: 后退 %dcm (v 该降 %.3f)", GRASP_VERIFY_BACK_CM, (double)dv_exp);
      JsonDocument p(&g_js_alloc);
      p["throttle"] = -GRASP_THROTTLE; p["steering"] = 0; p["distance_cm"] = GRASP_VERIFY_BACK_CM;
      track::hint_motion(0.0f, -dv_exp);
      pend_v = tr.v; pend_um = tr.u; pend_cm = (float)GRASP_VERIFY_BACK_CM;
      motion_cmds++;
      const uint32_t tv = act_wheels(gen, "move", p);
      wait_fresh_pos(tv, consumed_upd);
      float vu = 0, vv = 0;
      if (track::last_ok() && track::state() == track::State::Tracking && track::last_center(&vu, &vv)) {
        const float dvv = vv - tr.v;
        if (dvv > -0.4f * dv_exp) {
          ai::logf("[grasp] 验证失败: 退 %dcm 后 v 只变了 %+.3f(该 %.3f) ⇒ 锁的是随车固定的东西, 中止",
                   GRASP_VERIFY_BACK_CM, (double)dvv, (double)(-dv_exp));
          return R::NoProgress;
        }
        ai::logf("[grasp] 验证通过: v 变了 %+.3f ⇒ 目标随车动, 继续", (double)dvv);
      } else {
        ai::logf("[grasp] 验证时没跟上(多半被遮挡) ⇒ 保守继续");
      }
      req_verify = false;
      continue;                    // 退开了：下一轮会重新前进到可夹点
    }

    // ④ 到位 → 合爪（exec 的 grasp 含合后自动抬臂）
    ai::logf("[grasp] 已到位 u=%.3f v=%.3f conf=%.2f → 合爪", (double)tr.u, (double)tr.v, (double)tr.conf);
    const float u_pre = tr.u, v_pre = tr.v;
    JsonDocument g(&g_js_alloc); g["act"] = "grasp";
    exec::act("arm", g.as<JsonObjectConst>());
    ai::settle_arm(gen, 1500);
    return verify_grasp(gen, u_pre, v_pre, consumed_upd);
  }
  ai::logf("[grasp] 迭代超限, 中止");
  return R::TimedOut;
}

static void worker(void*) {
  for (;;) {
    xSemaphoreTake(s_notify, portMAX_DELAY);
    Req r;
    {
      xSemaphoreTake(s_mtx, portMAX_DELAY);
      r = s_req;
      s_req.pending = false;
      xSemaphoreGive(s_mtx);
    }
    s_running = true;
    const unsigned long gen = ai::generation();
    ai::logf("[grasp] 开始: 目标「%s」画面(%.3f,%.3f) 框(%.3f,%.3f)",
             r.name, (double)r.x, (double)r.y, (double)r.w, (double)r.h);
    track::seed(r.name, r.x, r.y, r.w, r.h);
    const R res = run(gen);
    // 回执刻意用"可能/确定"两档：AI 侧据此决定要不要重试。只有证据强时才敢说"确定未夹住"。
    const char* txt = res == R::Grasped ? "已夹取(可能)" : res == R::Empty ? "确定未夹住(方块仍在地面原位)" :
                      res == R::Lost ? "跟丢目标" :
                      res == R::Interrupted ? "被打断" : res == R::NoFrame ? "取帧失败" :
                      res == R::NoProgress ? "目标不随动作移动/后退超限, 中止" : "超时未到位";
    ai::logf("[grasp] 结束: %s", txt);
    snprintf(s_last_result, sizeof(s_last_result), "%s", txt);   // 给 AI 侧当回执
    track::stop();          // 夹完/中止后不再跟踪该目标
    track::set_reacquire(true);   // 恢复默认：AI/手机侧的单点跟踪仍允许重捕
    s_running = false;
  }
}

void init() {
  if (s_task) return;
  s_mtx = xSemaphoreCreateMutex();
  s_notify = xSemaphoreCreateBinary();
  static StackType_t* s_stack = nullptr;
  static StaticTask_t s_tcb;   // TCB 必须留内部 RAM
  // ⚠️ xTaskCreate* 的 usStackDepth 单位是**字**(StackType_t=4B)，不是字节：声明 4096 字 ⇒ 要
  // 分配 4096*4=16KB。旧代码写成 malloc(8192 字节) 却声明 8192 字(32KB)——任务实际只有 8KB 栈，
  // 越界 24KB 写进相邻 PSRAM（栈哨兵也落在 32KB 处，检查不到），静默踩坏堆 → 偶发卡死/取帧失败。
  const int kStackWords = 4096;   // 16KB：够 run() + track::update_from_fb 的调用深度
  s_stack = (StackType_t*)heap_caps_malloc((size_t)kStackWords * sizeof(StackType_t), MALLOC_CAP_SPIRAM);
  if (s_stack) {
    s_task = xTaskCreateStaticPinnedToCore(worker, "grasp", kStackWords, nullptr, 3, s_stack, &s_tcb, 1);
  } else {
    xTaskCreatePinnedToCore(worker, "grasp", kStackWords, nullptr, 3, &s_task, 1);
  }
  blog::logf(blog::AI, "[grasp] worker 就绪");
}

bool request(float x, float y, float w, float h, const char* name) {
  if (!s_mtx) return false;
  if (!(x >= 0.0f && x <= 1.0f && y >= 0.0f && y <= 1.0f)) return false;
  xSemaphoreTake(s_mtx, portMAX_DELAY);
  if (s_req.pending || s_running) { xSemaphoreGive(s_mtx); return false; }
  s_req.x = x; s_req.y = y; s_req.w = w; s_req.h = h;
  const char* nm = (name && name[0]) ? name : "目标";
  strncpy(s_req.name, nm, sizeof(s_req.name) - 1);
  s_req.name[sizeof(s_req.name) - 1] = 0;
  s_req.pending = true;
  xSemaphoreGive(s_mtx);
  xSemaphoreGive(s_notify);
  return true;
}

bool busy() {
  if (!s_mtx) return false;
  return s_req.pending || s_running;
}

void cancel() { ai::cancel(ai::StopMode::All); }   // 代际号自增 → run 循环下一拍退出

const char* last_result() { return s_last_result; }

}  // namespace grasp
