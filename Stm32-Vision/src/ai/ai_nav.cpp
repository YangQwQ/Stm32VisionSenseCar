#include "src/ai/ai_nav.h"
#include "src/ai/ai_client.h"   // ai::logf / ai::generation
#include "src/ai/ai_mem.h"      // s_car_x/y/heading, AI_PI, car_update_pose
#include "src/ai/ai_alloc.h"    // g_js_alloc
#include "src/exec/direct_exec.h"

#include <math.h>               // fabsf/fminf/sqrtf/atan2f/fmodf
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 等待轮子停下的检测循环数(约 50ms 一拍)。
#define NAV_WAIT_LOOPS 120
// 轮子 PWM 停稳后固定缓冲: 余速滑行(MV_COAST)/补转已结束但车身惯性还在缓慢漂, 立即抓帧仍会取到
// 模糊帧 → AI 空等稳定浪费一轮。
#define AI_SETTLE_DWELL_MS 60
// 机械臂静止后固定缓冲: 让舵机把动作走完收尾(S 形缓动/持续步进/grasp 合爪已收敛后)。
#define AI_ARM_DWELL_MS 50

// 角度归一化到 (-180,180]; 仅本文件内用, 可内联。
static float wrap180f(float a) {
  a = fmodf(a, 360.0f);
  if (a > 180.0f) a -= 360.0f;
  else if (a < -180.0f) a += 360.0f;
  return a;
}

// 等待本轮定距/定角到段自停(loop 的 update_tick 会按时长停轮); 中断或超时即退出。
static void wait_wheels(unsigned long gen) {
  for (int i = 0; i < NAV_WAIT_LOOPS; i++) {
    if (gen != ai::generation()) return;
    if (!exec::wheels_moving()) return;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// 出帧前限时等停稳(最多 max_ms, 超时照常出帧)。到点自停后电机仍有余速滑行, 此刻抓帧会模糊、
// px/py 算偏。⚠️ wheels_moving 只认 PWM 命令(命令停 ≠ 车身停), 故等停后再固定 dwell 让残余慢漂走完。
void ai::settle_wheels(unsigned long gen, int max_ms) {
  for (int t = 0; t < max_ms; t += 20) {
    if (gen != ai::generation() || !exec::wheels_moving()) break;   // PWM 停稳即跳出(不等满 max_ms)
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  for (int d = 0; d < AI_SETTLE_DWELL_MS && gen == ai::generation(); d += 20)  // 停稳后缓冲残余慢漂
    vTaskDelay(pdMS_TO_TICKS(20));
}

// 出帧前等机械臂动作到位(上限 max_ms, 超时照常出帧)。wheels_moving 只盯轮子 PWM, 机械臂(S 形缓动/
// 持续步进/grasp 抬臂)一概不覆盖, 故需单独等到 arm_moving 收敛, 夹取+抬臂才落在两帧间隙里。
void ai::settle_arm(unsigned long gen, int max_ms) {
  for (int t = 0; t < max_ms; t += 20) {
    if (gen != ai::generation() || !exec::arm_moving()) break;   // 臂到位即跳出(不等满 max_ms)
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  for (int d = 0; d < AI_ARM_DWELL_MS && gen == ai::generation(); d += 20)  // 臂静止后再缓冲动作收尾
    vTaskDelay(pdMS_TO_TICKS(20));
}

// 本地巡航核心(不调云端): 从当前位姿导航到全局目标 (tx,ty), 距目标 ≤stop_cm 且正对时停。无里程计,
// 按定距/定角时长近似 + 姿态累积开环死航。返回 Reached/Interrupted/TimedOut。
ai::NavR ai::navigate_to(unsigned long gen, float tx, float ty, float stop_cm) {
  const float throttle = 0.5f;   // 巡航推进油门(中速)
  int spins = 0, flips = 0, last_dir = 0, backs = 0;
  for (int it = 0; it < NAV_MAX_ITERS; it++) {
    if (gen != ai::generation()) return NavR::Interrupted;
    float dx = tx - ai::s_car_x, dy = ty - ai::s_car_y;
    float dist = sqrtf(dx * dx + dy * dy);
    float thg = atan2f(dx, dy) * 180.0f / ai::AI_PI;        // 目标全局方位(heading=0 时朝 Y+)
    float rel = wrap180f(ai::s_car_heading + thg);          // 目标相对车头偏角, 正=右
    if (dist <= stop_cm && fabsf(rel) <= NAV_ALIGN_DEG) {   // 到位 = 距离够近 **且** 正对目标
      JsonDocument s(&g_js_alloc); s["scope"] = "all"; exec::act("stop", s.as<JsonObjectConst>());
      return NavR::Reached;
    }
    if (fabsf(rel) > NAV_ALIGN_DEG) {                   // 偏太多先原地转向对齐
      // ★ "已在停距内却没正对"**绝不能原地转**: 原地转的平移在小 dist 下会改掉方位 ⇒ 左右来回死转
      // (本函数唯一发散路径), 且低姿两指会扫走目标。处方: 抬臂 + 拉开一次, 之后交回画面。
      if (dist <= stop_cm) {
        // 停距内却没正对: 能否原地转取决于目标够不够远 —— 太近时车体(低姿两指)会扫到它, 先直线拉开;
        // 够远则只转对准(下一轮 dist≤stop 且正对即到位)。
        float ax0 = 0, ah0 = 0;
        bool low = exec::arm_pos(&ax0, &ah0) && ah0 <= AI_ARM_LOW_H_CM;
        float spin_safe = low ? NAV_SPIN_SAFE_LOW_CM : NAV_SPIN_SAFE_HIGH_CM;
        if (dist <= spin_safe) {                       // 太近不敢转: 直线拉开(方向按目标半球选, 纯平移
          if (low) {                                   // 最安全), 拉到 dist>spin_safe 就只转对准。
            JsonDocument f(&g_js_alloc); f["act"] = "fold";
            exec::act("arm", f.as<JsonObjectConst>());   // 低姿别拖着方块动
            wait_wheels(gen);
          }
          if (backs++ < NAV_BACKOFF_TRIES) {
            bool behind = fabsf(rel) > 90.0f;          // 目标在前半球⇒倒车拉距离; 在侧后方⇒前进才拉得开
            float need = spin_safe - dist + 1.0f;      // 只拉到"敢转"的距离即可(不用拉满 stop_cm), +1 越过门限
            if (need > NAV_MAX_SEG_CM) need = (float)NAV_MAX_SEG_CM;
            JsonDocument p(&g_js_alloc);
            p["throttle"] = behind ? throttle : -throttle; p["steering"] = 0;
            p["distance_cm"] = (int)need;
            exec::act("move", p.as<JsonObjectConst>()); ai::car_update_pose("move", p.as<JsonObjectConst>());
            ai::logf("[ai] 导航: 已在 %.0fcm 内却偏 %d°(太近不敢原地转), 先%s %dcm 拉开到可转距离再对准",
                     dist, (int)rel, behind ? "前进" : "退", (int)need);
            wait_wheels(gen);
            continue;
          }
          // 拉开重对后仍不正对 ⇒ 可疑的已经不是转向, 而是**这条坐标本身**。
          JsonDocument s(&g_js_alloc); s["scope"] = "all"; exec::act("stop", s.as<JsonObjectConst>());
          ai::logf("[ai] 导航: 车已在 %.0fcm 内却仍偏 %d° ⇒ 这条坐标可疑(真在停距内不该偏这么多), "
                   "停车交回画面微操", dist, (int)rel);
          return NavR::Reached;
        }
        // 够远不会扫到: 落到下面的原地转向分支, 只转对准不推进。
      }
      int dir = rel > 0 ? 1 : -1;
      if (last_dir && dir != last_dir) flips++;
      last_dir = dir;
      if (++spins > NAV_MAX_SPINS || flips > NAV_MAX_FLIPS) {
        JsonDocument s(&g_js_alloc); s["scope"] = "all"; exec::act("stop", s.as<JsonObjectConst>());
        ai::logf("[ai] 导航中止: 转向 %d 次/换向 %d 次仍未对准(目标还在 %.0fcm 外) ⇒ 停止打转, 交回 AI",
                 spins, flips, dist);
        return NavR::TimedOut;
      }
      int ang = (int)fminf(fabsf(rel), (float)NAV_MAX_SPIN_DEG);
      JsonDocument p(&g_js_alloc); p["dir"] = dir; p["speed"] = 800; p["angle_deg"] = ang;   // 全系统统一 800 档
      exec::act("spin", p.as<JsonObjectConst>()); ai::car_update_pose("spin", p.as<JsonObjectConst>());
      wait_wheels(gen);
    } else {                                            // 已正对: 定距直行一段(分步收敛)
      float seg = fminf(dist - stop_cm, (float)NAV_MAX_SEG_CM);
      if (seg < 1.0f) seg = 1.0f;
      JsonDocument p(&g_js_alloc); p["throttle"] = throttle; p["steering"] = 0; p["distance_cm"] = (int)seg;
      exec::act("move", p.as<JsonObjectConst>()); ai::car_update_pose("move", p.as<JsonObjectConst>());
      wait_wheels(gen);
    }
  }
  JsonDocument s(&g_js_alloc); s["scope"] = "all"; exec::act("stop", s.as<JsonObjectConst>());
  return NavR::TimedOut;
}