#pragma once
#include <Arduino.h>

// 本地巡航 + 出帧前运动等待。纯本地、不调云端: /move to 与 AI 的 approach 都由 navigate_to 落地;
// settle_* 供出帧前等车/臂停稳。
namespace ai {

// 出帧前限时等轮子停稳的上限(定距/定角到点自停后仍有余速滑行)。
#define AI_SETTLE_MAX_MS 700

// "低姿"高度线: 低于它即两指已贴地前伸(见 navigate_to 的旋转安全距离分档)。
#define AI_ARM_LOW_H_CM 2.5f

// ---- 本地巡航参数(/move to 与 approach 共用) ----
#define NAV_STOP_CM_GOTO 6       // /move to 到位容差 cm(无里程计开环, 留松量)
#define NAV_ALIGN_DEG 15         // 目标偏角超过此值先原地转向对齐, 否则直行推进
#define NAV_MAX_SPIN_DEG 60      // 单次原地转向的角度上限(分步收敛)
#define NAV_MAX_SEG_CM 25        // 单次定距推进上限 cm(分步收敛、防冲)
#define NAV_MAX_ITERS 200        // 巡航最大迭代步数(防死循环)
#define NAV_MAX_SPINS 8          // 单次巡航的原地转向次数上限
#define NAV_MAX_FLIPS 2          // 转向方向反转次数上限(超过 = 在来回蹭)
#define NAV_BACKOFF_TRIES 2      // 太近拉开最多拉几次(拉不开再判坐标可疑)
#define NAV_SPIN_SAFE_HIGH_CM 8  // 抬臂时的旋转安全距离: 以下旋转可能撞到目标
#define NAV_SPIN_SAFE_LOW_CM 12  // 低姿时的旋转安全距离(两指前伸, 扫到范围更大)

// 巡航出口: Reached=到位 / Interrupted=被新任务或手动打断 / TimedOut=迭代或转向预算超限。
enum class NavR : uint8_t { Reached, Interrupted, TimedOut };

// 本地巡航核心(不调云端): 从当前位姿导航到全局目标 (tx,ty), 距目标 ≤stop_cm 且正对时停。无里程计,
// 按定距/定角时长近似 + 姿态累积开环死航; 剩余误差交给 AI 视觉微操。
NavR navigate_to(unsigned long gen, float tx, float ty, float stop_cm);

// 出帧前限时等停稳(最多 max_ms, 超时照常出帧, 绝不长时间阻塞决策)。gen 变化即提前退出。
void settle_wheels(unsigned long gen, int max_ms);
void settle_arm(unsigned long gen, int max_ms);

}  // namespace ai