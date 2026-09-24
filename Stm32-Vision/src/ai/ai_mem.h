#pragma once
#include <Arduino.h>     // size_t / millis
#include <ArduinoJson.h>

// 空间记忆 + 车姿态(拆分自 ai_client)。g_mem 等仅本模块内可见。
namespace ai {

// 角度→弧度换算系数(内部连接性, 各 TU 自持一份; 供 ai_client.cpp 的 navigate_to 使用)。
const float AI_PI = 3.14159265358979f;

// 夹取依据的时效(轮): 目标必须在这几轮内被画面看到过, 才允许 arm_pose 下探/arm clip(见
// mem_grasp_evidence)。与记忆喂回的"给精确坐标"时效刻意同值 —— "喂给你厘米坐标"和"允许你夹"
// 是同一个时效, 于是提示词、记忆行、闸门用的是同一句话(列在这里是因为 ai_client 的拒绝回告也
// 要报这个数字)。
const int AI_GRASP_STALE = 3;

// 夹爪实际够得着的范围(车头系 前 cm / 横向 cm): 只在闸门里用于把"够不着"与"没看见"分开回告,
// 给足余量(臂可达 4~15cm), 不做精判 —— 精判仍由画面与 AI 自己负责。
const float AI_GRASP_FWD_MIN = 0.0f;
const float AI_GRASP_FWD_MAX = 22.0f;
// 横向容差是**相对爪口**算的(闸门里减去 ARM_LOW_LAT_CM), 所以它可以很紧: 横向是机械臂唯一
// 无法补偿的一轴, 方块 2.5cm 宽、爪口只略宽, 差 1cm 就必空夹, 而合爪时不会有任何报错。
// ⚠️ 原值 15.0f 等于没有判据(目标偏出 15cm 早就出画面了) —— 实测两次空夹的横向读数都只有 1cm,
// 全都从这条缝里放行。协调误差(单应解算 + 模型自估)在厘米级, 故留 1.5cm 而不是更紧。
const float AI_GRASP_LAT_MAX = 1.5f;

// 近场与"已到车头/爪后"的距离口径(车头系 前 cm)。**放这里是为了只有一份**: 闸门(ai_client)与
// 记忆行里那句"看不见该怎么办"(ai_mem 的 mem_feed)必须说同一个数 —— 否则 AI 读到的处方和程序
// 拦它的线对不上, 它会照着一个程序不认的距离去动作(同源不变量, 见 CLAUDE.md)。
// 近场线: 与 approach 的停距同值同口径(见 ai_client 的 AI_NEAR_FWD_CM 注释)。
const float AI_NEAR_FWD_CM = 15.0f;
// 已到车头/爪后: 报告的 7cm 折算成真距约 5cm, 已越过低姿爪口(ARM_LOW_X_CM=8) ⇒ 方块在臂下。
const float AI_ARM_UNDER_CM = 7.0f;
// 上面那条的**迟滞出门线**(施密特): 进门 ≤7cm, 要退到 10cm 以外才算真出去, 中间维持上一轮判定。
// 为什么不用"连续两轮都成立"去抖: 判据是单条观测的厘米坐标, 这一带解算误差就有 ±10cm 量级, 融合
// 估计在门线附近来回跳(实测 5.5/6.7/8.5cm), "连续两轮"永远凑不满 ⇒ 每轮都不发处方, 而相反的建议
// 一轮不落地发出去, AI 就在"前进1cm/后退3cm"之间蹭(实测 50 秒一次夹取都没试)。见 ai_client 的用法。
// ⚠️ 出门线别取到 AI_NEAR_BLIND_CM(12): 那正是**方块已在两指之间**时坐标会报的值(高报 ⇒ 真距 8cm 报
// 约 12cm), 一进迟滞带就把"已对准"也吞进去, 于是 AI 会在对齐好的那一刻被赶去收臂后退, 反而夹不上。
// ⚠️ mem_feed 的丢失文案也拿它当分档线(理由见 ai_mem.cpp): 迟滞只要开着, ufwd 就 ≤ 这条线, 于是
// "程序说在臂下"与"记忆行说在臂下"永远同时成立, 不会出现一个让抬臂后退、另一个让顶进的情形。
const float AI_ARM_UNDER_EXIT_CM = 10.0f;

// 横向的**硬**阈值(同样相对爪口, cm): 超过它就不是"没对准"而是"物理上夹不到", 合爪一律拦下。
// 与 AI_GRASP_LAT_MAX 的分工: 那个是**提醒线**(放行 + 让 AI 自己看画面修), 这个是**拦截线**;
// 只在合爪(clip/grasp)上判, **不拦 arm low** —— 降爪是准备动作, 拦它等于不让 AI 降爪。
// 取 3.0cm 的理由: 方块 2.5cm 宽、爪口只略宽, 偏出一个方块身位必然空咬且不报任何错。实测那次
// 空夹读数是"左5.1cm"(爪口另偏右 1cm ⇒ 实际偏 6.1cm); 即便按单应高报约 30% 的悲观估计折算,
// 3cm 仍有 2cm 以上真实偏差, 远超半个方块。低于提醒线的偏差仍按老办法放行让画面定夺。
// ⚠️ 补救动作必须与提示词的小角度上限**同源**: 这里只许给"最小步 spin, 转完看画面再补",
// **禁止**照 atan2 算出大角度点名 —— 旧闸门点名 47°/56° 与提示词"≤15°"打架, AI 转不够、闸门加码、
// 转过头把方块扫出视野, 是跟丢的直接成因(见 CLAUDE.md 已知问题 4)。
const float AI_GRASP_LAT_HARD_CM = 3.0f;

// 夹取依据"还算不算数"的运动容差: 观测之后车走过的**路程**/转过的角度在这个量内, 那条坐标就仍是
// "看到的位置"(推算误差小于夹取容差), 可以照它夹; 超了才算"用推算值代替画面", 必须重新看。
// 用路程而非净位移: 往返各 3cm 净位移为 0, 但位姿估计在两段里各累积了一次误差, 坐标已经偏了。
// 这条容差是配方的一部分 —— 实测有效的夹法是 arm low 之后补一小步(<3cm)顶进两指再 clip, 那一顶
// 必然让车位姿变化, 且此时目标已被夹爪挡住**没法重新 observe**: 若判"车动过就拒", 配方里的动作
// 永远进不来(手动能做、AI 做不到), AI 只能反复后退找画面 —— 实测就卡在这个极限环里。
const float AI_GRASP_MOVE_TOL_CM = 3.0f;
const float AI_GRASP_TURN_TOL_DEG = 6.0f;

// 车自身位姿(全局坐标 x,y cm + 车向角 heading, 命名空间共享: ai_mem 维护 / navigate_to 读用)。
extern float s_car_x, s_car_y;
extern int16_t s_car_heading;

// 新任务起点: 车位置=原点、车头=0°, 清空物体记忆(原 ai_worker 任务开头那几行)。
void mem_reset();
// 按定距/定角近似累积车姿态(move→平移, spin→转向)。
void car_update_pose(const char* type, const JsonObjectConst& p);
// AI 观测(rel_deg 相对车头, 正=右、负=左): 换算车头系后入表。
// 返回 false = 这次"可见"的观测没能记入(缺距离/像素越界), 调用方应回告 AI 防它以为已记住。
bool mem_observe(const char* name, bool visible, float rel_deg, float dist_cm);
// AI 观测(屏幕归一化像素 px,py): 单应解算车头系坐标后入表。返回值含义同上。
bool mem_observe_xy(const char* name, bool visible, float px, float py);
// 生成喂给 AI 的空间记忆文本(当前车头局部系精确坐标)。
void mem_feed(char* buf, size_t cap);
// 记忆里有没有**可用**的物体(观测过、且没旧到喂不回去)。供"还没锁定任何目标"这类引导语判断:
// 过滤条件与 mem_feed 逐字同源, 免得程序说"没有目标"而同一轮的记忆行里明明列着一个。
bool mem_have_any();
// 是否存在"曾被正确观测过、但现已不可见(被标记丢失)"的物体: stale 已超新鲜线(AI_GRASP_STALE)但
// 记录仍在 —— 即 mem_feed 里会显示"已N轮未见/位置过时"的那一类。与 mem_have_any()(完全没可用目标)
// 是两种不同情形: 这个回答的是"有物体, 只是最近看不到了", 供"目标可能被机械臂遮挡"这类提示判断。
// 过滤条件与 mem_grasp_evidence 的"新鲜"线同源(stale > 新鲜线 ⇒ 夹取依据也已过期, 闸门会拒)。
bool mem_have_lost();
// 在物体记忆表里定位目标全局坐标。
bool mem_find(const char* name, float* tx, float* ty);
// 这条记忆最近一次是"判历史已失效 ⇒ 整条重置"来的吗(见 ai_mem.cpp 的离群判定②)。
// 重置后这条坐标只由**一条曾被判离群**的观测撑着 —— 是记忆里最不可信的一种。供 approach 拦"照它
// 开环赶路": navigate_to 是全盲死航, 拿一条错的坐标冲过去不会顶偏, 而是**直接顶到方块**上。
// 一轮之后正常观测落地即自动解除(那时它已有两条互相支持), 不会把 approach 长期锁死。
bool mem_untrusted(const char* name);
// 夹取依据(arm_pose 下探 / arm clip 的闸门用): 从记忆里挑"最近 AI_GRASP_STALE 轮内被画面看到过"
// 的那个物体(最新鲜优先, 同龄取最近观测的), 输出名字、距上次看到几轮、当前车头系坐标(前+/右+)。
// ⚠️ 坐标取的是**融合中位数**——与 mem_feed 喂给 AI 的是同一个数, 这条同源关系是硬不变量:
// 闸门判的是"AI 照它拿到的坐标算出来的动作", 两边不同源就会造出 AI 无论如何都满足不了的指令。
// 返回 false = 没有可用的近期观测(记忆为空, 或最近的观测都已过期 —— 拿这种坐标去夹必然空夹)。
// 只判时效、**不判够不够得着**: 后者由闸门据坐标自行分类, 好把"没看见目标"和"看见了但还够不着"
// 分成两种回告(前者要让 AI 去 observe 锁定, 后者要它先推进对准 —— 混成一句话会让它白试)。
// car_moved 判据的量化版: moved_cm/turned_deg = 自那次观测以来车走过的路程与转过的角度。调用方
// 拿它跟 AI_GRASP_*_TOL_* 比, 决定这条坐标还准不准(而不是"动过没有"这种一刀切)。
// ⚠️ 但"融合值 vs 最新观测"的取舍**不在这里做**: 闸门一律用融合值(同源不变量, 见上), 融合值本身
// 准不准是另一个问题 —— 它由"夹完抬臂复看"(夹空看得见、能重来)兜底, 而不是由闸门一票否决。
bool mem_grasp_evidence(char* name, size_t cap, int* age_rounds, float* fwd_cm, float* lat_cm,
                        float* moved_cm, float* turned_deg);
// 每轮结束: 所有物体未观测则过期轮数 +1。
void mem_tick_stale();

}  // namespace ai