#pragma once
#include <Arduino.h>

// 运动到位验证模块（namespace mvfy）：
// 板端轻量视觉闭环，纯本地、不依赖云端 AI，弥补 move/spin「无里程计、开环按时长近似」的空洞。
// - 移动受阻检测（move）：定距移动期间采样缩略灰度，画面持续无变化 → 判疑似撞墙/卡死 → 停轮
// - 旋转实测角度（spin）：原地旋转期间持续估算实际转角（累计多拍，不受单拍延迟影响），
//   供 exec 在旋转结束后用「实测与目标差」自动补转，避免转过头/不到位
// ⚠️ 转角累计从起转第一拍就开始（起步 grace 只压"卡死"误判，不再跳过累计），且停轮后须由 exec
//   调 spin_stop() 进入沉降窗口继续测完滑行——否则实测角系统性偏小，补转会把车转过头。
// 软解（jpg2rgb565）在独立任务线程执行（MVFY_TASK_CORE），不阻塞 loop/WS/控制时序；
// exec 任一线程经 begin/should_stop/consume 与任务共享状态（临界区保护），最新结果用 poll 读取。
namespace mvfy {

// 状态机：空闲 / 正在验证移动 / 正在验证旋转。
enum Mode : uint8_t { IDLE, VERIFY_MOVE, VERIFY_SPIN };

// 启动：创建软解任务（setup 期调用一次，幂等）。
void init();

// 由 exec 发起一次移动到位验证：type = "move"/"spin"，target_deg 仅 spin 用（计划转角）。
// keep_grace=true（旋转补偿续测用）：同模式续测时只刷新节流、不重置起步 grace，避免补偿段因 grace 未过而测不到转角。
// 同模式续测传 target_deg<=0 时保留原目标（不再把目标刷成 0，调试日志里"目标"才始终可读）。
void begin(const char* type, float target_deg = 0, bool keep_grace = false);

// 结束验证（车轮停稳/任务收尾时调，释放帧缓冲预算、复位内部状态）。幂等。
void end();

// spin 到点停轮：轮子已停、画面本就不动，故暂停"卡死"判定，但保留采样与转角累计，
// 供 exec 在沉降窗口内测完滑行后再读最终实测角。下一次 begin 会自动恢复正常判定。
void spin_stop();

// spin：停轮后是否已测到"画面静止"（滑行结束、累积角已定）。exec 据此在沉降窗口内提前收，
// 不必死等 MVFY_SPIN_SETTLE_MS 上限——死等要么白拖时间，要么窗口短于一次采样而漏掉尾部滑行。
// 只在 spin_stop() 之后的沉降期可能为 true；begin()/end()/spin_stop() 都会复位。
bool spin_settled();

// 读取任务最新判定结果（loop 每拍轮询）：有「受阻 / 转不大」待停机请求 → true，exec 应停下并 consume()。
// 该函数仅读共享状态，不阻塞、不做软解。
bool should_stop();

// 消费停机请求并复位，避免同一受阻被反复触发。
void consume();

// 当前验证模式（供 exec 判断是否在做旋转补偿）。
Mode mode();

// spin：本次验证累计实测转角（deg，有符号=实际旋转增量）。exec 旋转结束后用它算残余误差补转。
float spin_delta_deg();

// spin：累计进 spin_delta_deg() 的有效样本数（无纹理/乱匹配/位移顶到搜索窗边的拍不计入）。
// exec 用它判测量可信度：样本太少说明这次实测不可信，宁可不补转。
int spin_delta_n();

// 相机是否可用（无摄像头时跳过视觉验证，退化回纯开环）。
bool available();

}  // namespace mvfy