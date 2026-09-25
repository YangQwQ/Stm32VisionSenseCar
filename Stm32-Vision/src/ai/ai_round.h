#pragma once
#include "src/ai/ai_client.h"   // TaskLocal

// AI 任务主体(拆分自 ai_client 的 ai_worker): 任务级多轮闭环 —— 取帧 → 组包 → 云端 → 校验 →
// 落地 → 反馈/历史 → 下一轮, 直到 done/超步数/被新目标打断。全部跨轮状态收在 ai_round.cpp 的
// RoundCtx 里(按值建在调用方 worker 栈上, 不额外分配)。
namespace ai {

// 执行一个 AI 任务(阻塞至任务结束)。t 由调用方持有并在本函数内被消费(结束时释放 text/ann/ctx)。
void round_run_task(TaskLocal& t);

}  // namespace ai