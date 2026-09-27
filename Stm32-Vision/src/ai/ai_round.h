#pragma once
#include "src/ai/ai_client.h"   // TaskLocal

// AI 任务主体(拆分自 ai_client 的 ai_worker): 任务级 agent 循环 —— 组包 → 云端(tool calls) → 落地工具
// (mem/car/look/finish) → 结果回喂 → 下一回合, 直到 finish/超回合数/被新目标打断。全部跨回合状态收在
// ai_round.cpp 的 RoundCtx 里(按值建在调用方 worker 的 16KB PSRAM 栈上; 只有历史环表另在 PSRAM 堆上)。
namespace ai {

// 执行一个 AI 任务(阻塞至任务结束)。t 由调用方持有并在本函数内被消费(结束时释放 text/ann/ctx)。
void round_run_task(TaskLocal& t);

}  // namespace ai