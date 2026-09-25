#pragma once
#include <Arduino.h>
#include "src/core/command.h"   // cmd::ReplyFn(结果回传通道)

// AI 结果回传队列 + 编辑图暂存。结果队列: worker → loop 单向(loop 侧 ai::update 排空发送 ai_result);
// 编辑图: WS 二进制上行 → 暂存环 → worker 任务起点取快照。
namespace ai {

// 建结果队列与编辑图互斥锁(ai::init 里调用一次)。
void result_init();

// 入队一条结果文本(worker 侧调用)。内部堆拷贝 + UTF-8 消毒, 由 loop 侧消费后释放。
void enqueue_result(const char* text, cmd::ReplyFn fn, void* ctx);

// 编辑图暂存容量/TTL(快照与写入两侧共用)。
#define AI_EDITED_SLOTS 3
#define AI_EDITED_IMG_MAX (128 * 1024)
#define AI_EDITED_IMG_TTL_MS 60000

// 取"最近 ≤3 张未过期编辑图"快照: 各拷独立 PSRAM 副本, 槽位按"新→旧"记入 order(order[0]=最新)。
// imgs/lens 按槽位下标填(未命中为 nullptr/0), order 未填置 -1; maxn=数组容量; 调用方负责 free。
int edited_snapshot(uint8_t** imgs, size_t* lens, int* order, int maxn);

}  // namespace ai