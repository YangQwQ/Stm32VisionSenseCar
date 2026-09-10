#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include "command.h"

// ai_client 模块：DIRECT 链路板载 AI 客户端。
// worker 任务负责 HTTP 调用（阻塞不卡 loop）；单槽"最新目标优先"+ 代际号实现
// 新目标/手动指令中止旧任务；结果经队列回 loop 上下文发送（复用 cmd::Reply 通道）。
// 编辑图（WS 二进制上行）暂存 PSRAM，供 ai_goal{use_image:true} 消费。
namespace ai {

void init();                    // 启动 worker 任务（setup net 之后调用一次）
void update();                  // loop 中调用：排空结果队列（发送 ai_result）

// 下发新目标。id 为对应 ai_goal 的词表 id（回填 ai_result）。reply/reply_ctx 为
// 结果回传通道（同 cmd::handle）；WS 来源的 reply_ctx 是堆拷贝后的 fd 指针（本模块
// 持有并在任务结束时释放），BLE 为 nullptr。
// one_shot=true：只执行一轮决策即收尾（/ai oneshot），区别于 ai_goal 的迭代闭环。
void set_goal(const char* text, bool use_image, const char* annotation, long id,
              cmd::ReplyFn reply, void* reply_ctx, bool one_shot = false);

// 中止任务的兜底 stop 模式：None=被用户指令接管（执行板已被新指令覆盖，不补发）；
// Wheels=手动 arm 打断（只停轮子）；All=取消/任务终结（全停）。
enum class StopMode : uint8_t { None, Wheels, All };

void cancel(StopMode m = StopMode::All);  // 中止当前任务（新目标 / 手动指令 / ai_cancel）
bool busy();                              // 是否有任务进行中（BLE status.ai_busy 用）

// AI 调试日志：始终写串口；ai_log 开关开启且任务回传通道在位时，同时把该行
// 以 {"type":"ai_log","text":..} 推送当前手机（复用 ai_result 的回传队列）。
void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// 暂存一张编辑图（WS 二进制上行，裸 JPEG，覆盖式）。TTL 见常量。
void set_edited_image(const uint8_t* data, size_t len);

}  // namespace ai