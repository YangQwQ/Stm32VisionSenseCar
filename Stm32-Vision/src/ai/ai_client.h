#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include "src/core/command.h"

// ai_client：DIRECT 链路板载 AI 客户端。worker 负责 HTTP 调用(阻塞不卡 loop), 单槽"最新目标优先"
// + 代际号中止旧任务; 任务主体在 ai_round, 结果队列/日志出口在 ai_result, 本文件只管槽/代际/API。
namespace ai {

// 任务槽内容(set_goal / goto_target 写入, worker 取走)。见 ai_client.cpp 的 g_slot。
struct TaskLocal {
  char* text = nullptr;   // 目标文本(堆)
  char* ann = nullptr;    // 标注 JSON 或 {x,y,w,h,label}(堆)
  bool use_image = false;
  bool one_shot = false;  // 单轮模式: 只执行一轮决策即收尾(/ai oneshot)
  long id = 0;            // 对应 ai_goal 的词表 id(回填 ai_result)
  unsigned long generation = 0;
  cmd::ReplyFn fn = nullptr;
  void* ctx = nullptr;    // 任务期 sink ctx(WS=堆 int*, BLE=nullptr)
  bool active = false;    // 槽是否已被 set_goal 激活
  float nav_x = 0, nav_y = 0;  // goto_target 目标坐标 / frame
  bool nav = false;             // 纯导航任务(/move to), 不走 AI 闭环
  bool nav_global = false;      // true=沿用全局系; false=以当前车位姿为新原点
};

void init();                    // 启动 worker 任务（setup net 之后调用一次）
void update();                  // loop 中调用：排空结果队列（发送 ai_result）
// ⚠️ 本头文件只声明; update / logf / set_edited_image 的实现都在 ai_result.cpp(结果队列与日志出口),
//    不是 ai_client.cpp —— 按文件名找定义会扑空。

// 下发新目标。id=对应 ai_goal 词表 id(回填 ai_result); reply/reply_ctx=结果回传通道(同 cmd::handle,
// WS 的 ctx 是堆拷贝 fd 指针、由本模块在任务结束时释放, BLE 为 nullptr); one_shot=只执行一轮。
void set_goal(const char* text, bool use_image, const char* annotation, long id,
              cmd::ReplyFn reply, void* reply_ctx, bool one_shot = false);

// 纯本地导航任务(/move to x y): 不调云端, 直接巡航到坐标 (x,y)。frame_global=false 以当前位姿为原点,
// true 沿用当前全局系。手动接管类。
void goto_target(float x, float y, bool frame_global, long id,
                 cmd::ReplyFn reply, void* reply_ctx);

// 中止任务的兜底 stop 模式：None=被用户指令接管（执行板已被新指令覆盖，不补发）；
// Wheels=手动 arm 打断（只停轮子）；All=取消/任务终结（全停）。
enum class StopMode : uint8_t { None, Wheels, All };

void cancel(StopMode m = StopMode::All);  // 中止当前任务（新目标 / 手动指令 / ai_cancel）
bool busy();                              // 是否有任务进行中（BLE status.ai_busy 用）

// AI 任务进行中"插话"：把用户补充文本追加进当前任务上下文，不打断任务（区别于 set_goal）。
// 返回是否有任务在跑（true=已入队，worker 下一轮连同 prompt 一起喂给模型）。
bool append_chat(const char* text);

// AI 调试日志：统一经 board_log 模块（blog::AI 来源）——始终写串口；`/log ai on`（或 all）
// 时以 {type:"log",params:{src:"ai",text}} 经统一队列转发手机（WS+BLE）。
void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// 暂存一张编辑图（WS 二进制上行，裸 JPEG，覆盖式）。TTL 见常量。
void set_edited_image(const uint8_t* data, size_t len);

// ---- 以下为模块内部协作入口(供 ai_round / ai_nav 用), 不对外 ----

// 是否有未消费的插话(持锁只看不动)。
bool chat_pending();
// 取走插话(持锁拷进 dst 并清标志; 不含 UTF-8 尾部裁剪, 由调用方处理)。
bool chat_take(char* dst, size_t cap);
// worker 任务忙标志(ai_round 在任务起点/终点按原位置置位)。
void set_busy(bool v);
// 本轮任务终结时写入的兜底 stop 模式(由中断方决定)。
int stop_mode();
// 代际号(每次 set_goal/goto_target/cancel 自增)。ai_nav/ai_round 传下来的 gen 与它不等即
// "本任务已被新目标/手动指令打断", 就地收手。
unsigned long generation();

}  // namespace ai