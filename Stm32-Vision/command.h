#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

// command 模块：解析统一词表 JSON（与手机端 CommandProto.gd / 架构文档 §5.2 逐字一致），
// 校验并分发。手动指令（move/stop/arm）立即翻译为 UART 帧并执行；
// config 落 NVS 并调度重启；ai_goal 交给 ai_client（DIRECT，本阶段桩）。
// 不直接依赖任何通道——回复经回调发回调用方（WS=当前 fd 文本 / BLE=status 通知）。
namespace cmd {

typedef void (*ReplyFn)(void* ctx, const char* text);

// 处理一条词表 JSON。has_frames=true 表示调用方支持二进制帧（WS）；
// snapshot/stream 在无帧通道（BLE）上仅回引导文本。
// reply/reply_ctx 可为 NULL（静默丢弃，仅记录日志）。
void handle(const char* json, bool has_frames, ReplyFn reply, void* reply_ctx);

// 图传开关全局状态：由 command 接收 stream 指令更新，WS 推流任务读取。
bool streaming();
void set_streaming(bool on);

// 使最新 WiFi 配置生效：在线重建 STA 连接（不重启，BLE 保活）。
// BLE 配网写 SSID/PASS 后由 ble 调用；command 的 config 分支内部亦调用。
void apply_network();

}  // namespace cmd
