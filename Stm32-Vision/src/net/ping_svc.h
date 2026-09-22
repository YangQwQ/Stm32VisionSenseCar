#pragma once
#include "src/core/command.h"

// ping 服务：对 IP/域名发起异步 ICMP echo（esp_ping），结果经 cmd 回复通道回报。
// 用在 /ping <目标>（如 /ping 192.168.1.1 或 /ping baidu.com）：板子去 ping 目标并
// 回一条 status 文本（含 成功数/平均/最小/最大延迟）。纯 /ping（无目标）仍由
// command 就地回 pong（测小车连通性）。
namespace ping {

// 发起一次 ping 会话（count 次，每次超时 3s）。target 为 IPv4 或域名。
// reply/reply_ctx 复用 cmd 回复通道：WS 场景内部会堆拷贝 fd（异步回调安全）；
// BLE 为 nullptr（仅进日志转发）。任务栈走 PSRAM，同时只允许一个会话（s_busy）。
// 返回 false = **未发起**（参数无效 / 上一个会话未结束 / 内部堆紧到起不来），调用方须自行回报
// 失败并带上体征——失败原因与堆体征另经 blog 日志通道报出（cat=all 时 WS+BLE 都能看到）。
bool start(const char* target, int count, cmd::ReplyFn reply, void* reply_ctx);

}  // namespace ping
