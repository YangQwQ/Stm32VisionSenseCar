#pragma once
#include <Arduino.h>

// TLS 传输(仅发送层): 复用连接 POST 到 AI 端点并读响应体。独立出 ai_http 模块,
// 与 ai_client 的 worker 状态机解耦; cancel/set_goal 通过 http_stop() 中断在途请求。
// g_client(连接) / g_last_status(最近状态码) 均 file-local 于 ai_http.cpp。
namespace ai {

// POST url(带 key 的 AI 接口)。成功返回 true 且尽量保留 g_client 连接供下次复用。
// gen 为代际号(当前未用, 保留入参以对齐原接口; 中断判断由 worker 负责)。
bool http_post(const char* url, const char* key, const char* body, String& resp,
               unsigned long gen);
// 最近一次响应的 HTTP 状态码(0=未知)。供 worker 做 4xx/429 快速失败判定。
int http_last_status();
// 立即中止在途请求/连接(cancel/set_goal/goto 打断用)。
void http_stop();

}  // namespace ai