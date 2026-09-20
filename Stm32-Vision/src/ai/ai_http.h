#pragma once
#include <Arduino.h>

// AI TLS 发送层(复用连接 POST 并读响应)。g_client/g_last_status 仅在本模块内。
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