#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include "src/ai/ai_prompt.h"   // PieceList(请求体碎片流)

// AI TLS 发送层(复用连接 POST 并读响应)。g_client/g_last_status 仅在本模块内。
namespace ai {

// POST url(带 key 的 AI 接口)。请求体以碎片列表给出, 由 HTTPClient::sendRequest 边取边发(总长即 Content-Length)。
// 成功返回 true 且尽量保留 g_client 连接供下次复用。
// gen 为代际号(当前未用; 中断判断由 worker 负责)。
// resp: 响应正文收进 PSRAM 增长缓冲(原来用 String ⇒ 整份响应体压在内部堆上, chunked 路径
// 还每 512B 一次 realloc)。PSRAM 常年富余, 内部堆才是瓶颈。
bool http_post(const char* url, const char* key, PieceList& body, PsaBuf& resp,
               unsigned long gen);
// 最近一次响应的 HTTP 状态码(0=未知)。供 worker 做 4xx/429 快速失败判定。
int http_last_status();
// 最近一次非 200 响应的错误体(空串=该次没读到)。用于把云端的**具体拒绝原因**上报给手机。
const char* http_last_error();
// 立即中止在途请求/连接(cancel/set_goal/goto 打断用)。
void http_stop();

// 从响应提取工具调用, 规范化进 out["calls"]: [{"id","name","args"}(args=模型给的参数 JSON 原文)...]。
// 返回调用数(0 = 没调工具: 模型只回了文本 / 思考吃光预算被 length 截断 / 解析失败)。
// 同时打印 reasoning_content; 无调用时打一条诊断。out 用 PSRAM 池(调用方传 g_js_alloc 的文档)。
int extract_tool_calls(const PsaBuf& resp, JsonDocument& out, bool* broken = nullptr);

}  // namespace ai