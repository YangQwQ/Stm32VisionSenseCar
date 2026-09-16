#include "src/core/board_log.h"
#include <ArduinoJson.h>
#include <string.h>    // strdup/memcpy
#include <stdlib.h>    // free
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// 统一板端日志模块实现：
//   logf() 任意任务可调用——先写串口（带 [类] 前缀），类别开启时把 JSON 排队，
//   由专用转发任务统一排空并经注册的发送器发手机（WS 广播 + BLE 通知）。
// 转发默认全关；仅当手机开 exec/ai/all 对应项时才入队，避免空闲刷屏。

namespace blog {

static void sanitize_utf8(char* s);   // 前置声明（定义在 logf 之后）

static bool g_en[CAT_MAX] = {false};
static bool g_all = false;
static SendFn g_sender = nullptr;
static QueueHandle_t g_q = nullptr;

static const char* const k_name[CAT_MAX] = {
    "exec", "ai", "net", "cam", "ws", "cmd", "ble", "sys"};

bool enabled(Cat c) { return c < CAT_MAX && (g_all || g_en[c]); }

void enable(Cat c, bool on) { if (c < CAT_MAX) g_en[c] = on; }

void set_all(bool on) { g_all = on; }

void set_forwarder(SendFn fn) { g_sender = fn; }

void logf(Cat c, const char* fmt, ...) {
  if (c >= CAT_MAX) return;             // 非法类别：静默丢弃，防 k_name 越界
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  buf[sizeof(buf) - 1] = 0;  // 截断防越界
  va_end(ap);

  if (g_all || g_en[c]) {
    // 转发副本做 WS 消毒（残缺 UTF-8 → '?'，1:1），串口仍留原始便于排查。
    char scr[256];
    memcpy(scr, buf, sizeof(scr));
    scr[sizeof(scr) - 1] = 0;
    sanitize_utf8(scr);
    // 组 {type:"log", params:{src, text}}，挤出阻塞式入队（满则丢，自带 free，无 UAF/泄漏）。
    JsonDocument d;
    d["type"] = "log";
    d["params"]["src"] = k_name[c];
    d["params"]["text"] = scr;
    String s;
    serializeJson(d, s);
    if (g_q) {
      char* copy = strdup(s.c_str());
      if (copy && xQueueSend(g_q, &copy, 0) != pdTRUE) free(copy);
    }
  }
  Serial.printf("[%s] %s\n", k_name[c], buf);
}

// 就地把一条待发文本转为合法 UTF-8（RFC6455 TEXT 帧必须为 UTF-8）。
// 云端 AI 响应被日志原样嵌入（如 "HTTP %s body=%s" / "原始读取"）时，偶发残缺 UTF-8
// 或非法字节；若原样送进 WS TEXT 帧，手机 Godot 会以关闭码 1007 断链。与 ai_client 的
// sanitize_ws_utf8 同一策略：1:1 替换（不扩容），仅改待发副本，串口仍留原始便于排查。
static void sanitize_utf8(char* s) {
  char* w = s;
  const unsigned char* p = (const unsigned char*)s;
  while (*p) {
    unsigned char c = *p;
    int need = 0;
    if (c < 0x20 || c == 0x7f) { *w++ = '?'; p++; continue; }   // 控制字符
    if (c < 0x80) { *w++ = (char)c; p++; continue; }             // 合法 ASCII
    if (c >= 0xC2 && c <= 0xDF) need = 1;                        // 二/三/四字节引导（续字节非法→整体替换）
    else if (c >= 0xE0 && c <= 0xEF) need = 2;
    else if (c >= 0xF0 && c <= 0xF4) need = 3;                   // 其余首字节非法
    bool ok = need > 0;
    for (int i = 1; ok && i <= need; i++) {
      unsigned char cc = p[i];
      if (!cc || !(cc >= 0x80 && cc <= 0xBF)) ok = false;        // 续字节缺失/越界（含被截断串尾）
    }
    if (ok) { for (int i = 0; i <= need; i++) *w++ = (char)p[i]; p += need + 1; }
    else { *w++ = '?'; p += 1; }                                 // 非法/残缺：单字节替换
  }
  *w = 0;
}

// 转发任务：阻塞在队列上，取出带发送器发给手机（WS 广播 + BLE 通知）。
static void forward_task(void* arg) {
  if (!g_q) vTaskDelete(NULL);          // 队列创建失败则退出，避免永久阻塞
  for (;;) {
    char* s = nullptr;
    if (xQueueReceive(g_q, &s, portMAX_DELAY) == pdTRUE && s) {
      if (g_sender) g_sender(s);
      free(s);
    }
  }
}

void init() {
  if (g_q) return;                      // 幂等
  g_q = xQueueCreate(32, sizeof(char*));
  if (!g_q) return;                     // 队列创建失败：保持静默，转发自然关闭
  // 栈给足 8192 与 ws_stream 一致：发送器内含 ws_send_text + BLE notify，避免栈溢出。
  xTaskCreatePinnedToCore(forward_task, "blog_fwd", 8192, nullptr, 1, nullptr, 1);
}

}  // namespace blog