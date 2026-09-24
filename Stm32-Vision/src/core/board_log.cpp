#include "src/core/board_log.h"
#include <ArduinoJson.h>
#include <stdio.h>     // snprintf（state_text）
#include <string.h>    // strdup/memcpy
#include <stdlib.h>    // free
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_heap_caps.h>   // MALLOC_CAP_SPIRAM：转发任务栈放 PSRAM

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
static StackType_t*  s_fwd_stack = nullptr;  // 转发任务栈：放 PSRAM，抬内部 DMA 块水位
static StaticTask_t  s_fwd_tcb;

static const char* const k_name[CAT_MAX] = {
    "exec", "ai", "net", "cam", "ws", "cmd", "ble", "sys"};

bool enabled(Cat c) { return c < CAT_MAX && (g_all || g_en[c]); }

// 类别位与 all 互斥（单选，见头文件）：这样每条 /log 回执说的"只开/只关这一类"才算数。
void enable(Cat c, bool on) {
  if (c >= CAT_MAX) return;
  if (on) {
    g_all = false;          // 选具体类别 = 收窄：否则遗留的 all 会把这次"只开 ai"放大成全类别
    g_en[c] = true;
    return;
  }
  if (g_all) {
    // all 开着时关单个类别：把 all 降级成"其余类别各自开"，否则这条 off 名不副实（回执说关闭，日志照发）。
    // 先清 all 再置其余：跨任务读到的中间态只会少发一拍，不会多发出用户已关的类别。
    g_all = false;
    for (int i = 0; i < (int)CAT_MAX; i++) g_en[i] = (i != (int)c);
    return;
  }
  g_en[c] = false;
}

void set_all(bool on) {
  g_all = on;
  // all 期内的类别位无意义（且关 all 后会变成残留），一律清掉：all 开=全类别、all 关=真正全关。
  for (int i = 0; i < (int)CAT_MAX; i++) g_en[i] = false;
}

void state_text(char* buf, size_t n) {
  if (!buf || !n) return;
  buf[0] = 0;
  if (g_all) { snprintf(buf, n, "all(全部)"); return; }
  size_t w = 0;
  for (int i = 0; i < (int)CAT_MAX; i++) {
    if (!g_en[i]) continue;
    int m = snprintf(buf + w, n - w, "%s%s", w ? "," : "", k_name[i]);
    if (m < 0 || (size_t)m >= n - w) { buf[n - 1] = 0; return; }   // 截断即止（类别名短，实际不会走到）
    w += (size_t)m;
  }
  if (!w) snprintf(buf, n, "-");
}

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

// 转发一条任意长度文本（AI 思考等）为 {type:"log",params:{src,text}}。
// 手工拼接 JSON（含必做的转义），副本整体放 PSRAM——不用 ArduinoJson：大文本逐字符
// 转义会先落到内部堆，几十 KB 思考直接威胁 DMA 块红线。转发开关同 logf；不打串口。
void forward_text(Cat c, const char* text) {
  if (c >= CAT_MAX) return;
  if (!(g_all || g_en[c])) return;      // 与 logf 同一开关
  if (!text) return;
  const char* sc = k_name[c];
  // 前缀 {"type":"log","params":{"src":" + src + 常量 + 转义后的 text + "}}
  const char* pre = "{\"type\":\"log\",\"params\":{\"src\":\"";
  const char* pre2 = "\",\"text\":\"";
  const char* suf = "\"}}";
  size_t pre_len = strlen(pre) + strlen(sc) + strlen(pre2);

  // 第一趟：数出 text 转义 + 清洗后的长度（JSON 引号/反斜杠/控制符会扩容）。
  size_t body = 0;
  for (const unsigned char* p = (const unsigned char*)text; *p;) {
    unsigned char cc = *p;
    // 控制符 / 需转义字符：一律 JSON 转义或替换
    if (cc == '"' || cc == '\\')           { body += 2; p++; continue; }
    if (cc == '\n')                        { body += 2; p++; continue; }   // \n
    if (cc == '\r')                        { body += 2; p++; continue; }   // \r
    if (cc == '\t')                        { body += 2; p++; continue; }   // \t
    if (cc < 0x20 || cc == 0x7f)           { body += 1; p++; continue; }   // 其余控制符→'?'
    if (cc < 0x80)                         { body += 1; p++; continue; }   // 普通 ASCII(数字/英文/标点)原样
    // 多字节 UTF-8：校验引导符 + 续字节，合法整段原样，非法首字节→'?'（同 sanitize_utf8 策略）
    int need = 0;
    if (cc >= 0xC2 && cc <= 0xDF) need = 1;
    else if (cc >= 0xE0 && cc <= 0xEF) need = 2;
    else if (cc >= 0xF0 && cc <= 0xF4) need = 3;
    if (need == 0)                         { body += 1; p++; continue; }   // 非法首字节→'?'
    bool ok = true;
    for (int i = 1; ok && i <= need; i++) {
      unsigned char co = p[i];
      if (!co || !(co >= 0x80 && co <= 0xBF)) ok = false;
    }
    if (!ok) { body += 1; p++; }
    else     { body += 1 + need; p += 1 + need; }
  }
  size_t cap = pre_len + body + strlen(suf) + 1;
  char* buf = (char*)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
  if (!buf) return;                       // PSRAM 不足：静默丢弃（宁缺毋滥）
  char* w = buf;
  memcpy(w, pre, strlen(pre)); w += strlen(pre);
  memcpy(w, sc, strlen(sc));   w += strlen(sc);
  memcpy(w, pre2, strlen(pre2)); w += strlen(pre2);
  for (const unsigned char* p = (const unsigned char*)text; *p;) {
    unsigned char cc = *p;
    if (cc == '"' || cc == '\\') { *w++ = '\\'; *w++ = (char)cc; p++; continue; }
    if (cc == '\n') { *w++ = '\\'; *w++ = 'n'; p++; continue; }
    if (cc == '\r') { *w++ = '\\'; *w++ = 'r'; p++; continue; }
    if (cc == '\t') { *w++ = '\\'; *w++ = 't'; p++; continue; }
    if (cc < 0x20 || cc == 0x7f) { *w++ = '?'; p++; continue; }
    if (cc < 0x80) { *w++ = (char)cc; p++; continue; }   // 普通 ASCII(数字/英文/标点)原样
    int need = 0;
    if (cc >= 0xC2 && cc <= 0xDF) need = 1;
    else if (cc >= 0xE0 && cc <= 0xEF) need = 2;
    else if (cc >= 0xF0 && cc <= 0xF4) need = 3;
    if (need == 0) { *w++ = '?'; p++; continue; }
    bool ok = true;
    for (int i = 1; ok && i <= need; i++) {
      unsigned char co = p[i];
      if (!co || !(co >= 0x80 && co <= 0xBF)) ok = false;
    }
    if (!ok) { *w++ = '?'; p++; }
    else     { for (int i = 0; i <= need; i++) *w++ = (char)p[i]; p += 1 + need; }
  }
  memcpy(w, suf, strlen(suf) + 1);        // 含结尾 '\0'
  if (g_q) {
    if (xQueueSend(g_q, &buf, 0) != pdTRUE) heap_caps_free(buf);  // 满则释放，自带 free
  } else {
    heap_caps_free(buf);
  }
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
  // 栈先试 PSRAM（内部堆紧，见 heap_watch）：把 8KB 让回内部 DMA 池；失败退回内部栈保转发可用。
  if (!s_fwd_stack) s_fwd_stack = (StackType_t*)heap_caps_malloc(8192, MALLOC_CAP_SPIRAM);
  if (s_fwd_stack) {
    xTaskCreateStaticPinnedToCore(forward_task, "blog_fwd", 8192, nullptr, 1, s_fwd_stack, &s_fwd_tcb, 1);
  } else {
    xTaskCreatePinnedToCore(forward_task, "blog_fwd", 8192, nullptr, 1, nullptr, 1);
  }
}

}  // namespace blog