#include "ai_client.h"
#include "config.h"
#include "camera.h"
#include "uart.h"
#include "wifi_net.h"

#include <WiFiClientSecure.h>
#include <esp_timer.h>
#include <stdlib.h>  // malloc/free/strtol
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <ArduinoJson.h>
#include <esp_heap_caps.h>  // MALLOC_CAP_SPIRAM

// 决策频率 / 步数上限（对慢速小车足够，节奏见架构文档）。
#define AI_INTERVAL_MS 2500
#define AI_MAX_STEPS_PER_GOAL 120
#define AI_EDITED_IMG_MAX (128 * 1024)
#define AI_EDITED_IMG_TTL_MS 60000
#define AI_CONNECT_TIMEOUT_MS 10000
#define AI_HTTP_TIMEOUT_MS 30000
#define AI_WAIT_FB_MIN_MS 10000   // wait 反馈节流：同一动作少于此间隔只回一条

// ArduinoJson 内存池改用 PSRAM，避免其小分配每轮在内部堆上反复申请/释放，
// 与 TLS 缓冲交错把内部堆切成碎块（导致握手 -17040/-32512 失败）。
struct PsramAllocator : public ArduinoJson::Allocator {
  void* allocate(size_t n) override { return heap_caps_malloc(n, MALLOC_CAP_SPIRAM); }
  void deallocate(void* p) override { if (p) heap_caps_free(p); }
  void* reallocate(void* p, size_t n) override { return heap_caps_realloc(p, n, MALLOC_CAP_SPIRAM); }
};
static PsramAllocator g_js_alloc;

// ---------------- worker / 槽 / 队列 ----------------

struct TaskLocal {
  char* text = nullptr;   // 目标文本（堆）
  char* ann = nullptr;    // 标注 JSON 或 {x,y,w,h,label}（堆）
  bool use_image = false;
  bool one_shot = false;  // 单轮模式：只执行一轮决策即收尾（/ai oneshot）
  long id = 0;            // 对应 ai_goal 的词表 id（回填 ai_result）
  unsigned long generation = 0;
  cmd::ReplyFn fn = nullptr;
  void* ctx = nullptr;    // 任务期 sink ctx（WS=堆 int*，BLE=nullptr）
  bool active = false;    // 槽是否已被 set_goal 激活
};

// 结果队列项：每项独立持有消息文本与恢复用的（fn, 本次堆拷贝 ctx）。
struct ResultItem {
  char* text = nullptr;
  cmd::ReplyFn fn = nullptr;
  void* ctx = nullptr;    // 每次入队新建的 int*（WS）；BLE nullptr
};

static TaskLocal g_slot;
static SemaphoreHandle_t g_mtx = nullptr;   // 保护 g_slot / m_generation
static SemaphoreHandle_t g_notify = nullptr; // 唤醒 worker 的二进制信号量
static QueueHandle_t g_result_q = nullptr;
static volatile unsigned long m_generation = 0;  // 代际号：每次 set_goal/cancel 自增
static volatile bool m_busy = false;
static TaskHandle_t g_worker = nullptr;

// 编辑图暂存（PSRAM）+ 时间戳；worker 内只读快照由 set_edited_image/取图互斥。
static uint8_t* g_edited = nullptr;
static size_t g_edited_len = 0;
static uint64_t g_edited_ts = 0;
static SemaphoreHandle_t g_img_mtx = nullptr;

// TLS 客户端（worker 唯一实例）：cancel/set_goal 可从其他任务 stop() 中止在途请求。
static WiFiClientSecure g_client;

// 最近一条下发执行板的是否持续型（sink：stop 兜底判定）。任务起点复位。
static volatile bool g_last_continuous = false;

// 最近一条持续指令的类型（0=无/1=move/2=arm）。兜底 stop 时 Wheels 模式只适用于轮子残留。
static volatile int g_last_cont_type = 0;

// 本轮任务终结时的兜底 stop 模式，由打断方写入；worker 出口统一解析一次。
static volatile int g_stop_mode = (int)ai::StopMode::All;
static volatile uint64_t g_last_wait_fb_ms = 0;  // wait 反馈节流时间戳

static void enqueue_result(const char* text, cmd::ReplyFn fn, void* ctx);

// ---------------- 结果队列（worker → loop） ----------------

void enqueue_result(const char* text, cmd::ReplyFn fn, void* ctx) {
  ResultItem* it = (ResultItem*)malloc(sizeof(ResultItem));
  if (!it) return;
  size_t n = strlen(text);
  it->text = (char*)malloc(n + 1);
  if (!it->text) { free(it); return; }
  memcpy(it->text, text, n + 1);
  // WS：为本次结果单独堆拷贝 fd（loop 发送后释放）；BLE ctx 已为 nullptr。
  it->fn = fn;
  it->ctx = ctx ? new int(*(int*)ctx) : nullptr;
  if (xQueueSend(g_result_q, &it, 0) != pdTRUE) { free(it->text); free(it->ctx); free(it); }
}

void ai::update() {
  ResultItem* it = nullptr;
  while (g_result_q && xQueueReceive(g_result_q, &it, 0) == pdTRUE) {
    if (it) {
      if (it->fn) it->fn(it->ctx, it->text);
      free(it->text);
      if (it->ctx) delete (int*)it->ctx;
      free(it);
    }
  }
}

// ---------------- 编辑图暂存 ----------------

void ai::set_edited_image(const uint8_t* data, size_t len) {
  if (!data || len == 0 || len > AI_EDITED_IMG_MAX) return;
  xSemaphoreTake(g_img_mtx, portMAX_DELAY);
  if (!g_edited) g_edited = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
  if (g_edited) {
    memcpy(g_edited, data, len);
    g_edited_len = len;
    g_edited_ts = esp_timer_get_time();
  }
  xSemaphoreGive(g_img_mtx);
  Serial.printf("[ai] 收到编辑图 %u B\n", (unsigned)len);
}

static bool take_edited(uint8_t* buf, size_t cap, size_t* out_len) {
  bool ok = false;
  xSemaphoreTake(g_img_mtx, portMAX_DELAY);
  if (g_edited && g_edited_len > 0 &&
      (esp_timer_get_time() - g_edited_ts) < (uint64_t)AI_EDITED_IMG_TTL_MS * 1000 &&
      g_edited_len <= cap) {
    memcpy(buf, g_edited, g_edited_len);
    *out_len = g_edited_len;
    ok = true;
  }
  xSemaphoreGive(g_img_mtx);
  return ok;
}

// ---------------- PSRAM 增长缓冲 ----------------

struct PsaBuf {
  char* p = nullptr;
  size_t len = 0, cap = 0;
  bool ok = true;
  ~PsaBuf() { if (p) free(p); }
  bool ensure(size_t need) {
    if (len + need + 1 <= cap) return true;
    size_t nc = cap ? cap * 2 : 1024 * 1024;
    while (nc < len + need + 1) nc *= 2;
    char* np = (char*)heap_caps_realloc(p, nc, MALLOC_CAP_SPIRAM);
    if (!np) { ok = false; return false; }
    p = np; cap = nc; return true;
  }
  void put(const char* s) { if (!ok || !ensure(strlen(s)) ) { ok = false; return; } memcpy(p + len, s, strlen(s)); len += strlen(s); p[len] = 0; }
  void put(char c) { if (!ok || !ensure(1)) { ok = false; return; } p[len++] = c; p[len] = 0; }
};

// JSON 字符串转义（内部受控输入，转义引号与反斜杠足够）。
static void esc_append(PsaBuf& b, const char* s) {
  b.put('"');
  for (const char* c = s; *c; c++) {
    if (*c == '"' || *c == '\\') { b.put('\\'); b.put(*c); }
    else b.put(*c);
  }
  b.put('"');
}

static void b64_append(PsaBuf& b, const uint8_t* in, size_t inlen) {
  static const char T[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t i = 0;
  while (i + 2 < inlen) {
    uint32_t v = (in[i] << 16) | (in[i+1] << 8) | in[i+2];
    b.put(T[(v >> 18) & 63]); b.put(T[(v >> 12) & 63]); b.put(T[(v >> 6) & 63]); b.put(T[v & 63]);
    i += 3;
  }
  if (i + 1 == inlen) {
    uint32_t v = in[i] << 16;
    b.put(T[(v >> 18) & 63]); b.put(T[(v >> 12) & 63]); b.put('='); b.put('=');
  } else if (i + 2 == inlen) {
    uint32_t v = (in[i] << 16) | (in[i+1] << 8);
    b.put(T[(v >> 18) & 63]); b.put(T[(v >> 12) & 63]); b.put(T[(v >> 6) & 63]); b.put('=');
  }
}

// ---------------- AI 请求构建 ----------------

static void img_block(PsaBuf& b, const uint8_t* data, size_t len) {
  b.put("{\"type\":\"image_url\",\"image_url\":{\"url\":\"data:image/jpeg;base64,");
  b64_append(b, data, len);
  b.put("\",\"detail\":\"low\"}}");
}

// 构建请求 body。goal 目标文本；ann 标注 JSON 或空；hint 为纠正/引导语（重试或死循环打断）；
// last_cmd 上一步已下发指令的短描述（状态而非历史，帮助 AI 判断上一步效果）；
// exec_state 执行板状态一行文本（无数据为空串）。
// 系统提示词 + 目标 + 标注先组进 PSRAM 缓冲，再整体 JSON 转义（内含引号）。
// 有 edited 时带编辑图；有 prev 时带上一帧，与当前帧双帧对比供运动感知。
static void build_body(PsaBuf& b, const char* goal, const char* ann, const char* hint,
                       const char* last_cmd, const char* exec_state,
                       const uint8_t* frame, size_t frame_len,
                       const uint8_t* prev, size_t prev_len,
                       bool use_edited, const uint8_t* edited, size_t edited_len) {
  PsaBuf sys;
  sys.put("你是「小车+机械臂」视觉控制大脑。画面中操作者的红色标注（方框/箭头/文字）必须优先遵循。");
  sys.put("当前任务目标：");
  sys.put(goal);
  if (ann && ann[0]) { sys.put("（操作者标注区域："); sys.put(ann); sys.put("）"); }
  sys.put("上一步已下发：");
  sys.put(last_cmd && last_cmd[0] ? last_cmd : "无");
  sys.put("。每次只输出一个合法 JSON：");
  sys.put("{\"type\":\"move\",\"params\":{\"throttle\":0.3,\"steering\":0,\"distance_cm\":30},\"reason\":\"..\"} 移动/转向：加 distance_cm 定距、angle_deg 定角，否则持续移动；低速优先 throttle/steering≤0.5；");
  sys.put("或 {\"type\":\"arm\",\"params\":{\"act\":\"lift_up\",\"dist_cm\":15},\"reason\":\"..\"} act 取 lift_up/lift_down/reach_forward/reach_backward/clip/release；与操作者交接物品时先停稳、伸到其手边再 release；");
  sys.put("或 {\"type\":\"stop\",\"params\":{\"scope\":\"all\"},\"reason\":\"..\",\"done\":true} 立即停车并结束当前任务：任务完成/目标达成/需完全收手时带 done:true；仅临时停车继续观察则不带 done：");
  sys.put("或 {\"type\":\"wait\",\"reason\":\"..\"} 保持当前所有动作不变，原地等待观察：当还在运动中没到目标、或者画面没变化、或者还没锁定目标时，用 wait；");
  sys.put("规则：1.只输出 JSON，每次只规划一步。2.画面多轮无变化时先小幅转向环视探索；障碍物挡路则尝试绕行；绕行多轮仍无进展才 stop 并说明原因。3.停车信号只认明确手势：掌心正对镜头且五指张开、在镜头前持续上下/左右挥手、或人持续挡在车前，此时才 stop；人只是坐着、抬手或手指出现在画面里，不是停车信号。4.单手指向或手臂指向某一方向=操作者的方向指示，应朝该方向移动；画面中标出目标时朝标注区域移动，不要因为出现人手就停车。5.目标为空或“巡视”时持续小幅转向环视四周。6.reason 一句中文简要解释。");
  if (hint && hint[0]) { sys.put("注意："); sys.put(hint); }

  b.put("{\"model\":");
  esc_append(b, cfg::ai_model().c_str());
  b.put(",\"messages\":[{\"role\":\"system\",\"content\":");
  esc_append(b, sys.p ? sys.p : "");
  // 结束 system 对象后接下一消息
  b.put("},{\"role\":\"user\",\"content\":[{\"type\":\"text\",\"text\":");
  PsaBuf ut;  // user 文本：执行板状态 + 画面引导（整体转义一次）
  if (exec_state && exec_state[0]) { ut.put(exec_state); ut.put("；"); }
  if (frame) {
    if (prev) {
      ut.put("下面先给出上一帧、再给出当前帧：请对比这两帧，判断画面中移动的人手或物体大致朝哪个方向移动、在强调或指向哪一边；若某区域有移动，据其方向回应。");
    } else {
      ut.put("当前画面如下。");
    }
  } else {
    // 无摄像头降级：不让 AI 编造画面，只凭目标与状态规划保守指令
    ut.put("注意：摄像头不可用，当前无实时画面可分析，请勿假设或编造画面内容。只依据上述目标与执行板状态，稳妥地规划一步指令（优先短距离 move 或直接 stop）。");
  }
  esc_append(b, ut.p ? ut.p : "");
  b.put("}");
  if (frame) {
    // 图片块间需逗号分隔；首个（text 之后）不加。修 multi-image 缺逗号导致的 400。
    b.put(",");
    bool first = true;
    auto img = [&](const uint8_t* d, size_t n) {
      if (!first) b.put(',');
      img_block(b, d, n);
      first = false;
    };
    if (use_edited && edited) img(edited, edited_len);
    if (prev) img(prev, prev_len);   // 上一帧在前，供对比运动
    img(frame, frame_len);
  }
  b.put("]}],\"max_tokens\":1024,\"reasoning_effort\":\"low\",\"response_format\":{\"type\":\"json_object\"}}");
}

// 指令短描述（供"上一步已下发"拼接与死循环判定；含运动数值，便于识别"相同指令"）。
static void fmt_last(char* buf, size_t cap, const char* type, const JsonObjectConst& p) {
  if (!strcmp(type, "move")) {
    float th = p["throttle"] | 0.0f;
    float st = p["steering"] | 0.0f;
    int dc = p["distance_cm"] | 0;
    int ad = p["angle_deg"] | 0;
    if (dc) snprintf(buf, cap, "move %dcm th=%.1f", dc, th);
    else if (ad) snprintf(buf, cap, "move %d度 th=%.1f", ad, th);
    else snprintf(buf, cap, "move 持续 th=%.1f st=%.1f", th, st);
  } else if (!strcmp(type, "arm")) {
    const char* act = p["act"] | "";
    int pd = p["dist_cm"] | 0;
    if (pd) snprintf(buf, cap, "arm %s %dcm", act, pd);
    else snprintf(buf, cap, "arm %s 持续", act);
  } else {
    snprintf(buf, cap, "stop");
  }
}

// ---------------- 输出校验（防误动作） ----------------
// 校验并规范化 AI 输出到 out{type,params,reason}。返回 nullptr 通过；否则返回错误字符串。
static const char* validate_cmd(const char* content, JsonDocument& out, char* err_buf, size_t err_cap) {
  // 剥代码块/首尾空白（Arduino String 无 find/left，用 indexOf/substring）
  String c = content;
  c.trim();
  if (c.startsWith("```")) {
    int e = c.indexOf('\n');
    if (e >= 0) c = c.substring(e + 1);
    c.trim();
    if (c.endsWith("```")) c = c.substring(0, c.length() - 3);
    c.trim();
  }

  JsonDocument doc;
  if (deserializeJson(doc, c)) {
    snprintf(err_buf, err_cap, "AI 返回非 JSON：%s", c.substring(0, 80).c_str());
    return err_buf;
  }
  const char* type = doc["type"] | "";
  if (strcmp(type, "move") && strcmp(type, "stop") && strcmp(type, "arm") && strcmp(type, "wait")) {
    return "AI 输出非法 type";
  }
  if (!doc["params"].is<JsonObject>() && strcmp(type, "stop") && strcmp(type, "wait")) {
    return "AI 输出缺 params";
  }
  if (!strcmp(type, "arm")) {
    const char* act = doc["params"]["act"] | "";
    static const char* acts[] = {"lift_up","lift_down","reach_forward","reach_backward","clip","release", nullptr};
    bool good = false;
    for (int i = 0; acts[i]; i++) if (!strcmp(act, acts[i])) { good = true; break; }
    if (!good) { snprintf(err_buf, err_cap, "AI arm 非法 act=%s", act); return err_buf; }
  }

  out["type"] = type;
  JsonObject p = out["params"].to<JsonObject>();
  if (doc["params"].is<JsonObject>()) {
    JsonVariantConst src = doc["params"];
    // 数值钳制
    float th = doc["params"]["throttle"] | 0.0f;
    float st = doc["params"]["steering"] | 0.0f;
    p["throttle"] = constrain(th, -1.0f, 1.0f);
    p["steering"] = constrain(st, -1.0f, 1.0f);
    int dc = doc["params"]["distance_cm"] | 0;
    int ad = doc["params"]["angle_deg"] | 0;
    int pd = doc["params"]["dist_cm"] | 0;
    if (src["distance_cm"].is<int>() && dc) p["distance_cm"] = constrain(dc, 0, 500);
    if (src["angle_deg"].is<int>() && ad) p["angle_deg"] = constrain(ad, 0, 500);
    if (src["dist_cm"].is<int>() && pd) p["dist_cm"] = constrain(pd, 0, 500);
    const char* act = doc["params"]["act"] | "";
    if (act[0]) p["act"] = act;
    const char* scope = doc["params"]["scope"] | "all";
    p["scope"] = !strcmp(scope, "wheels") ? "wheels" : (!strcmp(scope, "arm") ? "arm" : "all");
  }
  const char* reason = doc["reason"] | "";
  if (reason[0]) out["reason"] = reason;
  if (doc["done"].is<bool>() && doc["done"].as<bool>()) out["done"] = true;  // 任务完结标记
  return nullptr;
}

// ---------------- HTTPS POST（keep-alive 复用连接） ----------------
// 直接走 TLS socket 裸写 HTTP/1.1，绕开 HTTPClient 库——其 cookie/Date 解析
// 会引入 libc time/gmtime/mktime/strptime 等函数（被链接脚本强制放 IRAM，
// 实测导致 iram0 溢出）。DeepSeek 边缘响应为 chunked（无 Content-Length），
// 见下方按 CL / chunked 两种分支读取。
// 连接策略：同一 host 复用 TLS 连接（Connection: keep-alive），省掉每轮 ~500ms
// 握手；复用连接被服务端空闲断开/半开时，http_post 检测后重建一次立即重发。
// 响应按块严格消费（chunked 终止块后吞 trailer 到空行），保证流干净可复用。

// 单次请求-响应交换（连接须已就绪）。成功返回 true 且保持连接供下次复用；
// 任何协议/IO 异常返回 false（http_post 决定重建或弃连）。
static bool http_exchange(const String& host_s, const char* key, const char* path_s,
                          const char* body, String& resp, bool* reusable) {
  resp = "";
  *reusable = true;   // 默认可复用；响应头明确 Connection: close 则置 false（下轮直接新建）
  g_client.printf("POST %s HTTP/1.1\r\n", path_s);
  g_client.printf("Host: %s\r\n", host_s.c_str());
  g_client.printf("Authorization: Bearer %s\r\n", key);
  g_client.print("Content-Type: application/json\r\n");
  g_client.printf("Content-Length: %d\r\n", (int)strlen(body));
  g_client.print("Connection: keep-alive\r\n\r\n");

  // 诊断：body 可能达数十 KB，一次 write 成败难定位。分段发，打印每段进度与耗时。
  size_t blen = strlen(body);
  Serial.printf("[ai] body len=%u\n", (unsigned)blen);
  const uint8_t* bp = (const uint8_t*)body;
  const size_t SEG = 4096;
  size_t sent = 0;
  unsigned long t_start = millis();
  size_t seg = 0;
  unsigned long t_last_write = millis();  // 连续无进展起点（防复用死连接傻等 30s）
  while (sent < blen) {
    size_t n = (blen - sent < SEG) ? (blen - sent) : SEG;
    unsigned long ts = millis();
    size_t w = g_client.write(bp + sent, n);
    unsigned long dt = millis() - ts;
    if (w > 0) { seg++; sent += w; t_last_write = millis(); if (dt > 100) Serial.printf("[ai] seg %u sent=%u ~%u ms\n", seg, (unsigned)sent, (unsigned)dt); continue; }
    // 返回 0 或负：发送卡住/失败。连续 3s 无进展即视为连接已死，弃连让上层重建。
    if (millis() - t_last_write > 3000) {
      Serial.printf("[ai] 发送停滞 sent=%u/%u，弃连\n", (unsigned)sent, (unsigned)blen);
      return false;
    }
    if (millis() - t_start > AI_HTTP_TIMEOUT_MS) break;
    delay(10);
  }
  Serial.printf("[ai] body 发送完成 sent=%u/%u ~%u ms\n", (unsigned)sent, (unsigned)blen,
                (unsigned)(millis() - t_start));
  if (sent < blen) return false;

  // 读响应头（上限 4KB；复用连接被服务端关闭时 connected()=false，快速失败）
  String hdr;
  unsigned long t0 = millis();
  while (hdr.length() < 4096 && millis() - t0 < AI_HTTP_TIMEOUT_MS) {
    while (g_client.available()) {
      int c = g_client.read();
      if (c < 0) break;
      hdr += (char)c;
      if (hdr.endsWith("\r\n\r\n")) break;
    }
    if (hdr.endsWith("\r\n\r\n")) break;
    if (!g_client.connected()) { Serial.println("[ai] 响应头读取中连接关闭"); return false; }
    delay(5);
  }
  if (!hdr.endsWith("\r\n\r\n")) { Serial.println("[ai] HTTP 响应头超时/不完整"); return false; }
  // 去掉状态行前残留的空白（前一响应/复用连接遗留的换行），保证 startsWith 判断准确
  while (hdr.length() > 0 && (hdr[0] == '\r' || hdr[0] == '\n')) hdr = hdr.substring(1);
  // 服务端明确 Connection: close（大小写不敏感）：本连接不可复用，下轮直接新建
  String hlc = hdr; hlc.toLowerCase();
  if (hlc.indexOf("connection: close") >= 0) *reusable = false;
  // 提前解析 Content-Length / 传输编码 / 内容类型，按实际帧格式读取响应体
  int cl = 0;
  int idx = hdr.indexOf("Content-Length:");
  if (idx >= 0) { String v = hdr.substring(idx + 15); v.trim(); cl = v.toInt(); }
  bool chunked = hlc.indexOf("transfer-encoding: chunked") >= 0;
  bool sse = hlc.indexOf("content-type: text/event-stream") >= 0;
  if (!hdr.startsWith("HTTP/1.1 200") && !hdr.startsWith("HTTP/1.0 200")) {
    // 非 2xx：把错误响应体（前 512B）也打出来，便于定位 400 的具体 message
    String errbody;
    int want = cl > 0 ? (cl < 512 ? cl : 512) : 0;
    long te = millis();
    while (want > 0 && (int)errbody.length() < want && millis() - te < 2000) {
      while (want > 0 && g_client.available() && (int)errbody.length() < want) errbody += (char)g_client.read();
      delay(3);
    }
    Serial.printf("[ai] HTTP %s body=%s", hdr.c_str(), errbody.c_str());
    return false;
  }
  // 响应体用 PsaBuf（PSRAM）累积，避免逐字符 String grow 在内部堆反复 realloc 制造碎片。
  PsaBuf pb;
  if (cl > 0) {
    unsigned long tr = millis();
    while ((int)pb.len < cl && millis() - tr < AI_HTTP_TIMEOUT_MS) {
      while (g_client.available() && (int)pb.len < cl) {
        int c = g_client.read();
        if (c < 0) break;
        pb.put((char)c);
      }
      if ((int)pb.len >= cl) break;
      if (!g_client.connected()) break;  // 服务端提前关连接
      delay(5);
    }
    if ((int)pb.len < cl) { Serial.printf("[ai] HTTP body 未读完 got=%d cl=%d\n", (int)pb.len, cl); return false; }
  } else if (chunked) {
    // chunked 解码：严格按块边界消费；0 块后吞 trailer 到空行，连接保持干净供复用。
    // 抗抖：DeepSeek 边缘偶发在 chunk 边界缺 \r\n，会把后续 size 行错位 2 字节，
    // 旧实现直接判死→弃连→重握手。这里两层容错：
    //  1) 块尾只吞判定字节，非 \r\n 即压回（pend），不吞掉下一 size 行首字节；
    //  2) size 行解析失败时向后扫描最近一个合法 size 行重同步（限时限行），
    //     仍失败才弃连让上层重建。
    unsigned long tk = millis();
    auto read_byte = [&]() -> int {   // 读一个字节，带总体超时；连接关闭立即失败
      while (millis() - tk < AI_HTTP_TIMEOUT_MS) {
        if (g_client.available() > 0) { int c = g_client.read(); if (c >= 0) return c; }
        if (!g_client.connected()) return -1;
        delay(2);
      }
      return -1;
    };
    int pend = -1;                    // 单字节压回（块尾容错：缺 \r\n 时把字节还给下一行）
    auto next_byte = [&]() -> int {
      if (pend >= 0) { int c = pend; pend = -1; return c; }
      return read_byte();
    };
    // 读一行（忽略 \r，遇 \n 停，\n 已消费）。cap=保留字符上限（超长丢弃但继续
    // 读到 \n 保持行对齐）；tmo_ms=单行超时。返回 0=正常、-1=EOF/连接关闭。
    auto read_line = [&](String& out, size_t cap, unsigned long tmo_ms) -> int {
      out = "";
      unsigned long tl = millis();
      while (millis() - tl < tmo_ms) {
        int c = next_byte();
        if (c < 0) return -1;
        if (c == '\r') continue;
        if (c == '\n') return 0;
        if (out.length() < cap) out += (char)c;
      }
      return 0;   // 超时：返回已收内容，由外层 parse 失败走重同步/弃连
    };
    // 严格十六进制 chunk-size 解析（支持 ";ext" 扩展与首尾空白；垃圾行报错而非静默截断）
    auto parse_size = [](String line, int* out) -> bool {
      line.trim();
      int semi = line.indexOf(';');          // chunk 扩展：忽略其后内容
      if (semi >= 0) line = line.substring(0, semi);
      if (line.length() == 0 || line.length() > 8) return false;
      long v = 0;
      for (int i = 0; i < (int)line.length(); i++) {
        char c = line[i];
        int d;
        if (c >= '0' && c <= '9') d = c - '0';
        else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
        else return false;
        v = (v << 4) | d;
      }
      *out = (int)v;
      return true;
    };
    for (;;) {                        // 循环解析各块
      String line;                    // 读 chunk-size 行（忽略 \r，遇到 \n 停）
      if (read_line(line, 64, 5000) < 0) { Serial.printf("[ai] chunked size 行读取失败\n"); return false; }
      int sz = 0;
      if (!parse_size(line, &sz)) {
        // 重同步：向后逐行找合法 size 行（限 3s / 5 行），吸收帧错位；
        // 找不到才按原逻辑打印诊断并弃连。
        bool resynced = false;
        int rs = 0;
        unsigned long t_rs = millis();
        while (!resynced && rs < 5 && millis() - t_rs < 3000) {
          String cand;
          if (read_line(cand, 32, 500) < 0) break;
          int cs = 0;
          if (parse_size(cand, &cs) && cs >= 0 && cs <= 262144) { sz = cs; resynced = true; }
          rs++;
        }
        if (!resynced) {
          Serial.printf("[ai] chunked size 行异常: '%.32s'\n", line.c_str());
          // 诊断：帧格式错位定位。打印响应头尾部与后续原始字节（hex），
          // 确认是服务端帧格式异常、还是读取层错位（首行被吞字节）。
          String htail = hdr.length() > 48 ? hdr.substring(hdr.length() - 48) : hdr;
          htail.replace("\r", "\\r"); htail.replace("\n", "\\n");
          Serial.printf("[ai] hdr尾: %s\n", htail.c_str());
          Serial.print("[ai] 流hex: ");
          uint8_t dbg[64]; int dn = 0;
          while (dn < 64 && g_client.available()) { int dc = g_client.read(); if (dc < 0) break; dbg[dn++] = (uint8_t)dc; }
          for (int i = 0; i < dn; i++) Serial.printf("%02X ", dbg[i]);
          Serial.printf("(%d B)\n", dn);
          return false;
        }
        Serial.printf("[ai] chunked 重同步 → sz=%d\n", sz);
      }
      if (sz == 0) {
        // 终止块：吞掉 trailer 行直到空行（keep-alive 复用必须把流清干净；连接若已关也视为干净结束）
        for (;;) {
          String t;
          if (read_line(t, 256, 5000) < 0) break;
          if (t.length() == 0) break;
        }
        break;
      }
      if (sz < 0 || sz > 262144) { Serial.println("[ai] chunked 块过大"); return false; }
      for (int i = 0; i < sz; i++) {
        int c = next_byte();
        if (c < 0) { Serial.printf("[ai] HTTP chunked 中断 got=%d\n", (int)pb.len); return false; }
        pb.put((char)c);
      }
      // 块尾容错：只吞判定字节。标准 \r\n / 仅 \n 都接受；否则视为缺块尾，
      // 把该字节压回给下一 size 行（服务端缺 \r\n 时不再错位 2 字节）。
      int t1 = next_byte();
      if (t1 < 0) { Serial.println("[ai] chunked 块尾缺失"); return false; }
      if (t1 == '\r') {
        int t2 = next_byte();
        if (t2 < 0) { Serial.println("[ai] chunked 块尾缺失"); return false; }
        if (t2 != '\n') pend = t2;   // \r 后非 \n（异常帧），还给下一行
      } else if (t1 != '\n') {
        pend = t1;                    // 缺 \r\n：首字节是下一 size 行内容，压回
      }
    }
    Serial.printf("[ai] HTTP chunked 解码 %d B\n", (int)pb.len);
  } else if (sse) {
    // SSE（text/event-stream，未走 chunked 帧）：按行读，累积 data: 负载；
    // [DONE] 事件或连接关闭即结束。产物为拼接后的 JSON 文本，交 extract_content 解析。
    unsigned long ts2 = millis();
    String ln;
    bool sse_end = false;
    while (!sse_end && millis() - ts2 < AI_HTTP_TIMEOUT_MS && g_client.connected()) {
      while (g_client.available() && !sse_end) {
        int c = g_client.read();
        if (c < 0) break;
        if (c == '\n') {
          String d = ln; ln = "";
          d.trim();
          if (d.startsWith("data:")) {
            String payload = d.substring(5); payload.trim();
            if (payload == "[DONE]") { sse_end = true; break; }
            pb.put(payload.c_str());
            pb.put('\n');   // 多条 data 负载间保留换行，便于 JSON 拼接/调试
          }
        } else if (c != '\r') {
          ln += (char)c;
        }
      }
      if (!g_client.connected()) break;
      delay(2);
    }
    Serial.printf("[ai] SSE 解码 %d B\n", (int)pb.len);
  } else {
    // 无 CL、无 chunked、无 SSE：裸读直到连接关闭或超时（10s 封顶），尽量救回响应体。
    unsigned long tr3 = millis();
    while (millis() - tr3 < 10000) {
      while (g_client.available()) {
        int c = g_client.read();
        if (c < 0) break;
        pb.put((char)c);
      }
      if (!g_client.connected()) break;
      delay(5);
    }
    Serial.printf("[ai] 原始读取 %d B\n", (int)pb.len);
  }
  resp = pb.p ? pb.p : "";
  return pb.len > 0;
}

static bool http_post(const char* url, const char* key, const char* body, String& resp,
                      unsigned long gen) {
  // 解析 [scheme://]host[:port]/path
  const char* p = strstr(url, "://");
  p = p ? p + 3 : url;
  const char* slash = strchr(p, '/');
  String host_s = slash ? String(p, slash - p) : String(p);
  String path_s = slash ? String(slash) : String("/");
  int port = 443;
  int colon = host_s.indexOf(':');
  if (colon >= 0) { port = host_s.substring(colon + 1).toInt(); host_s = host_s.substring(0, colon); }

  // keep-alive：同一 host 且连接仍活则复用，否则重建
  static String s_host;   // 当前保持 keep-alive 的 host（空=未建立）
  bool reuse = !s_host.isEmpty() && s_host == host_s && g_client.connected();
  if (!reuse) {
    g_client.stop();
    g_client.setInsecure();  // 开发期简单可靠；上线建议嵌入 CA
    g_client.setConnectionTimeout(AI_CONNECT_TIMEOUT_MS);
    g_client.setTimeout(AI_HTTP_TIMEOUT_MS);
    // 诊断：TLS 握手需内部 RAM 连续块（默认 mbedTLS in/out 各 16KB），
    // 打印当前内部堆/最大连续块，便于确认是否堆耗尽/碎片化。
    Serial.printf("[ai] TLS前 freeHeap=%u maxBlock=%u freePsram=%u\n",
                  ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getFreePsram());
    unsigned long t_conn = millis();
    if (!g_client.connect(host_s.c_str(), port)) { s_host = ""; Serial.println("[ai] TLS 连接失败"); return false; }
    // 诊断：握手耗时（毫秒），区分"网络慢/超时"与"阻塞卡顿"
    Serial.printf("[ai] TLS 握手完成 ~%u ms（新建）\n", (unsigned)(millis() - t_conn));
    s_host = host_s;
  } else {
    Serial.println("[ai] TLS 复用（keep-alive）");
  }

  bool reusable = true;
  bool ok = http_exchange(host_s, key, path_s.c_str(), body, resp, &reusable);
  if (!ok && reuse) {
    // 复用连接被服务端空闲断开/半开：重建一次立即重发，比走调用方退避快得多。
    if (gen != m_generation) { g_client.stop(); s_host = ""; return false; }  // 已被 cancel/新目标打断：不重建
    Serial.println("[ai] 复用连接失效，重建重试");
    g_client.stop(); s_host = "";
    g_client.setInsecure();
    g_client.setConnectionTimeout(AI_CONNECT_TIMEOUT_MS);
    g_client.setTimeout(AI_HTTP_TIMEOUT_MS);
    unsigned long t_conn = millis();
    if (g_client.connect(host_s.c_str(), port)) {
      Serial.printf("[ai] TLS 重连完成 ~%u ms\n", (unsigned)(millis() - t_conn));
      s_host = host_s;
      ok = http_exchange(host_s, key, path_s.c_str(), body, resp, &reusable);
    } else {
      s_host = "";
      Serial.println("[ai] TLS 重连失败");
    }
  }
  if (!ok || !reusable) { g_client.stop(); s_host = ""; }   // 响应流损坏/失败，或服务端要求 close：弃连，下次重建
  return ok;
}

// 从响应提取 choices[0].message.content；同时打印 reasoning_content（思考过程，截断防刷屏）。
static bool extract_content(const String& resp, String& content) {
  // 若响应是 SSE 文本（chunked-SSE 解码产物）：按行取 data: 负载拼成最终 JSON
  String payload = resp;
  if (resp.startsWith("data:") || resp.indexOf("\ndata:") >= 0) {
    payload = "";
    int i = 0, n = resp.length();
    while (i <= n) {
      int j = resp.indexOf('\n', i);
      if (j < 0) j = n;
      String ln = resp.substring(i, j);
      i = j + 1;
      ln.trim();
      if (ln.startsWith("data:")) {
        String d = ln.substring(5); d.trim();
        if (d == "[DONE]") break;
        payload += d;
      }
    }
  }
  JsonDocument doc(&g_js_alloc);
  if (deserializeJson(doc, payload)) return false;
  const char* rc = doc["choices"][0]["message"]["reasoning_content"] | "";
  if (rc[0]) {
    String r = rc;
    if (r.length() > 200) r = "..." + r.substring(r.length() - 200);
    Serial.printf("[ai] 思考: %s\n", r.c_str());
  }
  const char* c = doc["choices"][0]["message"]["content"] | "";
  content = c;
  return content.length() > 0;
}

// ---------------- 结果文本构建 ----------------

static String build_feedback(unsigned long id, const JsonDocument& cmd) {
  JsonDocument out(&g_js_alloc);
  out["type"] = "ai_result";
  if (id) out["id"] = (long)id;
  JsonObject p = out["params"].to<JsonObject>();
  const char* reason = cmd["reason"] | "";
  p["reason"] = reason;
  if (cmd["error"]) p["error"] = cmd["error"].as<const char*>();
  if (cmd["done"].is<bool>()) p["done"] = cmd["done"].as<bool>();
  // 内嵌词表指令（供 Godot 显示）
  if (cmd["type"] ) {
    JsonObject inner = p["command"].to<JsonObject>();
    inner["type"] = cmd["type"].as<const char*>();
    if (cmd["params"].is<JsonObject>()) inner["params"] = cmd["params"].as<JsonObjectConst>();
    const char* rs = cmd["reason"] | "";
    if (rs[0]) inner["reason"] = rs;
  }
  String s;
  serializeJson(out, s);
  return s;
}

// ---------------- worker 任务 ----------------

static void ai_worker(void*) {
  for (;;) {
    xSemaphoreTake(g_notify, portMAX_DELAY);

    TaskLocal t;
    xSemaphoreTake(g_mtx, portMAX_DELAY);
    if (g_slot.active) {
      t = g_slot;
      g_slot.text = nullptr; g_slot.ann = nullptr; g_slot.ctx = nullptr; g_slot.active = false;
    }
    xSemaphoreGive(g_mtx);
    if (!t.text) continue;
    g_last_continuous = false;  // 本任务尚未下发过持续指令（防上一任务残留标志误判）
    g_last_cont_type = 0;
    m_busy = true;
    // 摄像头可用性在任务起点判定（init 后即定）：不可用则整轮走无画面降级
    bool cam_ok = cam::available();
    Serial.printf("[ai] 任务开始 gen=%lu text=%s%s\n", t.generation, t.text, cam_ok ? "" : "（无摄像头→无画面模式）");
    // 打印实际端点/模型，便于排查 404/401 等云端拒绝（配错路径是常见原因）
    Serial.printf("[ai] 端点=%s 模型=%s key=%s\n", cfg::ai_url().c_str(), cfg::ai_model().c_str(),
                  cfg::ai_key().isEmpty() ? "空" : "已配置");

    unsigned long steps = 0;
    bool done = false;
    const char* fail = nullptr;
    char err_buf[160];
    // 死循环防线状态：上一步指令短描述 / 连续相同指令计数 / 已注入引导标记。
    char last_cmd[48] = {0}, cur_cmd[48] = {0};
    int stall = 0;
    bool stall_hint = false;

    // 编辑图一次性取快照（供整轮任务复用，避免中途被覆盖）。
    uint8_t* edited = nullptr; size_t edited_len = 0;
    if (t.use_image) {
      edited = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
      if (edited && !take_edited(edited, AI_EDITED_IMG_MAX, &edited_len)) { free(edited); edited = nullptr; }
    }
    // 上一轮画面帧（供双帧运动感知；任务期复用，首轮为空则只发当前帧）。
    uint8_t* prev = nullptr; size_t prev_len = 0;
    // 本轮帧的 PSRAM 副本（任务期复用）：抓帧后立即把字节拷进来并归还相机缓冲，
    // 避免 AI 在做慢速 TLS 请求期间长时间占用 fb_count=2 的缓冲池把推流饿死。
    uint8_t* cur = nullptr; size_t cur_len = 0;

    // 兜底 stop（任务终结出口统一解析一次）。g_stop_mode 由打断方写入：
    // None=手动 move/stop 接管（用户指令已覆盖，不补停）；Wheels=手动 arm（只停轮子）；All=其余。
    // 仅当存在持续指令残留才补。
    auto resolve_stop = [&]() {
      if (g_stop_mode == (int)ai::StopMode::None) return;
      if (!g_last_continuous) return;
      // Wheels 模式只对轮子持续残留停轮子；臂持续残留（或未知）必须全停。
      const char* scope = (g_stop_mode == (int)ai::StopMode::Wheels && g_last_cont_type == 1) ? "wheels" : "all";
      JsonDocument d; d["scope"] = scope;   // d 即 stop 的 params 对象
      uart::act("stop", d.as<JsonObjectConst>());
      Serial.printf("[ai] 兜底 stop scope=%s\n", scope);
      g_last_continuous = false;
    };

    while (!done) {
      uint64_t step_ts = esp_timer_get_time();  // 本轮起点（周期控制基准）
      // 中止检查（代际号变化即本任务作废）；兜底停统一在任务出口解析。
      if (t.generation != m_generation) { Serial.println("[ai] 被新目标/手动中断"); break; }
      if (!net::is_connected()) { snprintf(err_buf, sizeof(err_buf), "WiFi 掉线"); fail = err_buf; break; }
      if (cfg::ai_key().isEmpty()) { snprintf(err_buf, sizeof(err_buf), "未配置 AI Key"); fail = err_buf; break; }

      // 取当前帧（失败重试，避免推流占缓冲时一次失败即判死）；无摄像头则跳过，走无画面降级
      camera_fb_t* fb = nullptr;
      if (cam_ok) {
        for (int fr = 0; fr < 3 && !fb; fr++) {
          fb = cam::grab();
          if (!fb && fr < 2) vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (!fb) { snprintf(err_buf, sizeof(err_buf), "取帧失败"); fail = err_buf; break; }
      }
      const uint8_t* frame = fb ? fb->buf : nullptr;
      size_t frame_len = fb ? fb->len : 0;

      // 抓帧后立即拷入 PSRAM 副本并归还相机缓冲：AI 的 HTTPS（TLS 握手+请求）
      // 慢则数秒，期间一直占着 fb 会把 fb_count=2 的缓冲池耗尽，饿死并行推流。
      // 帧数据后续（body 组装、prev 双帧）一律用这份副本。
      if (fb && frame_len > 0 && frame_len <= AI_EDITED_IMG_MAX) {
        if (!cur) cur = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
        if (cur) { memcpy(cur, frame, frame_len); cur_len = frame_len; }
        else cur_len = 0;
      }
      cam::return_frame(fb);
      // 副本就绪才认本帧（fb 已归还，直接指向裸 fb->buf 即为悬垂）：过大或拷入失败则按无画面处理。
      if (cur_len > 0) { frame = cur; frame_len = cur_len; }
      else { frame = nullptr; frame_len = 0; }

      bool got = false;
      for (int attempt = 0; attempt < 2 && !done; attempt++) {
        // 引导语：重试纠正 / 死循环打断（连续多轮相同指令）
        const char* hint = attempt ? "上次输出非法，请只输出合法 JSON 词表指令。"
                                   : (stall_hint ? "画面与指令多轮无进展：请先小幅转向环视探索，或判断任务无法达成则输出 stop。" : "");
        PsaBuf body;
        char st[64];
        const char* stp = uart::read_state(st, sizeof(st)) ? st : "";  // 执行板状态（无数据为空）
        build_body(body, t.text, t.ann, hint, last_cmd, stp,
                   frame, frame_len, prev, prev_len, edited != nullptr, edited, edited_len);
        if (!body.ok) { fail = "组装请求 body 失败"; break; }

        String resp;
        bool http_ok = false;
        for (int nr = 0; nr < 3 && !http_ok; nr++) {   // 网络失败指数退避重试（任务串行，代价可控）
          if (http_post(cfg::ai_url().c_str(), cfg::ai_key().c_str(), body.p, resp, t.generation)) { http_ok = true; break; }
          if (t.generation != m_generation) { done = true; break; }  // 被中止，静默作废
          if (nr < 2) { vTaskDelay(pdMS_TO_TICKS(500 << nr)); Serial.printf("[ai] 网络失败重试 %d\n", nr + 1); }
        }
        if (done) break;
        if (!http_ok) { fail = "AI 请求失败"; break; }
        if (t.generation != m_generation) { done = true; break; }  // 在途结果作废

        String content;
        if (!extract_content(resp, content)) {
          // 解码成功但无有效内容（瞬态错误体/空 content 等）：打印原始片段便于定位
          Serial.printf("[ai] 响应无内容，原始(前120B): %s\n", resp.substring(0, 120).c_str());
          fail = "AI 响应无内容";
          continue;  // 空内容→重试，不终止
        }

        JsonDocument cmdD;
        const char* verr = validate_cmd(content.c_str(), cmdD, err_buf, sizeof(err_buf));
        if (!verr) {
          const char* type = cmdD["type"] | "";
          JsonObjectConst params = cmdD["params"].as<JsonObjectConst>();
          if (!strcmp(type, "wait")) {
            // wait=空操作：不下发执行板，保持当前动作，任务继续观察。
            // 视为有意进展：复位死循环计数，避免"等待"被当成无进展注入引导。
            stall = 0; stall_hint = false;
            uint64_t now = (uint64_t)(esp_timer_get_time() / 1000);
            if (now - g_last_wait_fb_ms >= AI_WAIT_FB_MIN_MS) {  // 反馈节流，防每轮刷屏
              g_last_wait_fb_ms = now;
              String fb = build_feedback(t.id, cmdD);
              enqueue_result(fb.c_str(), t.fn, t.ctx);
            }
            got = true;
            break;
          }
          // 校验通过 → 执行 move/arm/stop
          uart::act(type, params);
          g_last_continuous = uart::is_continuous(type, params);
          g_last_cont_type = g_last_continuous ? (!strcmp(type, "move") ? 1 : 2) : 0;
          Serial.printf("[ai] 执行 %s%s\n", type, g_last_continuous ? "（持续）" : "");
          String fb = build_feedback(t.id, cmdD);
          enqueue_result(fb.c_str(), t.fn, t.ctx);
          // 死循环防线：连续多轮下发相同指令 → 下轮注入引导语让 AI 主动变化
          fmt_last(cur_cmd, sizeof(cur_cmd), type, params);
          if (strcmp(cur_cmd, last_cmd)) { stall = 0; stall_hint = false; }
          else if (++stall >= 4 && !stall_hint) { stall_hint = true; Serial.println("[ai] 多轮无进展，注入引导"); }
          snprintf(last_cmd, sizeof(last_cmd), "%s", cur_cmd);
          // stop 仅停车，不再结束任务；任务由手动中断/cancel 或步数上限结束
          got = true;
          break;
        } else {
          Serial.printf("[ai] 校验: %s\n", verr);
          fail = verr;  // 重试一次前暂存
        }
      }
      // 用当前帧（PSRAM 副本 cur）更新"上一帧"，供下轮双帧对比运动。相机缓冲已在上方提前归还。
      if (frame) {
        if (!prev) prev = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
        if (prev && frame_len <= AI_EDITED_IMG_MAX) { memcpy(prev, frame, frame_len); prev_len = frame_len; }
        else prev_len = 0;
      }

      if (done) break;
      // 单轮模式（/ai oneshot）：一轮决策即收尾。本轮无有效输出（网络/解码/校验失败）
      // 时把原因作为 error 回报，不跳 3s 继续观察；持续指令残留由任务出口 resolve_stop 补停。
      if (t.one_shot) {
        if (!got && fail) {
          JsonDocument e(&g_js_alloc);
          e["error"] = fail;
          String s = build_feedback(t.id, e);
          enqueue_result(s.c_str(), t.fn, t.ctx);
        }
        done = true;
        break;
      }
      if (!got) {
        // 本轮无有效输出（网络/解码/校验瞬态失败）：不终结任务，跳过本轮继续观察，
        // 由 AI 输出 stop(done) / 手动中断 / 步骤上限收尾。多等 3s 给瞬态错误
        // （限流/边缘抖动）喘息，避免密集重试把服务端打得更紧。
        Serial.printf("[ai] 本轮无有效输出，跳过继续（%s）\n", fail ? fail : "未知");
        vTaskDelay(pdMS_TO_TICKS(3000));
      }

      if (++steps >= AI_MAX_STEPS_PER_GOAL) {
        JsonDocument e(&g_js_alloc); e["done"] = true; String s = build_feedback(t.id, e); enqueue_result(s.c_str(), t.fn, t.ctx);
        break;
      }
      // 周期控制：以 2000ms 为节奏，扣掉本轮已耗时（含抓帧/HTTP/校验），快路径约 3.2s 一轮
      uint64_t el = esp_timer_get_time() - step_ts;
      long rem = (long)AI_INTERVAL_MS - (long)(el / 1000);
      if (rem > 0) vTaskDelay(pdMS_TO_TICKS(rem));
    }

    if (edited) free(edited);
    if (cur) free(cur);          // 本轮帧 PSRAM 副本
    if (prev) free(prev);        // 上一帧 PSRAM 副本
    if (t.ctx) delete (int*)t.ctx;  // 任务期 sink fd
    free(t.text); free(t.ann);
    resolve_stop();   // 统一兜底：持续指令残留即补停
    m_busy = false;
    Serial.printf("[ai] 任务结束 gen=%lu\n", t.generation);
  }
}

// ---------------- 对外 ----------------

void ai::init() {
  if (g_worker) return;
  g_mtx = xSemaphoreCreateMutex();
  g_notify = xSemaphoreCreateBinary();
  g_img_mtx = xSemaphoreCreateMutex();
  g_result_q = xQueueCreate(8, sizeof(ResultItem*));
  xTaskCreatePinnedToCore(ai_worker, "ai_worker", 16384, nullptr, 2, &g_worker, 1);
  Serial.println("[ai] worker 就绪");
}

void ai::set_goal(const char* text, bool use_image, const char* annotation, long id,
                  cmd::ReplyFn reply, void* reply_ctx, bool one_shot) {
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  ++m_generation;
  // 替换旧槽（旧字符串为空因 worker 已取走；残余则释放）
  if (g_slot.text) free(g_slot.text);
  if (g_slot.ann) free(g_slot.ann);
  if (g_slot.ctx) delete (int*)g_slot.ctx;
  g_slot.text = strdup(text ? text : "");
  g_slot.ann = (annotation && annotation[0]) ? strdup(annotation) : nullptr;
  g_slot.use_image = use_image;
  g_slot.one_shot = one_shot;
  g_slot.id = id;
  g_slot.generation = m_generation;
  g_slot.fn = reply;
  g_slot.ctx = reply_ctx;   // WS: 堆 int*；BLE: nullptr
  g_slot.active = true;
  xSemaphoreGive(g_mtx);
  // 尝试掐断在途请求，让 worker 尽快回到循环取新槽
  g_client.stop();
  g_stop_mode = (int)StopMode::All;   // 新目标打断旧任务：残留持续指令在旧任务出口补停
  xSemaphoreGive(g_notify);
}

void ai::cancel(StopMode m) {
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  bool was_active = g_slot.active;  // 仅当确有任务在跑才上报，避免手动指令刷屏
  ++m_generation;         // 使在途结果作废
  if (g_slot.text) { free(g_slot.text); g_slot.text = nullptr; }
  if (g_slot.ann) { free(g_slot.ann); g_slot.ann = nullptr; }
  if (g_slot.ctx) { delete (int*)g_slot.ctx; g_slot.ctx = nullptr; }
  g_slot.active = false;
  xSemaphoreGive(g_mtx);
  g_client.stop();
  g_stop_mode = (int)m;   // 手动 move/stop 接管=None（不补停）；arm=Wheels；ai_cancel=All
  if (was_active) Serial.println("[ai] cancel");
}

bool ai::busy() { return m_busy; }