#include "ai_client.h"
#include "config.h"
#include "camera.h"
#include "uart.h"
#include "wifi_net.h"

#include <WiFiClientSecure.h>
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// 决策频率 / 步数上限（对慢速小车足够，节奏见架构文档）。
#define AI_INTERVAL_MS 2500
#define AI_MAX_STEPS_PER_GOAL 120
#define AI_EDITED_IMG_MAX (128 * 1024)
#define AI_EDITED_IMG_TTL_MS 60000
#define AI_CONNECT_TIMEOUT_MS 10000
#define AI_HTTP_TIMEOUT_MS 30000

// ---------------- worker / 槽 / 队列 ----------------

struct TaskLocal {
  char* text = nullptr;   // 目标文本（堆）
  char* ann = nullptr;    // 标注 JSON 或 {x,y,w,h,label}（堆）
  bool use_image = false;
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

// 构建请求 body。goal 目标文本；ann 标注 JSON 或空；corrective 为重试纠正语。
// 系统提示词 + 目标 + 标注先组进 PSRAM 缓冲，再整体 JSON 转义（内含引号）。
// 有 edited 时带双图（编辑图作意图锚点 + 当前帧作实时反馈），否则仅当前帧。
static void build_body(PsaBuf& b, const char* goal, const char* ann, const char* corrective,
                       const uint8_t* frame, size_t frame_len,
                       bool use_edited, const uint8_t* edited, size_t edited_len) {
  PsaBuf sys;
  sys.put("你是「小车+机械臂」的视觉控制大脑。摄像头画面来自车载相机；画面中可能有操作者的红色标注（方框/箭头/文字），必须优先遵循。");
  sys.put("当前任务目标：");
  sys.put(goal);
  if (ann && ann[0]) { sys.put("（操作者标注区域："); sys.put(ann); sys.put("）"); }
  sys.put("每次只能输出【一个】合法 JSON 指令，格式严格如下：");
  sys.put("{\"type\":\"move\",\"params\":{\"throttle\"..,\"steering\"..},\"reason\":\"..\"} 持续移动，直到收到 stop；");
  sys.put("或 {\"type\":\"move\",\"params\":{\"throttle\":0.5,\"steering\":0,\"distance_cm\":30},\"reason\":\"..\"} 指定距离移动；");
  sys.put("或 {\"type\":\"move\",\"params\":{\"throttle\":0,\"steering\":0.8,\"angle_deg\":90},\"reason\":\"..\"} 指定角度转向；");
  sys.put("或 {\"type\":\"arm\",\"params\":{\"act\":\"lift_up\"..},\"reason\":\"..\"} act.其中 lift_up/lift_down/reach_forward/reach_backward/clip/release；");
  sys.put("或 {\"type\":\"arm\",\"params\":{\"act\":\"reach_forward\",\"dist_cm\":15},\"reason\":\"..\"} 指定距离；");
  sys.put("或 {\"type\":\"stop\",\"params\":{\"scope\":\"all\"},\"reason\":\"..\"} 任务完成、无法继续或需立即停车时。");
  sys.put("规则：1.只输出 JSON，禁止多余文字/代码块标记。2.每次只规划一步。3.默认低速谨慎，优先用指定距离/角度版本。4.只能依据当前画面判断执行结果，若多轮无变化则输出 stop。5.reason 一句中文简要解释。");
  if (corrective && corrective[0]) { sys.put("注意："); sys.put(corrective); }

  b.put("{\"model\":");
  esc_append(b, cfg::ai_model().c_str());
  b.put(",\"messages\":[{\"role\":\"system\",\"content\":");
  esc_append(b, sys.p ? sys.p : "");

  b.put(",{\"role\":\"user\",\"content\":[{\"type\":\"text\",\"text\":\"请基于当前画面给出下一步动作。\"},");
  if (use_edited && edited) img_block(b, edited, edited_len);
  img_block(b, frame, frame_len);
  b.put("]}],\"max_tokens\":256,\"temperature\":0.2,\"response_format\":{\"type\":\"json_object\"}}");
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
  if (strcmp(type, "move") && strcmp(type, "stop") && strcmp(type, "arm")) {
    return "AI 输出非法 type";
  }
  if (!doc["params"].is<JsonObject>() && strcmp(type, "stop")) {
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
  return nullptr;
}

// ---------------- HTTPS POST ----------------
// 直接走 TLS socket 裸写 HTTP/1.1，绕开 HTTPClient 库——其 cookie/Date 解析
// 会引入 libc time/gmtime/mktime/strptime 等函数（被链接脚本强制放 IRAM，
// 实测导致 iram0 溢出）。DeepSeek 返回固定 JSON，Content-Length 定长可可靠读完。

static bool http_post(const char* url, const char* key, const char* body, String& resp) {
  // 解析 https://host[:port]/path
  const char* p = url + 8;  // 跳过 "https://"
  const char* slash = strchr(p, '/');
  String host_s = slash ? String(p, slash - p) : String(p);
  String path_s = slash ? String(slash) : String("/");
  int port = 443;
  int colon = host_s.indexOf(':');
  if (colon >= 0) { port = host_s.substring(colon + 1).toInt(); host_s = host_s.substring(0, colon); }

  g_client.stop();
  g_client.setInsecure();  // 开发期简单可靠；上线建议嵌入 CA
  g_client.setConnectionTimeout(AI_CONNECT_TIMEOUT_MS);
  g_client.setTimeout(AI_HTTP_TIMEOUT_MS);
  if (!g_client.connect(host_s.c_str(), port)) { Serial.println("[ai] TLS 连接失败"); return false; }

  g_client.printf("POST %s HTTP/1.1\r\n", path_s.c_str());
  g_client.printf("Host: %s\r\n", host_s.c_str());
  g_client.printf("Authorization: Bearer %s\r\n", key);
  g_client.print("Content-Type: application/json\r\n");
  g_client.printf("Content-Length: %d\r\n", (int)strlen(body));
  g_client.print("Connection: close\r\n\r\n");
  g_client.write((const uint8_t*)body, strlen(body));

  // 读响应头（上限 4KB）
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
    delay(5);
  }
  if (!hdr.endsWith("\r\n\r\n")) { g_client.stop(); return false; }
  if (!hdr.startsWith("HTTP/1.1 200") && !hdr.startsWith("HTTP/1.0 200")) {
    g_client.stop();
    Serial.printf("[ai] HTTP %s", hdr.c_str());
    return false;
  }

  // 按 Content-Length 读 body（上限 64KB 防呆）
  int cl = 0;
  int idx = hdr.indexOf("Content-Length:");
  if (idx >= 0) { String v = hdr.substring(idx + 15); v.trim(); cl = v.toInt(); }
  if (cl <= 0 || cl > 65536) { g_client.stop(); return false; }
  resp = "";
  resp.reserve(cl + 1);
  t0 = millis();
  while ((int)resp.length() < cl && millis() - t0 < AI_HTTP_TIMEOUT_MS) {
    while (g_client.available() && (int)resp.length() < cl) {
      int c = g_client.read();
      if (c < 0) break;
      resp += (char)c;
    }
    delay(5);
  }
  g_client.stop();
  return (int)resp.length() >= cl;
}

// 从响应提取 choices[0].message.content。
static bool extract_content(const String& resp, String& content) {
  JsonDocument doc;
  if (deserializeJson(doc, resp)) return false;
  const char* c = doc["choices"][0]["message"]["content"] | "";
  content = c;
  return content.length() > 0;
}

// ---------------- 结果文本构建 ----------------

static String build_feedback(unsigned long id, const JsonDocument& cmd) {
  JsonDocument out;
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
    m_busy = true;
    Serial.printf("[ai] 任务开始 gen=%lu text=%s\n", t.generation, t.text);

    unsigned long steps = 0;
    bool done = false;
    const char* fail = nullptr;
    char err_buf[160];

    // 编辑图一次性取快照（供整轮任务复用，避免中途被覆盖）。
    uint8_t* edited = nullptr; size_t edited_len = 0;
    if (t.use_image) {
      edited = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
      if (edited && !take_edited(edited, AI_EDITED_IMG_MAX, &edited_len)) { free(edited); edited = nullptr; }
    }

    while (!done) {
      // 中止检查（代际号变化即本任务作废）
      if (t.generation != m_generation) { Serial.println("[ai] 被新目标/手动中断"); break; }
      if (!net::is_connected()) { snprintf(err_buf, sizeof(err_buf), "WiFi 掉线"); fail = err_buf; break; }
      if (cfg::ai_key().isEmpty()) { snprintf(err_buf, sizeof(err_buf), "未配置 AI Key"); fail = err_buf; break; }

      // 取当前帧
      camera_fb_t* fb = cam::grab();
      if (!fb) { snprintf(err_buf, sizeof(err_buf), "取帧失败"); fail = err_buf; break; }
      const uint8_t* frame = fb->buf; size_t frame_len = fb->len;

      bool got = false;
      for (int attempt = 0; attempt < 2 && !done; attempt++) {
        const char* corrective = attempt ? "上次输出非法，请只输出合法 JSON 词表指令。" : "";
        PsaBuf body;
        build_body(body, t.text, t.ann, corrective,
                   frame, frame_len, edited != nullptr, edited, edited_len);
        if (!body.ok) { fail = "组装请求 body 失败"; break; }

        String resp;
        if (!http_post(cfg::ai_url().c_str(), cfg::ai_key().c_str(), body.p, resp)) {
          if (t.generation != m_generation) { done = true; break; }  // 被中止，静默作废
          fail = "AI 请求失败"; break;
        }
        if (t.generation != m_generation) { done = true; break; }  // 在途结果作废

        String content;
        if (!extract_content(resp, content)) { fail = "AI 响应无内容"; break; }

        JsonDocument cmdD;
        const char* verr = validate_cmd(content.c_str(), cmdD, err_buf, sizeof(err_buf));
        if (!verr) {
          // 校验通过 → 执行
          const char* type = cmdD["type"] | "";
          JsonObjectConst params = cmdD["params"].as<JsonObjectConst>();
          uart::act(type, params);
          Serial.printf("[ai] 执行 %s\n", type);
          String fb = build_feedback(t.id, cmdD);
          enqueue_result(fb.c_str(), t.fn, t.ctx);
          if (!strcmp(type, "stop")) { done = true; break; }
          got = true;
          break;
        } else {
          Serial.printf("[ai] 校验: %s\n", verr);
          fail = verr;  // 重试一次前暂存
        }
      }
      cam::return_frame(fb);

      if (done) break;
      if (!got) { JsonDocument e; e["error"] = fail; String s = build_feedback(t.id, e); enqueue_result(s.c_str(), t.fn, t.ctx); break; }

      if (++steps >= AI_MAX_STEPS_PER_GOAL) {
        JsonDocument e; e["done"] = true; String s = build_feedback(t.id, e); enqueue_result(s.c_str(), t.fn, t.ctx);
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(AI_INTERVAL_MS));
    }

    if (edited) free(edited);
    if (t.ctx) delete (int*)t.ctx;  // 任务期 sink fd
    free(t.text); free(t.ann);
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
                  cmd::ReplyFn reply, void* reply_ctx) {
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  ++m_generation;
  // 替换旧槽（旧字符串为空因 worker 已取走；残余则释放）
  if (g_slot.text) free(g_slot.text);
  if (g_slot.ann) free(g_slot.ann);
  if (g_slot.ctx) delete (int*)g_slot.ctx;
  g_slot.text = strdup(text ? text : "");
  g_slot.ann = (annotation && annotation[0]) ? strdup(annotation) : nullptr;
  g_slot.use_image = use_image;
  g_slot.id = id;
  g_slot.generation = m_generation;
  g_slot.fn = reply;
  g_slot.ctx = reply_ctx;   // WS: 堆 int*；BLE: nullptr
  g_slot.active = true;
  xSemaphoreGive(g_mtx);
  // 尝试掐断在途请求，让 worker 尽快回到循环取新槽
  g_client.stop();
  xSemaphoreGive(g_notify);
}

void ai::cancel() {
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  ++m_generation;         // 使在途结果作废
  if (g_slot.text) { free(g_slot.text); g_slot.text = nullptr; }
  if (g_slot.ann) { free(g_slot.ann); g_slot.ann = nullptr; }
  if (g_slot.ctx) { delete (int*)g_slot.ctx; g_slot.ctx = nullptr; }
  g_slot.active = false;
  xSemaphoreGive(g_mtx);
  g_client.stop();
  Serial.println("[ai] cancel");
}

bool ai::busy() { return m_busy; }