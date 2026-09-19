#include "src/ai/ai_client.h"
#include "src/ai/ai_prompt.h"   // PsaBuf + build_body(请求体组装)
#include "src/ai/ai_mem.h"     // 空间记忆/车姿态: mem_reset/mem_feed/mem_find/...
#include "src/ai/ai_http.h"    // TLS 发送层: http_post/http_last_status/http_stop
#include "src/net/config.h"
#include "src/cam/camera.h"
#include "src/exec/direct_exec.h"
#include "src/net/wifi_net.h"
#include "src/ai/ground_proj.h"
#include "src/core/board_log.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>   // 统一处理 TLS/content-length/chunked, 替代手写 http_exchange
#include <esp_timer.h>
#include <stdlib.h>  // malloc/free/strtol
#include <math.h>    // fabsf
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <ArduinoJson.h>
#include <esp_heap_caps.h>  // MALLOC_CAP_SPIRAM
#include <mbedtls/platform.h>  // mbedtls_platform_set_calloc_free(TLS 内存搬到 PSRAM)
#include <string.h>         // memset/strncpy(空间记忆表)
#include <stdarg.h>         // vsnprintf(ai::logf)

// 决策频率 / 步数上限
#define AI_INTERVAL_MS 1500
#define AI_MAX_STEPS_PER_GOAL 120
#define AI_EDITED_IMG_MAX (128 * 1024)
#define AI_EDITED_IMG_TTL_MS 60000
#define AI_WAIT_FB_MIN_MS 10000   // wait 反馈节流: 同一动作少于此间隔只回一条
#define AI_MOVE_CAP_MS 2000       // AI 持续 move 单次行驶时限(无里程计兜底, 防决策间隔内盲走撞墙)
#define AI_MAX_NET_FAIL 4         // 连续"无有效输出"轮数上限: 超过即中止任务并回报(防云端持续无响应时无限空转)
#define AI_HIST_N 8               // 历史环条数(AI 决策 + 插话共用, 满员淘汰最旧)

// 本地巡航(approach / /move to)参数: 定距段长、初对齐阈值、单次转向上限等。
#define AI_APPROACH_STOP_CM 15   // AI 自动靠近的到位距离(之后交给 AI 微操)
#define NAV_STOP_CM_GOTO 6       // /move to 的到位容差 cm(无里程计开环, 留一点松量)
#define NAV_ALIGN_DEG 25         // 目标偏角超过此值先原地转向对齐, 否则直行推进
#define NAV_MAX_SPIN_DEG 60      // 单次原地转向的角度上限(分步收敛)
#define NAV_MAX_SEG_CM 25        // 单次定距推进上限 cm(分步收敛、防冲)
#define NAV_MAX_ITERS 200        // 巡航最大迭代步数(防死循环)
#define NAV_WAIT_LOOPS 120       // 每段等待轮子停下的检测循环数(约 50ms 一拍)

// ArduinoJson 内存池改用 PSRAM, 避免其小分配每轮在内部堆上反复申请/释放, 
// 与 TLS 缓冲交错把内部堆切成碎块(导致握手 -17040/-32512 失败)。
struct PsramAllocator : public ArduinoJson::Allocator {
  void* allocate(size_t n) override { return heap_caps_malloc(n, MALLOC_CAP_SPIRAM); }
  void deallocate(void* p) override { if (p) heap_caps_free(p); }
  void* reallocate(void* p, size_t n) override { return heap_caps_realloc(p, n, MALLOC_CAP_SPIRAM); }
};
static PsramAllocator g_js_alloc;

// ---------------- worker / 槽 / 队列 ----------------

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

// 结果队列项: 每项独立持有消息文本与恢复用的(fn, 本次堆拷贝 ctx)。
struct ResultItem {
  char* text = nullptr;
  cmd::ReplyFn fn = nullptr;
  void* ctx = nullptr;    // 每次入队新建的 int*(WS); BLE nullptr
};

static TaskLocal g_slot;
static SemaphoreHandle_t g_mtx = nullptr;   // 保护 g_slot / m_generation
static SemaphoreHandle_t g_notify = nullptr; // 唤醒 worker 的二进制信号量
static QueueHandle_t g_result_q = nullptr;
static volatile unsigned long m_generation = 0;  // 代际号: 每次 set_goal/cancel 自增
static volatile bool m_busy = false;
static TaskHandle_t g_worker = nullptr;

// 编辑图暂存(PSRAM)+ 时间戳; worker 内只读快照由 set_edited_image/取图互斥。
static uint8_t* g_edited = nullptr;
static size_t g_edited_len = 0;
static uint64_t g_edited_ts = 0;
static SemaphoreHandle_t g_img_mtx = nullptr;

// AI 任务进行中"插话"缓冲(ai_chat 写入、worker 每轮消费一次, 受 g_mtx 保护)。
// 一次性消费: 喂进下一轮 prompt 后清空, 避免重复塞给 AI。
static char g_chat[256] = {0};
static bool g_chat_has = false;

// 角度归一化到 (-180,180](方向计算统一口径)。
static float wrap180f(float a) {
  a = fmodf(a, 360.0f);
  if (a > 180.0f) a -= 360.0f;
  else if (a < -180.0f) a += 360.0f;
  return a;
}

// 最近一条下发执行板的是否持续型(sink: stop 兜底判定)。任务起点复位。
static volatile bool g_last_continuous = false;

// 最近一条持续指令的类型(0=无/1=move/2=arm)。兜底 stop 时 Wheels 模式只适用于轮子残留。
static volatile int g_last_cont_type = 0;

// 本轮任务终结时的兜底 stop 模式, 由打断方写入; worker 出口统一解析一次。
static volatile int g_stop_mode = (int)ai::StopMode::All;
static volatile uint64_t g_last_wait_fb_ms = 0;  // wait 反馈节流时间戳

static void enqueue_result(const char* text, cmd::ReplyFn fn, void* ctx);

// ---------------- 结果队列(worker → loop) ----------------

// 就地把一条 WS 文本转为合法 UTF-8(RFC6455 文本帧必须为 UTF-8)。
// 云端 AI 响应/日志偶发混入残缺 UTF-8 序列或非法字节, 若原样塞进 WS TEXT 帧, 
// 手机 Godot 会以关闭码 1007(Invalid frame payload data)断链。此时整条链路
// 的中文保持不变, 仅把控制字符与非法/残缺序列替换为 '?'(1:1, 不扩容)。
static void sanitize_ws_utf8(char* s) {
  char* w = s;
  const unsigned char* p = (const unsigned char*)s;
  while (*p) {
    unsigned char c = *p;
    int need = 0;
    if (c < 0x20 || c == 0x7f) { *w++ = '?'; p++; continue; }   // 控制字符
    if (c < 0x80) { *w++ = (char)c; p++; continue; }             // 合法 ASCII
    if (c >= 0xC2 && c <= 0xDF) need = 1;
    else if (c >= 0xE0 && c <= 0xEF) need = 2;
    else if (c >= 0xF0 && c <= 0xF4) need = 3;                   // 其余字节非法
    bool ok = need > 0;
    for (int i = 1; ok && i <= need; i++) {
      unsigned char cc = p[i];
      if (!cc || !(cc >= 0x80 && cc <= 0xBF)) ok = false;        // continuation 缺失/越界(含被截断的串尾)
    }
    if (ok) { for (int i = 0; i <= need; i++) *w++ = (char)p[i]; p += need + 1; }
    else { *w++ = '?'; p += 1; }                                 // 非法首字节/残缺序列: 单字节替换
  }
  *w = 0;
}

void enqueue_result(const char* text, cmd::ReplyFn fn, void* ctx) {
  ResultItem* it = (ResultItem*)malloc(sizeof(ResultItem));
  if (!it) return;
  size_t n = strlen(text);
  it->text = (char*)malloc(n + 1);
  if (!it->text) { free(it); return; }
  memcpy(it->text, text, n + 1);
  sanitize_ws_utf8(it->text);   // 统一 WS 文本消毒: 任何 enqueue 出口都走这里, 防 1007 断链
  // WS: 为本次结果单独堆拷贝 fd(loop 发送后释放); BLE ctx 已为 nullptr。
  it->fn = fn;
  it->ctx = ctx ? new int(*(int*)ctx) : nullptr;
  if (xQueueSend(g_result_q, &it, 0) != pdTRUE) { free(it->text); free(it->ctx); free(it); }
}

// AI 调试日志: 经 board_log(blog::AI)统一输出; /log ai(或 all)时转发手机。
void ai::logf(const char* fmt, ...) {
  char buf[256];   // 栈缓冲保持小，防小栈任务(WS/httpd/BLE)里大局部压栈溢出崩溃
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  buf[sizeof(buf) - 1] = 0;  // 截断防越界
  va_end(ap);
  // 统一走板端日志模块: 始终写串口(带 [ai] 前缀); /log ai on 时经日志队列转发手机。
  // 消息内统一以 "[ai] " 开头, 与 blog::logf 的类别前缀重合 → 剥掉一段防显示成 "[ai] [ai]"。
  const char* p = buf;
  if (strncmp(p, "[ai] ", 5) == 0) p += 5;
  blog::logf(blog::AI, "%s", p);
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
  blog::logf(blog::AI, "收到编辑图 %u B", (unsigned)len);
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

// 裁剪字符串尾部的残缺 UTF-8 序列: 固定缓冲截断常切在汉字中间, 残留半个字节会让云端判
// "invalid unicode code point" 400。原地修改; 合法 UTF-8 输入不受影响。
static void utf8_clamp_tail(char* buf) {
  size_t n = strlen(buf), e = n, ncont = 0;
  while (e > 0) {
    unsigned char ch = (unsigned char)buf[e - 1];
    if ((ch & 0xC0) == 0x80) { e--; ncont++; continue; }        // 续字节: 继续回退
    int need;
    if (ch < 0x80) break;                                       // ASCII 结尾: 完整
    else if ((ch & 0xE0) == 0xC0) need = 1;
    else if ((ch & 0xF0) == 0xE0) need = 2;
    else if ((ch & 0xF8) == 0xF0) need = 3;
    else break;                                                 // 非法引导字节: 不动
    if (ncont < need) e--;                                      // 续字节不足: 连同引导字节一起删
    break;
  }
  if (e != n) buf[e] = 0;
}

// 等待本轮定距/定角到段自停(loop 的 update_tick 会按时长停轮); 中断或超时即退出。
static void wait_wheels(unsigned long gen) {
  for (int i = 0; i < NAV_WAIT_LOOPS; i++) {
    if (gen != m_generation) return;
    if (!exec::wheels_moving()) return;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// 本地巡航核心(纯本地, 不调云端): 把车从当前位姿导航到全局目标 (tx,ty), 距目标 ≤stop_cm 停。
// 无里程计: 按定距/定角时长近似 + 姿态累积做开环死航; 到位后剩余误差交给 AI 视觉微操兜底。
// 返回 Reached(到位)/ Interrupted(被新任务/手动打断)/ TimedOut(迭代超限收敛)。
enum class NavR : uint8_t { Reached, Interrupted, TimedOut };
static NavR navigate_to(unsigned long gen, float tx, float ty, float stop_cm) {
  const float throttle = 0.5f;   // 巡航推进油门(中速)
  for (int it = 0; it < NAV_MAX_ITERS; it++) {
    if (gen != m_generation) return NavR::Interrupted;
    float dx = tx - ai::s_car_x, dy = ty - ai::s_car_y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist <= stop_cm) { JsonDocument s(&g_js_alloc); s["scope"] = "all"; exec::act("stop", s.as<JsonObjectConst>()); return NavR::Reached; }
    float thg = atan2f(dx, dy) * 180.0f / ai::AI_PI;        // 目标全局方位(heading=0 时朝 Y+)
    float rel = wrap180f(ai::s_car_heading + thg);          // 目标相对车头偏角, 正=右
    if (fabsf(rel) > NAV_ALIGN_DEG) {                   // 偏太多先原地转向对齐
      int ang = (int)fminf(fabsf(rel), (float)NAV_MAX_SPIN_DEG);
      JsonDocument p(&g_js_alloc); p["dir"] = rel > 0 ? 1 : -1; p["speed"] = 850; p["angle_deg"] = ang;
      exec::act("spin", p.as<JsonObjectConst>()); ai::car_update_pose("spin", p.as<JsonObjectConst>());
      wait_wheels(gen);
    } else {                                            // 否则定距直行一段(分步收敛)
      float seg = fminf(dist - stop_cm, (float)NAV_MAX_SEG_CM);
      if (seg < 1.0f) seg = 1.0f;
      JsonDocument p(&g_js_alloc); p["throttle"] = throttle; p["steering"] = 0; p["distance_cm"] = (int)seg;
      exec::act("move", p.as<JsonObjectConst>()); ai::car_update_pose("move", p.as<JsonObjectConst>());
      wait_wheels(gen);
    }
  }
  JsonDocument s(&g_js_alloc); s["scope"] = "all"; exec::act("stop", s.as<JsonObjectConst>());
  return NavR::TimedOut;
}

// (请求体组装 build_body 已拆分至 src/ai/ai_prompt.cpp)

// 指令短描述(供"上一步已下发"拼接与死循环判定; 含运动数值, 便于识别"相同指令")。
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
  } else if (!strcmp(type, "arm_pose")) {
    float px = p["x"] | 0.0f;
    float ph = p["h"] | 0.0f;
    snprintf(buf, cap, "arm_pose x=%.0f h=%.0f", px, ph);
  } else if (!strcmp(type, "spin")) {
    int dd = p["dir"] | 0;
    snprintf(buf, cap, "spin %d", dd);
  } else {
    snprintf(buf, cap, "stop");
  }
}

// ---------------- 输出校验(防误动作) ----------------
// 校验并规范化 AI 输出到 out{type,params,reason}。返回 nullptr 通过; 否则返回错误字符串。
static const char* validate_cmd(const char* content, JsonDocument& out, char* err_buf, size_t err_cap) {
  // 剥代码块/首尾空白(Arduino String 无 find/left, 用 indexOf/substring)
  String c = content;
  c.trim();
  if (c.startsWith("```")) {
    int e = c.indexOf('\n');
    if (e >= 0) c = c.substring(e + 1);
    c.trim();
    if (c.endsWith("```")) c = c.substring(0, c.length() - 3);
    c.trim();
  }

  JsonDocument doc(&g_js_alloc);  // PSRAM 池, 避免内部堆碎片
  if (deserializeJson(doc, c)) {
    snprintf(err_buf, err_cap, "AI 返回非 JSON: %s", c.substring(0, 80).c_str());
    return err_buf;
  }
  const char* type = doc["type"] | "";
  if (strcmp(type, "move") && strcmp(type, "stop") && strcmp(type, "arm") &&
      strcmp(type, "wait") && strcmp(type, "spin") && strcmp(type, "arm_pose") &&
      strcmp(type, "approach")) {
    return "AI 输出非法 type";
  }
  if (!doc["params"].is<JsonObject>() && strcmp(type, "stop") && strcmp(type, "wait") &&
      strcmp(type, "approach")) {
    return "AI 输出缺 params";
  }
  if (!strcmp(type, "arm")) {
    const char* act = doc["params"]["act"] | "";
    static const char* acts[] = {"lift_up","lift_down","reach_forward","reach_backward","clip","release","fold", nullptr};
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
    int sd = doc["params"]["dir"] | 0;
    int ss = doc["params"]["speed"] | 900;  // 原地旋转默认转速(实测 <700 拖不动, 须给足)
    int sa = doc["params"]["angle_deg"] | 0;
    p["dir"] = constrain(sd, -1, 1);     // 原地旋转方向: ±1/0
    p["speed"] = constrain(ss, 0, 1000); // 原地旋转单轮 pwm
    if (src["angle_deg"].is<int>() && sa) p["angle_deg"] = constrain(sa, 0, 500); // 定角微操
    const char* act = doc["params"]["act"] | "";
    if (act[0]) p["act"] = act;
    const char* scope = doc["params"]["scope"] | "all";
    p["scope"] = !strcmp(scope, "wheels") ? "wheels" : (!strcmp(scope, "arm") ? "arm" : "all");
    // arm_pose 指定位姿: x=轴前方 cm(可达 4..15), h=夹爪中心离地高度 cm。arm_pose 内部还有可达域检查。
    float px = doc["params"]["x"] | 0.0f;
    float ph = doc["params"]["h"] | 0.0f;
    if (src["x"].is<float>() || src["x"].is<int>()) p["x"] = constrain(px, 0.0f, 20.0f);
    if (src["h"].is<float>() || src["h"].is<int>()) p["h"] = constrain(ph, -2.0f, 25.0f);
    const char* tgt = doc["params"]["target"] | "";
    if (tgt[0]) p["target"] = tgt;
  }
  const char* reason = doc["reason"] | "";
  if (reason[0]) out["reason"] = reason;
  const char* tg = doc["task_goal"] | "";
  if (tg[0]) out["task_goal"] = tg;   // 选项: 把插话/新意图提升为当前任务目标(AI 显式标记)
  if (doc["done"].is<bool>() && doc["done"].as<bool>()) out["done"] = true;  // 任务完结标记
  // carry_prev:true = 下一轮希望同时收到本轮画面做对比(目标锁定/追踪、判断移动后目标方位)。
  // 本字段不进 params, 仅作 worker 决策是否带 prev 的信号。
  if (doc["carry_prev"].is<bool>() && doc["carry_prev"].as<bool>()) out["carry_prev"] = true;
  // observe:true = AI 的空间观测(name/rel_deg/dist_cm/visible), 透传给 worker 更新物体记忆表。
  if (doc["observe"].is<JsonObject>()) out["observe"] = doc["observe"].as<JsonObjectConst>();
  return nullptr;
}

// ---------------- HTTPS POST / chunked 解码已拆分至 src/ai/ai_http.cpp ----------------
// (http_post / read_chunked_body / ai_tls_calloc / ai_tls_free 及已废弃的手写 http_exchange
// 均移入独立 TLS 传输模块; worker 经 ai::http_post / ai::http_last_status / ai::http_stop 调用)

// 从响应提取 choices[0].message.content; 同时打印 reasoning_content(思考过程, 截断防刷屏)。
// broken: 解析失败时置 true(多半是传输层截断/复用残留, 调用方应弃用复用连接)
static bool extract_content(const String& resp, String& content, bool* broken = nullptr) {
  // 若响应是 SSE 文本(chunked-SSE 解码产物): 按行取 data: 负载拼成最终 JSON
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
  // 剥离开头泄漏的 chunked size 行(形如 "XX\r\n{" / "XX\n{", XX 为 1-8 位十六进制长度)。
  // 复用连接偶发把某次响应的 size 行错位拼进下一条响应体顶部, deserializeJson 直接失败; 
  // JSON 响应永远以 '{' 起始、永不始于 hex, 故只在"hex+换行+{" 时剥离, 绝不误伤正文。
  {
    auto ishex = [](char c){ return (c>='0'&&c<='9')||(c>='a'&&c<='f')||(c>='A'&&c<='F'); };
    for (int s = 0; s < 4; s++) {              // 最多剥 4 段(防多块错位叠加)
      const char* q = payload.c_str();
      int i = 0;
      while (i < 8 && ishex(q[i])) i++;        // 扫描十六进制前缀
      if (i == 0) break;                       // 开头非 hex, 非泄漏
      int j = i;
      if (q[j] == '\r') j++;
      if (q[j] == '\n') j++;
      if (q[j] != '{') break;                  // 后随非 '{', 按正文处理
      payload = payload.substring(j);          // 剥掉这段 size 行
    }
  }
  JsonDocument doc(&g_js_alloc);
  if (deserializeJson(doc, payload)) {
    if (broken) *broken = true;  // 解析失败即视为连接可疑: 截断/残留污染, 调用方弃用复用连接
    // 解析失败诊断: 打印完整 payload + 结构判据(判断是状体被拼断/chunked 泄漏/SSE 多段拼接)。
    auto sanitize = [](String s) {
      for (int i = 0; i < s.length(); i++) {
        unsigned char ch = (unsigned char)s[i];
        if (ch < 0x20 || ch == '\x7f') s[i] = '?';
      }
      return s;
    };
    // 结构判据: 是否含 "\ndata:"(SSE 多帧)、是否含多个 "{"id"..."}(多 JSON 拼接)、是否含 chunk 大小泄漏前缀。
    bool sse = payload.indexOf("\ndata:") >= 0 || payload.startsWith("data:");
    int multi_json = 0;
    for (int i = payload.indexOf("{\"id\""); i >= 0 && i < payload.length(); i = payload.indexOf("{\"id\"", i + 1)) multi_json++;
    String full = sanitize(payload);
    ai::logf("[ai] JSON失败 len=%d sse=%d 多json=%d 截断=%d 全文:%s",
             (int)payload.length(), sse ? 1 : 0, multi_json,
             (payload.length() > 0 && payload[payload.length() - 1] != '}' && payload[payload.length() - 1] != ']') ? 1 : 0,
             full.c_str());
    return false;
  }
  const char* rc = doc["choices"][0]["message"]["reasoning_content"] | "";
  if (rc[0]) {
    // 完整思考只打串口（流式，不申请缓冲/不占栈，也不转发手机）；调试/评估用，不回投模型。
    Serial.print("[ai] 思考: "); Serial.println(rc);
  }
  const char* c = doc["choices"][0]["message"]["content"] | "";
  // 注意: content 为空但 reasoning 有值 = 模型只给了思考没给正式回答(安全终止/条件触达)
  if (!c[0] && rc[0]) {
    Serial.print("[ai] content为空但reasoning有值("); Serial.print((unsigned)strlen(rc)); Serial.print("字节): "); Serial.println(rc);
  }
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
  // 内嵌词表指令(供 Godot 显示)
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
    if (!t.text && !t.nav) continue;

    // 纯导航任务(/move to): 不调云端, 直接本地巡航到目标坐标即回报。
    if (t.nav) {
      m_busy = true;
      if (!t.nav_global) { ai::s_car_x = 0; ai::s_car_y = 0; ai::s_car_heading = 0; }  // local=以当前位姿为新原点
      NavR r = navigate_to(t.generation, t.nav_x, t.nav_y, NAV_STOP_CM_GOTO);
      JsonDocument f(&g_js_alloc);
      f["type"] = "ai_result";
      if (t.id) f["id"] = (long)t.id;
      f["params"]["reason"] = r == NavR::Reached ? "已到达目标坐标" : "导航被中断";
      f["params"]["done"] = (r == NavR::Reached);
      String s; serializeJson(f, s);
      enqueue_result(s.c_str(), t.fn, t.ctx);
      blog::logf(blog::AI, "导航结束 rel=%d", (int)r);
      if (t.ctx) delete (int*)t.ctx;
      m_busy = false;
      continue;
    }

    g_last_continuous = false;  // 本任务尚未下发过持续指令(防上一任务残留标志误判)
    g_last_cont_type = 0;
    m_busy = true;
    // 新任务 = 新坐标系: 车位置=原点、初始车头=0°, 清空上一任务的空间记忆。
    ai::mem_reset();
    // 摄像头可用性在任务起点判定(init 后即定): 不可用则整轮走无画面降级
    bool cam_ok = cam::available();
    ai::logf("[ai] 任务开始 gen=%lu text=%s%s", t.generation, t.text, cam_ok ? "" : "(无摄像头→无画面模式)");
    // 单应状态诊断: 若未就绪, 所有 px/py 观测都会被拒, 直接可看出问题
    if (!ground::ready()) ai::logf("[ai] 警告: 单应未就绪(像素观测将全部被拒绝)");
    else {
      float ex, ey;
      ground::screen_to_world(0.5f, 0.5f, &ex, &ey);
      ai::logf("[ai] 单应OK 中心→(%.0f,%.0f)", ex, ey);
    }
    // 打印实际端点/模型, 便于排查 404/401 等云端拒绝(配错路径是常见原因)
    ai::logf("[ai] 端点=%s 模型=%s key=%s", cfg::ai_url().c_str(), cfg::ai_model().c_str(),
                  cfg::ai_key().isEmpty() ? "空" : "已配置");

    unsigned long steps = 0;
    bool done = false;
    bool sent_done = false;      // 是否已确报过终态 done(正常/单轮/超步数/超连续失败)
    bool interrupted = false;    // 是否因新目标/手动中断退出(代际变更), 此时不发补发 done
    const char* fail = nullptr;
    char err_buf[160];
    // 死循环防线状态: 上一步指令短描述 / 连续相同指令计数 / 已注入引导标记。
    char last_cmd[48] = {0}, cur_cmd[48] = {0};
    unsigned long last_act_ms = 0;  // 上次真正下执行/微操指令的时刻(ms), 供给 AI 算"距上次决策多久"
    int net_fail = 0;           // 连续"无有效输出"轮数(网络/解析失败), 用于退避与上限收尾
    int stall = 0;
    bool stall_hint = false;
    bool want_prev = false;     // AI 上轮 carry_prev=true → 本轮带上 prev 帧做对比
    char task_note[192] = {0};  // AI 写入的任务笔记(目标外观/计划), 每轮喂回
    struct { char name[48]; bool done; } s_tasks[8] = {{0}};  // AI 维护的任务列表(JSON 更新)
    int s_task_n = 0;
    // 当前任务目标(独立 user 消息展示; 可被 task_goal 热替换)
    char goal_now[256];
    snprintf(goal_now, sizeof(goal_now), "%s", t.text ? t.text : "");
    // 历史环(PSRAM 动态分配): AI 决策(assistant) + 插话(user) 共用一条管理, 满员淘汰最旧。
    char* hist_text[AI_HIST_N] = {nullptr};
    const char* hist_role[AI_HIST_N] = {nullptr};   // "assistant" / "user"
    int hist_n = 0;
    char task_remind[160] = {0};  // 插话即将被冲掉前的提醒(提示补 task_goal 更新目标), 一次性消费
    auto ps_dup = [](const char* s) -> char* {   // 拷贝到 PSRAM(供历史条用)
      size_t n = strlen(s);
      char* p = (char*)heap_caps_malloc(n + 1, MALLOC_CAP_SPIRAM);
      if (p) memcpy(p, s, n + 1);
      return p;
    };
    auto hist_add = [&](const char* role, const char* text) {   // 推入历史环(满则冲掉最旧)
      char* dup = ps_dup(text);
      if (!dup) return;
      if (hist_n < AI_HIST_N) { hist_role[hist_n] = role; hist_text[hist_n] = dup; hist_n++; }
      else {
        if (hist_role[0] && !strcmp(hist_role[0], "user"))   // 被冲掉的恰是插话: 提醒先 task_goal 更新目标
          snprintf(task_remind, sizeof(task_remind),
                   "较早的一条操作者插话即将被历史丢弃; 若它表达了新任务/新目标而你还未用 task_goal 更新, 请现在更新。");
        free(hist_text[0]);
        memmove(hist_text, hist_text + 1, (AI_HIST_N - 1) * sizeof(hist_text[0]));
        memmove(hist_role, hist_role + 1, (AI_HIST_N - 1) * sizeof(hist_role[0]));
        hist_text[AI_HIST_N - 1] = dup; hist_role[AI_HIST_N - 1] = role;
      }
    };

    // 编辑图一次性取快照(供整轮任务复用, 避免中途被覆盖)。
    uint8_t* edited = nullptr; size_t edited_len = 0;
    if (t.use_image) {
      edited = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
      if (edited && !take_edited(edited, AI_EDITED_IMG_MAX, &edited_len)) { free(edited); edited = nullptr; }
    }
    // 本轮帧的 PSRAM 副本(任务期复用): 抓帧后立即把字节拷进来并归还相机缓冲, 
    // 避免 AI 在做慢速 TLS 请求期间长时间占用 fb_count=2 的缓冲池把推流饿死。
    uint8_t* cur = nullptr; size_t cur_len = 0;
    // 上一帧 PSRAM 副本(周期性双帧运动对比用; 任务期复用)。
    uint8_t* prev = nullptr; size_t prev_len = 0;

    // 兜底 stop(任务终结出口统一解析一次)。g_stop_mode 由打断方写入: 
    // None=手动 move/stop 接管(用户指令已覆盖, 不补停); Wheels=手动 arm(只停轮子); All=其余。
    // 仅当存在持续指令残留才补。
    auto resolve_stop = [&]() {
      if (g_stop_mode == (int)ai::StopMode::None) return;
      if (!g_last_continuous) return;
      // Wheels 模式只对轮子持续残留停轮子; 臂持续残留(或未知)必须全停。
      const char* scope = (g_stop_mode == (int)ai::StopMode::Wheels && g_last_cont_type == 1) ? "wheels" : "all";
      JsonDocument d; d["scope"] = scope;   // d 即 stop 的 params 对象
      exec::act("stop", d.as<JsonObjectConst>());
      blog::logf(blog::AI, "兜底 stop scope=%s", scope);
      g_last_continuous = false;
    };

    while (!done) {
      uint64_t step_ts = esp_timer_get_time();  // 本轮起点(周期控制基准)
      fail = nullptr;  // 每轮重置, 避免沿用上轮错误文本误导日志/回报
      // 中止检查(代际号变化即本任务作废); 兜底停统一在任务出口解析。
      if (t.generation != m_generation) { blog::logf(blog::AI, "被新目标/手动中断"); interrupted = true; break; }
      if (!net::is_connected()) { snprintf(err_buf, sizeof(err_buf), "WiFi 掉线"); fail = err_buf; break; }
      if (cfg::ai_key().isEmpty()) { snprintf(err_buf, sizeof(err_buf), "未配置 AI Key"); fail = err_buf; break; }

      // 取当前帧(失败重试, 避免推流占缓冲时一次失败即判死); 无摄像头则跳过, 走无画面降级
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

      // 抓帧后立即拷入 PSRAM 副本并归还相机缓冲: AI 的 HTTPS(TLS 握手+请求)
      // 慢则数秒, 期间一直占着 fb 会把 fb_count=2 的缓冲池耗尽, 饿死并行推流。
      // 帧数据后续(body 组装)一律用这份副本。
      if (fb && frame_len > 0 && frame_len <= AI_EDITED_IMG_MAX) {
        if (!cur) cur = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
        if (cur) { memcpy(cur, frame, frame_len); cur_len = frame_len; }
        else cur_len = 0;
      }
      cam::return_frame(fb);
      // 副本就绪才认本帧(fb 已归还, 直接指向裸 fb->buf 即为悬垂): 过大或拷入失败则按无画面处理。
      if (cur_len > 0) { frame = cur; frame_len = cur_len; }
      else { frame = nullptr; frame_len = 0; }

      bool got = false;
      for (int attempt = 0; attempt < 2 && !done; attempt++) {
        // 引导语: 重试纠正 / 死循环打断(连续多轮相同指令, 带重复指令名便于 AI 自我纠正)
        char hint_buf[192];
        const char* hint;
        if (attempt) {
          hint = "上次输出非法, 请只输出合法 JSON 词表指令。";
        } else if (stall_hint) {
          snprintf(hint_buf, sizeof(hint_buf),
                   "连续多轮重复「%s」无进展: 先改变观察角度重新确认目标(抬/落机械臂、后退或环视), 若夹爪已夹紧或无法达成则输出 stop。", last_cmd);
          hint = hint_buf;
        } else {
          hint = "";
        }
        PsaBuf body;
        char st[192];  // exec 状态缓冲(含不可达诊断, 需足量避免截断)
        const char* stp = exec::read_state(st, sizeof(st)) ? st : "";  // 本地直驱状态(无执行板, 状态本地合成)
        // 每轮取一次插话(一次性消费, 读完清空): 推入历史环作 user 消息, 随环一起保留/冲掉。
        char chat_now[256] = {0};
        xSemaphoreTake(g_mtx, portMAX_DELAY);
        if (g_chat_has) { strncpy(chat_now, g_chat, sizeof(chat_now) - 1); utf8_clamp_tail(chat_now); g_chat_has = false; }
        xSemaphoreGive(g_mtx);
        if (chat_now[0]) { hist_add("user", chat_now); }   // 插话进入共享历史(操作者话语)
        // 上一帧是否带上: 仅由 AI 上轮 carry_prev=true 决定(锁定/追踪意图), 其余保持单帧省开销。
        bool use_prev = want_prev && prev_len > 0;
        // 操作者参考图: 仅首轮带一次(初始目标外观参考); 之后不续带(已去掉 carry_user)。
        bool use_edited_now = (steps == 0) && edited != nullptr;
        // 渲染任务列表喂回: 任务列表: 1.出门[完成] 2.右转[未完成] ...
        char task_s[320] = {0};
        if (s_task_n > 0) {
          int tp2 = snprintf(task_s, sizeof(task_s), "任务列表: ");
          for (int ti = 0; ti < s_task_n && tp2 < (int)sizeof(task_s) - 48; ti++)
            tp2 += snprintf(task_s + tp2, sizeof(task_s) - tp2, "%d.%s[%s] ",
                            ti + 1, s_tasks[ti].name, s_tasks[ti].done ? "完成" : "未完成");
        }
        // 合并本轮提示(在插话 hist_add 之后构建, 确保其触发的 task_remind 本轮可见): 
        // 一次性提醒(插话将冲掉→提示补 task_goal)在前, 死循环引导在后
        char hint_cb[384] = {0};
        if (task_remind[0]) { snprintf(hint_cb, sizeof(hint_cb), "%s", task_remind); task_remind[0] = 0; }
        if (hint && hint[0]) {
          size_t hl = strlen(hint_cb);
          if (hl) hint_cb[hl++] = '; ';
          snprintf(hint_cb + hl, sizeof(hint_cb) - hl, "%s", hint);
        }
        // 历史环逐条喂给 build_body(assistant=AI决策 / user=插话, 真多轮对话)
        build_body(body, goal_now, t.ann, hint_cb,
                   (const char* const*)hist_role, (const char* const*)hist_text, hist_n,
                   stp,
                   last_act_ms ? (unsigned)((esp_timer_get_time() / 1000 - last_act_ms) / 1000) : 0u,
                   task_note, task_s,
                   frame, frame_len, prev, use_prev ? prev_len : 0,
                   use_prev, use_edited_now, edited, edited_len);
        if (!body.ok) { fail = "组装请求 body 失败"; break; }

        String resp;
        bool http_ok = false;
        for (int nr = 0; nr < 3 && !http_ok; nr++) {   // 网络失败指数退避重试(任务串行, 代价可控)
          if (ai::http_post(cfg::ai_url().c_str(), cfg::ai_key().c_str(), body.p, resp, t.generation)) { http_ok = true; break; }
          if (t.generation != m_generation) { done = true; interrupted = true; break; }  // 被中止, 静默作废
          if (ai::http_last_status() >= 400 && ai::http_last_status() < 500) {  // 4xx 重发同 body 必然再拒, 快速失败
            fail = "云端拒绝(4xx), 疑似参数或限流";
            break;
          }
          if (nr < 2) { vTaskDelay(pdMS_TO_TICKS(500 << nr)); blog::logf(blog::AI, "网络失败重试 %d", nr + 1); }
        }
        if (done) break;
        if (!http_ok) { if (!fail) fail = "AI 请求失败"; break; }
        if (t.generation != m_generation) { done = true; interrupted = true; break; }  // 在途结果作废

        String content;
        bool body_broken = false;
        if (!extract_content(resp, content, &body_broken)) {
          // 解码成功但无有效内容(瞬态错误体/空 content 等): 打印原始片段便于定位
          blog::logf(blog::AI, "响应无内容, 原始(前120B): %s", resp.substring(0, 120).c_str());
          fail = "AI 响应无内容";
          if (body_broken) ai::http_stop();  // 传输层截断/残留: 弃用复用连接, 下次全新握手防污染
          continue;  // 空内容→重试, 不终止
        }

        JsonDocument cmdD(&g_js_alloc);  // PSRAM 池: 避免每轮在内部堆分配/释放制造碎片(freeHeap 泄漏嫌疑)
        const char* verr = validate_cmd(content.c_str(), cmdD, err_buf, sizeof(err_buf));
        if (!verr) {
          const char* type = cmdD["type"] | "";
          JsonObjectConst params = cmdD["params"].as<JsonObjectConst>();
          if (!strcmp(type, "approach")) {
            // 本地自动靠近: 不点云端, 直接按记忆目标巡航到近距, 交还 AI 继续微操。
            const char* tgt = params["target"] | "";
            float tx, ty;
            bool found = ai::mem_find(tgt[0] ? tgt : nullptr, &tx, &ty);
            const char* why = nullptr;
            bool arrived = false;
            if (!found) {
              why = tgt[0] ? "approach 目标不在记忆里, 请先 observe 锁定" : "approach 无可用目标记忆, 请先 observe";
            } else {
              NavR r = navigate_to(t.generation, tx, ty, AI_APPROACH_STOP_CM);
              if (r == NavR::Interrupted) { interrupted = true; done = true; break; }   // 被接管: 本任务作废
              arrived = (r == NavR::Reached);
              why = arrived ? "已自动靠近目标, 交还你微操" : "靠近收敛结束, 由你继续";
              last_act_ms = (unsigned long)(esp_timer_get_time() / 1000);
            }
            JsonDocument cmdF(&g_js_alloc);
            cmdF["type"] = "approach";
            cmdF["reason"] = why;
            cmdF["params"]["target"] = tgt[0] ? tgt : "(最近)";
            String fb = build_feedback(t.id, cmdF);
            enqueue_result(fb.c_str(), t.fn, t.ctx);
            stall = 0; stall_hint = false;    // 本地巡航视为有意推进, 不复位死循环判据
            got = true;
            net_fail = 0;
            break;
          }
          if (!strcmp(type, "wait")) {
            // wait=空操作: 不下发执行板, 保持当前动作, 任务继续观察。
            // 视为有意进展: 复位死循环计数, 避免"等待"被当成无进展注入引导。
            stall = 0; stall_hint = false;
            uint64_t now = (uint64_t)(esp_timer_get_time() / 1000);
            if (now - g_last_wait_fb_ms >= AI_WAIT_FB_MIN_MS) {  // 反馈节流, 防每轮刷屏
              g_last_wait_fb_ms = now;
              String fb = build_feedback(t.id, cmdD);
              enqueue_result(fb.c_str(), t.fn, t.ctx);
            }
            got = true;
            break;
          }
          // 校验通过 → 执行 move/arm/stop(本板直驱, 不再经执行板), 记录时刻供时间感知。
          exec::act(type, params);
          last_act_ms = (unsigned long)(esp_timer_get_time() / 1000);
          // 空间记忆: 先解析 AI observe(用**动作前**的车位姿把观测转全局), 再按 move/spin 更新车姿态。
          if (cmdD["observe"].is<JsonObject>()) {
            JsonObjectConst ob = cmdD["observe"].as<JsonObjectConst>();
            const char* nm = ob["name"] | "";
            bool vis = ob["visible"] | true;
            if (nm[0]) {
              // 优先用屏幕像素 px/py(单应解算, 精度高); 越界/解算失败或没报像素则回退 rel_deg/dist。
              bool has_px = ob["px"].is<float>() || ob["px"].is<int>();
              bool has_py = ob["py"].is<float>() || ob["py"].is<int>();
              if (has_px && has_py) {
                if (!ai::mem_observe_xy(nm, vis, ob["px"] | 0.0f, ob["py"] | 0.0f))
                  ai::mem_observe(nm, vis, ob["rel_deg"] | 0.0f, ob["dist_cm"] | 0.0f);
              } else {
                ai::mem_observe(nm, vis, ob["rel_deg"] | 0.0f, ob["dist_cm"] | 0.0f);
              }
            }
          }
          ai::car_update_pose(type, params);
          // 无里程计兜底: AI 无定距的持续 move 单次最多行驶 AI_MOVE_CAP_MS, 到期由 exec 自动停轮; 
          // 带 distance_cm 的定距 move 已在 send_move 按时长近似自停, 不叠加持续兜底。
          if (!strcmp(type, "move") && !params["distance_cm"].is<int>() &&
              fabsf(params["throttle"] | 0.0f) > 0.001f)
            exec::set_move_cap_ms(AI_MOVE_CAP_MS);
          g_last_continuous = exec::is_continuous(type, params);
          g_last_cont_type = g_last_continuous ? (!strcmp(type, "move") ? 1 : 2) : 0;
          // 用带参数的短描述日志(如 "arm_pose x=10 h=3" / "arm lift_up / move 6cm"), 供对动作看得清。
          fmt_last(cur_cmd, sizeof(cur_cmd), type, params);
          ai::logf("[ai] 执行 %s%s", cur_cmd, g_last_continuous ? "(持续)" : "");
          String fb = build_feedback(t.id, cmdD);
          enqueue_result(fb.c_str(), t.fn, t.ctx);
          // 死循环防线: 连续多轮下发相同指令 → 下轮注入引导语让 AI 主动变化
          if (strcmp(cur_cmd, last_cmd)) { stall = 0; stall_hint = false; }
          else if (++stall >= 3 && !stall_hint) { stall_hint = true; blog::logf(blog::AI, "多轮无进展, 注入引导"); }
          snprintf(last_cmd, sizeof(last_cmd), "%s", cur_cmd);
          // AI 显式要求保留上一帧(锁定/追踪): 下轮带上 prev; 否则按兜底节奏走
          want_prev = cmdD["carry_prev"].is<bool>() && cmdD["carry_prev"].as<bool>();
          // 任务笔记/进度: AI 写入并持续喂回(目标外观/计划/进行到哪)
          const char* tn = cmdD["task_note"] | "";
          if (tn[0] && strcmp(tn, task_note)) {
            strncpy(task_note, tn, sizeof(task_note) - 1); task_note[sizeof(task_note) - 1] = 0;
            ai::logf("[ai] 任务笔记: %s", task_note);
          }
          // 任务列表: 全量重写(新建/重组时用, 低频); AI 每轮只看到当前状态喂回。
          if (cmdD["tasks"].is<JsonArray>()) {
            JsonArrayConst ta = cmdD["tasks"].as<JsonArrayConst>();
            int n = 0;
            for (JsonObjectConst it : ta) {
              if (n >= 8) break;
              const char* nm = it["name"] | "";
              if (!nm[0]) continue;
              strncpy(s_tasks[n].name, nm, 47); s_tasks[n].name[47] = 0;
              s_tasks[n].done = it["done"] | false;
              n++;
            }
            if (n > 0 || s_task_n > 0) { s_task_n = n; ai::logf("[ai] 任务列表更新(%d项)", s_task_n); }
          }
          // 任务状态标记: 增量(第 N 项完成/未完成, index 从 1 起)
          if (cmdD["task_done"].is<JsonObject>()) {
            int idx = (cmdD["task_done"]["index"] | 0) - 1;
            if (idx >= 0 && idx < s_task_n) {
              s_tasks[idx].done = cmdD["task_done"]["done"] | false;
              ai::logf("[ai] 任务%d → %s", idx + 1, s_tasks[idx].done ? "完成" : "未完成");
            }
          }
          // 发给 AI 的"上一步已下发": 指令 + 完整 reason(含目标方位), 替代双帧供跨轮衔接。
          // 末尾整字符钳制(utf8_clamp_tail)防半个中文字节被云端判 400, 但不做长度截断。
          const char* r = cmdD["reason"] | "";
          {
            PsaBuf ld;   // 指令+完整 reason
            ld.put(cur_cmd); ld.put(", 原因: "); ld.put(r);
            utf8_clamp_tail(ld.p);
            // AI 决策(assistant) 直接滚入历史环(省去中转拷贝一次); ld.p 作用域末自动释放
            if (ld.len) hist_add("assistant", ld.p);
          }
          // AI 显式 task_goal: 把插话/新意图提升为目标(只替换目标 message, 不碰 system → 保缓存)
          const char* tg = cmdD["task_goal"] | "";
          if (tg[0] && strcmp(tg, goal_now)) {
            snprintf(goal_now, sizeof(goal_now), "%s", tg);
            utf8_clamp_tail(goal_now);
            ai::logf("[ai] 目标更新: %s", goal_now);
          }
          // stop + done:true = AI 自判任务完成: 停车并结束任务(build_feedback 已带 done 上报手机端); 
          // 不带 done 的 stop 仅临时停车观察, 任务继续下一轮。严格按 bool 判定, 防模型输出字符串 "true"。
          if (!strcmp(type, "stop") && cmdD["done"].is<bool>() && cmdD["done"].as<bool>()) {
            done = true;
            sent_done = true;   // 已向手机确报终态, 任务出口不再补发
            blog::logf(blog::AI, "AI 判定任务完成(stop+done)");
          }
          got = true;
          break;
        } else {
          blog::logf(blog::AI, "校验: %s", verr);
          fail = verr;  // 重试一次前暂存
        }
      }
      if (done) break;
      // 单轮模式(/ai oneshot): 一轮决策即收尾。本轮无有效输出(网络/解码/校验失败)
      // 时把原因作为 error 回报, 不跳 3s 继续观察; 持续指令残留由任务出口 resolve_stop 补停。
      if (t.one_shot) {
        if (!got && fail) {
          JsonDocument e(&g_js_alloc);
          e["error"] = fail;
          String s = build_feedback(t.id, e);
          enqueue_result(s.c_str(), t.fn, t.ctx);
        }
        // 单轮收尾: 补一条带 done 的完成通知, 让手机端复位「发送」并显示任务结束。
        JsonDocument e(&g_js_alloc);
        e["done"] = true;
        String s = build_feedback(t.id, e);
        enqueue_result(s.c_str(), t.fn, t.ctx);
        sent_done = true;   // 单轮已确报终态
        done = true;
        break;
      }
      if (!got) {
        // 网络/解析持续失败: 逐轮指数退避(3s→6s→12s→24s 封顶), 避免每轮狂建连接把云端打得更紧; 
        // 429 限流给更长喘息(30s)再继续; 失败原因同步推手机。
        if (++net_fail >= AI_MAX_NET_FAIL) {
          const char* reason = "云端持续无响应(疑似限流), 任务已中止, 请稍后重试";
          JsonDocument e(&g_js_alloc);
          e["error"] = reason;
          e["done"] = true;   // Bug1: 终结必带 done, 让手机端把「中止」复位为「发送」
          sent_done = true;
          String s = build_feedback(t.id, e);
          enqueue_result(s.c_str(), t.fn, t.ctx);
          blog::logf(blog::AI, "连续网络失败超限, 任务中止");
          break;
        }
        int wait = ai::http_last_status() == 429 ? 30000 : (3000 << (net_fail > 3 ? 3 : net_fail - 1));
        ai::logf("[ai] 本轮无有效输出(%s), %ds 后重试", fail ? fail : "未知", wait / 1000);
        vTaskDelay(pdMS_TO_TICKS(wait));
      } else {
        net_fail = 0;
      }
      ai::mem_tick_stale();   // 每轮结束: 未观测的物体过期轮数 +1

      // 滚动缓存上一帧: 仅当 AI 下一轮 carry_prev=true 时才发送作对比(锁定/追踪场景), 
      // 其余单帧以节省云端处理开销。AI 不要求即永不发送此帧。
      if (frame) {
        if (!prev) prev = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
        if (prev && frame_len <= AI_EDITED_IMG_MAX) { memcpy(prev, frame, frame_len); prev_len = frame_len; }
        else prev_len = 0;
      }

      if (++steps >= AI_MAX_STEPS_PER_GOAL) {
        JsonDocument e(&g_js_alloc); e["done"] = true; String s = build_feedback(t.id, e); enqueue_result(s.c_str(), t.fn, t.ctx);
        sent_done = true;
        break;
      }
      // 周期控制: 以 AI_INTERVAL_MS 为下限节奏, 扣掉本轮已耗时(含抓帧/HTTP/校验), 
      // 服务端快(单帧 ~2s)时立即进入下一轮, 慢时由服务端耗时主导。
      uint64_t el = esp_timer_get_time() - step_ts;
      long rem = (long)AI_INTERVAL_MS - (long)(el / 1000);
      if (rem > 0) vTaskDelay(pdMS_TO_TICKS(rem));
    }

    // 任务终结补发 done: 覆盖失败/掉线等"只报 error 不带 done"的终态(Bug1), 
    // 让手机端把「中止」复位为「发送」。被新目标/手动中断(interrupted)时跳过, 
    // 避免把下一任务的按钮状态误复位。正常 stop+done / 单轮 / 超步数已置 sent_done, 不再补发。
    if (!interrupted && !sent_done) {
      JsonDocument e(&g_js_alloc);
      e["done"] = true;
      if (fail) e["error"] = fail;
      String s = build_feedback(t.id, e);
      enqueue_result(s.c_str(), t.fn, t.ctx);
    }

    if (edited) free(edited);
    if (cur) free(cur);          // 本轮帧 PSRAM 副本
    if (prev) free(prev);        // 上一帧 PSRAM 副本
    for (int i = 0; i < AI_HIST_N; i++) free(hist_text[i]);   // 历史环 PSRAM
    if (t.ctx) delete (int*)t.ctx;  // 任务期 sink fd
    ai::logf("[ai] 任务结束 gen=%lu", t.generation);
    free(t.text); free(t.ann);
    resolve_stop();   // 统一兜底: 持续指令残留即补停
    m_busy = false;
  }
}

// ---------------- 对外 ----------------

void ai::init() {
  if (g_worker) return;
  blog::logf(blog::AI, "build=hwaes_int8192_v4  (TLS: 硬件AES/INTERNAL/8192; ws_stream/ping_svc 栈已调大)");
  ground::init();   // 屏幕→地面单应拟合 + 诊断日志(见 ground_proj)
  g_mtx = xSemaphoreCreateMutex();
  g_notify = xSemaphoreCreateBinary();
  g_img_mtx = xSemaphoreCreateMutex();
  g_result_q = xQueueCreate(8, sizeof(ResultItem*));
  xTaskCreatePinnedToCore(ai_worker, "ai_worker", 16384, nullptr, 2, &g_worker, 1);
  blog::logf(blog::AI, "worker 就绪");
}

void ai::set_goal(const char* text, bool use_image, const char* annotation, long id,
                  cmd::ReplyFn reply, void* reply_ctx, bool one_shot) {
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  ++m_generation;
  g_chat_has = false;   // 新目标清除上一任务的残留插话, 避免串任务
  // 替换旧槽(旧字符串为空因 worker 已取走; 残余则释放)
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
  g_slot.ctx = reply_ctx;   // WS: 堆 int*; BLE: nullptr
  g_slot.nav = false;       // 新 AI 目标覆盖可能的纯导航残留
  g_slot.active = true;
  xSemaphoreGive(g_mtx);
  // 尝试掐断在途请求, 让 worker 尽快回到循环取新槽
  ai::http_stop();
  g_stop_mode = (int)StopMode::All;   // 新目标打断旧任务: 残留持续指令在旧任务出口补停
  xSemaphoreGive(g_notify);
}

// /move to x y: 板端本地巡航到坐标(不调 AI)。local=以当前位姿为新原点; 
// global=沿用当前全局系(可与 AI/历史导航共用坐标系)。手动接管类: 打断在途任务。
void ai::goto_target(float x, float y, bool frame_global, long id, cmd::ReplyFn reply, void* reply_ctx) {
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  ++m_generation;               // 新导航接管: 在途 AI/导航作废
  g_chat_has = false;           // 清上一任务残留插话
  if (g_slot.text) free(g_slot.text);
  if (g_slot.ann) free(g_slot.ann);
  if (g_slot.ctx) delete (int*)g_slot.ctx;
  g_slot.nav = true;
  g_slot.nav_x = x; g_slot.nav_y = y;
  g_slot.nav_global = frame_global;
  g_slot.text = nullptr; g_slot.ann = nullptr;
  g_slot.use_image = false; g_slot.one_shot = false;
  g_slot.id = id;
  g_slot.generation = m_generation;
  g_slot.fn = reply;
  g_slot.ctx = reply_ctx;   // 直接接管 command.cpp 预建的堆 fd(对齐 set_goal), 由 nav 分支释放
  g_slot.active = true;
  xSemaphoreGive(g_mtx);
  ai::http_stop();              // 掐断在途 AI 请求, worker 尽快回到循环取导航槽
  g_stop_mode = (int)StopMode::All;
  xSemaphoreGive(g_notify);
}

void ai::cancel(StopMode m) {
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  bool was_active = g_slot.active;  // 仅当确有任务在跑才上报, 避免手动指令刷屏
  ++m_generation;         // 使在途结果作废
  if (g_slot.text) { free(g_slot.text); g_slot.text = nullptr; }
  if (g_slot.ann) { free(g_slot.ann); g_slot.ann = nullptr; }
  if (g_slot.ctx) { delete (int*)g_slot.ctx; g_slot.ctx = nullptr; }
  g_slot.nav = false;
  g_slot.active = false;
  xSemaphoreGive(g_mtx);
  ai::http_stop();
  g_stop_mode = (int)m;   // 手动 move/stop 接管=None(不补停); arm=Wheels; ai_cancel=All
  if (was_active) blog::logf(blog::AI, "cancel");
}

bool ai::busy() { return m_busy; }

bool ai::append_chat(const char* text) {
  if (!text || !text[0]) return false;
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  bool fed = m_busy;  // 仅当有任务在跑才接收插话
  if (fed) {
    strncpy(g_chat, text, sizeof(g_chat) - 1);
    g_chat[sizeof(g_chat) - 1] = 0;
    g_chat_has = true;
    blog::logf(blog::AI, "插话入队: %s", text);
  }
  xSemaphoreGive(g_mtx);
  return fed;
}