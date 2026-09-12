#include "ai_client.h"
#include "config.h"
#include "camera.h"
#include "direct_exec.h"
#include "wifi_net.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>   // 统一处理 TLS/content-length/chunked，替代手写 http_exchange
#include <esp_timer.h>
#include <stdlib.h>  // malloc/free/strtol
#include <math.h>    // fabsf
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <ArduinoJson.h>
#include <esp_heap_caps.h>  // MALLOC_CAP_SPIRAM
#include <mbedtls/platform.h>  // mbedtls_platform_set_calloc_free（TLS 内存搬到 PSRAM）
#include <string.h>         // memset/strncpy（空间记忆表）
#include <stdarg.h>         // vsnprintf（ai::logf）

// 决策频率 / 步数上限
#define AI_INTERVAL_MS 1500
#define AI_MAX_STEPS_PER_GOAL 120
#define AI_EDITED_IMG_MAX (128 * 1024)
#define AI_EDITED_IMG_TTL_MS 60000
#define AI_CONNECT_TIMEOUT_MS 10000
#define AI_HTTP_TIMEOUT_MS 30000
#define AI_WAIT_FB_MIN_MS 10000   // wait 反馈节流：同一动作少于此间隔只回一条
#define AI_MOVE_CAP_MS 2000       // AI 持续 move 单次行驶时限（无里程计兜底，防决策间隔内盲走撞墙）
#define AI_MAX_NET_FAIL 4         // 连续"无有效输出"轮数上限：超过即中止任务并回报（防云端持续无响应时无限空转）

// ================== 画面标定（摄像头左前高位、俯视；改这些数即可应对镜头被碰歪） ==================
// 以小车为原点：+y=车前，−y=车后，+x=车右，−x=车左。单位 cm，均为实测近似值，AI 只当粗参考。
// 竖轴（屏幕纵带中点）对应前向距离：底贴爪→中→中上→顶
#define CAL_FWD_BOTTOM_CM  0      // 屏幕最底横带   ≈ 贴近夹爪正下方
#define CAL_FWD_MID_CM     46     // 屏幕中心横带   ≈ 车前 46cm（正前，车轴线）
#define CAL_FWD_MIDTOP_CM  120    // 中心到顶部中间 ≈ 车前 120cm
#define CAL_FWD_TOP_CM     200    // 屏幕顶部横带   ≈ 车前 200cm（顶部已超出可测，估计值）
// 中横带左右两个外侧中点对应横向偏移（本机左前安装故左右不对称：右侧视野更宽）
#define CAL_SIDE_LEFT_CM   -45    // 屏幕左侧 1/4   ≈ 车左 45cm（(0.25,0.5) 实测）
#define CAL_SIDE_RIGHT_CM  14     // 屏幕右侧 3/4   ≈ 车右 14cm（(0.75,0.5) 实测）

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

// 最近一次响应的 HTTP 状态码（HTTPClient 写入，供上层 4xx 快速失败判定；0=未知）。
static int g_last_status = 0;

// ============ 全局坐标系 + 物体记忆（空间记忆） ============
// 车自身位姿：全局坐标 (x,y) cm + 车向角 heading。坐标系：任务开始车位置=原点，
// 初始车头=Y 正方向=0°，逆时针为正（x 向右）；右转(顺时针) heading 减小。
// 无里程计，位姿靠 move 定距 / spin 定角标定近似累积；AI 每次视觉观测会刷新物体
// 坐标，故里程漂移只在两次观测之间短暂存在，不累积误导。
static float s_car_x = 0, s_car_y = 0;
static int16_t s_car_heading = 0;

// 物体记忆表（通用：AI 觉得值得记的都记）。程序存全局坐标，喂给 AI 时一律换算成
// "当前车头局部系"（相对车头角度+距离），AI 零换算。stale=每轮未观测+1（过期不清空）。
// 每条保留最近 AI_OBS_N 次车头系观测，取中位数融合——DeepSeek 单次报的像素/角度方差大
// （轮间跳变、reason 与 observe 字段不一致），中位数对单次离谱值鲁棒，防记忆被污染。
#define AI_MEM_MAX 8
#define AI_OBS_N 5
static struct {
  char name[16];
  float gx, gy;        // 全局坐标 cm（由观测中位数合成）
  uint32_t t_ms;       // 最近观测时刻
  int16_t stale;       // 0=新鲜；每轮未观测 +1；>20 时不再喂回
  bool valid;
  float hx[AI_OBS_N], hy[AI_OBS_N];  // 车头系观测环形缓冲（记录时各自的车头角在 hh）
  int16_t hh[AI_OBS_N];              // 每条观测对应的车头角（融合时转全局用）
  uint8_t hn, hi;                    // 已存数量 / 写指针
} g_mem[AI_MEM_MAX];
static const float AI_PI = 3.14159265358979f;

// 取数组前 n 个元素的中位数（n≤AI_OBS_N，插入排序后取中间，仅用于观测融合）
static float median_n(const float* a, int n) {
  float t[AI_OBS_N];
  memcpy(t, a, n * sizeof(float));
  for (int i = 1; i < n; i++) {
    float v = t[i]; int j = i - 1;
    while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
    t[j + 1] = v;
  }
  return t[n / 2];
}

// 最近一条下发执行板的是否持续型（sink：stop 兜底判定）。任务起点复位。
static volatile bool g_last_continuous = false;

// 最近一条持续指令的类型（0=无/1=move/2=arm）。兜底 stop 时 Wheels 模式只适用于轮子残留。
static volatile int g_last_cont_type = 0;

// 本轮任务终结时的兜底 stop 模式，由打断方写入；worker 出口统一解析一次。
static volatile int g_stop_mode = (int)ai::StopMode::All;
static volatile uint64_t g_last_wait_fb_ms = 0;  // wait 反馈节流时间戳

// ai_log 回推 sink：worker 任务起点捕获当前回传通道（t.fn/t.ctx），供 logf 复用
// （enqueue_result 会堆拷贝 ctx，跨任务排空安全）。
static cmd::ReplyFn g_log_fn = nullptr;
static void* g_log_ctx = nullptr;

static void enqueue_result(const char* text, cmd::ReplyFn fn, void* ctx);

// ---------------- 结果队列（worker → loop） ----------------

// 就地把一条 WS 文本转为合法 UTF-8（RFC6455 文本帧必须为 UTF-8）。
// 云端 AI 响应/日志偶发混入残缺 UTF-8 序列或非法字节，若原样塞进 WS TEXT 帧，
// 手机 Godot 会以关闭码 1007（Invalid frame payload data）断链。此时整条链路
// 的中文保持不变，仅把控制字符与非法/残缺序列替换为 '?'（1:1，不扩容）。
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
      if (!cc || !(cc >= 0x80 && cc <= 0xBF)) ok = false;        // continuation 缺失/越界（含被截断的串尾）
    }
    if (ok) { for (int i = 0; i <= need; i++) *w++ = (char)p[i]; p += need + 1; }
    else { *w++ = '?'; p += 1; }                                 // 非法首字节/残缺序列：单字节替换
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
  sanitize_ws_utf8(it->text);   // 统一 WS 文本消毒：任何 enqueue 出口都走这里，防 1007 断链
  // WS：为本次结果单独堆拷贝 fd（loop 发送后释放）；BLE ctx 已为 nullptr。
  it->fn = fn;
  it->ctx = ctx ? new int(*(int*)ctx) : nullptr;
  if (xQueueSend(g_result_q, &it, 0) != pdTRUE) { free(it->text); free(it->ctx); free(it); }
}

// AI 调试日志：始终写串口；ai_log 开关开启且任务通道在位时，同时推手机当前行。
void ai::logf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  buf[sizeof(buf) - 1] = 0;  // 截断防越界
  va_end(ap);
  Serial.printf("%s\n", buf);
  if (!cmd::ai_log() || !g_log_fn) return;
  JsonDocument d(&g_js_alloc);
  d["type"] = "ai_log";
  d["params"]["text"] = buf;
  String s;
  serializeJson(d, s);
  enqueue_result(s.c_str(), g_log_fn, g_log_ctx);
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

// JSON 字符串转义：引号/反斜杠 + 控制字符（\n \r \t 等）。模型 reason 可能含换行，裸发会让云端
// 400 "invalid unicode/character"；中文等多字节字节序原样透传（UTF-8 合法）。
static void esc_append(PsaBuf& b, const char* s) {
  b.put('"');
  for (const char* c = s; *c; c++) {
    unsigned char ch = (unsigned char)*c;
    if (ch == '"' || ch == '\\') { b.put('\\'); b.put((char)ch); }
    else if (ch == '\n') { b.put('\\'); b.put('n'); }
    else if (ch == '\r') { b.put('\\'); b.put('r'); }
    else if (ch == '\t') { b.put('\\'); b.put('t'); }
    else if (ch < 0x20) { char h[7]; snprintf(h, sizeof(h), "\\u%04X", ch); b.put(h); }
    else b.put((char)ch);
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

// ============ 屏幕像素 → 地面坐标（单应矩阵） ============
// 相机固定俯视车前地面，无畸变广角，屏幕归一化 (u,v) → 车头系地面 (x右+, y前+) 为
// 三维单应变换（8 自由度），由实测点最小二乘拟合，天然吸收相机安装偏差。
// 求解用 Householder QR（板端）：正规方程法在 ESP32 单精度 FPU 下条件数平方放大会崩
// （曾算出 H=(480,-567)）；QR 不放大条件数，double 软件模拟也够。QR 失败/回验超差时
// 回退 PC 预计算的硬编码常量。标定点 CAL_N 可增（≥4），新增实测点直接往表里加即可。
// 实测校准点（屏幕归一化 0..1 → 车头系地面 cm）：
#define CAL_N 14
static const double CAL_UV[CAL_N][2] = {
  {0.5,0.25},{0.25,0.375},{0.25,0.5},{0.25,0.75},{0.25,1.0},{0.0,1.0},{0.0,0.75},
  {0.75,0.125},{0.5,1.0},{0.75,0.25},{0.75,0.5},{0.75,0.75},{1.0,0.5},{1.0,0.75}
};
static const double CAL_XY[CAL_N][2] = {
  {40,134},{-50,242},{-45,112},{-38,48},{-34,23},{-64,36},{-105,90},
  {78,85},{-15,14},{45,55},{16,26},{1.5,12},{23,13},{7,6}
};

// 当前生效的单应 + 归一化参数（QR 求解后为动态值；回退时为 fallback 常量）
static double H[8];
static double SU_MU, SU_S, SV_MU, SV_S, SX_MU, SX_S, SY_MU, SY_S;
static bool s_h_ok = false;

// PC 高精度预计算常量（QR 不可靠时的回退；由上述 CAL_* 经 DLT 归一化最小二乘得出）
static const double SU_MU_FB = 0.5,                 SU_S_FB = 0.2857142857142857;
static const double SV_MU_FB = 0.6071428571428571,  SV_S_FB = 0.25;
static const double SX_MU_FB = -10.035714285714286, SX_S_FB = 40.107142857142868;
static const double SY_MU_FB = 64.0,                SY_S_FB = 49.0;
static const double H_FALLBACK[8] = {
  0.60143113350335875, -0.24375293926870129, 0.085187899550564716,
  -0.75630851952496025, -0.76604382688550554, -0.6000131115682783,
  0.43120841755020961, 0.45749816959869866
};

// Householder QR 求解超定最小二乘 Ah≈b（m=2N 方程, n=8 未知）→ h 写回 H。
// A 按列主元反射逐步上三角化，b 同步施加反射，最后回代。返回 false=奇异/数值失败。
static bool qr_fit(const double A[2*CAL_N][8], const double b[2*CAL_N], double h[8]) {
  const int m = 2 * CAL_N, n = 8;
  double R[32][8];   // m×n 工作副本（CAL_N≤16 时 m≤32）
  double qb[32];
  for (int i = 0; i < m; i++) { for (int j = 0; j < n; j++) R[i][j] = A[i][j]; qb[i] = b[i]; }
  for (int k = 0; k < n; k++) {
    double n2 = 0;
    for (int i = k; i < m; i++) n2 += R[i][k] * R[i][k];
    if (n2 < 1e-30) return false;
    double alpha = (R[k][k] >= 0 ? -1 : 1) * sqrt(n2);
    // v = x - alpha·e1（存 R[k..][k]），归一化
    R[k][k] -= alpha;
    double vn = 0;
    for (int i = k; i < m; i++) vn += R[i][k] * R[i][k];
    vn = sqrt(vn);
    if (vn < 1e-15) { R[k][k] = alpha; continue; }   // 该列已正交，跳过
    for (int i = k; i < m; i++) R[i][k] /= vn;
    // 反射作用到右侧列与 b
    for (int j = k + 1; j < n; j++) {
      double dot = 0;
      for (int i = k; i < m; i++) dot += R[i][k] * R[i][j];
      for (int i = k; i < m; i++) R[i][j] -= 2 * dot * R[i][k];
    }
    double db = 0;
    for (int i = k; i < m; i++) db += R[i][k] * qb[i];
    for (int i = k; i < m; i++) qb[i] -= 2 * db * R[i][k];
    R[k][k] = alpha;   // 存 R 对角
  }
  // 回代解 R h = qb（R 上三角在 R[0..n-1][0..n-1]）
  for (int i = n - 1; i >= 0; i--) {
    double s = qb[i];
    for (int j = i + 1; j < n; j++) s -= R[i][j] * h[j];
    if (fabs(R[i][i]) < 1e-15) return false;
    h[i] = s / R[i][i];
  }
  return true;
}

// 前向声明：homography_fit 回验需要（定义在下方）
static bool screen_to_world(float u, float v, float* x, float* y);

// 启动：QR 求解归一化单应；回验校准点误差 ≤5cm 则采用，否则回退硬编码常量。
static bool homography_fit(void) {
  // 归一化参数（均值 + 平均绝对偏差）
  double um=0,vm=0,xm=0,ym=0;
  for (int i = 0; i < CAL_N; i++) { um+=CAL_UV[i][0]; vm+=CAL_UV[i][1]; xm+=CAL_XY[i][0]; ym+=CAL_XY[i][1]; }
  um/=CAL_N; vm/=CAL_N; xm/=CAL_N; ym/=CAL_N;
  double us=0,vs=0,xs=0,ys=0;
  for (int i = 0; i < CAL_N; i++) {
    us+=fabs(CAL_UV[i][0]-um); vs+=fabs(CAL_UV[i][1]-vm);
    xs+=fabs(CAL_XY[i][0]-xm); ys+=fabs(CAL_XY[i][1]-ym);
  }
  us=us/CAL_N; vs=vs/CAL_N; xs=xs/CAL_N; ys=ys/CAL_N;
  if (us<1e-9) us=1; if (vs<1e-9) vs=1; if (xs<1e-9) xs=1; if (ys<1e-9) ys=1;
  double A[32][8], b[32];
  for (int i = 0; i < CAL_N; i++) {
    double u = (CAL_UV[i][0]-um)/us, v = (CAL_UV[i][1]-vm)/vs;
    double x = (CAL_XY[i][0]-xm)/xs, y = (CAL_XY[i][1]-ym)/ys;
    A[2*i][0]=u; A[2*i][1]=v; A[2*i][2]=1; A[2*i][3]=0; A[2*i][4]=0; A[2*i][5]=0; A[2*i][6]=-u*x; A[2*i][7]=-v*x; b[2*i]=x;
    A[2*i+1][0]=0; A[2*i+1][1]=0; A[2*i+1][2]=0; A[2*i+1][3]=u; A[2*i+1][4]=v; A[2*i+1][5]=1; A[2*i+1][6]=-u*y; A[2*i+1][7]=-v*y; b[2*i+1]=y;
  }
  double h[8];
  bool ok = qr_fit(A, b, h);
  if (ok) {
    SU_MU=um; SU_S=us; SV_MU=vm; SV_S=vs; SX_MU=xm; SX_S=xs; SY_MU=ym; SY_S=ys;
    for (int i = 0; i < 8; i++) H[i] = h[i];
    s_h_ok = true;
    float maxerr = 0;
    for (int i = 0; i < CAL_N; i++) {
      float ex, ey;
      screen_to_world((float)CAL_UV[i][0], (float)CAL_UV[i][1], &ex, &ey);
      float e = sqrtf((ex-(float)CAL_XY[i][0])*(ex-(float)CAL_XY[i][0]) +
                      (ey-(float)CAL_XY[i][1])*(ey-(float)CAL_XY[i][1]));
      if (e > maxerr) maxerr = e;
    }
    if (maxerr <= 5.f) { Serial.printf("[ai] 单应QR求解成功 回验最大误差=%.1fcm\n", maxerr); return true; }
    Serial.printf("[ai] 单应QR回验超差(%.1fcm)，回退硬编码\n", maxerr);
  } else {
    Serial.println("[ai] 单应QR求解失败，回退硬编码");
  }
  // 回退：PC 预计算常量
  SU_MU=SU_MU_FB; SU_S=SU_S_FB; SV_MU=SV_MU_FB; SV_S=SV_S_FB;
  SX_MU=SX_MU_FB; SX_S=SX_S_FB; SY_MU=SY_MU_FB; SY_S=SY_S_FB;
  for (int i = 0; i < 8; i++) H[i] = H_FALLBACK[i];
  s_h_ok = true;
  return true;
}

// 屏幕归一化像素 (u,v) → 车头系地面 (x右+, y前+) cm。
// 单应只在标定区域内可信，区域外分母趋零会剧烈外推（AI 报错/越界像素时可达数千 cm）。
// 故限制：输入须在 0..1，输出须在可接受地面范围，否则返回 false（调用方回退 rel_deg）。
static bool screen_to_world(float u, float v, float* x, float* y) {
  if (!s_h_ok) return false;
  if (u < 0.f || u > 1.f || v < 0.f || v > 1.f) return false;
  double un = (u - SU_MU) / SU_S, vn = (v - SV_MU) / SV_S;   // 归一化
  double d = H[6] * un + H[7] * vn + 1.0;
  if (fabs(d) < 1e-9) return false;
  float xx = (float)(((H[0] * un + H[1] * vn + H[2]) / d) * SX_S + SX_MU);
  float yy = (float)(((H[3] * un + H[4] * vn + H[5]) / d) * SY_S + SY_MU);
  if (xx < -150.f || xx > 150.f || yy < -20.f || yy > 300.f) return false;  // 防外推爆炸
  *x = xx; *y = yy;
  return true;
}

// ============ 空间记忆辅助（全局位姿 + 物体记忆表） ============
// AI 每步执行 move/spin 后调用：按定距/定角近似累积车姿态。持续(无定距/定角)移动
// 位移未知，不改姿态（记忆会随每轮 stale++ 表不可靠）。
static void car_update_pose(const char* type, const JsonObjectConst& p) {
  if (!strcmp(type, "move")) {
    float th = p["throttle"] | 0.0f;
    int cm = p["distance_cm"] | 0;
    if (cm > 0 && fabsf(th) > 0.001f) {
      float d = cm * (th < 0 ? -1.0f : 1.0f);        // 后退反向
      float h = s_car_heading * AI_PI / 180.0f;
      s_car_x += sinf(h) * d;                        // heading=0 → 朝 Y+
      s_car_y += cosf(h) * d;
      ai::logf("[ai] 车位置(%.0f,%.0f) 车向%d°", s_car_x, s_car_y, (int)s_car_heading);
    }
  } else if (!strcmp(type, "spin")) {
    int dir = p["dir"] | 0;
    int ang = p["angle_deg"] | 0;
    if (dir != 0 && ang > 0) {
      int d = dir > 0 ? -ang : ang;                  // 右转=顺时针=heading 减小
      s_car_heading += d;
      if (s_car_heading > 180) s_car_heading -= 360;
      else if (s_car_heading < -180) s_car_heading += 360;
      ai::logf("[ai] 车向%d°", (int)s_car_heading);
    }
  }
}

// 把一次"车头系坐标"观测写入物体记忆表（同名刷新，满则覆盖最旧）。
// 观测历史按各自记录时的车头角转成全局坐标后取中位数——DeepSeek 单次观测方差大，
// 中位数对单次离谱值鲁棒；车头角变化也被正确补偿（各观测转同一全局系再融合）。
static void mem_store(const char* name, float wx, float wy) {
  int slot = -1, oldest = 0;
  for (int i = 0; i < AI_MEM_MAX; i++) {
    if (g_mem[i].valid && g_mem[i].name[0] && !strcmp(g_mem[i].name, name)) { slot = i; break; }
    if (!g_mem[i].valid) { slot = i; break; }
    if (g_mem[i].t_ms < g_mem[oldest].t_ms) oldest = i;
  }
  if (slot < 0) slot = oldest;                        // 满：覆盖最旧
  strncpy(g_mem[slot].name, name, 15); g_mem[slot].name[15] = 0;
  // 压入本次观测（记录当前车头角）
  g_mem[slot].hx[g_mem[slot].hi] = wx;
  g_mem[slot].hy[g_mem[slot].hi] = wy;
  g_mem[slot].hh[g_mem[slot].hi] = s_car_heading;
  g_mem[slot].hi = (g_mem[slot].hi + 1) % AI_OBS_N;
  if (g_mem[slot].hn < AI_OBS_N) g_mem[slot].hn++;
  // 历史观测各自转全局后取中位数
  float gxl[AI_OBS_N], gyl[AI_OBS_N];
  for (int i = 0; i < g_mem[slot].hn; i++) {
    int idx = (g_mem[slot].hi - g_mem[slot].hn + i + AI_OBS_N) % AI_OBS_N;  // 最旧→最新
    float h = g_mem[slot].hh[idx] * AI_PI / 180.0f;
    float ch = cosf(h), sh = sinf(h);
    gxl[i] = s_car_x - g_mem[slot].hx[idx] * ch + g_mem[slot].hy[idx] * sh;
    gyl[i] = s_car_y + g_mem[slot].hx[idx] * sh + g_mem[slot].hy[idx] * ch;
  }
  g_mem[slot].gx = median_n(gxl, g_mem[slot].hn);
  g_mem[slot].gy = median_n(gyl, g_mem[slot].hn);
  g_mem[slot].t_ms = millis();
  g_mem[slot].stale = 0;
  g_mem[slot].valid = true;
  ai::logf("[ai] 观测 %s 车头系(%.0f,%.0f) 融合n=%d → 全局(%.0f,%.0f)", name, wx, wy,
           g_mem[slot].hn, g_mem[slot].gx, g_mem[slot].gy);
}

// AI 观测（rel_deg 相对车头，正=右、负=左）：换算车头系后入表。
static void mem_observe(const char* name, bool visible, float rel_deg, float dist_cm) {
  if (!visible || !name[0] || dist_cm <= 0) {   // 未见：由 mem_tick_stale 每轮 +1
    if (name[0]) ai::logf("[ai] 观测 %s 不可见", name);
    return;
  }
  float r = rel_deg * AI_PI / 180.0f;
  mem_store(name, sinf(r) * dist_cm, cosf(r) * dist_cm);   // rel 正=右 → wx 右+
}

// AI 观测（屏幕归一化像素 px,py）：单应解算车头系坐标后入表（精度远高于目测距离）。
// 返回 false = 像素越界/解算失败，调用方应回退 rel_deg 路径。
static bool mem_observe_xy(const char* name, bool visible, float px, float py) {
  if (!visible || !name[0]) {   // 未见：由 mem_tick_stale 每轮 +1
    if (name[0]) ai::logf("[ai] 观测 %s 不可见", name);
    return true;
  }
  float wx, wy;
  if (!screen_to_world(px, py, &wx, &wy)) {
    ai::logf("[ai] 观测 %s 像素(%.2f,%.2f) 越界/解算失败", name, px, py);
    return false;
  }
  mem_store(name, wx, wy);
  return true;
}

// 每轮结束：所有物体未观测则过期轮数 +1（过期不清空，仅标记）。
static void mem_tick_stale(void) {
  for (int i = 0; i < AI_MEM_MAX; i++)
    if (g_mem[i].valid) g_mem[i].stale++;
}

// 生成喂给 AI 的空间记忆文本：一律当前车头局部系（相对角度+距离），AI 零换算。
static void mem_feed(char* buf, size_t cap) {
  int n = snprintf(buf, cap, "车向:%d°", (int)s_car_heading);
  for (int i = 0; i < AI_MEM_MAX && n < (int)cap - 64; i++) {
    if (!g_mem[i].valid || g_mem[i].stale > 20) continue;   // 太久(>20轮)才不喂，避免环视中记忆过早消失
    float dx = g_mem[i].gx - s_car_x, dy = g_mem[i].gy - s_car_y;
    float dist = sqrtf(dx * dx + dy * dy);
    float thg = atan2f(dx, dy) * 180.0f / AI_PI;      // 全局角（相对 Y+，逆时针+）
    int rel = (int)roundf(s_car_heading - thg);        // rel 正=右
    rel = (rel + 540) % 360 - 180;                     // wrap -180..180
    n += snprintf(buf + n, cap - n, "；%s %s%u°约%.0fcm",
                  g_mem[i].name, rel >= 0 ? "右偏" : "左偏", (unsigned)abs(rel), dist);
    if (g_mem[i].stale > 0)
      n += snprintf(buf + n, cap - n, "（%d轮前）", (int)g_mem[i].stale);
  }
  ai::logf("[ai] 记忆 %s", buf);   // 喂回内容同步到 ai_log，便于观察 AI 看到的物体位置理解
}

// 构建请求 body。goal 目标文本；ann 标注 JSON 或空；hint 为纠正/引导语（重试或死循环打断）；
// last_cmd 上一步已下发指令的短描述（含 reason，供 AI 判断上一步效果与目标方位）；
// exec_state 执行板状态一行文本（无数据为空串）；hist 为"近几步动作+进度"摘要（方位记忆）。
// 系统提示词 + 目标 + 标注先组进 PSRAM 缓冲，再整体 JSON 转义（内含引号）。
// 有 edited 时带编辑图；prev 非空时带上一帧做周期性双帧运动对比，否则单帧。
static void build_body(PsaBuf& b, const char* goal, const char* ann, const char* hint,
                       const char* last_cmd, const char* exec_state, unsigned last_age_s,
                       const char* hist,
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
  sys.put("{\"type\":\"move\",\"params\":{\"throttle\":0.3,\"steering\":0,\"distance_cm\":30},\"reason\":\"..\"} 移动/转向：离目标距离明确时务必加 distance_cm 定距（已生效、按时长近似，幅度宜小防过冲）；也要 angle_deg 定角；不加则持续移动；低速优先 throttle/steering≤0.5；");
  sys.put("或 {\"type\":\"spin\",\"params\":{\"dir\":1,\"angle_deg\":90},\"reason\":\"..\"} 原地旋转(dir: +1右转(顺时针)/-1左转(逆时针)/0停)：保持车头朝向不变原地转动视角，是观察环境/环视四周的推荐转弯方式，须配合前轮保持直行；可选 angle_deg 定角转指定度数（已生效、近似），小幅微调或转够观察角度用；");
  sys.put("或 {\"type\":\"arm\",\"params\":{\"act\":\"lift_up\",\"dist_cm\":15},\"reason\":\"..\"} act 取 lift_up/lift_down/reach_forward/reach_backward/clip/release/home（home=收臂折叠回平台：第一节竖直、第二节折回，摄像头回最高位扩大视野，用于避开盲区重新观察）；与操作者交接物品时先停稳、伸到其手边再 release；");
  sys.put("或 {\"type\":\"arm_pose\",\"params\":{\"x\":10,\"h\":4},\"reason\":\"..\"} 直接把夹爪末端移动到指定位姿（一步到位）：x=轴前方 cm（可达约4..15），h=夹爪中心离地高度 cm（越高夹爪越抬、越低越贴近地面）。夹取前最推荐用它把夹爪调到与目标高度匹配；不可达时不会移动，请改 x/h 重试；");
  sys.put("或 {\"type\":\"stop\",\"params\":{\"scope\":\"all\"},\"reason\":\"..\",\"done\":true} 立即停车并结束当前任务：任务完成/目标达成/需完全收手时带 done:true；仅临时停车继续观察则不带 done：");
  sys.put("或 {\"type\":\"wait\",\"reason\":\"..\"} 保持当前所有动作不变，原地等待观察：当还在运动中没到目标、或者画面没变化、或者还没锁定目标时，用 wait；");
  sys.put("规则：1. 只输出 JSON，每次只规划一步；回复务必简短——思考放在 reason（中文一句），不要输出长篇推理或解释。2.画面多轮无变化、或需要观察环境/还没锁定目标时，优先用 原地旋转(spin) 小幅环视探索视角；障碍物挡路则尝试绕行；绕行多轮仍无进展才 stop 并说明原因。3.停车信号只认明确手势：掌心正对镜头且五指张开、在镜头前持续上下/左右挥手、或人持续挡在车前，此时才 stop；人只是坐着、抬手或手指出现在画面里，不是停车信号。4.单手指向或手臂指向某一方向=操作者的方向指示，应朝该方向移动；画面中标出目标时朝标注区域移动，不要因为出现人手就停车。5.若目标是纯判断/评估类（含“判断”“是否”“能不能”“可达”“能不能到达”等），以判断优先于移动：不要朝标注区域移动，输出 stop 或 wait 并在 reason 里直接给出结论（如“该位置在对面，不可到达”）。6.目标为空或“巡视”时持续小幅原地旋转(spin)环视四周。7.结合上一轮执行情况与本轮画面判断目标是否已达成或仍在视野：画面暂未出现目标时，优先按规则10做盲区判定，勿因单帧未见就继续前进或误判已完成。8. reason 一句中文简要解释，但必须含目标方位：相对小车的左/中/右 + 距离（约x cm）+ 是否已贴近/被夹爪遮挡；若目标消失，写明最后已知方位与推断（如“最后在偏左约10cm，现在应在车头右前方，右转找回”），供下轮决定转向或后退，避免走过头后盲目环视。9. 摄像头俯视能看到小车整体（含夹爪）。画面无镜像、无颠倒，所见即真实方位：画面左侧=小车左前方、画面右侧=小车右前方、画面中央=正前方；夹爪指向前=小车正前方。直接按画面判断左右，禁止任何镜像/翻转/视差换算。10.目标先前可见且在近处、随后画面中消失（尤其上一步是前进靠近）——先不要假定\"进了盲区\"或\"被机械臂挡了\"：机械臂已很少遮挡目标。目标消失的常见原因是 ①移出画面边缘（转向过头/偏离），②已过近落进两夹板之间（这时两板根部可见但目标被板挡住）。此时绝对禁止继续往前撞；优先用空间记忆（车向+已记物体方位）推断目标现在应在哪个方向，据此转向找回或小距离后退让它重新入画，确认方位后再靠近。仅当确系被机械臂本体遮挡（收臂/伸出/高位时臂体正挡在目标方向）才用 收臂 home 抬摄像头重新观察。11.最后贴近夹取时目标必然被夹爪遮住而消失，属正常：按最后已知位置直接 clip，不要再前进。12.夹爪是两片平行夹板，装机在高位侧俯视时可看到夹爪；“爪:合”只代表夹爪伺服已闭合到位，绝不代表夹住了物体。判断是否夹住必须验证：夹取后执行 arm lift_up 抬臂，目标被带离地面、或仍在两板间随夹爪抬起才算夹住；目标仍在地面或夹爪空合则未夹住，应 release 后重试或调整高度。13.合爪前确认目标位于两夹板之间且贴近夹板根部；目标在两板外侧或下方较远处时合爪必然夹空。夹取类任务不得仅凭“爪:合”或画面接近就 stop done 宣告成功，必须先完成抬臂验证。14.机械臂已很少遮挡目标（摄像头高位俯视），目标不可见几乎都是移出画面边缘或落进两板之间。遇目标暂时不可见：先凭空间记忆（车向 + 已记物体全局方位）推断目标应在哪个方向，转向对准那个方向找回；原地小幅旋转换视角也行，但不要无方向地连续原地打转或盲目前进。15.可输出额外字段 \"carry_prev\":true——当你需要下一轮同时收到本轮画面做前后对比时置位：例如已锁定目标/正在追踪、或即将移动担心目标进盲区需要对比判定时。下轮你会同时收到上一帧与当前帧，据此判断目标是否移动/进入盲区。无需对比时保持单帧以节省开销。16.状态里“抓手:前Xcm 高Ycm”是夹爪末端轴前方与离地高度（前向运动学），据此判断能否/还需伸多远、目标在当前抓手高度是否合适；继续朝受限方向动作不会改变位置时（到顶/缩到底），应换方向或调整姿态。17.发现目标在夹爪轴线的左前方/右前方时，务必先原地转向（spin 或 move+steering）把夹爪轴线对准目标方向，再小步前进；绝对不要朝着斜侧目标直线猛冲——目标很近会撞倒、较远会错过或钻进两夹板间隙。对准后每步前进量宜小（≤10cm），并走一步停一步复查画面。18.夹取前必须先把夹爪高度调到与目标匹配，夹爪悬空过高必然夹空：目标贴地时（沙漏/水瓶等在地面上）h 取 2~5cm；目标有一定高度时估其腰部离地高度，两夹板中心对准腰部。判断目标高度参考画面：夹爪两板张开时可见，其张开宽度约 3~4cm 可当标尺，对比目标在画面中的相对大小估计高度。对准后先用 arm_pose 把夹爪调到该高度，再前进贴近到两板之间，最后 clip；夹空（抬臂验证目标未离地）后，调整高度 h 重试而非反复在原高度空夹。19.可输出额外字段 \"observe\":{\"name\":\"沙漏\",\"px\":0.36,\"py\":0.62,\"visible\":true} 记录画面中物体的位置：name=物体名（你想记住的都行），px/py=物体中心在画面中的归一化坐标，坐标系与你看到的画面完全一致、不要做任何翻转/镜像/变换：左上角=(0,0)、右下角=(1,1)、画面中心=(0.5,0.5)、正右=px增大、正下=py增大。visible=false 表示该物体当前不在画面中。报位置时优先用 px/py（程序会换算成地面坐标，比目测距离准得多）；只有无法给出像素时才用 rel_deg/dist_cm 兜底（rel_deg=相对当前车头角度正=右负=左，dist_cm=距离）。程序会记住并后续以\"当前车头局部系\"喂回给你（如\"车向:35°；沙漏 右偏20°约25cm\"）。车向角 0°=任务开始时的车头方向，逆时针为正（左转为正）；程序已按当前车向把记忆换算好，你直接照此判断方位即可、无需自己换算。注意：记忆仅供参考，画面所见永远为准——看到就刷新 observe，看不到就报 visible=false；画面中物体位置与记忆明显不符说明物体被移动或车已转向，一律以画面为准更新。目标不在画面时，用记忆里的全局坐标结合当前车向，推断它应在车头局部系的哪个方向（右偏/左偏多少度、约多远），直接转向那个方向去找，不要无方向瞎转。20.状态行\"抓手:前Xcm 高Ycm\"与\"车向\"均为程序合成/累积的近似值，做粗粒度参考即可，精细动作仍以画面判断为准。");
  { // 画面标定：从顶部 CAL_* 宏读取，改一处即可应对镜头松动后整体调参
    char cal[768];
    snprintf(cal, sizeof(cal),
             "画面标定（摄像头左前高位俯视，无镜像；数值为近似值，画面所见永远优先）：屏幕越往下越近——"
             "最底部横带贴近夹爪（0cm）、屏幕中心约车前%dcm正前、中心到顶部一半约%dcm、顶部约%dcm；"
             "中横带左边缘中点约车左%dcm、右边缘中点约车右%dcm（左右不对称因镜头偏左）。"
             "这套标定假设小车前方地面基本水平；若目标明显不在同一高度（桌面/台阶/被垫高，或前方高低差），"
             "该距离标定不成立、勿套用；此时只据画面判断大致方向与偏向，距离以状态或合理推断给个大约范围。"
             "近距时夹爪相对目标高低看不清，一律以状态抓手:前Xcm 高Ycm 为准微调，不要靠画面猜高矮。",
             CAL_FWD_MID_CM, CAL_FWD_MIDTOP_CM, CAL_FWD_TOP_CM, CAL_SIDE_LEFT_CM, CAL_SIDE_RIGHT_CM);
    sys.put(cal);
  }
  if (hint && hint[0]) { sys.put("注意："); sys.put(hint); }

  b.put("{\"model\":");
  esc_append(b, cfg::ai_model().c_str());
  b.put(",\"messages\":[{\"role\":\"system\",\"content\":");
  esc_append(b, sys.p ? sys.p : "");
  // 结束 system 对象后接下一消息
  b.put("},{\"role\":\"user\",\"content\":[{\"type\":\"text\",\"text\":");
  PsaBuf ut;  // user 文本：执行板状态 + 上一指令时长 + 进度/近几步 + 画面引导（整体转义一次）
  if (exec_state && exec_state[0]) { ut.put(exec_state); ut.put("；"); }
  if (last_age_s > 0) {
    char age[32]; snprintf(age, sizeof(age), "上一指令约%us前执行；", last_age_s);
    ut.put(age);
  }
  if (hist && hist[0]) { ut.put(hist); ut.put("；"); }
  { // 空间记忆喂回（车向 + 已记物体，当前车头局部系）
    char mem_s[192];
    mem_feed(mem_s, sizeof(mem_s));
    if (mem_s[0]) { ut.put(mem_s); ut.put("；"); }
  }
  if (frame) {
    if (prev && prev_len > 0) {
      ut.put("下面按顺序给出：标注图（若有）、上一帧、当前帧。请对比上一帧与当前帧，判断画面中移动的人手/物体大致朝哪个方向移动；若上一步动作已让目标消失，据两帧差异推断目标方位与盲区。");
    } else {
      ut.put("当前画面如下：");
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
    if (prev && prev_len > 0) img(prev, prev_len);  // 上一帧在前，供对比运动（周期性启用）
    img(frame, frame_len);
  }
  b.put("]}],\"max_tokens\":8192,\"reasoning_effort\":\"low\",\"response_format\":{\"type\":\"json_object\"}}");
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

  JsonDocument doc(&g_js_alloc);  // PSRAM 池，避免内部堆碎片
  if (deserializeJson(doc, c)) {
    snprintf(err_buf, err_cap, "AI 返回非 JSON：%s", c.substring(0, 80).c_str());
    return err_buf;
  }
  const char* type = doc["type"] | "";
  if (strcmp(type, "move") && strcmp(type, "stop") && strcmp(type, "arm") &&
      strcmp(type, "wait") && strcmp(type, "spin") && strcmp(type, "arm_pose")) {
    return "AI 输出非法 type";
  }
  if (!doc["params"].is<JsonObject>() && strcmp(type, "stop") && strcmp(type, "wait")) {
    return "AI 输出缺 params";
  }
  if (!strcmp(type, "arm")) {
    const char* act = doc["params"]["act"] | "";
    static const char* acts[] = {"lift_up","lift_down","reach_forward","reach_backward","clip","release","home", nullptr};
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
    int ss = doc["params"]["speed"] | 900;  // 原地旋转默认转速（实测 <700 拖不动，须给足）
    int sa = doc["params"]["angle_deg"] | 0;
    p["dir"] = constrain(sd, -1, 1);     // 原地旋转方向：±1/0
    p["speed"] = constrain(ss, 0, 1000); // 原地旋转单轮 pwm
    if (src["angle_deg"].is<int>() && sa) p["angle_deg"] = constrain(sa, 0, 500); // 定角微操
    const char* act = doc["params"]["act"] | "";
    if (act[0]) p["act"] = act;
    const char* scope = doc["params"]["scope"] | "all";
    p["scope"] = !strcmp(scope, "wheels") ? "wheels" : (!strcmp(scope, "arm") ? "arm" : "all");
    // arm_pose 指定位姿：x=轴前方 cm（可达 4..15），h=夹爪中心离地高度 cm。arm_pose 内部还有可达域检查。
    float px = doc["params"]["x"] | 0.0f;
    float ph = doc["params"]["h"] | 0.0f;
    if (src["x"].is<float>() || src["x"].is<int>()) p["x"] = constrain(px, 0.0f, 20.0f);
    if (src["h"].is<float>() || src["h"].is<int>()) p["h"] = constrain(ph, -2.0f, 25.0f);
  }
  const char* reason = doc["reason"] | "";
  if (reason[0]) out["reason"] = reason;
  if (doc["done"].is<bool>() && doc["done"].as<bool>()) out["done"] = true;  // 任务完结标记
  // carry_prev:true = 下一轮希望同时收到本轮画面做对比（目标锁定/追踪、判断移动后目标方位）。
  // 本字段不进 params，仅作 worker 决策是否带 prev 的信号。
  if (doc["carry_prev"].is<bool>() && doc["carry_prev"].as<bool>()) out["carry_prev"] = true;
  // observe:true = AI 的空间观测（name/rel_deg/dist_cm/visible），透传给 worker 更新物体记忆表。
  if (doc["observe"].is<JsonObject>()) out["observe"] = doc["observe"].as<JsonObjectConst>();
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

#if 0  // 手写传输层已废弃：其 chunked 投机容错曾把响应体读错位成散文。改用 HTTPClient（见 http_post）。
// 单次请求-响应交换（连接须已就绪）。成功返回 true 且保持连接供下次复用；
// 任何协议/IO 异常返回 false（http_post 决定重建或弃连）。
static bool http_exchange(const String& host_s, const char* key, const char* path_s,
                          const char* body, String& resp, bool* reusable) {
  resp = "";
  *reusable = true;   // 默认可复用；响应头明确 Connection: close 则置 false（下轮直接新建）
  g_last_status = 0;  // 每次交换重置，避免沿用上次状态
  g_client.printf("POST %s HTTP/1.1\r\n", path_s);
  g_client.printf("Host: %s\r\n", host_s.c_str());
  g_client.printf("Authorization: Bearer %s\r\n", key);
  g_client.print("Content-Type: application/json\r\n");
  g_client.print("Accept: application/json\r\n");
  g_client.print("User-Agent: VisionS3/1.0\r\n");
  g_client.printf("Content-Length: %d\r\n", (int)strlen(body));
  g_client.print("Connection: keep-alive\r\n\r\n");

  // 诊断：body 可能达数十 KB，一次 write 成败难定位。分段发，打印每段进度与耗时。
  size_t blen = strlen(body);
  ai::logf("[ai] body %u B", (unsigned)blen);
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
  ai::logf("[ai] body 发送完成 %u/%u ~%u ms", (unsigned)sent, (unsigned)blen,
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
    if (!g_client.connected()) { ai::logf("[ai] 响应头读取中连接关闭（云端主动断开）"); return false; }
    delay(5);
  }
  if (!hdr.endsWith("\r\n\r\n")) { ai::logf("[ai] HTTP 响应头超时/不完整"); return false; }
  // 去掉状态行前残留的空白（前一响应/复用连接遗留的换行），保证 startsWith 判断准确
  while (hdr.length() > 0 && (hdr[0] == '\r' || hdr[0] == '\n')) hdr = hdr.substring(1);
  // 记录状态码，供上层 4xx 快速失败（400 重试无意义）
  int st_sp = hdr.indexOf(' ');
  if (st_sp >= 0 && hdr.length() >= st_sp + 4) g_last_status = hdr.substring(st_sp + 1, st_sp + 4).toInt();
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
    // 非 2xx：把状态行与错误响应体（前 100B）推到手机，便于定位 429 限流 / 400 参数等
    String errbody;
    int want = cl > 0 ? (cl < 512 ? cl : 512) : 0;
    long te = millis();
    while (want > 0 && (int)errbody.length() < want && millis() - te < 2000) {
      while (want > 0 && g_client.available() && (int)errbody.length() < want) errbody += (char)g_client.read();
      delay(3);
    }
    int cr = hdr.indexOf('\r');
    String status = cr > 0 ? hdr.substring(0, cr) : hdr;
    ai::logf("[ai] HTTP 非200 %s 响应体:%.100s", status.c_str(), errbody.c_str());
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
    // chunked 解码（严格、无投机）：每个字节/行按独立空闲超时读取，逐字节精确计数。
    // 上一版用「全局 30s 时钟 + 块尾字节压回(pend) + size 行前向重同步」做容错，但这些
    // 投机在大响应慢流时会误读块边界——只吞到半截 size 行、按错误块长读取，导致响应体被
    // 裁剪/错位成散文（`{"id":xxx` 后接错位文本），deserializeJson 必然失败。这里改为：
    // 严格按块边界逐字节计数，块尾必须 \r\n（或 \n），任何不完整一律判失败并返回 false，
    // 由上层丢弃该连接、换新连接重试。宁可多次失败重试，也绝不再产出错位响应体。
    const unsigned long idle_ms = 2500;   // 单个读操作空闲超时：无新字节即判断流
    auto read_byte_idle = [&]() -> int {
      unsigned long t0 = millis();
      while (millis() - t0 < idle_ms) {
        if (g_client.available() > 0) { int c = g_client.read(); if (c >= 0) return c; }
        if (!g_client.connected()) return -1;
        delay(1);
      }
      return -1;
    };
    // 读一行（忽略 \r，遇 \n 停并消费）。返回 0=正常、-1=超时/断流；超时绝不返回半截。
    auto read_line_strict = [&](String& out, size_t cap) -> int {
      out = "";
      for (;;) {
        int c = read_byte_idle();
        if (c < 0) return -1;
        if (c == '\r') continue;
        if (c == '\n') return 0;
        if (out.length() < cap) out += (char)c;
      }
    };
    // 严格十六进制 chunk-size 解析（支持 ";ext" 扩展与首尾空白；垃圾行直接报错）
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
      String line;
      if (read_line_strict(line, 64) < 0) { Serial.println("[ai] chunked size 行读取失败"); return false; }
      int sz = 0;
      if (!parse_size(line, &sz)) {
        Serial.printf("[ai] chunked size 行异常: '%.32s'\n", line.c_str());
        return false;
      }
      if (sz == 0) {
        // 终止块 0：吞 trailer 到空行（单请求后即重建连接，清不清流都无碍）
        for (;;) { String t; if (read_line_strict(t, 256) < 0) break; if (t.length() == 0) break; }
        break;
      }
      if (sz < 0 || sz > 262144) { Serial.println("[ai] chunked 块过大"); return false; }
      for (int i = 0; i < sz; i++) {        // 逐字节精确读块数据，缺一字节即失败
        int c = read_byte_idle();
        if (c < 0) { Serial.printf("[ai] HTTP chunked 中断 got=%d\n", (int)pb.len); return false; }
        pb.put((char)c);
      }
      // 块尾：严格 \r\n（或仅 \n）。不完整即失败，不猜字节、不压回。
      int t1 = read_byte_idle();
      if (t1 < 0) { Serial.println("[ai] chunked 块尾缺失"); return false; }
      if (t1 == '\r') {
        int t2 = read_byte_idle();
        if (t2 < 0 || t2 != '\n') { Serial.println("[ai] chunked 块尾缺失"); return false; }
      } else if (t1 != '\n') {
        Serial.println("[ai] chunked 块尾错位"); return false;
      }
    }
    ai::logf("[ai] HTTP chunked 解码 %d B", (int)pb.len);
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
    ai::logf("[ai] SSE 解码 %d B", (int)pb.len);
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
#endif  // http_exchange 已废弃，改用 HTTPClient

static bool http_post(const char* url, const char* key, const char* body, String& resp,
                      unsigned long gen) {
  (void)gen;  // 代际/中断判断由调用方（worker 循环）负责；cancel 走 g_client.stop()
  g_last_status = 0;  // 入口重置，避免沿用上一轮 4xx/429 误判
  ai::logf("[ai] TLS前 freeHeap=%u maxBlock=%u freePsram=%u",
           ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getFreePsram());
  unsigned long t0 = millis();
  // 连接策略：尽量复用同一条 TLS 连接（省握手、降延迟）。复用的风险是上一次成功响应在
  // ssl_ctx/HTTPClient 留下未读尽的残留，或连接在决策间隙被服务端/半开关掉，导致下一次
  // mbedtls_ssl_write 报 -0x7100 BAD_INPUT_DATA（→-3）。处理：pass0 直接复用当前 g_client；
  // 一旦发送失败立刻 stop 掉，pass1 用全新握手重发同一 body（只多这一轮），把多轮 -3 阵发
  // 收敛成"一次复用失败 + 一次新握手成功"。success 时保留连接供下轮复用。
  static bool s_tls_cfg = false;
  if (!s_tls_cfg) {
    g_client.setInsecure();                 // 开发期信任自签；上线建议改 CA 校验
    g_client.setConnectionTimeout(AI_CONNECT_TIMEOUT_MS);
    g_client.setTimeout(AI_HTTP_TIMEOUT_MS);
    s_tls_cfg = true;
  }
  for (int pass = 0; pass < 2; pass++) {
    if (pass == 1 && g_client.connected()) { g_client.stop(); }  // 复用失败：彻底断开，全新握手
    HTTPClient http;
    http.setReuse(true);                    // 成功即保留连接供下轮复用
    if (!http.begin(g_client, url)) { Serial.println("[ai] HTTPClient begin 失败"); return false; }
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", String("Bearer ") + String(key));
    int code = http.POST(body);             // body 为 PSRAM C 串；流式发送、不整体拷内部堆
    g_last_status = code;
    ai::logf("[ai] POST 完成 code=%d ~%u ms rssi=%d", code, (unsigned)(millis() - t0), (int)WiFi.RSSI());
    if (code <= 0) {
      ai::logf("[ai] HTTP POST 失败 code=%d pass%d rssi=%d", code, pass + 1, (int)WiFi.RSSI());
      if (pass == 0) { g_client.stop(); http.end(); continue; }   // 复用失败：断开，pass1 全新握手
      g_client.stop(); http.end();
      return false;
    }
    if (code != 200) {
      String eb = http.getString();
      if (eb.length() > 128) eb = eb.substring(0, 128);
      ai::logf("[ai] HTTP 非200 %d 响应体:%.100s", code, eb.c_str());
      http.end();
      return false;
    }
    resp = http.getString();                // HTTPClient 已自动解码 chunked；模型简短，落堆可接受
    http.end();                             // 成功即保留连接，下轮复用
    return resp.length() > 0;
  }
  return false;
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
  // 剥离开头泄漏的 chunked size 行（形如 "XX\r\n{" / "XX\n{"，XX 为 1-8 位十六进制长度）。
  // 复用连接偶发把某次响应的 size 行错位拼进下一条响应体顶部，deserializeJson 直接失败；
  // JSON 响应永远以 '{' 起始、永不始于 hex，故只在"hex+换行+{" 时剥离，绝不误伤正文。
  {
    auto ishex = [](char c){ return (c>='0'&&c<='9')||(c>='a'&&c<='f')||(c>='A'&&c<='F'); };
    for (int s = 0; s < 4; s++) {              // 最多剥 4 段（防多块错位叠加）
      const char* q = payload.c_str();
      int i = 0;
      while (i < 8 && ishex(q[i])) i++;        // 扫描十六进制前缀
      if (i == 0) break;                       // 开头非 hex，非泄漏
      int j = i;
      if (q[j] == '\r') j++;
      if (q[j] == '\n') j++;
      if (q[j] != '{') break;                  // 后随非 '{'，按正文处理
      payload = payload.substring(j);          // 剥掉这段 size 行
    }
  }
  JsonDocument doc(&g_js_alloc);
  if (deserializeJson(doc, payload)) {
    // 解析失败诊断：打印完整 payload + 结构判据（判断是状体被拼断/chunked 泄漏/SSE 多段拼接）。
    auto sanitize = [](String s) {
      for (int i = 0; i < s.length(); i++) {
        unsigned char ch = (unsigned char)s[i];
        if (ch < 0x20 || ch == '\x7f') s[i] = '?';
      }
      return s;
    };
    // 结构判据：是否含 "\ndata:"(SSE 多帧)、是否含多个 "{"id"..."}(多 JSON 拼接)、是否含 chunk 大小泄漏前缀。
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
    String r = rc;
    if (r.length() > 120) r = r.substring(0, 120);
    ai::logf("[ai] 思考: %s", r.c_str());
  }
  const char* c = doc["choices"][0]["message"]["content"] | "";
  // 注意：content 为空但 reasoning 有值 = 模型只给了思考没给正式回答（安全终止/条件触达）
  if (!c[0] && rc[0]) {
    String rr = rc;
    if (rr.length() > 120) rr = rr.substring(0, 120);
    ai::logf("[ai] content为空但reasoning有值(%u字节): %s", (unsigned)strlen(rc), rr.c_str());
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
    g_log_fn = t.fn;            // ai_log 回推通道：本任务周期内有效
    g_log_ctx = t.ctx;
    g_last_continuous = false;  // 本任务尚未下发过持续指令（防上一任务残留标志误判）
    g_last_cont_type = 0;
    m_busy = true;
    // 新任务 = 新坐标系：车位置=原点、初始车头=0°，清空上一任务的空间记忆。
    s_car_x = 0; s_car_y = 0; s_car_heading = 0;
    memset(g_mem, 0, sizeof(g_mem));
    // 摄像头可用性在任务起点判定（init 后即定）：不可用则整轮走无画面降级
    bool cam_ok = cam::available();
    ai::logf("[ai] 任务开始 gen=%lu text=%s%s", t.generation, t.text, cam_ok ? "" : "（无摄像头→无画面模式）");
    // 单应状态诊断：若未就绪，所有 px/py 观测都会被拒，直接可看出问题
    if (!s_h_ok) ai::logf("[ai] 警告：单应未就绪（像素观测将全部被拒绝）");
    else {
      float ex, ey;
      screen_to_world(0.5f, 0.5f, &ex, &ey);
      ai::logf("[ai] 单应OK 中心→(%.0f,%.0f) H0-7=(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",
               ex, ey, H[0], H[1], H[2], H[3], H[4], H[5], H[6], H[7]);
    }
    // 打印实际端点/模型，便于排查 404/401 等云端拒绝（配错路径是常见原因）
    ai::logf("[ai] 端点=%s 模型=%s key=%s", cfg::ai_url().c_str(), cfg::ai_model().c_str(),
                  cfg::ai_key().isEmpty() ? "空" : "已配置");

    unsigned long steps = 0;
    bool done = false;
    const char* fail = nullptr;
    char err_buf[160];
    // 死循环防线状态：上一步指令短描述 / 连续相同指令计数 / 已注入引导标记。
    char last_cmd[48] = {0}, cur_cmd[48] = {0};
    char last_disp[256] = {0};  // 发给 AI 的"上一步已下发"（指令+reason，含目标方位，供延续判断）
    // 近几步动作环形缓冲（方位记忆）：让 AI 知道自己转过几次/走过多远，防重复打转。
    char act_hist[4][48] = {{0}};
    int act_hist_n = 0;
    // 任务内指令类型计数（进度）：让 AI 数得清已发几次 move/spin/...，防"补发"重复执行。
    static const char* k_type_names[] = {"move", "spin", "arm", "arm_pose", "stop", "wait"};
    int act_cnt[sizeof(k_type_names) / sizeof(k_type_names[0])] = {0};
    unsigned long last_act_ms = 0;  // 上次真正下执行/微操指令的时刻（ms），供给 AI 算"距上次决策多久"
    int net_fail = 0;           // 连续"无有效输出"轮数（网络/解析失败），用于退避与上限收尾
    int stall = 0;
    bool stall_hint = false;
    bool want_prev = false;     // AI 上轮 carry_prev=true → 本轮带上 prev 帧做对比

    // 编辑图一次性取快照（供整轮任务复用，避免中途被覆盖）。
    uint8_t* edited = nullptr; size_t edited_len = 0;
    if (t.use_image) {
      edited = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
      if (edited && !take_edited(edited, AI_EDITED_IMG_MAX, &edited_len)) { free(edited); edited = nullptr; }
    }
    // 本轮帧的 PSRAM 副本（任务期复用）：抓帧后立即把字节拷进来并归还相机缓冲，
    // 避免 AI 在做慢速 TLS 请求期间长时间占用 fb_count=2 的缓冲池把推流饿死。
    uint8_t* cur = nullptr; size_t cur_len = 0;
    // 上一帧 PSRAM 副本（周期性双帧运动对比用；任务期复用）。
    uint8_t* prev = nullptr; size_t prev_len = 0;

    // 兜底 stop（任务终结出口统一解析一次）。g_stop_mode 由打断方写入：
    // None=手动 move/stop 接管（用户指令已覆盖，不补停）；Wheels=手动 arm（只停轮子）；All=其余。
    // 仅当存在持续指令残留才补。
    auto resolve_stop = [&]() {
      if (g_stop_mode == (int)ai::StopMode::None) return;
      if (!g_last_continuous) return;
      // Wheels 模式只对轮子持续残留停轮子；臂持续残留（或未知）必须全停。
      const char* scope = (g_stop_mode == (int)ai::StopMode::Wheels && g_last_cont_type == 1) ? "wheels" : "all";
      JsonDocument d; d["scope"] = scope;   // d 即 stop 的 params 对象
      exec::act("stop", d.as<JsonObjectConst>());
      Serial.printf("[ai] 兜底 stop scope=%s\n", scope);
      g_last_continuous = false;
    };

    while (!done) {
      uint64_t step_ts = esp_timer_get_time();  // 本轮起点（周期控制基准）
      fail = nullptr;  // 每轮重置，避免沿用上轮错误文本误导日志/回报
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
      // 帧数据后续（body 组装）一律用这份副本。
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
        // 引导语：重试纠正 / 死循环打断（连续多轮相同指令，带重复指令名便于 AI 自我纠正）
        char hint_buf[192];
        const char* hint;
        if (attempt) {
          hint = "上次输出非法，请只输出合法 JSON 词表指令。";
        } else if (stall_hint) {
          snprintf(hint_buf, sizeof(hint_buf),
                   "连续多轮重复「%s」无进展：先改变观察角度重新确认目标（抬/落机械臂、后退或环视），若夹爪已夹紧或无法达成则输出 stop。", last_cmd);
          hint = hint_buf;
        } else {
          hint = "";
        }
        // 进度与近几步动作摘要（方位记忆）：供 AI 知道自己转了几次/走过多远，防重复打转
        char hist_s[256] = {0};
        int hp = 0;
        // 任务内指令计数：move×N spin×M ...（仅列出 >0 的，AI 据此不再"补发"已完成的动作）
        hp += snprintf(hist_s + hp, sizeof(hist_s) - hp, "已发[");
        int kc = 0;
        for (int ki = 0; ki < (int)(sizeof(act_cnt) / sizeof(act_cnt[0])) && hp < (int)sizeof(hist_s) - 64; ki++) {
          if (!act_cnt[ki]) continue;
          hp += snprintf(hist_s + hp, sizeof(hist_s) - hp, "%s%s×%d", kc++ ? "," : "", k_type_names[ki], act_cnt[ki]);
        }
        if (!kc) hp += snprintf(hist_s + hp, sizeof(hist_s) - hp, "无");
        hp += snprintf(hist_s + hp, sizeof(hist_s) - hp, "] 近几步：[");
        for (int hi = 0; hi < act_hist_n && hp < (int)sizeof(hist_s) - 48; hi++)
          hp += snprintf(hist_s + hp, sizeof(hist_s) - hp, "%s%s", hi ? "," : "", act_hist[hi]);
        hp += snprintf(hist_s + hp, sizeof(hist_s) - hp, "]；第%lu步", (unsigned long)(steps + 1));
        ai::logf("[ai] %s", hist_s);  // 任务进度与近几步：手机端 /ai_log 直接可见
        PsaBuf body;
        char st[96];  // exec 状态缓冲（含限位提示，需足量避免截断）
        const char* stp = exec::read_state(st, sizeof(st)) ? st : "";  // 本地直驱状态（无执行板，状态本地合成）
        // 上一帧是否带上：仅由 AI 上轮 carry_prev=true 决定（锁定/追踪意图），其余保持单帧省开销。
        bool use_prev = want_prev && prev_len > 0;
        build_body(body, t.text, t.ann, hint, last_disp, stp,
                   last_act_ms ? (unsigned)((esp_timer_get_time() / 1000 - last_act_ms) / 1000) : 0u,
                   hist_s,
                   frame, frame_len, prev, use_prev ? prev_len : 0,
                   edited != nullptr, edited, edited_len);
        if (!body.ok) { fail = "组装请求 body 失败"; break; }

        String resp;
        bool http_ok = false;
        for (int nr = 0; nr < 3 && !http_ok; nr++) {   // 网络失败指数退避重试（任务串行，代价可控）
          if (http_post(cfg::ai_url().c_str(), cfg::ai_key().c_str(), body.p, resp, t.generation)) { http_ok = true; break; }
          if (t.generation != m_generation) { done = true; break; }  // 被中止，静默作废
          if (g_last_status >= 400 && g_last_status < 500) {  // 4xx 重发同 body 必然再拒，快速失败
            fail = "云端拒绝(4xx)，疑似参数或限流";
            break;
          }
          if (nr < 2) { vTaskDelay(pdMS_TO_TICKS(500 << nr)); Serial.printf("[ai] 网络失败重试 %d\n", nr + 1); }
        }
        if (done) break;
        if (!http_ok) { if (!fail) fail = "AI 请求失败"; break; }
        if (t.generation != m_generation) { done = true; break; }  // 在途结果作废

        String content;
        if (!extract_content(resp, content)) {
          // 解码成功但无有效内容（瞬态错误体/空 content 等）：打印原始片段便于定位
          Serial.printf("[ai] 响应无内容，原始(前120B): %s\n", resp.substring(0, 120).c_str());
          fail = "AI 响应无内容";
          continue;  // 空内容→重试，不终止
        }

        JsonDocument cmdD(&g_js_alloc);  // PSRAM 池：避免每轮在内部堆分配/释放制造碎片（freeHeap 泄漏嫌疑）
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
          // 校验通过 → 执行 move/arm/stop（本板直驱，不再经执行板），记录时刻供时间感知。
          exec::act(type, params);
          last_act_ms = (unsigned long)(esp_timer_get_time() / 1000);
          // 空间记忆：按 move/spin 定距/定角更新车姿态；解析 AI observe 更新物体记忆表。
          car_update_pose(type, params);
          if (cmdD["observe"].is<JsonObject>()) {
            JsonObjectConst ob = cmdD["observe"].as<JsonObjectConst>();
            const char* nm = ob["name"] | "";
            bool vis = ob["visible"] | true;
            if (nm[0]) {
              // 优先用屏幕像素 px/py（单应解算，精度高）；越界/解算失败或没报像素则回退 rel_deg/dist。
              bool has_px = ob["px"].is<float>() || ob["px"].is<int>();
              bool has_py = ob["py"].is<float>() || ob["py"].is<int>();
              if (has_px && has_py) {
                if (!mem_observe_xy(nm, vis, ob["px"] | 0.0f, ob["py"] | 0.0f))
                  mem_observe(nm, vis, ob["rel_deg"] | 0.0f, ob["dist_cm"] | 0.0f);
              } else {
                mem_observe(nm, vis, ob["rel_deg"] | 0.0f, ob["dist_cm"] | 0.0f);
              }
            }
          }
          // 无里程计兜底：AI 无定距的持续 move 单次最多行驶 AI_MOVE_CAP_MS，到期由 exec 自动停轮；
          // 带 distance_cm 的定距 move 已在 send_move 按时长近似自停，不叠加持续兜底。
          if (!strcmp(type, "move") && !params["distance_cm"].is<int>() &&
              fabsf(params["throttle"] | 0.0f) > 0.001f)
            exec::set_move_cap_ms(AI_MOVE_CAP_MS);
          g_last_continuous = exec::is_continuous(type, params);
          g_last_cont_type = g_last_continuous ? (!strcmp(type, "move") ? 1 : 2) : 0;
          ai::logf("[ai] 执行 %s%s", type, g_last_continuous ? "（持续）" : "");
          String fb = build_feedback(t.id, cmdD);
          enqueue_result(fb.c_str(), t.fn, t.ctx);
          // 死循环防线：连续多轮下发相同指令 → 下轮注入引导语让 AI 主动变化
          fmt_last(cur_cmd, sizeof(cur_cmd), type, params);
          for (int ki = 0; ki < (int)(sizeof(act_cnt) / sizeof(act_cnt[0])); ki++)
            if (!strcmp(type, k_type_names[ki])) { act_cnt[ki]++; break; }  // 任务进度计数
          if (strcmp(cur_cmd, last_cmd)) { stall = 0; stall_hint = false; }
          else if (++stall >= 3 && !stall_hint) { stall_hint = true; Serial.println("[ai] 多轮无进展，注入引导"); }
          snprintf(last_cmd, sizeof(last_cmd), "%s", cur_cmd);
          // 记录进"近几步"环形缓冲（最多 4 条，满则覆盖最旧）：方位/动作记忆防重复打转
          if (act_hist_n < 4) { strncpy(act_hist[act_hist_n++], cur_cmd, 47); act_hist[act_hist_n - 1][47] = 0; }
          else { memmove(act_hist[0], act_hist[1], 3 * 48); strncpy(act_hist[3], cur_cmd, 47); act_hist[3][47] = 0; }
          // AI 显式要求保留上一帧（锁定/追踪）：下轮带上 prev；否则按兜底节奏走
          want_prev = cmdD["carry_prev"].is<bool>() && cmdD["carry_prev"].as<bool>();
          // 发给 AI 的"上一步已下发"：指令 + reason，替代双帧供跨轮衔接（目标方位/执行情况）。
          // reason 较长时按 UTF-8 安全截断（%.*s 会截断半个中文字节 → 云端 400 invalid unicode）。
          const char* r = cmdD["reason"] | "";
          int rn = (int)strlen(r);
          if (rn > 128) {
            rn = 128;
            while (rn > 0 && ((unsigned char)r[rn] & 0xC0) == 0x80) rn--;  // 回退到字符边界
          }
          snprintf(last_disp, sizeof(last_disp), "%s，原因：%.*s", cur_cmd, rn, r);
          // stop + done:true = AI 自判任务完成：停车并结束任务（build_feedback 已带 done 上报手机端）；
          // 不带 done 的 stop 仅临时停车观察，任务继续下一轮。严格按 bool 判定，防模型输出字符串 "true"。
          if (!strcmp(type, "stop") && cmdD["done"].is<bool>() && cmdD["done"].as<bool>()) {
            done = true;
            Serial.println("[ai] AI 判定任务完成（stop+done）");
          }
          got = true;
          break;
        } else {
          Serial.printf("[ai] 校验: %s\n", verr);
          fail = verr;  // 重试一次前暂存
        }
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
        // 单轮收尾：补一条带 done 的完成通知，让手机端复位「发送」并显示任务结束。
        JsonDocument e(&g_js_alloc);
        e["done"] = true;
        String s = build_feedback(t.id, e);
        enqueue_result(s.c_str(), t.fn, t.ctx);
        done = true;
        break;
      }
      if (!got) {
        // 网络/解析持续失败：逐轮指数退避（3s→6s→12s→24s 封顶），避免每轮狂建连接把云端打得更紧；
        // 429 限流给更长喘息（30s）再继续；失败原因同步推手机。
        if (++net_fail >= AI_MAX_NET_FAIL) {
          const char* reason = "云端持续无响应（疑似限流），任务已中止，请稍后重试";
          JsonDocument e(&g_js_alloc);
          e["error"] = reason;
          String s = build_feedback(t.id, e);
          enqueue_result(s.c_str(), t.fn, t.ctx);
          Serial.println("[ai] 连续网络失败超限，任务中止");
          break;
        }
        int wait = g_last_status == 429 ? 30000 : (3000 << (net_fail > 3 ? 3 : net_fail - 1));
        ai::logf("[ai] 本轮无有效输出（%s），%ds 后重试", fail ? fail : "未知", wait / 1000);
        vTaskDelay(pdMS_TO_TICKS(wait));
      } else {
        net_fail = 0;
      }
      mem_tick_stale();   // 每轮结束：未观测的物体过期轮数 +1

      // 滚动缓存上一帧：仅当 AI 下一轮 carry_prev=true 时才发送作对比（锁定/追踪场景），
      // 其余单帧以节省云端处理开销。AI 不要求即永不发送此帧。
      if (frame) {
        if (!prev) prev = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
        if (prev && frame_len <= AI_EDITED_IMG_MAX) { memcpy(prev, frame, frame_len); prev_len = frame_len; }
        else prev_len = 0;
      }

      if (++steps >= AI_MAX_STEPS_PER_GOAL) {
        JsonDocument e(&g_js_alloc); e["done"] = true; String s = build_feedback(t.id, e); enqueue_result(s.c_str(), t.fn, t.ctx);
        break;
      }
      // 周期控制：以 AI_INTERVAL_MS 为下限节奏，扣掉本轮已耗时（含抓帧/HTTP/校验），
      // 服务端快（单帧 ~2s）时立即进入下一轮，慢时由服务端耗时主导。
      uint64_t el = esp_timer_get_time() - step_ts;
      long rem = (long)AI_INTERVAL_MS - (long)(el / 1000);
      if (rem > 0) vTaskDelay(pdMS_TO_TICKS(rem));
    }

    if (edited) free(edited);
    if (cur) free(cur);          // 本轮帧 PSRAM 副本
    if (prev) free(prev);        // 上一帧 PSRAM 副本
    if (t.ctx) delete (int*)t.ctx;  // 任务期 sink fd
    ai::logf("[ai] 任务结束 gen=%lu", t.generation);   // 先于注销通道，确保此行也能回推
    g_log_fn = nullptr; g_log_ctx = nullptr;  // 注销 ai_log 回推通道
    free(t.text); free(t.ann);
    resolve_stop();   // 统一兜底：持续指令残留即补停
    m_busy = false;
  }
}

// ---------------- 对外 ----------------

// mbedTLS 内存搬到 PSRAM：TLS 握手/传输要连续分配 in+out 缓冲（各 16KB）和 context，
// 内部堆碎片化时（maxBlock<12K 日志频繁 code=-1/-3/-11）握手必败。把 mbedTLS 内部所有
// 分配改为优先 PSRAM（8MB 充足），内部堆只留少量连续块即可握手。PSRAM 满则回退内部堆。
static void* ai_tls_calloc(size_t n, size_t s) {
  void* p = heap_caps_calloc(n, s, MALLOC_CAP_SPIRAM);
  if (!p) p = heap_caps_calloc(n, s, MALLOC_CAP_INTERNAL);
  return p;
}
static void ai_tls_free(void* p) { heap_caps_free(p); }

void ai::init() {
  if (g_worker) return;
  Serial.println("[ai] build=hwaes_int8192_v4  （TLS: 硬件AES/INTERNAL/8192；ws_stream/ping_svc 栈已调大）");
  if (homography_fit()) {
    Serial.printf("[ai] 单应拟合成功 H=(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)\n",
                  H[0], H[1], H[2], H[3], H[4], H[5], H[6], H[7]);
    float tx = 0, ty = 0;
    bool ok1 = screen_to_world(0.5f, 0.5f, &tx, &ty);
    Serial.printf("[ai] 单应验证 中心%s(%.1f,%.1f) ", ok1 ? "" : "FAIL ", tx, ty);
    tx = ty = 0;
    bool ok2 = screen_to_world(0.75f, 0.5f, &tx, &ty);
    Serial.printf("右下%s(%.1f,%.1f) ", ok2 ? "" : "FAIL ", tx, ty);
    tx = ty = 0;
    bool ok3 = screen_to_world(0.5f, 0.25f, &tx, &ty);
    Serial.printf("上方%s(%.1f,%.1f)\n", ok3 ? "" : "FAIL ", tx, ty);
  } else {
    Serial.println("[ai] 单应拟合失败（校准点不足/退化）");
  }
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