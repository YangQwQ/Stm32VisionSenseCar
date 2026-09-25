#include "src/ai/ai_result.h"
#include "src/ai/ai_client.h"   // ai::logf / ai::update / ai::set_edited_image 声明
#include "src/core/board_log.h"
#include "src/core/lock_guard.h"   // ScopedLock(g_img_mtx 的取放)

#include <esp_heap_caps.h>
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdarg.h>

// 结果队列项: 每项独立持有消息文本与恢复用的(fn, 本次堆拷贝 ctx)。
struct ResultItem {
  char* text = nullptr;
  cmd::ReplyFn fn = nullptr;
  void* ctx = nullptr;    // 每次入队新建的 int*(WS); BLE nullptr
};

static QueueHandle_t g_result_q = nullptr;

// 用户编辑图暂存(PSRAM 环形, 保留最近 3 张)+ 时间戳/插入序号; worker 内只读快照由
// set_edited_image/取图互斥。AI 侧记为 image1(最新)/image2/image3, 供 carry_image:"image1~3" 按需查看。
static uint8_t* g_edited[AI_EDITED_SLOTS] = {};
static size_t g_edited_len[AI_EDITED_SLOTS] = {};
static uint64_t g_edited_ts[AI_EDITED_SLOTS] = {};
static uint32_t g_edited_seq[AI_EDITED_SLOTS] = {};  // 插入序号(越大越新), 排序"新→旧"用
static uint32_t g_edited_seq_cnt = 0;
static SemaphoreHandle_t g_img_mtx = nullptr;

void ai::result_init() {
  g_img_mtx = xSemaphoreCreateMutex();
  g_result_q = xQueueCreate(8, sizeof(ResultItem*));
}

// 就地把一条 WS 文本转为合法 UTF-8(RFC6455 文本帧必须为 UTF-8): 非法/残缺序列会让手机端以
// 关闭码 1007 断链。中文保持不变, 仅把控制字符与非法/残缺序列 1:1 替换为 '?'(不扩容)。
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

void ai::enqueue_result(const char* text, cmd::ReplyFn fn, void* ctx) {
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

void ai::set_edited_image(const uint8_t* data, size_t len) {
  if (!data || len == 0 || len > AI_EDITED_IMG_MAX) return;
  int w = -1;
  {
    ScopedLock lk(g_img_mtx);
    // 找写入槽: 优先空槽; 全满则环形覆盖最旧一张(仅保留最近 3 张)。
    for (int i = 0; i < AI_EDITED_SLOTS; i++) if (!g_edited[i] || g_edited_len[i] == 0) { w = i; break; }
    if (w < 0) {   // 全满: 覆盖最早插入的一张(seq 最小)
      w = 0;
      for (int i = 1; i < AI_EDITED_SLOTS; i++) if (g_edited_seq[i] < g_edited_seq[w]) w = i;
    }
    if (!g_edited[w]) g_edited[w] = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
    if (g_edited[w]) {
      memcpy(g_edited[w], data, len);
      g_edited_len[w] = len;
      g_edited_ts[w] = esp_timer_get_time();
      g_edited_seq[w] = ++g_edited_seq_cnt;
    }
  }
  blog::logf(blog::AI, "收到编辑图 %u B (槽%d)", (unsigned)len, w);
}

int ai::edited_snapshot(uint8_t** imgs, size_t* lens, int* order, int maxn) {
  if (maxn > AI_EDITED_SLOTS) maxn = AI_EDITED_SLOTS;
  for (int i = 0; i < maxn; i++) { imgs[i] = nullptr; lens[i] = 0; order[i] = -1; }
  int vn = 0;
  ScopedLock lk(g_img_mtx);   // 整段持锁到函数出口(内含多处 continue 跳过)
  struct { int idx; uint32_t seq; } vt[AI_EDITED_SLOTS];
  uint64_t now = esp_timer_get_time();
  for (int i = 0; i < AI_EDITED_SLOTS; i++) {
    if (!g_edited[i] || g_edited_len[i] == 0) continue;
    if ((now - g_edited_ts[i]) >= (uint64_t)AI_EDITED_IMG_TTL_MS * 1000) continue;   // 过期不算
    uint8_t* b = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
    if (!b) continue;
    memcpy(b, g_edited[i], g_edited_len[i]);
    imgs[i] = b; lens[i] = g_edited_len[i];
    vt[vn].idx = i; vt[vn].seq = g_edited_seq[i]; vn++;
  }
  for (int k = 0; k < vn; k++) {   // 选择排序: 新→旧
    int m = -1; uint32_t ms = 0;
    for (int j = 0; j < vn; j++) if (vt[j].seq > ms) { ms = vt[j].seq; m = j; }
    order[k] = vt[m].idx; vt[m].seq = 0;
  }
  return vn;
}