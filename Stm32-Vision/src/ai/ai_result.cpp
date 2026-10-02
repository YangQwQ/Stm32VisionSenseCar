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

// 结果队列项: 文本本体放进尾部弹性数组, 一次 malloc 到手(省下独立 text 分配)。
struct ResultItem {
  cmd::ReplyFn fn;
  // WS 的目标 fd 直接内联在本块里(原来每次入队还要 new 一个 int —— 一条消息多一个 4B 的独立
  // 内部堆块, 而内部堆最缺的就是"最大连续块", 这种碎渣正是碎片化的主要来源)。-1 = 走 BLE(无 fd)。
  int fd;
  char text[];           // 消息文本(已消毒), 与结构体同一块分配
};

static QueueHandle_t g_result_q = nullptr;

// 用户编辑图暂存(PSRAM 环形, 保留最近 3 张)+ 时间戳/插入序号; worker 内只读快照由
// set_edited_image/取图互斥。任务起点由 worker 取快照并统一编号(ImageN), 供 look(image=[N]) 按需回看。
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
  size_t n = strlen(text);
  // ★ 走 PSRAM：这条**每轮 AI 要发十几~几十条**、文本变长（任务面板/记忆快照可到 KB 级），队列深 8
  //   ⇒ 未消费前多个大块并存。落在默认(内部)堆时，实测把内部 DMA 池的"最大连续块"打到 1KB 上下
  //   （AI 任务期间 `[水位] 内部DMA块告急` 几乎持续不断）。消费方 ai::update 用同一个 heap_caps_free。
  ResultItem* it = (ResultItem*)heap_caps_malloc(sizeof(ResultItem) + n + 1, MALLOC_CAP_SPIRAM);
  if (!it) return;
  memcpy(it->text, text, n + 1);
  sanitize_ws_utf8(it->text);   // 统一 WS 文本消毒: 任何 enqueue 出口都走这里, 防 1007 断链
  it->fn = fn;
  it->fd = ctx ? *(int*)ctx : -1;   // WS: 本次结果的 fd 拷贝(loop 发送后随整块释放); BLE 无 fd
  if (xQueueSend(g_result_q, &it, 0) != pdTRUE) heap_caps_free(it);
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
  blog::log_line(blog::AI, p);   // 已格式化, 直接输出(免二次 vsnprintf)
}

void ai::update() {
  ResultItem* it = nullptr;
  while (g_result_q && xQueueReceive(g_result_q, &it, 0) == pdTRUE) {
    if (it) {
      if (it->fn) it->fn(it->fd >= 0 ? (void*)&it->fd : nullptr, it->text);
      heap_caps_free(it);   // 配 enqueue_result 的 MALLOC_CAP_SPIRAM
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

int ai::edited_snapshot(uint8_t** imgs, size_t* lens, int maxn) {
  if (maxn > AI_EDITED_SLOTS) maxn = AI_EDITED_SLOTS;
  for (int i = 0; i < maxn; i++) { imgs[i] = nullptr; lens[i] = 0; }
  int vn = 0;
  ScopedLock lk(g_img_mtx);   // 整段持锁到函数出口(内含多处 continue 跳过)
  uint64_t now = esp_timer_get_time();
  for (int i = 0; i < AI_EDITED_SLOTS; i++) {
    if (!g_edited[i] || g_edited_len[i] == 0) continue;
    if ((now - g_edited_ts[i]) >= (uint64_t)AI_EDITED_IMG_TTL_MS * 1000) continue;   // 过期不算
    uint8_t* b = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
    if (!b) continue;
    memcpy(b, g_edited[i], g_edited_len[i]);
    imgs[i] = b; lens[i] = g_edited_len[i];
    vn++;
  }
  return vn;
}