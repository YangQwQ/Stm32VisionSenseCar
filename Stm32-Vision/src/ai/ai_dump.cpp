#include "src/ai/ai_dump.h"
#include "src/core/board_log.h"   // blog::enabled / blog::logf

#include <string.h>
#include <esp_heap_caps.h>        // MALLOC_CAP_SPIRAM
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// 每槽的帧缓冲**按需增长**（只按实际见过的最大帧分配，不预留满额）：VGA q10 一帧 40~70KB，
// 预留 6×128KB 会白占近 800KB PSRAM。增长按 8KB 步进，避免逐帧 realloc。
#define AI_DUMP_SLOTS 6
#define AI_DUMP_GROW_STEP (8 * 1024)
#define AI_DUMP_NOTE_CAP 96             // 标注上限(字节); 清单每条约 180 字节, 6 条约 1.1KB

namespace ai {

// 槽内元数据 + 帧缓冲(PSRAM)。seq==0 表示该槽"尚未定案"（正在进行的这一轮），对 HTTP 不可见。
static uint8_t* s_buf[AI_DUMP_SLOTS];
static size_t   s_cap[AI_DUMP_SLOTS];   // 已分配的字节数(单调增)
static size_t   s_len[AI_DUMP_SLOTS];
static uint32_t s_seq[AI_DUMP_SLOTS];
static uint32_t s_ms[AI_DUMP_SLOTS];
static bool     s_prev[AI_DUMP_SLOTS];
static char     s_note[AI_DUMP_SLOTS][AI_DUMP_NOTE_CAP];

static int      s_head = 0;      // 最新写入的槽位
static int      s_count = 0;     // 已用槽数(≤ SLOTS)
static bool     s_pending = false;  // 最新槽位尚未补标注 → 同一轮的重复 push 就地覆盖
static uint32_t s_seqn = 0;      // 序号发号器(只在定案时递增)
static SemaphoreHandle_t s_mtx = nullptr;

// 按 UTF-8 边界截断(截半个中文字节会让清单直接不是合法 UTF-8)并去掉会撑破手写 JSON 的字符。
static void note_sanitize(char* dst, size_t cap, const char* src) {
  size_t n = 0;
  for (const unsigned char* p = (const unsigned char*)src; *p && n + 1 < cap; p++) {
    unsigned char c = *p;
    if (c < 0x20) continue;                      // 控制字符(含 \n)直接丢, 清单是一行 JSON
    if (c == '"' || c == '\\') c = '\'';         // JSON 元字符换成等价可读字符
    if (c < 0x80) { dst[n++] = (char)c; continue; }
    // 多字节字符: 先确认整段都在、且放得下, 再整体拷入 —— 绝不落下半截 UTF-8
    int need = (c >= 0xF0) ? 4 : (c >= 0xE0) ? 3 : (c >= 0xC0) ? 2 : 0;
    if (need == 0) continue;                     // 孤立续接字节(上游数据已损坏): 丢
    size_t avail = 1;
    while (avail < (size_t)need && p[avail]) avail++;
    if (avail < (size_t)need || n + (size_t)need >= cap) break;
    for (int k = 0; k < need; k++) dst[n++] = (char)p[k];
    p += need - 1;
  }
  dst[n] = 0;
}

// 关掉留档(或首次 push 时的空表) → 把 PSRAM 全还给堆。调用方须持锁。
static void release_all(void) {
  for (int i = 0; i < AI_DUMP_SLOTS; i++) {
    if (s_buf[i]) { heap_caps_free(s_buf[i]); s_buf[i] = nullptr; }
    s_cap[i] = s_len[i] = 0; s_seq[i] = 0; s_note[i][0] = 0; s_prev[i] = false;
  }
  s_head = 0; s_count = 0; s_pending = false;
}

void dump_push(const uint8_t* img, size_t n, bool with_prev) {
  // 留档只在 blog 的 AI 类别开启时进行(/log ai on): 关着时一次也不 memcpy。
  if (!blog::enabled(blog::AI)) {
    if (s_mtx && s_count > 0) {
      if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(200)) == pdTRUE) { release_all(); xSemaphoreGive(s_mtx); }
    }
    return;
  }
  if (!s_mtx) {
    s_mtx = xSemaphoreCreateMutex();
    if (!s_mtx) return;
  }
  if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(500)) != pdTRUE) return;

  int idx;
  if (s_pending && s_count > 0) {
    idx = s_head;                     // 同一轮的重复 push(组包重试): 就地覆盖, 不新占槽
  } else {
    idx = (s_count == 0) ? 0 : (s_head + 1) % AI_DUMP_SLOTS;
    if (s_count < AI_DUMP_SLOTS) s_count++;
    s_head = idx;
    s_seq[idx] = 0;                   // 定案前不可见
    s_note[idx][0] = 0;
    s_pending = true;
    if (s_count == 1) blog::logf(blog::AI, "[ai] 抓帧留档开始(%d 槽, /log ai off 即释放)", AI_DUMP_SLOTS);
  }

  if (img && n > 0 && n <= AI_DUMP_FRAME_MAX) {
    if (s_cap[idx] < n) {
      size_t want = ((n + AI_DUMP_GROW_STEP - 1) / AI_DUMP_GROW_STEP) * AI_DUMP_GROW_STEP;
      uint8_t* p = (uint8_t*)heap_caps_realloc(s_buf[idx], want, MALLOC_CAP_SPIRAM);
      if (!p) {                          // PSRAM 紧张: 本轮不留画面(不影响 AI 链路本身)
        s_len[idx] = 0; s_ms[idx] = (uint32_t)millis(); s_prev[idx] = with_prev;
        xSemaphoreGive(s_mtx);
        return;
      }
      s_buf[idx] = p; s_cap[idx] = want;
    }
    memcpy(s_buf[idx], img, n);
    s_len[idx] = n;
  } else {
    s_len[idx] = 0;                      // 无画面/过大: 留一条空记录, 便于看出"某轮是瞎决策的"
  }
  s_ms[idx] = (uint32_t)millis();
  s_prev[idx] = with_prev;
  xSemaphoreGive(s_mtx);
}

void dump_note(const char* note) {
  if (!s_mtx || !note) return;
  if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(500)) != pdTRUE) return;
  if (s_pending && s_count > 0) {
    note_sanitize(s_note[s_head], sizeof(s_note[s_head]), note);
    s_seq[s_head] = ++s_seqn;            // 定案: 此刻起 HTTP 可见, 且序号单调递增供抓取端去重
    s_pending = false;
  }
  xSemaphoreGive(s_mtx);
}

// 第 i 新(0=最新)的槽位下标
static inline int slot_at(int i) { return (s_head - i + AI_DUMP_SLOTS * 2) % AI_DUMP_SLOTS; }

bool dump_manifest(char* buf, size_t cap) {
  if (!buf || cap < 64) return false;
  snprintf(buf, cap, "{\"slots\":%d,\"n\":0,\"frames\":[]}", AI_DUMP_SLOTS);   // 先摆一份合法空清单
  if (!s_mtx) return true;              // 从没留过档 → 就是空的, 不是失败
  if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(500)) != pdTRUE) return false;

  int n = 0;   // 只数"已定案"的槽: 正在进行的那一轮标注还没落, 取回去跟画面配不上号
  for (int i = 0; i < s_count; i++) if (s_seq[slot_at(i)]) n++;

  size_t p = snprintf(buf, cap, "{\"slots\":%d,\"n\":%d,\"frames\":[", AI_DUMP_SLOTS, n);
  bool ok = true;
  int w = 0;
  for (int i = 0; i < s_count; i++) {
    int idx = slot_at(i);
    if (!s_seq[idx]) continue;
    if (p + 220 > cap) { ok = false; break; }   // 宁可整份作废, 也不留半截非法 JSON 给抓取端
    p += snprintf(buf + p, cap - p, "%s{\"seq\":%u,\"ms\":%u,\"len\":%u,\"prev\":%s,\"note\":\"%s\"}",
                  w++ ? "," : "", (unsigned)s_seq[idx], (unsigned)s_ms[idx],
                  (unsigned)s_len[idx], s_prev[idx] ? "true" : "false", s_note[idx]);
  }
  if (ok && p + 3 <= cap) { buf[p++] = ']'; buf[p++] = '}'; buf[p] = 0; }
  else ok = false;
  xSemaphoreGive(s_mtx);

  if (!ok) snprintf(buf, cap, "{\"slots\":%d,\"n\":0,\"frames\":[]}", AI_DUMP_SLOTS);
  return ok;
}

size_t dump_copy(uint32_t seq, uint8_t* dst, size_t cap, uint32_t* ms) {
  if (!dst || cap == 0 || seq == 0 || !s_mtx) return 0;
  size_t out = 0;
  if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(500)) == pdTRUE) {
    for (int i = 0; i < s_count; i++) {
      int idx = slot_at(i);
      if (s_seq[idx] != seq) continue;
      if (s_len[idx] > 0 && s_len[idx] <= cap) { memcpy(dst, s_buf[idx], s_len[idx]); out = s_len[idx]; }
      if (ms) *ms = s_ms[idx];
      break;
    }
    xSemaphoreGive(s_mtx);
  }
  return out;
}

uint32_t dump_newest_seq(void) {
  if (!s_mtx) return 0;
  uint32_t out = 0;
  // 取锁超时也回 0: 调用方(长轮询)把"没拿到"与"还没有新帧"一样当作"再等等" —— 短暂超时
  // 下一拍就会重问, 不会漏帧; 若在这里阻塞等待, 反倒是把 httpd 会话钉在 AI 线程的锁上。
  if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(200)) == pdTRUE) {
    // 扫最大值而不是只看 s_head: 正在进行的这一轮 seq 还是 0(未定案), 不在扫描范围内;
    // 环形滚动后 s_head 也未必是序号最大的那个槽。
    for (int i = 0; i < s_count; i++) {
      uint32_t s = s_seq[slot_at(i)];
      if (s > out) out = s;
    }
    xSemaphoreGive(s_mtx);
  }
  return out;
}

}  // namespace ai
