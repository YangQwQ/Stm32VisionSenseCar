#include "src/core/heap_watch.h"
#include "src/core/board_log.h"

#include <Arduino.h>          // millis()（经 board_log.h 也能拿到，显式写出防将来重构断链）
#include <esp_heap_caps.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr uint32_t kSampleMs   = 20;      // 采样周期：够密，开销可忽略
constexpr int      kTaskStack  = 3072;    // 只做几次 heap_caps 查询，栈要得很少
constexpr uint32_t kDangerByte = 4096;    // DMA 块跌破此值 ⇒ 1600B 的 RX 缓冲已在悬崖边
constexpr uint32_t kWarnGapMs  = 2000;    // 告警限流

volatile uint32_t s_min_free = UINT32_MAX;
volatile uint32_t s_min_dma  = UINT32_MAX;
portMUX_TYPE      s_mux      = portMUX_INITIALIZER_UNLOCKED;

StackType_t*  s_stack = nullptr;
StaticTask_t  s_tcb;
TaskHandle_t  s_task  = nullptr;

inline uint32_t dma_block() {
  // RX 缓冲要 DMA 可达，所以闸门是"内部 ∩ DMA 可达"的最大连续块
  return heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
}

inline uint32_t internal_free() {
  return heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
}

void sampler(void*) {
  uint32_t last_warn = 0;
  for (;;) {
    const uint32_t f = internal_free();
    const uint32_t d = dma_block();

    portENTER_CRITICAL(&s_mux);
    if (f < s_min_free) s_min_free = f;
    if (d < s_min_dma)  s_min_dma  = d;
    portEXIT_CRITICAL(&s_mux);

    // 跌破危险线就告警（限流）：这一刻起 WiFi 随时可能分不到 RX 缓冲
    const uint32_t now = (uint32_t)millis();
    if (d < kDangerByte && (uint32_t)(now - last_warn) >= kWarnGapMs) {
      last_warn = now;
      blog::logf(blog::NET, "[水位] 内部DMA块告急：最大=%uB（阈值 %uB）内部空闲=%uk 累计最低=%uB",
                 (unsigned)d, (unsigned)kDangerByte, (unsigned)(f / 1024),
                 (unsigned)s_min_dma);
    }
    vTaskDelay(pdMS_TO_TICKS(kSampleMs));
  }
}

}  // namespace

void hwatch::init() {
  if (s_task) return;
  // 栈放 PSRAM：内部堆紧到 16KB 栈都可能分配失败（见 motion_verify 的实测），
  // TCB 必须留内部 RAM（FreeRTOS 断言）。
  if (!s_stack) s_stack = (StackType_t*)heap_caps_malloc(kTaskStack, MALLOC_CAP_SPIRAM);
  if (!s_stack) {
    blog::logf(blog::NET, "水位哨兵：PSRAM 栈分配失败，未启动");
    return;
  }
  s_task = xTaskCreateStaticPinnedToCore(sampler, "hwatch", kTaskStack, nullptr, 1,
                                         s_stack, &s_tcb, 1);
  blog::logf(blog::NET, "水位哨兵就绪（%ums 采样 / PSRAM 栈 %d）", (unsigned)kSampleMs, kTaskStack);
}

void hwatch::reset_min() {
  portENTER_CRITICAL(&s_mux);
  s_min_free = UINT32_MAX;
  s_min_dma  = UINT32_MAX;
  portEXIT_CRITICAL(&s_mux);
}

// 未启动（PSRAM 栈分配失败）时返回 0，而不是 UINT32_MAX：后者被体征行除成 4194303k，
// 看着像个天文数字的"安全水位"，恰好把"哨兵没起来"这件事伪装成"一切正常"。
// ⚠️ 但 0 本身仍有两义：**真触底**（确实一次都没凑出连续块）与**没测到**。两者严重性天差地别，
// 而体征行只把它除以 1024 印成 `0k`，读的人无从分辨——我自己就为此多绕了一圈推理。
// 所以调用方（command.cpp 的体征行）印之前必须先问 `ready()`，未起来时印 `--`。
uint32_t hwatch::min_free_internal() { return s_task ? s_min_free : 0; }
uint32_t hwatch::min_largest_dma()   { return s_task ? s_min_dma  : 0; }
// 当前值：**不走 s_task**（采样任务是 20ms 一次的，查的是最近一刻的存量），直接现查。
// 存在的理由：告警文案里的"当前最大块"此前只印在串口，网络侧（体征行）只有谷底 `DMA块最低`。
// 于是判"这个池是不是**结构性**贴着底"（当前就很小 ⇒ 与任务无关，是布局问题）还是
// "被某一轮任务压下去的"（当前正常、谷底为 0）时，手里只有一半数据 —— 这两种结论的处置完全相反。
uint32_t hwatch::cur_largest_dma()   { return dma_block(); }
uint32_t hwatch::cur_free_internal() { return internal_free(); }
bool     hwatch::ready()             { return s_task != nullptr; }

void hwatch::report(const char* tag) {
  blog::logf(blog::NET, "[水位] %s 最低：内部=%uk DMA块=%uk ｜ 当前：内部=%uk DMA块=%uk",
             tag ? tag : "",
             (unsigned)(s_min_free / 1024), (unsigned)(s_min_dma / 1024),
             (unsigned)(internal_free() / 1024), (unsigned)(dma_block() / 1024));
}
