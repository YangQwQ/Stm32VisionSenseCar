#include "src/ai/magnify.h"

#include <Arduino.h>
#include <img_converters.h>   // fmt2rgb888 / fmt2jpg_cb
#include <esp_heap_caps.h>    // heap_caps_malloc(MALLOC_CAP_SPIRAM)
#include "src/cam/camera.h"   // cam::lock_jpeg_dec/unlock_jpeg_dec：Tjpgd 非线程安全, 软解须经共享锁
#include "src/core/board_log.h"   // blog::logf(AI, ...)
#include "freertos/FreeRTOS.h"    // 解码失败重试要让一拍
#include "freertos/task.h"
#include <math.h>
#include <string.h>

namespace magnify {

// 解码档: **全分辨率**(1/1)。不要退回 1/2 —— 1/2 解码后再裁再放大, 放大的是一张"已经丢掉 3/4
// 像素"的图: 2× 档下可用的原生像素只有 1/4, 画面看着大了, 其实全是插值糊出来的, 该看不清的还是
// 看不清。全分辨率解码后裁出的 320×240 在 2× 档正好是原生像素 1:1, 细节是真的。
// 代价是缓冲与解码时间约 ×4(全在 PSRAM, 见下面的注释), 相对 AI 一轮几秒可忽略。
// ⚠️ 解码现在走 fmt2rgb888, 它**内部固定传 JPEG_IMAGE_SCALE_0**(to_bmp.c:178), 不接 scale 参数
// ⇒ 下面这两个常量只用于尺寸推导与日志, 不再真的喂给解码器。它们的取值(1/1→shift 0)必须与之一致。
static constexpr auto DECODE_SCALE = JPG_SCALE_NONE;   // = JPEG_IMAGE_SCALE_0, 即 1/1 全分辨率
                                                       // (auto 是因为该枚举的类型名在 C++ 侧不可见)
static constexpr int DECODE_SHIFT = 0;   // 必须与 DECODE_SCALE 一致: 1/1→0, 1/2→1
// 解码尺寸上界(像素数，不是边长): 只为挡住离谱分配。别拿它截 w/h —— 解码器按**源帧**右移
// DECODE_SHIFT 写，截了只护住后面的读、护不住它的写。所以只做"整体放弃"。留到 800×600: 摄像头
// 有 SVGA 回退档(camera.cpp), 上界得罩得住它。
static constexpr size_t MAX_PIX = 800 * 600;
// 裁框最小边长(解码像素): 再小就没有可放大的信息了(纯马赛克)。
static constexpr int MIN_SIDE = 12;
// 解码失败重试(见 crop_to_jpg 里的说明): 次数 / 每次间隔 / 总预算 ms。
static constexpr int DEC_RETRY_N         = 4;
static constexpr int DEC_RETRY_GAP_MS    = 15;
static constexpr int DEC_RETRY_BUDGET_MS = 200;

static uint8_t* s_src = nullptr; static size_t s_src_cap = 0;   // 解码缓冲(RGB888: 3 字节/像素)
static uint8_t* s_dst = nullptr; static size_t s_dst_cap = 0;   // 放大缓冲(RGB888: 3 字节/像素)
// 两块缓冲首次调用后**一直留着**(全分辨率 RGB888 下 s_src≈900KB + s_dst≈230KB, 占 PSRAM 约 14%):
// 每轮释放再申请同样大小反而是在 PSRAM 上反复撕口子，而这条链路对堆碎片极敏感。分辨率变了
// (改摄像头配置)时按需增长。改回 RGB565 则两块各减 1/3。
static int s_cost_ms = 0;
static int s_src_px = 0;

int last_cost_ms() { return s_cost_ms; }
int last_src_px() { return s_src_px; }

// 解码失败现场取证用：那一刻"内部 ∩ DMA 可达"的最大连续块。这是个**待证的怀疑**，不是结论——
// 已知的事实只有三条，都指向"解码器要一块连续内部 RAM，而那一刻没有"：
//   1) 失败是**快失败**：实测 2ms 返回，而正常一次解码要 330ms；
//   2) 同一瞬间 mvfy 的软解也失败（`mvfy move 取帧/软解失败`）—— 两条独立路径共用的只有解码器本身，
//      不是各自的输出缓冲；
//   3) 紧邻的日志里 `[水位] 内部DMA块告急：最大=1076B（阈值 4096B）内部空闲=10k`，即内部堆碎片化到
//      任何 4KB 连续块都拿不出来（heap_watch.h 已记本板"长期贴红线"）。
// 把"头尾标记"和这个块值一起打出来，下一次失败就能一刀切开两种可能：头=FFD8 尾=FFD9 说明字节是好的
// （那就是内存），头尾不对说明从相机拿到的就是半张图（该去查抓帧，别再查内存）。
static uint32_t in_dma_block() {
  return heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
}

// 编码输出回调: 直接写进调用方缓冲(省一次 malloc + 拷贝)。
// `index` 的语义(绝对偏移 / 恒为 0 由回调自己追加)本地无源码可查(预编译库、代理不通)，这里两种
// 都兼容: 只有 index 恰好等于已写长度时按 index 写，否则按追加 —— 无论哪种约定，落地的都是同一段
// 连续字节。缓冲不够时返回 0 并置 over，让 fmt2jpg_cb 自己失败退出。
struct Out { uint8_t* buf; size_t cap; size_t written; bool over; };

static size_t jpg_cb(void* arg, size_t index, const void* data, size_t len) {
  Out* o = (Out*)arg;
  // `!data` 是 jpge 收尾的那一次 `put_buf(NULL, 0)`, 不是错误、更不是溢出:
  // 上游 memory_stream::put_buf 的契约就是"pBuf 为空 ⇒ 收尾返回 true"(见库内实现)。
  // 早先把它和"缓冲不够"合并判成 over, 于是**每一次编码成功都会被判成失败**(written 已写满却报错)。
  if (!data) return 0;
  size_t at = (index == o->written) ? index : o->written;
  if (at + len > o->cap) { o->over = true; return 0; }
  memcpy(o->buf + at, data, len);
  o->written = at + len;
  return len;
}

static bool ensure(uint8_t** p, size_t* cap, size_t need) {
  if (need <= *cap) return true;
  size_t nc = (need + 16383) & ~(size_t)16383;
  uint8_t* nb = (uint8_t*)heap_caps_malloc(nc, MALLOC_CAP_SPIRAM);
  if (!nb) return false;
  if (*p) heap_caps_free(*p);
  *p = nb; *cap = nc;
  return true;
}

// [a,b) 夹进 [0,lim) 并保证至少 MIN_SIDE 宽: 先夹，再以中心为轴撑到 MIN_SIDE，最后再夹一次。
static void fit_span(int& a, int& b, int lim) {
  if (a < 0) a = 0;
  if (b > lim) b = lim;
  if (b - a < MIN_SIDE) {
    int c = (a + b) / 2;
    a = c - MIN_SIDE / 2;
    b = a + MIN_SIDE;
    if (a < 0) { a = 0; b = MIN_SIDE; }
    if (b > lim) { b = lim; a = lim - MIN_SIDE; if (a < 0) a = 0; }
  }
  if (b <= a) b = a + 1;
}

bool crop_to_jpg(const uint8_t* jpg, size_t len, int src_w, int src_h,
                 float x0, float y0, float x1, float y1,
                 uint8_t* out, size_t cap, size_t* out_len,
                 int out_w, int out_h, int quality) {
  if (out_len) *out_len = 0;
  s_src_px = 0;
  s_cost_ms = 0;   // 必须**在入口清零**：下面各失败分支不再更新它，留着上一次的值会让
                   // "生成失败(11ms)"这种日志看起来像"解码到一半就退了"，实测据此误判过一次
                   // (真实原因是早退，那 11ms 是上上次的耗时)。
  if (!jpg || len == 0 || !out || cap == 0 || out_w <= 0 || out_h <= 0) {
    blog::logf(blog::AI, "[放大镜] 入参不合法(jpg=%p len=%u out=%p cap=%u %dx%d)",
               (const void*)jpg, (unsigned)len, (void*)out, (unsigned)cap, out_w, out_h);
    return false;
  }
  const int w = src_w >> DECODE_SHIFT, h = src_h >> DECODE_SHIFT;
  if (w <= 0 || h <= 0 || (size_t)w * h > MAX_PIX) {
    blog::logf(blog::AI, "[放大镜] 源尺寸越界(%dx%d 上限%uB)", w, h, (unsigned)MAX_PIX);
    return false;
  }
  if (!ensure(&s_src, &s_src_cap, (size_t)w * h * 3)) {
    blog::logf(blog::AI, "[放大镜] 解码缓冲分配失败(需%uB 已有%uB 内部块=%uB)",
               (unsigned)((size_t)w * h * 3), (unsigned)s_src_cap, (unsigned)in_dma_block());
    return false;
  }
  if (!ensure(&s_dst, &s_dst_cap, (size_t)out_w * out_h * 3)) {
    blog::logf(blog::AI, "[放大镜] 放大缓冲分配失败(需%uB 已有%uB)",
               (unsigned)((size_t)out_w * out_h * 3), (unsigned)s_dst_cap);
    return false;
  }

  if (x0 > x1) { float t = x0; x0 = x1; x1 = t; }
  if (y0 > y1) { float t = y0; y0 = y1; y1 = t; }
  int cx0 = (int)floorf(x0 * w), cx1 = (int)ceilf(x1 * w);
  int cy0 = (int)floorf(y0 * h), cy1 = (int)ceilf(y1 * h);
  fit_span(cx0, cx1, w);
  fit_span(cy0, cy1, h);
  const int cw = cx1 - cx0, ch = cy1 - cy0;

  unsigned long t0 = millis();
  // Tjpgd/jpge 共用全局静态上下文、都非线程安全，故解码与编码整个持共享锁串行化
  // （camera.h lock_jpeg_dec）。⚠️ 早先这里断言"与 mvfy 并发会踩出坏图（Y 在、Cb/Cr 乱）"，
  // 该机理**已被证伪**：锁确在板上跑，放大帧仍 6/6 全坏（详见 CLAUDE.md「已知问题 1」）。
  // 持锁保留（确实该串行化），但别再把它当坏图的解释。
  cam::lock_jpeg_dec();
  // 失败重试: 解码失败是**快失败**(实测 2ms 就返回, 而正常一次要 330ms), 所以重试几乎不花时间;
  // 而"那块连续内部内存被瞬时占走"(相机抓帧 / mvfy 的软解 / TLS 收尾)是会自己松开的。
  // 总耗时封顶 DEC_RETRY_BUDGET_MS, 免得某次变成"解到底才失败"时被拖住。
  bool dec = false;
  for (int a = 0; a < DEC_RETRY_N && !dec; a++) {
    // 走 fmt2rgb888 而非 jpg2rgb565: 前者输出 RGB888, 绕开 RGB565 输出级(见 CLAUDE.md 里的判别实验)。
    // 它内部固定 JPEG_IMAGE_SCALE_0(1/1), 与 DECODE_SCALE/DECODE_SHIFT 的取值一致, 故尺寸假设不变。
    dec = fmt2rgb888(jpg, len, PIXFORMAT_JPEG, s_src);
    if (dec) break;
    if (a + 1 < DEC_RETRY_N && millis() - t0 < DEC_RETRY_BUDGET_MS) {
      vTaskDelay(pdMS_TO_TICKS(DEC_RETRY_GAP_MS));
    } else {
      break;
    }
  }
  cam::unlock_jpeg_dec();
  if (!dec) {
    s_cost_ms = (int)(millis() - t0);
    const uint8_t h0 = len >= 2 ? jpg[0] : 0, h1 = len >= 2 ? jpg[1] : 0;
    const uint8_t t1 = len >= 2 ? jpg[len - 1] : 0, t2 = len >= 2 ? jpg[len - 2] : 0;
    // 判据: 头=FFD8 且尾=FFD9 ⇒ 这段字节是完整 JPEG, 那失败就只能是解码器拿不到工作区
    // (看内部块值); 头尾不对 ⇒ 是从相机那边拿到的就是半张图, 得去查抓帧而不是查内存。
    blog::logf(blog::AI, "[放大镜] 解码失败(%dms 共试%d次 in=%uB 源%dx%d scale=%d 内部块=%uB 头=%02X%02X 尾%02X%02X)",
               s_cost_ms, DEC_RETRY_N, (unsigned)len, src_w, src_h, (int)DECODE_SCALE,
               (unsigned)in_dma_block(), h0, h1, t2, t1);
    return false;
  }
  s_src_px = w * h;

  // 放大用最近邻: 倍率是整数级(2~8×)，要的是"看清相对位置"，清晰的分块边界比双线性的插值糊更好读，
  // 也省掉一遍浮点乘加。
  for (int y = 0; y < out_h; y++) {
    int sy = cy0 + (int)((long)y * ch / out_h);
    if (sy >= cy1) sy = cy1 - 1;
    const uint8_t* srow = s_src + (size_t)sy * w * 3;   // 3 字节/像素
    uint8_t* drow = s_dst + (size_t)y * out_w * 3;
    for (int x = 0; x < out_w; x++) {
      int sx = cx0 + (int)((long)x * cw / out_w);
      if (sx >= cx1) sx = cx1 - 1;
      const uint8_t* p = srow + (size_t)sx * 3;
      uint8_t* q = drow + (size_t)x * 3;
      // ⚠️ 第一字节与第三字节**必须对调**：fmt2rgb888(JPEG→RGB888) 实际吐的是 B,G,R，而下面
      // fmt2jpg_cb(PIXFORMAT_RGB888) 要的是 R,G,B（to_jpg.cpp:55-62 会把三元组倒序写进 jpge 的
      // BGR 缓冲）。不换的后果是**整幅 R/B 互换**——灰地板看不出来（R=B 是不动点），但黄色方块会
      // 变青色。实测：真值 (255,255,92) → 板出 (100,248,248)。
      q[0] = p[2]; q[1] = p[1]; q[2] = p[0];
    }
  }

  Out o = { out, cap, 0, false };
  // jpge 编码同样是全局静态上下文（与 Tjpgd 同源问题），经同一把 cam 锁串行化。
  cam::lock_jpeg_dec();
  // PIXFORMAT_RGB888 分支把三元组倒序写进 jpge 的 BGR 缓冲(to_jpg.cpp:55-62) ⇒ 源字节须为
  // R,G,B。上面放大循环已把解码器吐出的 B,G,R 换成 R,G,B, 故这里直接配得上。
  bool ok = fmt2jpg_cb(s_dst, (size_t)out_w * out_h * 3, out_w, out_h,
                       PIXFORMAT_RGB888, quality, jpg_cb, &o);
  cam::unlock_jpeg_dec();
  s_cost_ms = (int)(millis() - t0);
  if (!ok || o.over || o.written == 0) {
    // 出不去时把三个量都打出来: over=1 就是 48KB 上限不够(放大图高频多、压不小), 该抬 AI_ZOOM_JPG_MAX
    blog::logf(blog::AI, "[放大镜] 编码失败(%dms ok=%d over=%d written=%u cap=%u %dx%d)",
               s_cost_ms, (int)ok, (int)o.over, (unsigned)o.written, (unsigned)cap, out_w, out_h);
    return false;
  }
  if (out_len) *out_len = o.written;
  return true;
}

}  // namespace magnify
