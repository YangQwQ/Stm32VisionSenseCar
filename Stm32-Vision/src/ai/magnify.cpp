#include "src/ai/magnify.h"

#include <Arduino.h>
#include <img_converters.h>   // fmt2jpg_cb
#include <esp_heap_caps.h>    // heap_caps_malloc(MALLOC_CAP_SPIRAM)
#include "src/cam/camera.h"   // cam::lock_jpeg_dec/unlock_jpeg_dec：Tjpgd 非线程安全, 编解码须经共享锁
#include "src/core/board_log.h"   // blog::logf(AI, ...)
#include <string.h>
#include "rom/tjpgd.h"        // TJpgDec: jd_prepare/jd_decomp —— 部分解码(只取中央)不撼全幅 RGB

namespace magnify {

static int s_cost_ms = 0;   // 上次放大镜总耗时(解码+重编码)，供调用方日志诊断用

int last_cost_ms() { return s_cost_ms; }

// 编码输出回调: 直接写进调用方缓冲(省一次 malloc+拷贝)。`index` 语义本地无源码可查, 两种约定
// 都兼容(仅当 index==已写长度时按 index 写, 否则追加); 缓冲不够返回 0 并置 over。
struct Out { uint8_t* buf; size_t cap; size_t written; bool over; };

static size_t jpg_cb(void* arg, size_t index, const void* data, size_t len) {
  Out* o = (Out*)arg;
  // `!data` 是 jpge 收尾的 put_buf(NULL,0), 不是错误/溢出(上游契约: pBuf 空 ⇒ 收尾返回 true)。
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

// ---------------- TJpgDec 部分解码裁中央 ----------------
// 逐 MCU 解码, 回调只把落在中央带的像素拷进 PSRAM; workbuf 亦走 PSRAM, 不碰内部堆/DMA。
namespace {

// 解码会话上下文：源 JPEG 字节流 / 中央目标区（像素，源坐标系）/ 输出 RGB 缓冲的布局。
struct CropCtx {
  const uint8_t* src; size_t len; size_t pos;   // TJpgDec 输入流（内存源）
  int src_w, src_h;                             // 整幅尺寸
  int cx0, cy0, cw, ch;                         // 中央带在源坐标系里的像素矩形（左/上/宽/高）
  uint8_t* rgb;                                 // 中央带 RGB888 缓冲（cw*ch*3，PSRAM）
};

// TJpgDec 流输入回调：把源缓冲字节喂给解码器。
static UINT jpg_in(JDEC* jd, BYTE* dst, UINT n) {
  CropCtx* c = (CropCtx*)jd->device;
  size_t avail = c->len - c->pos;
  if (n > avail) n = (UINT)avail;
  if (dst) memcpy(dst, c->src + c->pos, n);
  c->pos += n;
  return n;
}
// 注意: c->pos 是累计游标, 初始为 0; jd_prepare 先探头部、jd_decomp 从头读, 二者顺序调用共用同一游标。

// TJpgDec 输出回调: 每个 MCU 矩形回调一次, 只把落在中央带的像素拷进 rgb。
// 关键: bitmap 是完整 MCU 块的 RGB888, 行距 = 块宽×3(块宽 = jd->msx*8); 拿 rect 宽当行距会错位。
static UINT jpg_out(JDEC* jd, void* bitmap, JRECT* rect) {
  CropCtx* c = (CropCtx*)jd->device;
  const uint8_t* blk = (const uint8_t*)bitmap;
  const int blk_w = ((int)jd->msx << 3);          // 块宽 = MCU 横向块数 × 8
  const int bx0 = rect->left, bx1 = rect->right + 1, by0 = rect->top, by1 = rect->bottom + 1;
  // 该块与中央带的交集（源坐标）。
  const int oy0 = by0 > c->cy0 ? by0 : c->cy0;
  const int oy1 = by1 < c->cy0 + c->ch ? by1 : c->cy0 + c->ch;
  const int ox0 = bx0 > c->cx0 ? bx0 : c->cx0;
  const int ox1 = bx1 < c->cx0 + c->cw ? bx1 : c->cx0 + c->cw;
  if (oy0 >= oy1 || ox0 >= ox1) return 1;   // 该块全在中央带外，丢弃
  const int span = ox1 - ox0;
  for (int y = oy0; y < oy1; y++) {
    const uint8_t* s = blk + ((size_t)(y - by0) * blk_w + (ox0 - bx0)) * 3;
    uint8_t* d = c->rgb + ((size_t)(y - c->cy0) * c->cw + (ox0 - c->cx0)) * 3;
    // TJpgDec JDCS_RGB 输出的三元组内部是 B,G,R, 逐像素首尾对调回 R,G,B(否则整幅 R/B 互换)。
    // ⚠️ 索引必须同源: d/s 都是字节址, 像素 p 首字节 = p*3, 混用会逐列错位成竖条纹。
    for (int p = 0; p < span; p++) {
      d[p * 3 + 0] = s[p * 3 + 2];
      d[p * 3 + 1] = s[p * 3 + 1];
      d[p * 3 + 2] = s[p * 3 + 0];
    }
  }
  return 1;   // 非 0 = 继续
}

}  // namespace

bool crop_center_jpg(const uint8_t* jpg, size_t len, int src_w, int src_h,
                     uint8_t* out, size_t cap, size_t* out_len,
                     int out_w, int out_h, int quality) {
  s_cost_ms = 0;   // 必须在入口清零: 失败分支不更新它, 留着上次的值会误读耗时
  blog::logf(blog::AI, "[放大镜] crop_center: entry len=%u 幅=%dx%d cap=%u %dx%d", (unsigned)len, src_w, src_h, (unsigned)cap, out_w, out_h);
  if (out_len) *out_len = 0;
  if (!jpg || len == 0 || !out || cap == 0 || out_w <= 0 || out_h <= 0 || src_w <= 0 || src_h <= 0) {
    return false;
  }
  unsigned long t0 = millis();
  // 中央带 = (0.25,0.25)-(0.75,0.75)。取整到像素。
  const int cx0 = src_w / 4, cy0 = src_h / 4;
  const int cw_c = src_w / 2, ch_c = src_h / 2;   // 中央带在源里的宽高
  const size_t rgb_needed = (size_t)cw_c * ch_c * 3;
  uint8_t* rgb = (uint8_t*)heap_caps_malloc(rgb_needed, MALLOC_CAP_SPIRAM);
  if (!rgb) { blog::logf(blog::AI, "[放大镜] crop_center: 中央带缓冲分配失败(%uB)", (unsigned)rgb_needed); return false; }
  memset(rgb, 0, rgb_needed);

  CropCtx ctx = { jpg, len, 0, src_w, src_h, cx0, cy0, cw_c, ch_c, rgb };
  JDEC jdec;
  // TJpgDec workbuf：ROM 版固定 JD_FASTDECODE=0(基础优化)，仅需约 3.1KB；给 16KB 留余量。
  // 全部 PSRAM，不碰内部堆。
  static uint8_t* s_work = nullptr; static size_t s_work_cap = 0;
  if (!ensure(&s_work, &s_work_cap, 16 * 1024)) { heap_caps_free(rgb); return false; }

  cam::lock_jpeg_dec();   // TJpgDec 非线程安全，与 AI 其它软解串行
  unsigned long t_prep = millis();
  JRESULT r1 = jd_prepare(&jdec, jpg_in, s_work, s_work_cap, &ctx);
  blog::logf(blog::AI, "[放大镜] crop_center: prepare=%d(%llums) 幅=%dx%d 池=%uB", (int)r1,
             (unsigned long long)(millis() - t_prep), src_w, src_h, (unsigned)s_work_cap);
  JRESULT r = (r1 == JDR_OK) ? jd_decomp(&jdec, jpg_out, 0) : r1;
  cam::unlock_jpeg_dec();
  blog::logf(blog::AI, "[放大镜] crop_center: decomp=%d(%llums)", (int)r,
             (unsigned long long)(millis() - t_prep));
  if (r != JDR_OK) {
    blog::logf(blog::AI, "[放大镜] crop_center: 解码失败(prepare=%d decomp=%d)", (int)r1, (int)r);
    heap_caps_free(rgb);
    return false;
  }

  // 把中央带 RGB 重编码为 out_w×out_h JPEG。
  Out o = { out, cap, 0, false };
  unsigned long t_enc = millis();
  cam::lock_jpeg_dec();
  bool ok = fmt2jpg_cb(rgb, rgb_needed, (uint16_t)cw_c, (uint16_t)ch_c,
                       PIXFORMAT_RGB888, quality, jpg_cb, &o);
  cam::unlock_jpeg_dec();
  blog::logf(blog::AI, "[放大镜] crop_center: encode=%d(%llums)", (int)ok,
             (unsigned long long)(millis() - t_enc));
  heap_caps_free(rgb);
  if (!ok || o.over || o.written == 0) return false;
  if (out_len) *out_len = o.written;
  s_cost_ms = (int)(millis() - t0);
  return true;
}

}  // namespace magnify