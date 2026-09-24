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

// ---------------- TJpgDec 部分解码裁中央 ----------------
// 把整幅 JPEG 用 TJpgDec 逐 MCU 解码，out_func 回调里**只把落在中央区域的 MCU 像素**拷进 PSRAM 的
// 中央带 RGB 缓冲，其余丢弃。熵解码仍需整幅串行跑完（省内存不省 CPU），但 PSRAM 只划中央带那么大，
// 且 workbuf 全走 MALLOC_CAP_SPIRAM —— 不碰内部堆/DMA，规避 fmt2rgb888 软解整幅吃内部 RAM 的卡死。
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
// 注意: jpg_in 里 c->pos 是累计的, 但 jd_prepare 会先探测头部、jd_decomp 从头读 —— pos 必须从 0 起。
// 上面把 pos 偏移写在 c 里, 初始化时 pos=0 即可, prepare 与 decomp 共用同一游标(它们顺序调用)。

// TJpgDec 输出回调：每个 MCU 矩形回调一次，只把落在中央带的像素拷进 rgb。
// 关键：bitmap 是**完整 MCU 块**的 RGB888 像素，其行距 = 块宽×3，块宽 = jd->msx*8（YUV420 下 MCU=16×16）。
// 不能拿 rect 的宽当行距 —— 图像边缘的块被截断时 rect 宽 < 块宽，按 rect 宽读会错位(内容错)。
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
    // TJpgDec JDCS_RGB 输出的三元组内部是 B,G,R（与 fmt2rgb888 一致，产品已实测定标），逐像素把
    // 首尾对调回 R,G,B —— 否则整幅 R/B 互换：灰地看不出（R≈B 不动点），但青色方块变蓝、黄变青。
    // ⚠️ 三种索引必须同源：d/s 都是字节址，像素 p 的首字节 = p*3。展开版若把 d 当像素索引写
    // d[i]=s[i+2] 而 s 按字节走，就逐列错位成竖条纹（实测）。统一按 p*3 走最不易错。
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
  s_cost_ms = 0;   // 必须在入口清零：失败分支不再更新它，留着上次的值会让"失败(11ms)"误读成"解到一半就退"
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

  cam::lock_jpeg_dec();   // TJpgDec 非线程安全，与 mvfy/AI 其它软解串行
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