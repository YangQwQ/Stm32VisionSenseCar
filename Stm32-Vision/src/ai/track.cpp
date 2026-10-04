#include "src/ai/track.h"

#include <Arduino.h>          // millis
#include <esp_heap_caps.h>    // heap_caps_malloc(MALLOC_CAP_SPIRAM)
#include <math.h>             // sqrtf / lroundf
#include <string.h>           // memcpy / memset / strncpy
#include <stdlib.h>           // abs（魔棒区域生长的色度比较）
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "src/core/board_log.h"
#include "src/cam/camera.h"   // cam::set_track_mode：跟踪起停切 RGB565/JPEG（幂等）
#include "src/ai/dcf.h"       // 相关滤波跟踪内核（取代原来的穷举 ZNCC 搜索）
#include "Calibration.h"      // TRACK_*

namespace track {

// ============================ PSRAM 按需增长分配 ============================
// 抄 magnify::ensure：16KB 对齐增长，只按实际见过的最大需求分配，全走 PSRAM（不碰内部堆/DMA 块）。
static bool ensure(uint8_t** p, size_t* cap, size_t need) {
  if (need <= *cap) return true;
  size_t nc = (need + 16383) & ~(size_t)16383;
  uint8_t* nb = (uint8_t*)heap_caps_malloc(nc, MALLOC_CAP_SPIRAM);
  if (!nb) return false;
  if (*p) heap_caps_free(*p);
  *p = nb; *cap = nc;
  return true;
}

// ============================ RGB565 → luma / chroma（带抽点降采样） ============================
// 逐像素读 RGB565 帧缓冲（本板为 BE：高字节在前 = gggrrrrr 0 gggbbbbb），算
//   luma   = (77R + 150G + 29B) >> 8
//   chroma = max(R,G,B) - min(R,G,B)   ← 彩色小块在灰地面/灰机械臂上远高于亮度对比
// scale>0 时按 2^scale 抽点（不做均值），输出 gw = ceil(W/2^s) × gh = ceil(H/2^s)。
static void read_planes(const uint8_t* rgb, int W, int H, int scale,
                        uint8_t* lum, uint8_t* chr, int gw) {
  const int step = 1 << scale;
  for (int y = 0, oy = 0; y < H; y += step, oy++) {
    const uint8_t* row = rgb + (size_t)y * W * 2;
    uint8_t* dl = lum ? lum + (size_t)oy * gw : nullptr;
    uint8_t* dc = chr ? chr + (size_t)oy * gw : nullptr;
    for (int x = 0, ox = 0; x < W; x += step, ox++) {
      const uint16_t px = (uint16_t)(((uint16_t)row[x * 2] << 8) | row[x * 2 + 1]);
      int r = (px >> 11) & 0x1F; r = (r << 3) | (r >> 2);   // 5bit → 8bit
      int g = (px >> 5) & 0x3F;  g = (g << 2) | (g >> 4);   // 6bit → 8bit
      int b = px & 0x1F;         b = (b << 3) | (b >> 2);   // 5bit → 8bit
      if (dl) dl[ox] = (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
      if (dc) {
        int mx = r > g ? r : g; if (b > mx) mx = b;
        int mn = r < g ? r : g; if (b < mn) mn = b;
        dc[ox] = (uint8_t)(mx - mn);
      }
    }
  }
}

static int s_decode_ms = 0;
static int s_track_ms = 0;
// 分段耗时(us)与热点计数（诊断）：定位"搜索变慢"慢在哪一段。
static int s_us_disc = 0, s_us_search = 0;   // 分段耗时(us)：准备 / 搜索（诊断）
static float s_last_scale = 1.0f;   // 上次搜索胜出的倍率（诊断）
static float s_last_obj = 0.0f;     // 上帧用的目标边长(px)（诊断，转发自 dcf）
static float s_last_off = 0.0f;     // 上帧响应峰偏移/半径（诊断，转发自 dcf）
static float s_last_appear = 0.0f;  // 上帧与**起跟外观**的 NCC（诊断，转发自 dcf）
// 合爪自检探针（见 track.h）：两个点，下一次 update 顺便算它们的外观分
static volatile bool s_probe_on = false;
static float s_p0u = 0, s_p0v = 0, s_p1u = 0, s_p1v = 0;
static volatile float s_pa0 = 0, s_pa1 = 0;
static volatile unsigned long s_last_upd_ms = 0;   // 上次 update 完成时刻（夹取闭环判断"动作后的新位置"用）
static volatile unsigned long s_last_cap_ms = 0;   // 上次处理的帧的拍摄时刻（update 入口 stamp）：结果到达时刻
                                                   // 含 ~280ms 解码+搜索延迟，判"画面是否动作后拍的"得以此为准
static volatile bool s_last_ok = false;            // 最近一帧是否被采纳（PSR 达标）
static int s_wide_left = 0;                        // "预期宽窗"剩余帧数（hint_wide 设置，用完即回正常窗）

// 内部：从一帧一次抽出降采样 luma 与 chroma（任一可空）。
// 返回 0 成功；-1 参数非法；-2 输出缓冲不够；-4 帧不是 RGB565（如高清 JPEG 快照）。
static int decode_dual(const camera_fb_t* fb, int scale, uint8_t* lum, uint8_t* chr,
                       int out_cap, int* gw, int* gh) {
  s_decode_ms = 0;
  if (!fb || !fb->buf || (!lum && !chr) || scale < 0 || scale > 3 ||
      fb->width <= 0 || fb->height <= 0) return -1;
  if (fb->format == PIXFORMAT_JPEG) return -4;
  const int W = (int)fb->width, H = (int)fb->height;
  const int g_w = (W + (1 << scale) - 1) >> scale;
  const int g_h = (H + (1 << scale) - 1) >> scale;
  if (gw) *gw = g_w;
  if (gh) *gh = g_h;
  if ((long)g_w * g_h > out_cap) return -2;
  const unsigned long t0 = millis();
  read_planes(fb->buf, W, H, scale, lum, chr, g_w);
  s_decode_ms = (int)(millis() - t0);
  return 0;
}

int decode_gray(const camera_fb_t* fb, int scale, uint8_t* out, int out_cap, int* gw, int* gh) {
  return decode_dual(fb, scale, out, nullptr, out_cap, gw, gh);
}

int decode_chroma(const camera_fb_t* fb, int scale, uint8_t* out, int out_cap, int* gw, int* gh) {
  return decode_dual(fb, scale, nullptr, out, out_cap, gw, gh);
}

int last_decode_ms() { return s_decode_ms; }
int last_track_ms() { return s_track_ms; }

// 分段耗时(us)：取窗/加窗准备 + 相关滤波搜索。旧 ZNCC 时代的 coarse/refine/zncc/pos 已随其代码删除。
void last_phases_us(int* prep, int* search) {
  if (prep) *prep = s_us_disc;
  if (search) *search = s_us_search;
}

const char* state_name(State s) {
  switch (s) {
    case State::Idle:     return "idle";
    case State::Locking:  return "lock";
    case State::Tracking: return "track";
    case State::Lost:     return "lost";
  }
  return "?";
}

// ============================ 跟踪器 ============================
// 定位由 dcf（相关滤波）负责：整幅响应图一次 FFT 算出，自带 PSR 置信度，在线自适应目标外观。
namespace {

struct Tgt {
  State st = State::Idle;
  char  name[48] = {0};
  float u = 0, v = 0, conf = 0;
  int   miss = 0;                                   // 连续低分帧数
  int   frames = 0;                                 // 自锁定以来帧数
  float su = 0, sv = 0, sw = 0, sh = 0;             // 种子（归一化）
  // 精度改进：指数平滑 + 运动预测
  float smooth_u = 0, smooth_v = 0;                 // 平滑后位置（归一化，输出给控制闭环）
  float raw_u = 0, raw_v = 0;                       // 未平滑亚像素中心（手机叠加显示用，无拖尾）
  float alpha = TRACK_SMOOTH_ALPHA;                 // 平滑系数
  float pred_u = 0, pred_v = 0;                     // 上一帧→本帧的预测位移（归一化，用于门控搜索窗）
  bool  has_pred = false;                           // 第二帧起才有预测
};

Tgt s_t;
static bool s_reacq = true;   // 跟丢后是否允许宽搜重捕（grasp 里关掉，见 track.h）
static int  s_scale = TRACK_SCALE;   // 本次锁定的解码抽点（seed 时按目标大小自适应，见 seed）
// 追踪状态互斥：update/seed/stop 会改 s_t 与全局像素缓冲（s_lum/s_chr…），
// 而 update 现在可能被控制线程（grasp/AI）与图传线程（light 刷新）并发调用 —— 必须串行。
// light 路径用 try-lock（拿不到即跳过），控制路径阻塞取锁。
static SemaphoreHandle_t s_mtx = nullptr;
static void ensure_mtx() { if (!s_mtx) s_mtx = xSemaphoreCreateMutex(); }

// 取追踪互斥（**带超时**）：拿不到就记一条限频日志并返回 false。
// 真机上"整条闭环无声僵死"最难查，用超时把死等变成"日志里点名 + 调用方自愈"，而不是一起僵在 portMAX_DELAY。
static bool take_mtx(uint32_t ms) {
  if (!s_mtx) return true;   // 极端兜底（创建失败）：无锁运行
  if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(ms)) == pdTRUE) return true;
  static uint32_t s_warn_ms = 0;
  const uint32_t now = millis();
  if ((uint32_t)(now - s_warn_ms) > 2000) {
    s_warn_ms = now;
    blog::logf(blog::AI, "[track] 等追踪锁超时(%ums)：有任务长期持锁(上一次 update 卡住了?)", (unsigned)ms);
  }
  return false;
}
uint8_t* s_lum = nullptr;    size_t s_lum_cap = 0;     // 全幅 luma
uint8_t* s_chr = nullptr;    size_t s_chr_cap = 0;     // 全幅 chroma(max-min)
uint8_t* s_tmp = nullptr;    size_t s_tmp_cap = 0;     // 模板暂存（先算统计、达标才采用）
// （判别图/粗层/多尺度模板暂存等 ZNCC 时代的缓冲已随那套代码删除；现在只有 luma + chroma 两个平面。）

// 由归一化框算出全幅模板边长，钳到 [TW_MIN, TW_MAX] 且不超图。
void tmpl_dims(int gw, int gh, float wn, float hn, int* tw, int* th) {
  int a = (int)lroundf((wn > 0 ? wn : TRACK_SEED_W) * gw);
  int b = (int)lroundf((hn > 0 ? hn : TRACK_SEED_H) * gh);
  if (a < TRACK_TW_MIN) a = TRACK_TW_MIN;
  if (b < TRACK_TW_MIN) b = TRACK_TW_MIN;
  if (a > TRACK_TW_MAX) a = TRACK_TW_MAX;
  if (b > TRACK_TW_MAX) b = TRACK_TW_MAX;
  if (a > gw) a = gw;
  if (b > gh) b = gh;
  *tw = a; *th = b;
}

// （ZNCC / Trimmed-ZNCC / search_best / subpixel_fit / struct Best 已随旧搜索一并删除，
//   现在定位全靠 dcf 的相关滤波。）

// 某通道在 (ccx,ccy) 为中心、tw×th 框内的灰度标准差（判"这通道在这块有没有对比"）。框越界返回 0。
static float box_std(const uint8_t* g, int gw, int gh, int ccx, int ccy, int tw, int th) {
  if (!g) return 0.0f;
  int x0 = ccx - tw / 2, y0 = ccy - th / 2;
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x0 + tw > gw || y0 + th > gh) return 0.0f;
  float sum = 0, sq = 0;
  for (int y = 0; y < th; y++) {
    const uint8_t* r = g + (size_t)(y0 + y) * gw + x0;
    for (int x = 0; x < tw; x++) { const float v = r[x]; sum += v; sq += v * v; }
  }
  const float n = (float)(tw * th);
  const float m = sum / n;
  const float var = sq - sum * m;
  return var > 0 ? sqrtf(var) / sqrtf(n) : 0.0f;
}

// 取"某框最外圈(2px 带)"的色度均值 —— 当"四周/背景"的参考色。框会先被夹回图内；无效返回 -1。
static int ring_mean(const uint8_t* chr, int gw, int gh, int cx, int cy, int bw, int bh) {
  int x0 = cx - bw / 2, y0 = cy - bh / 2;
  if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0;
  if (x0 + bw > gw) x0 = gw - bw;
  if (y0 + bh > gh) y0 = gh - bh;
  if (bw < 4 || bh < 4 || x0 < 0 || y0 < 0) return -1;
  const int band = 2;
  long s = 0; int n = 0;
  for (int y = 0; y < bh; y++) {
    const bool ey = (y < band || y >= bh - band);
    const uint8_t* r = chr + (size_t)(y0 + y) * gw + x0;
    for (int x = 0; x < bw; x++) {
      if (!ey && x >= band && x < bw - band) continue;    // 只取外圈那一带
      s += r[x]; n++;
    }
  }
  return n ? (int)(s / n) : -1;
}

// 主体范围：以**框最外圈的色度**当"四周"参考，把框里"与四周差得够远"的像素认作主体，取质心与外接框。
//
// 为什么用边框而不是中心（用户提的，比我上一版靠谱）：中心那一点可能正好落在高光/反光/阴影上；而框是
// 围着目标画的，最外圈那一带基本就是背景 —— 拿它当"四周"的参考，"主体 vs 四周"这问题的两半就齐了。
// 也修掉上一版的坑：那版拿"框内色度直方图峰值"当主体色，而**平整的背景会在单个 bin 堆出高峰** ⇒ 峰值
// 选到背景 ⇒ "框心不像主体色" ⇒ **悄悄 return false**（连日志都没有，真机两次都白跑）。
// 现在：① 边框色当参考；② 主体太少就把"取参考的框"放大 40% 重算一次（用户提的"框放大点重算"）；
//      ③ **每条失败路径都打日志**，不再有静默 return；
//      ④ 不再做连通域生长（框是围着目标的，框内其它同色物罕见），也就不需要 PSRAM 工作缓冲。
static bool magic_range(const uint8_t* chr, int gw, int gh, int ccx, int ccy, int bw, int bh,
                        int* ox, int* oy, int* ow, int* oh) {
  if (!chr) return false;
  if (bw < 8 || bh < 8) {
    blog::logf(blog::AI, "[track] 魔棒: 框太小 %dx%d, 沿用原框", bw, bh);
    return false;
  }
  int x0 = ccx - bw / 2, y0 = ccy - bh / 2;
  if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0;
  if (x0 + bw > gw) x0 = gw - bw;
  if (y0 + bh > gh) y0 = gh - bh;
  if (x0 < 0 || y0 < 0) {
    blog::logf(blog::AI, "[track] 魔棒: 框出界 %dx%d @%d,%d, 沿用原框", bw, bh, x0, y0);
    return false;
  }
  const int area = bw * bh;
  for (int pass = 0; pass < 2; pass++) {
    const int rw = pass ? (bw * 7) / 5 : bw;   // 第 2 轮：取参考的框放大 40% 重算（往外多取一点背景）
    const int rh = pass ? (bh * 7) / 5 : bh;
    const int ref = ring_mean(chr, gw, gh, x0 + bw / 2, y0 + bh / 2, rw, rh);
    if (ref < 0) {
      blog::logf(blog::AI, "[track] 魔棒 pass%d: 取背景参考失败, 换一轮", pass);
      continue;
    }
    // 阈值**自适应**：固定阈值必然两头不讨好 —— 卡紧了只圈到方块最饱和的核心（真机实测 33x29 的框
    // 只圈出 11x12，模板/窗/搜索半径跟着变小、更容易丢），卡松了把影子地面也吃进来。
    // 所以从紧到松试，取第一个"圈得住又没圈满"的结果（用户提的"减太多了就放宽重算"）。
    const int tols[4] = { TRACK_MAGIC_TOL, 25, 15, 8 };
    for (int ti = 0; ti < 4; ti++) {
      const int ctol = tols[ti];
      int n = 0, ax = 0, ay = 0, minx = bw, maxx = -1, miny = bh, maxy = -1, achr = 0;
      for (int y = 0; y < bh; y++) {
        const uint8_t* r = chr + (size_t)(y0 + y) * gw + x0;
        for (int x = 0; x < bw; x++) {
          if (abs((int)r[x] - ref) <= ctol) continue;   // 像四周 ⇒ 不算主体
          n++; ax += x; ay += y; achr += (int)r[x];
          if (x < minx) minx = x; if (x > maxx) maxx = x;
          if (y < miny) miny = y; if (y > maxy) maxy = y;
        }
      }
      if (n == 0) {   // 这一档一个主体都没有：直接换下一档。⚠️ 不能往下走 —— ax/n 会整数除零直接崩
        blog::logf(blog::AI, "[track] 魔棒 pass%d tol=%d: 框%dx%d 背景色度=%d 主体=0/%d (0%%)",
                   pass, ctol, bw, bh, ref, area);
        continue;
      }
      const int bw2 = maxx - minx + 1, bh2 = maxy - miny + 1;   // 主体外接框
      const int dx2 = ax / n - bw / 2, dy2 = ay / n - bh / 2;   // 主体质心相对框中心的偏移
      blog::logf(blog::AI, "[track] 魔棒 pass%d tol=%d: 框%dx%d 背景色度=%d 主体=%d/%d (%d%%) 外接%dx%d 偏%+d,%+d",
                 pass, ctol, bw, bh, ref, n, area, n * 100 / area, bw2, bh2, dx2, dy2);
      // 接受三条（防两类坏结果）: ① 主体**实心**——填满自家外接框的三成(防零散噪声凑出个大外接框);
      // ② 外接框宽高≥10px(防 5px 残条); ③ 外接框宽高≥原框一半 且 质心偏移≤1/4框(防中心跑偏)。
      // 都不满足就沿用种子的框。
      const bool big_enough = (n * 10 >= bw2 * bh2 * 3) && (bw2 >= 10) && (bh2 >= 10);
      const bool shape_ok   = (bw2 * 2 >= bw) && (bh2 * 2 >= bh);
      const bool centered   = (abs(dx2) * 4 <= bw) && (abs(dy2) * 4 <= bh);
      if (big_enough && shape_ok && centered) {
        *ox = x0 + ax / n; *oy = y0 + ay / n;
        *ow = bw2; *oh = bh2;
        // 种子质量自证：主体色度均值须明显高于环带。真彩色方块 25~60 vs 地面 ~10；
        // 两者几乎无差 = 圈到的是地面纹理（实测锁错后整场跟的都是错的东西，外观自洽无从察觉）。
        blog::logf(blog::AI, "[track] 魔棒接受: 主体色度均值=%d 背景=%d%s",
                   achr / n, ref, (achr / n - ref >= 12) ? "" : " (主体与背景几乎无色差, 种子可疑!)");
        return true;
      }
      blog::logf(blog::AI, "[track] 魔棒 pass%d tol=%d 不接受(%s%s%s), 换下一档",
                 pass, ctol, big_enough ? "" : "主体太少", shape_ok ? "" : "只圈到一条边", centered ? "" : "质心偏太远");
    }
  }
  blog::logf(blog::AI, "[track] 魔棒: 两轮×四档阈值都没圈出合理主体, 沿用原框");
  return false;
}

// 种子吸附：用户/AI 标的是"大致位置"，在 TRACK_SNAP_WIN 窗口内用**色度**峰值把种子精确吸到目标上。
// ⚠️ 用**固定小窗**求峰值，不能用模板框：目标常比模板小得多，用大框会把色度均值稀释到十几（实测 38×38 框
// 里目标只占 ~110px，均值仅 12，被地面拉平），导致永远过不了门槛。小窗贴到目标核心，均值可达 ~100。
// 返回是否吸附成功（= 找到了明显的彩色目标）。ox/oy 回填吸附后的中心。
bool snap_seed(const uint8_t* chr, int gw, int gh, int ccx, int ccy, int bw, int* ox, int* oy) {
  if (!chr) return false;
  const int h2 = bw / 2;                      // 采样步长 2 ⇒ 每边取 h2 个样本
  const int nsamp = h2 * h2;
  const int wu = (int)(TRACK_SNAP_WIN * gw), wv = (int)(TRACK_SNAP_WIN * gh);
  // 基准：标记点本身所在的框内色度均值（吸附后必须明显优于它，否则说明标记点已经在目标上/附近没更彩的东西）
  int s0 = 0;
  {
    const int x0 = ccx - h2, y0 = ccy - h2;
    if (x0 < 0 || y0 < 0 || x0 + bw > gw || y0 + bw > gh) return false;
    for (int yy = 0; yy < bw; yy += 2) {
      const uint8_t* r = chr + (size_t)(y0 + yy) * gw + x0;
      for (int xx = 0; xx < bw; xx += 2) s0 += r[xx];
    }
  }
  // 闸门①：标记处**本身**就没有足够色度 ⇒ 直接不吸附。此时窗口里的"色度峰"多半是地面反光/杂色，
  // 吸上去会把模板取到反光上，下一帧反光一变就"全画面一个点都找不到"（真机实测两次 已吸附 的运行
  // 都是这么死的，而未吸附那次直接夹取成功）。色度吸附只对"确实有颜色"的目标可靠。
  const int base_mean = nsamp ? (s0 / nsamp) : 0;
  if (base_mean < TRACK_SNAP_MIN_CHROMA) return false;
  const int need = (s0 * TRACK_SNAP_GAIN) / 100;   // 吸附门槛 = 基准 × 增益系数/100
  int best = -1, bx = ccx, by = ccy, bdist = 0;
  for (int y = ccy - wv; y <= ccy + wv; y += 2) {
    for (int x = ccx - wu; x <= ccx + wu; x += 2) {
      const int x0 = x - h2, y0 = y - h2;
      if (x0 < 0 || y0 < 0 || x0 + bw > gw || y0 + bw > gh) continue;
      int s = 0;
      for (int yy = 0; yy < bw; yy += 2) {
        const uint8_t* r = chr + (size_t)(y0 + yy) * gw + x0;
        for (int xx = 0; xx < bw; xx += 2) s += r[xx];
      }
      if (s < need) continue;                      // 色度没有明显优于标记点 ⇒ 不是我们要吸的目标
      const int dx = x - ccx, dy = y - ccy;
      const int d = dx * dx + dy * dy;
      if (best < 0 || d < bdist) { best = s; bdist = d; bx = x; by = y; }
    }
  }
  if (best < 0) return false;   // 窗口内没有更"有颜色"的目标 ⇒ 不吸附（保持用户标记点）
  *ox = bx; *oy = by;
  return true;
}

// 学判别式：目标块(**正样本**) vs 外圈背景(**负样本**)，在 (色度, 亮度) 二维特征上求一个线性投影方向 w。
// 这就是"区分目标和背景"的答案 —— 普通 NCC 只问"这块像不像模板"，从不问"它跟背景差多少"，
// 于是带纹理的白地板也能拿到不错的分数。把**当处背景**作为负样本学进去后，投影出的图里背景被压暗、
// 目标凸显。代价：只在锁定时学一次。
//   pos = 框内中心区(内 0.6)；neg = 外圈(1.15×~1.9× 框)剔除内区后的环带。

// 每帧把 (色度, 亮度) 投到判别方向 → 一张"目标亮/背景暗"的标量图。
// +128 偏移后夹到 0..255：ZNCC 会减均值，常数偏移对结果无影响。

// 最近邻缩放模板（多尺度搜索用：逼近时目标变大，固定尺寸模板必然失配）。

// 缓冲的均值与去均值平方和的开方（供缩放后的模板重算统计）。

}  // namespace

void hint_motion(float du, float dv) {
  ensure_mtx();
  if (s_mtx && xSemaphoreTake(s_mtx, 0) != pdTRUE) return;   // 拿不到就算了（下次 update 仍走图像预测）
  s_t.pred_u = du; s_t.pred_v = dv;
  s_t.has_pred = true;
  if (s_mtx) xSemaphoreGive(s_mtx);
}
void hint_wide(float du, float dv) {
  ensure_mtx();
  if (s_mtx && xSemaphoreTake(s_mtx, 0) != pdTRUE) return;
  s_t.pred_u = du; s_t.pred_v = dv;
  s_t.has_pred = true;
  s_wide_left = 3;                              // 接下来三帧走宽窗+恢复期门槛（预期大位移，非丢后重捕）
  if (s_mtx) xSemaphoreGive(s_mtx);
}

bool seed(const char* name, float cx, float cy, float w, float h) {
  if (!name || !name[0]) return false;
  if (w <= 0) w = TRACK_SEED_W;
  if (h <= 0) h = TRACK_SEED_H;
  if (cx < 0) cx = 0; if (cx > 1) cx = 1;
  if (cy < 0) cy = 0; if (cy > 1) cy = 1;
  ensure_mtx();
  const bool lk = take_mtx(2000);
  strncpy(s_t.name, name, sizeof(s_t.name) - 1);
  s_t.name[sizeof(s_t.name) - 1] = 0;
  s_t.su = cx; s_t.sv = cy; s_t.sw = w; s_t.sh = h;
  s_t.st = State::Locking;
  s_t.miss = 0; s_t.frames = 0; s_t.conf = 0;
  // 解码抽点随目标大小自适应: 目标太小(远/小物件)时, 1/2 抽点下只剩十几个像素, 而 DCF 的模板/窗
  // 有 20px 下限, 模板里大半是背景, 外观分在门槛边缘震荡, 车一动就跌破 ⇒ 降低抽点让目标够大。
  // u/v 是归一化坐标, 抽点不影响几何; 代价是取图耗时和内存随抽点下降而增大(均走 PSRAM)。
  {
    const float full_px = w * 640.0f;                 // 全幅宽(VGA)下的目标边长
    int sc = TRACK_SCALE;
    while (sc > 0 && full_px / (float)(1 << sc) < 24.0f) sc--;
    s_scale = sc;
  }
  if (lk) xSemaphoreGive(s_mtx);
  cam::set_track_mode(true);   // 进入跟踪：相机切 RGB565（幂等，已切则不重开；阻塞百 ms 级）
  return true;
}

// 内部：真正跑追踪。调用方已持 s_mtx。
static Result update_locked(const camera_fb_t* fb, bool light) {
  s_track_ms = 0;
  Result r = { false, s_t.u, s_t.v, s_t.conf, s_t.st };
  if (s_t.st == State::Idle) return r;
  // light（手机标记刷新）：只在已锁定的 Tracking 态补一刀位置；锁定/丢失态一律不碰（避免干扰控制线程）。
  if (light && s_t.st != State::Tracking) return r;
  if (!fb || !fb->buf || fb->width <= 0 || fb->height <= 0) return r;
  s_last_cap_ms = millis();   // 喂帧侧 grab_fresh 排空后才取，此刻 ≈ 拍摄时刻（后面的解码+搜索延迟不算在内）

  const int gw = ((int)fb->width + (1 << s_scale) - 1) >> s_scale;
  const int gh = ((int)fb->height + (1 << s_scale) - 1) >> s_scale;
  if (!ensure(&s_lum, &s_lum_cap, (size_t)gw * gh)) return r;
  if (!ensure(&s_chr, &s_chr_cap, (size_t)gw * gh)) return r;
  int dw = 0, dh = 0;
  if (decode_dual(fb, s_scale, s_lum, s_chr, (int)s_lum_cap, &dw, &dh) != 0) {
    blog::logf(blog::AI, "[track] 取图失败(scale=%d %ux%u)", s_scale,
               (unsigned)fb->width, (unsigned)fb->height);
    return r;
  }
  unsigned long t0 = millis();

  // ---- 首次锁定：吸附种子 → 学判别式 → 在判别图上抽模板 ----
  if (s_t.st == State::Locking) {
    int tw = 0, th = 0;
    tmpl_dims(gw, gh, s_t.sw, s_t.sh, &tw, &th);
    int ccx = (int)lroundf(s_t.su * gw);
    int ccy = (int)lroundf(s_t.sv * gh);
    // 种子吸附：把"大致标记位置"精确吸到目标上（色度峰值）。手标常偏几十像素，不吸附就锁到地面。
    // ⚠️ 只在**色度确实占优**时才吸附：目标亮度主导时，窗口里的色度峰多半是地面反光/杂色，吸上去会把
    // 模板取到反光上 —— 下一帧反光一变就"全画面一个点都找不到"（真机实测：两次 已吸附 的运行都是这样
    // 第 2 帧就 conf=0.00 死掉，而未吸附那次直接夹取成功）。
    int sx0 = ccx, sy0 = ccy;
    const bool chroma_dominant =
        box_std(s_chr, gw, gh, ccx, ccy, tw, th) > box_std(s_lum, gw, gh, ccx, ccy, tw, th) * 1.5f;
    const bool snapped = chroma_dominant && snap_seed(s_chr, gw, gh, ccx, ccy, tw, &sx0, &sy0);
    if (snapped) { ccx = sx0; ccy = sy0; }

    // ---- 交给相关滤波内核起跟 ----
    // 只用灰度（s_lum）：相关滤波每帧在线更新自己，不需要色度/判别式那套"学一次就不再更新"的东西
    //（真机实测那套在车一转/一逼近后 conf 只会缓慢退化到 0.55 卡死）。
    // 起跟前做一次"这块够不够有对比"的体检：太平/太素的话先验不可靠。
    // ★ 魔棒式取范围（见 magic_range）：以**框最外圈的色度**当"四周"参考，把框里"与四周差得够远"的像素
    //   认作主体，用它的质心与外形取代"你画的框"来定中心与尺寸 —— 影子/地面就不会进模板了。
    // 框里难免带进影子/地面，而框直接决定 DCF 的模板与窗 ⇒ 影子进了模板就会被跟着走。
    int magic_px = 0;
    {
      int mx = 0, my = 0, mw = 0, mh = 0;
      if (magic_range(s_chr, gw, gh, ccx, ccy, tw, th, &mx, &my, &mw, &mh)) {
        magic_px = mw > mh ? mw : mh;
        blog::logf(blog::AI, "[track] '%s' 魔棒范围 %dx%d (质心偏移 %+d,%+d)", s_t.name, mw, mh, mx - ccx, my - ccy);
        ccx = mx; ccy = my;
        if (mw >= 6 && mh >= 6) { tw = mw; th = mh; }   // 模板尺寸改用主体自己的外接范围
      }
    }
    const float lstd = box_std(s_lum, gw, gh, ccx, ccy, tw, th);
    if (lstd < TRACK_MIN_STD) {
      blog::logf(blog::AI, "[track] '%s' 区域太素(亮度std=%.1f), 无法跟踪", s_t.name, (double)lstd);
      s_t.st = State::Idle;
      cam::set_track_mode(false);
      r.st = State::Idle;
      return r;
    }
    // 目标尺寸：优先用"魔棒范围"的边长（主体自己的大小），拿不到才退回用户框的归一化尺寸。
    const float obj_norm = (magic_px >= 6)
        ? ((float)magic_px / (float)gw)
        : fmaxf(s_t.sw > 0 ? s_t.sw : TRACK_SEED_W, s_t.sh > 0 ? s_t.sh : TRACK_SEED_H);
    float psr0 = 0;
    if (!dcf::start(s_lum, s_chr, gw, gh, (ccx + 0.5f) / gw, (ccy + 0.5f) / gh, obj_norm, &psr0)) {
      blog::logf(blog::AI, "[track] '%s' 相关滤波起跟失败(内存不足?)", s_t.name);
      s_t.st = State::Idle;
      cam::set_track_mode(false);
      r.st = State::Idle;
      return r;
    }
    s_t.u = (ccx + 0.5f) / gw;
    s_t.v = (ccy + 0.5f) / gh;
    s_t.conf = 1.0f;
    s_t.st = State::Tracking;
    s_t.frames = 0;
    s_last_ok = true;                 // 锁定这一帧本身就是有效位置（夹取闭环可立刻采用）
    s_last_upd_ms = millis();
    // 初始化平滑输出和预测（首次锁定无历史位移），raw 先对齐到锁定点
    s_t.smooth_u = s_t.u;
    s_t.smooth_v = s_t.v;
    s_t.raw_u = s_t.u;
    s_t.raw_v = s_t.v;
    s_t.has_pred = false;
    s_t.pred_u = 0; s_t.pred_v = 0;
    r.ok = true; r.u = s_t.u; r.v = s_t.v; r.conf = 1.0f; r.st = s_t.st;
    blog::logf(blog::AI, "[track] 锁定 '%s' 相关滤波 目标%.0fpx 中心(%.3f,%.3f) 取图%ums%s "
               "[色度std=%.1f 亮度std=%.1f] [车身mask=%s] [色度门=%s]",
               s_t.name, (double)(obj_norm * gw), s_t.u, s_t.v, (unsigned)s_decode_ms,
               snapped ? " [已吸附]" : " [未吸附]",
               (double)box_std(s_chr, gw, gh, ccx, ccy, tw, th), (double)lstd,
               dcf::mask_on() ? "开" : "关(种子在车身里)",
               dcf::sat_on() ? "开" : "关(目标无彩色)");
    return r;
  }

  // ---- 相关滤波更新（取代原来的"建判别图 → 粗层 → 多尺度粗搜 → 全幅精修"） ----
  {
    const int64_t t0d = esp_timer_get_time();
    s_us_disc = 0;
    // 搜索窗心：优先用**控制量先验**（夹取下一步动作会带来的位移，由 grasp 的 hint_motion 喂进来），
    // 再叠加图像位移预测。相关滤波只需要一个"目标大概在哪"的窗心。
    float pu = s_t.raw_u, pv = s_t.raw_v;
    if (s_t.has_pred) { pu += s_t.pred_u; pv += s_t.pred_v; }
    // wide：丢失重捕，或 grasp 宣告的"预期宽窗"（近场后退/大转角后的一两帧——窗心已在 hint
    // 位置，宽窗只为容住比预测更大的实际位移；不是丢后瞎找）。
    const bool wide_frame = s_wide_left > 0;
    const bool wide = (s_t.st == State::Lost) || wide_frame;
    if (wide_frame) s_wide_left--;
    // 接收门限：PSR。已在跟(keep)比重新捕获(lost 放宽窗重捕)松一些。**低于门限的帧在 dcf 内部就被
    // 整个丢弃**（位置/尺度/滤波器都不动），所以这里只看 du.ok。
    // 预期宽窗帧与正常跟踪同一门槛（TRACK_PSR_KEEP 已按恢复期校准）：大转角/大位移后响应要几帧才恢复锐利
    const float need_psr = (s_t.st == State::Tracking) ? TRACK_PSR_KEEP : TRACK_PSR_LO;
    // 尺寸修正只给"丢失重捕"的宽窗：预期宽窗（自己动作后的恢复期）失败时位置本就在原地，
    // 按 best_scale 放缩会让 s_obj 两帧翻 4 倍（实测 39→109px）
    const bool lost_wide = wide && (s_t.st == State::Lost);
    dcf::Upd du = dcf::update(s_lum, s_chr, gw, gh, pu, pv, wide, need_psr, lost_wide);
    s_us_search = (int)(esp_timer_get_time() - t0d);
    s_last_scale = du.scale;
    s_last_obj = du.obj_px;      // 诊断：本帧目标边长(px)
    s_last_off = du.off_frac;    // 诊断：峰偏移/半径（接近 1 = 目标贴窗边）
    s_last_appear = du.appear;   // 诊断：与起跟外观的 NCC（低 = 已经不是那个物体了）
    // 合爪自检探针：顺便算两个给定点"像不像目标"（不动任何跟踪状态）
    if (s_probe_on) {
      s_pa0 = dcf::appear_at(s_lum, gw, gh, s_p0u, s_p0v);
      // 夹持位**在邻域里取最大**：夹持位会随车姿俯仰/臂姿态漂，只探单点必然落空
      //（真机实测：单点判空夹连续误报）。9 个点各一次采样，整个验证阶段多花 ~10ms，无所谓。
      s_pa1 = dcf::appear_at(s_lum, gw, gh, s_p1u, s_p1v);
      for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++) {
          if (!dx && !dy) continue;
          const float a = dcf::appear_at(s_lum, gw, gh,
                                         s_p1u + (float)dx * GRASP_HOLD_SCAN_R,
                                         s_p1v + (float)dy * GRASP_HOLD_SCAN_R);
          if (a > s_pa1) s_pa1 = a;
        }
    }
    s_t.conf = du.ok ? (du.psr / 20.0f) : 0.0f;   // PSR→0..1 的置信度（仅上报/显示用）
    if (s_t.conf > 1.0f) s_t.conf = 1.0f;
    s_track_ms = (int)(millis() - t0);
    s_last_upd_ms = millis();   // 报告"本帧处理完了"：夹取闭环据此判断拿到了动作之后的新位置
    s_last_ok = du.ok;          // 本帧是否被采纳（PSR 达标）——夹取闭环必须看这个，别拿旧位置当新位置
    if (du.ok) {
      const float ru = du.u, rv2 = du.v;
      s_t.raw_u = ru; s_t.raw_v = rv2;     // 未平滑：手机标记用（不拖尾）
      const float prev_su = s_t.smooth_u, prev_sv = s_t.smooth_v;
      s_t.smooth_u = s_t.alpha * ru + (1.0f - s_t.alpha) * s_t.smooth_u;
      s_t.smooth_v = s_t.alpha * rv2 + (1.0f - s_t.alpha) * s_t.smooth_v;
      if (s_t.frames > 0) {
        s_t.pred_u = s_t.smooth_u - prev_su; s_t.pred_v = s_t.smooth_v - prev_sv;
        s_t.has_pred = true;
      }
      s_t.u = s_t.smooth_u; s_t.v = s_t.smooth_v;   // 平滑后：控制闭环用
      s_t.miss = 0;
      s_t.st = State::Tracking;
      s_t.frames++;
      r.ok = true;
    } else if (wide_frame) {
      // 预期宽窗恢复期：PSR 还没恢复锐利是正常的，不计 miss、不判丢（宽窗帧用完自然回到正常门）
    } else {
      s_t.miss++;
      if (!s_reacq && s_t.miss >= TRACK_LOST_N) {
        // 不允许重捕：跟丢即终止，不做宽搜（grasp 闭环用 —— 宽搜抓到什么都可能，不如老实失败）
        if (s_t.st != State::Idle) {
          blog::logf(blog::AI, "[track] '%s' 丢失(PSR=%.1f), 已禁用重捕 → 停止跟踪", s_t.name, (double)du.psr);
          s_t.st = State::Idle;
          dcf::stop();
          cam::set_track_mode(false);   // 跟踪结束：相机切回 JPEG 常态
        }
      } else if (s_t.st != State::Lost && s_t.miss >= TRACK_LOST_N) {
        s_t.st = State::Lost;
        blog::logf(blog::AI, "[track] '%s' 丢失(PSR=%.1f), 放宽搜索窗重捕", s_t.name, (double)du.psr);
      } else if (s_t.st == State::Lost && s_t.miss >= TRACK_LOST_N + TRACK_REACQ_MAX) {
        blog::logf(blog::AI, "[track] '%s' 重捕失败, 停止跟踪", s_t.name);
        s_t.st = State::Idle;
        dcf::stop();
        cam::set_track_mode(false);   // 跟踪结束：相机切回 JPEG 常态
      }
    }
    r.u = s_t.u; r.v = s_t.v; r.conf = s_t.conf; r.st = s_t.st;
    return r;
  }
}

// 公开入口：加锁包住 update_locked。light 用 try-lock（拿不到即跳过，绝不阻塞控制线程）；
// 控制路径阻塞取锁。追踪状态与全局像素缓冲都经这里改，保证 grasp/AI 与图传 light 刷新串行。
Result update_from_fb(const camera_fb_t* fb, bool light) {
  ensure_mtx();
  if (!s_mtx) return update_locked(fb, light);   // 极端兜底（互斥创建失败）
  if (light) {
    if (xSemaphoreTake(s_mtx, 0) != pdTRUE) {
      Result r = { false, s_t.u, s_t.v, s_t.conf, s_t.st };
      return r;
    }
    Result r = update_locked(fb, true);
    xSemaphoreGive(s_mtx);
    return r;
  }
  if (!take_mtx(3000)) { Result r = { false, s_t.u, s_t.v, s_t.conf, s_t.st }; return r; }
  Result r = update_locked(fb, light);
  xSemaphoreGive(s_mtx);
  return r;
}

// 种子框(AI 标的位置): auto_grasp 播种前登记, 供 app_httpd 周期上报 —— 让手机在切跟踪画面**之前**
// 的最后一帧上把框画出来, 人能看见 AI 到底标到了哪(标歪/标大时一眼看出, 不必等夹空)。
static float s_seed[4] = { 0, 0, 0, 0 };
static uint32_t s_seed_ms = 0;

void stop() {
  ensure_mtx();
  const bool lk = take_mtx(2000);
  s_t.st = State::Idle;
  s_seed_ms = 0;                // 种子框一并作废: 否则跟踪都停了板子还在给手机推这个框
  s_t.miss = 0;
  s_t.name[0] = 0;
  s_last_ok = false;
  s_wide_left = 0;
  s_scale = TRACK_SCALE;        // 抽点回默认（下次 seed 按目标大小重选）
  dcf::stop();                  // 相关滤波状态一并清掉（下次要重新 start）
  if (lk) xSemaphoreGive(s_mtx);
  cam::set_track_mode(false);   // 跟踪结束：相机切回 JPEG 常态（幂等；未在跟踪时为空操作）
}

bool active() { return s_t.st != State::Idle; }
void set_reacquire(bool on) { s_reacq = on; }
State state() { return s_t.st; }
const char* target_name() { return s_t.name; }

bool last_center(float* u, float* v) {
  // 只有锁定成功(Tracking)或丢失后保留的上次位置(Lost)才算"有效中心"。
  // Locking 期间 s_t.u/v 还是上一次的旧值(初值 0,0)，报出去会把目标显示/告知在左上角。
  if (s_t.st != State::Tracking && s_t.st != State::Lost) return false;
  if (u) *u = s_t.u;
  if (v) *v = s_t.v;
  return true;
}

bool last_raw_center(float* u, float* v) {
  // 同上，但给的是未平滑的亚像素中心（手机叠加显示用，避免平滑造成的固有拖尾）。
  if (s_t.st != State::Tracking && s_t.st != State::Lost) return false;
  if (u) *u = s_t.raw_u;
  if (v) *v = s_t.raw_v;
  return true;
}

float last_conf() { return s_t.conf; }
float last_scale() { return s_last_scale; }
float last_obj_px() { return s_last_obj; }
float last_off_frac() { return s_last_off; }
float last_appear() { return s_last_appear; }
bool dcf_mask_on() { return dcf::mask_on(); }
void  probe_set(float u0, float v0, float u1, float v1) {
  s_p0u = u0; s_p0v = v0; s_p1u = u1; s_p1v = v1; s_probe_on = true;
}
void  probe_off() { s_probe_on = false; }
float probe_appear0() { return s_pa0; }
float probe_appear1() { return s_pa1; }
unsigned long last_update_ms() { return s_last_upd_ms; }
unsigned long last_capture_ms() { return s_last_cap_ms; }

void set_seed_box(float u, float v, float w, float h) {
  s_seed[0] = u; s_seed[1] = v; s_seed[2] = w; s_seed[3] = h;
  s_seed_ms = (uint32_t)(esp_timer_get_time() / 1000);
  blog::logf(blog::AI, "[track] AI 标框 (%.3f,%.3f) %.3fx%.3f → 手机将在切跟踪画面前画出", (double)u, (double)v, (double)w, (double)h);
}

bool seed_box(float* u, float* v, float* w, float* h) {
  const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
  if (s_seed_ms == 0 || (uint32_t)(now - s_seed_ms) > 4000) return false;   // 覆盖松爪+降臂+播种重初始化整个准备期
  if (u) *u = s_seed[0];
  if (v) *v = s_seed[1];
  if (w) *w = s_seed[2];
  if (h) *h = s_seed[3];
  return true;
}
bool last_ok() { return s_last_ok; }
void search_radius(float* ru, float* rv) { dcf::radius_norm(ru, rv); }

}  // namespace track
