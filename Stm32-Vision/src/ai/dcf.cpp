// ============================ 相关滤波跟踪内核实现（见 dcf.h 的设计说明） ============================
#include "src/ai/dcf.h"
#include "src/ai/track_mask.h"   // 车身/夹爪所占画面区域的粗网格（由 armLowMask.png 生成）

#include <math.h>
#include <string.h>
#include <esp_heap_caps.h>

namespace dcf {

// ---- 可调参数（调这些就够，别动结构） ----
static const int   N        = 32;      // 频域网格边长（必须是 2 的幂）。64→32：搜索/FFT 4 倍提速，
                                       // 换取跟踪帧率 ~2 倍 —— 大转角不再是"一次 0.2u 的瞬移"，滤波器
                                       // 能骑上平滑旋转（实测大转角跟丢的主因就是帧间跳变太大）。
                                       // 代价：响应图/外观分辨率减半，亚像素拟合补偿，PSR 门待验证。
static const int   NN       = N * N;
static const float K_WIN    = 6.0f;    // 源窗边长 = 目标边长 × 此倍数 ⇒ 搜索半径 ≈ 2.5×目标。
                                      // ⚠️ 必须罩住"一步动作的位移"：原地转 25° 位移≈0.15归一化(=48px@320)，
                                      // 而目标才 ~23px —— K_WIN=4 时半径只有 1.5×23=35px，一转就出窗（真机实测
                                      // 一转就 conf 崩到 0.16）。6 时半径 57px，留了余量。
static const float ETA      = 0.125f;  // 滤波器在线学习率（MOSSE 论文默认；越大越跟手、越小越稳）
static const float SIGMA    = 2.0f;    // 期望响应（高斯）的标准差（网格单位）
static const float EPS      = 1e-3f;   // 分母保护
// 尺度候选：正常跟踪跑 3 档；丢失时跑大档（归一化窗 ⇒ 放尺度 = 放搜索半径）
static const float SC_NORMAL[3] = { 0.85f, 1.00f, 1.20f };
static const float SC_WIDE[7]   = { 0.50f, 0.70f, 1.00f, 1.40f, 2.00f, 2.80f, 4.00f };   // 丢失重捕用

// ---- 缓冲（全 PSRAM） ----
static float* s_in  = nullptr;   // 输入窗（复数交错）
static float* s_g   = nullptr;   // 期望响应（频域）
static float* s_num = nullptr;   // 滤波器分子（频域复数）
static float* s_den = nullptr;   // 滤波器分母（频域实数，虚部恒 0）
static float* s_resp = nullptr;  // 响应（时域）
// 都是"一块有边界的斑"——这道门是唯一能把它们分开的东西。c_tol<=0 表示不用（目标无彩色）。
static float* s_win = nullptr;   // Hann 窗（实）
static float* s_seed = nullptr;  // **起跟那一帧的窗**（零均值、单位模）——每帧比对外观的绝对参照
static float  s_col[2 * N];      // 列变换暂存（内部 RAM：小且频繁）

// 外观锚定门：本帧（按峰值位置）与种子的中心区 NCC 低于此值 ⇒ "已经不是那个物体了"：
// 本帧整个不采纳，也不训练滤波器。
// 阈值**卡在实测两类之间**：真目标全程 0.45~1.00（贴近夹爪、被部分遮挡时会掉到 0.45~0.50），
// 臂/夹爪 0.20~0.43。早先设 0.50 会把真目标在最后关头丢掉（run11 `#9 外观0.45` 被拒 → 丢 → 重捕
// 抓到臂上 → 判"太近"猛退 → 中止），故降到 0.45。
static const float APPEAR_MIN = 0.45f;
// **重捕档（wide）单独放低**：真机实测方块贴近夹爪被部分遮挡时外观掉到 0.4 上下，用 0.45 救不回来
// （run9 `#15 外观0.43` 就是靠它救的）。但不能太松：臂/夹爪最高能到 0.43，0.35 会把臂也放进来
// （run11 重捕抓到臂上、外观 0.36~0.43）。取 0.40。
static const float APPEAR_MIN_REACQ = 0.40f;
// 锚定自适应：appear ≥ 这个值才允许把锚定往当前外观带（APPEAR_ETA/帧）。见 seed_blend 的说明。
// 0.70→0.45：短距离逼近时外观每步掉 ~0.2（视角+遮挡），旧值 0.70 让锚定在 0.7 以下就冻结，
// 冻结的锚定追不上衰减 ⇒ 门 0.45 击穿 ⇒ 收官必丢。降到与门同高 = 每个采纳帧都跟着衰减走；
// "锁到别的东西"的突变仍会被门拦住（一步掉到门下），锚定来不及学进去。
static const float APPEAR_LEARN = 0.45f;
static const float APPEAR_ETA   = 0.15f;

static bool  s_on = false;
static int   s_gw = 0, s_gh = 0;
static float s_obj = 0;           // 目标在灰度图中的边长(px)，随尺度搜索缓慢更新
static float s_prev_obj = 0;      // 上一**采纳**帧的 s_obj：尺寸跳变否决用（刚体一帧不会翻倍）
static float s_cx = 0, s_cy = 0; // 目标中心(平面 px)
static float s_scale = 1.0f;     // 相对起跟时的尺度（对外报告）
static float s_appear = 0.0f;    // 最近一次算出的"与起跟外观的 NCC"（被拒帧/重捕帧沿用，避免日志里出现 0 被误读）
// 车身 mask 是否启用：**起跟位置若落在车身里就整场关闭**（否则目标本身在夹爪里时会被直接挡死）。
static bool  s_use_mask = false;

// ---- 色度加权门（luma 为主、饱和度只当加分/减分）----
// 起跟时若判定"目标本身有颜色"（目标中心小片的饱和度明显高于窗四周）就开启：
// 之后每帧在响应图上，把"饱和度不像目标"的格子**压低**。典型收益 —— 地面反射的深色臂影在灰度里
// 和深色方块几乎一样（真机实测：锁上去后车转 35° u 纹丝不动），但它的饱和度远低于红方块 ⇒ 被压下去。
// 注意这**不是**"纯色度锁定"：找目标仍是 luma，色度只做否决；色度漂了最多退化成原行为。
static const uint8_t* s_chrp = nullptr;   // 当前帧饱和度平面（空 = 不做门）
static int   s_chrw = 0, s_chrh = 0;
static bool  s_sat_on = false;            // 本次跟跟是否开着色度门
static float s_sat_ref = 0.0f;            // 目标的饱和度参考值（起跟时量）
static const float SAT_MIN_DIFF = 25.0f;  // 目标中心比四周高这么多才算"有颜色"
static const float SAT_TOL      = 45.0f;  // 某格饱和度与目标差这么多 ⇒ 不像目标
static const float SAT_PENALTY  = 0.25f;  // 不像目标的格子：正向响应乘它（负响应不动，免得抬高）

static void* psram(size_t n) { return heap_caps_malloc(n, MALLOC_CAP_SPIRAM); }

// ---------------- radix-2 复数 FFT（自带实现，避免依赖库 API） ----------------
// d 为复数交错数组(2n 个 float)。inv=false 正变换 / true 逆变换（含 1/n）。
static void fft1(float* d, int n, bool inv) {
  for (int i = 1, j = 0; i < n; i++) {          // 位反转置换
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      float tr = d[2 * i], ti = d[2 * i + 1];
      d[2 * i] = d[2 * j]; d[2 * i + 1] = d[2 * j + 1];
      d[2 * j] = tr; d[2 * j + 1] = ti;
    }
  }
  for (int len = 2; len <= n; len <<= 1) {      // 逐级蝶形
    const double ang = (inv ? 2.0 : -2.0) * 3.14159265358979323846 / (double)len;
    const float wr = cosf((float)ang), wi = sinf((float)ang);
    const int half = len >> 1;
    for (int i = 0; i < n; i += len) {
      float cwr = 1.0f, cwi = 0.0f;
      for (int k = 0; k < half; k++) {
        const int a = i + k, b = a + half;
        const float ur = d[2 * a], ui = d[2 * a + 1];
        const float vr = d[2 * b], vi = d[2 * b + 1];
        const float xr = vr * cwr - vi * cwi, xi = vr * cwi + vi * cwr;
        d[2 * a] = ur + xr; d[2 * a + 1] = ui + xi;
        d[2 * b] = ur - xr; d[2 * b + 1] = ui - xi;
        const float nwr = cwr * wr - cwi * wi;
        cwi = cwr * wi + cwi * wr; cwr = nwr;
      }
    }
  }
  if (inv) { const float s = 1.0f / (float)n; for (int i = 0; i < 2 * n; i++) d[i] *= s; }
}

// 二维 FFT：先行后列（就地）
static void fft2(float* d, bool inv) {
  for (int y = 0; y < N; y++) fft1(d + y * 2 * N, N, inv);
  for (int x = 0; x < N; x++) {
    for (int y = 0; y < N; y++) { s_col[2 * y] = d[(y * N + x) * 2]; s_col[2 * y + 1] = d[(y * N + x) * 2 + 1]; }
    fft1(s_col, N, inv);
    for (int y = 0; y < N; y++) { d[(y * N + x) * 2] = s_col[2 * y]; d[(y * N + x) * 2 + 1] = s_col[2 * y + 1]; }
  }
}

// ---------------- 采样与取窗 ----------------
static inline float sample(const uint8_t* g, float x, float y) {  int x0 = (int)floorf(x), y0 = (int)floorf(y);
  const float fx = x - (float)x0, fy = y - (float)y0;
  if (x0 < 0) x0 = 0; else if (x0 >= s_gw - 1) x0 = s_gw - 2;
  if (y0 < 0) y0 = 0; else if (y0 >= s_gh - 1) y0 = s_gh - 2;
  const uint8_t* r0 = g + (size_t)y0 * s_gw + x0;
  const uint8_t* r1 = r0 + s_gw;
  const float a = (float)r0[0] + ((float)r0[1] - (float)r0[0]) * fx;
  const float b = (float)r1[0] + ((float)r1[1] - (float)r1[0]) * fx;
  return a + (b - a) * fy;
}

// 采样**饱和度平面**（色度门用）。步长/边界逻辑与 sample 一致，只是换一块缓冲。
static inline float sample_chr(float x, float y) {
  int x0 = (int)floorf(x), y0 = (int)floorf(y);
  const float fx = x - (float)x0, fy = y - (float)y0;
  if (x0 < 0) x0 = 0; else if (x0 >= s_chrw - 1) x0 = s_chrw - 2;
  if (y0 < 0) y0 = 0; else if (y0 >= s_chrh - 1) y0 = s_chrh - 2;
  const uint8_t* r0 = s_chrp + (size_t)y0 * s_chrw + x0;
  const uint8_t* r1 = r0 + s_chrw;
  const float a = (float)r0[0] + ((float)r0[1] - (float)r0[0]) * fx;
  const float b = (float)r1[0] + ((float)r1[1] - (float)r1[0]) * fx;
  return a + (b - a) * fy;
}

// 以 (cx,cy) 为中心、源边长 W，双线性重采样到 N×N，乘 Hann 窗并去均值 → s_in
static void extract(const uint8_t* g, float cx, float cy, float W) {
  const float step = W / (float)N;
  const float sx = cx - W * 0.5f + step * 0.5f;
  const float sy = cy - W * 0.5f + step * 0.5f;
  float mean = 0;
  for (int j = 0; j < N; j++) {
    for (int i = 0; i < N; i++) {
      const float v = sample(g, sx + (float)i * step, sy + (float)j * step);
      s_in[(j * N + i) * 2] = v;
      mean += v;
    }
  }
  mean /= (float)NN;
  for (int k = 0; k < NN; k++) {
    s_in[k * 2] = (s_in[k * 2] - mean) * s_win[k];   // 去均值(Hann 后仍有直流) + 加窗
    s_in[k * 2 + 1] = 0.0f;
  }
}

// ---------------- 种子外观锚定 ----------------
// ⚠️ **只比中心区**：窗 = K_WIN×目标 ⇒ 整窗 97% 是背景，拿整窗做 NCC 量到的是"地面长得一不一样"，
// 不是"还是不是那个方块"（真机实测：方块还好好跟着时外观也从 1.00 衰减到 0.31，而锁到夹爪上时是 0.53
// —— 两者重叠、阈值根本分不开）。目标本体只占中心 ±(N/2)/K_WIN ≈ ±5 格，所以只取中心这块比。
// 记下"起跟时目标本来的样子"：把该区存成**零均值、单位模**，之后每帧与它做 NCC。
// 区外留 0，于是点积可以照常走满 N×N（区外两边都乘了 0）。
static const int APPEAR_R = N / 12 + 1;   // 中心区半宽（格）：目标本体占 ±N/(2*K_WIN)≈±N/12，外扩 1 格。
                                          // 随 N 缩放（64→6、32→3），保证外观比的是"目标本体"而不是混进背景

static void seed_capture() {
  const int c = N / 2, R = APPEAR_R;
  float m = 0; int cnt = 0;
  for (int j = c - R; j <= c + R; j++)
    for (int i = c - R; i <= c + R; i++) { m += s_in[(j * N + i) * 2]; cnt++; }
  m /= (float)cnt;
  for (int k = 0; k < NN; k++) s_seed[k] = 0.0f;
  float nrm = 0;
  for (int j = c - R; j <= c + R; j++)
    for (int i = c - R; i <= c + R; i++) {
      const int k = j * N + i;
      const float v = s_in[k * 2] - m;
      s_seed[k] = v; nrm += v * v;
    }
  const float inv = nrm > 1e-6f ? 1.0f / sqrtf(nrm) : 0.0f;
  for (int k = 0; k < NN; k++) s_seed[k] *= inv;
}

// 把锚定**慢慢**带向当前外观：只在"很像"（appear ≥ APPEAR_LEARN）时才带。
// 为什么需要：锚定拿的是"起跟那一帧"，而逼近过程中目标在画面里的样子会持续变（透视/大小）。
// 一直拿最初那一帧比，正常逼近也会把外观拉到门限以下（真机实测 run13：从 60cm 一路直走逼近，
// 外观 1.00→0.54→0.25 直接丢掉，且重捕也回不来）。带得慢，是为了让"锁到别的东西"这种**突变**
// 来不及被学进去 —— 突变那一帧外观会一步掉到 0.3 上下，不满足"很像"，锚定原地不动。
static void seed_blend() {
  const int c = N / 2, R = APPEAR_R;
  float m = 0; int cnt = 0;
  for (int j = c - R; j <= c + R; j++)
    for (int i = c - R; i <= c + R; i++) { m += s_in[(j * N + i) * 2]; cnt++; }
  m /= (float)cnt;
  float nrm = 0;
  for (int j = c - R; j <= c + R; j++)
    for (int i = c - R; i <= c + R; i++) { const float v = s_in[(j * N + i) * 2] - m; nrm += v * v; }
  if (nrm <= 1e-6f) return;
  const float inv = 1.0f / sqrtf(nrm);
  float n2 = 0;
  for (int j = c - R; j <= c + R; j++)
    for (int i = c - R; i <= c + R; i++) {
      const int k = j * N + i;
      const float v = (s_in[k * 2] - m) * inv;                    // 当前窗：零均值、单位模
      s_seed[k] = (1.0f - APPEAR_ETA) * s_seed[k] + APPEAR_ETA * v;
      n2 += s_seed[k] * s_seed[k];
    }
  const float inv2 = n2 > 1e-6f ? 1.0f / sqrtf(n2) : 0.0f;
  for (int k = 0; k < NN; k++) s_seed[k] *= inv2;
}

// 当前窗中心区与种子的 NCC。s_seed 已是零均值单位模，这里只需给当前区去均值并归一化。
static float seed_ncc() {
  const int c = N / 2, R = APPEAR_R;
  float m = 0; int cnt = 0;
  for (int j = c - R; j <= c + R; j++)
    for (int i = c - R; i <= c + R; i++) { m += s_in[(j * N + i) * 2]; cnt++; }
  m /= (float)cnt;
  float dot = 0, nrm = 0;
  for (int j = c - R; j <= c + R; j++)
    for (int i = c - R; i <= c + R; i++) {
      const int k = j * N + i;
      const float v = s_in[k * 2] - m;
      dot += v * s_seed[k]; nrm += v * v;
    }
  if (nrm <= 1e-6f) return 0.0f;
  float cc = dot / sqrtf(nrm);
  if (cc < -1.0f) cc = -1.0f; else if (cc > 1.0f) cc = 1.0f;
  return cc;
}

// ---------------- 初始化 ----------------
bool init() {
  if (s_on) return true;
  s_in   = (float*)psram(sizeof(float) * 2 * NN);
  s_g    = (float*)psram(sizeof(float) * 2 * NN);
  s_num  = (float*)psram(sizeof(float) * 2 * NN);
  s_den  = (float*)psram(sizeof(float) * 2 * NN);
  s_resp = (float*)psram(sizeof(float) * 2 * NN);
  s_win  = (float*)psram(sizeof(float) * NN);
  s_seed = (float*)psram(sizeof(float) * NN);
  if (!s_in || !s_g || !s_num || !s_den || !s_resp || !s_win || !s_seed) return false;
  // Hann 窗 + 高斯期望响应（只在 0..N-1 的非周期窗上，避免 FFT 边界跳变）
  for (int j = 0; j < N; j++)
    for (int i = 0; i < N; i++) {
      const float wx = 0.5f - 0.5f * cosf(2.0f * 3.14159265f * (float)i / (float)(N - 1));
      const float wy = 0.5f - 0.5f * cosf(2.0f * 3.14159265f * (float)j / (float)(N - 1));
      s_win[j * N + i] = wx * wy;
    }
  memset(s_g, 0, sizeof(float) * 2 * NN);
  float gsum = 0;
  for (int j = 0; j < N; j++)
    for (int i = 0; i < N; i++) {
      const float dx = (float)(i - N / 2), dy = (float)(j - N / 2);
      const float v = expf(-(dx * dx + dy * dy) / (2.0f * SIGMA * SIGMA));
      s_g[(j * N + i) * 2] = v; gsum += v;
    }
  for (int k = 0; k < NN; k++) s_g[k * 2] /= gsum;   // 归一化（响应峰值≈1）
  fft2(s_g, false);                                  // 期望响应转频域，之后训练时直接用
  s_on = true;
  return true;
}

void stop() { s_on = false; s_obj = 0; s_scale = 1.0f; }
bool active() { return s_on; }

void radius_norm(float* ru, float* rv) {
  const float rpx = 0.5f * (K_WIN - 1.0f) * s_obj;   // 源像素半径 = (W-s_obj)/2
  if (ru) *ru = s_gw > 0 ? rpx / (float)s_gw : 0.0f;
  if (rv) *rv = s_gh > 0 ? rpx / (float)s_gh : 0.0f;
}

bool mask_on() { return s_use_mask; }
bool sat_on() { return s_sat_on; }

float appear_at(const uint8_t* gray, int gw, int gh, float u, float v) {
  if (!s_on || !gray || gw != s_gw || gh != s_gh || s_obj <= 0.0f) return 0.0f;
  extract(gray, u * (float)gw, v * (float)gh, s_obj * K_WIN);
  return seed_ncc();
}

// ---------------- 起跟 ----------------
bool start(const uint8_t* gray, const uint8_t* chr, int gw, int gh, float u, float v, float obj_norm, float* psr) {
  if (!init() || !gray || gw < 8 || gh < 8) return false;
  s_gw = gw; s_gh = gh;
  s_chrp = chr; s_chrw = gw; s_chrh = gh;   // 供色度门采样（可为空）

  s_obj = obj_norm * (float)(gw > gh ? gw : gh);          // 目标边长(px)
  s_prev_obj = 0;                                         // 新目标：首帧无否决基准
  if (s_obj < 20.0f) s_obj = 20.0f;   // 目标尺寸下限：远距离框小 ⇒ s_obj 小 ⇒ 窗小 ⇒ 搜索半径不够、一步就出窗（用户实测"拉框小了容易跟丢"）。20px 保证半径≥2.5×20=50px
  s_cx = u * (float)gw; s_cy = v * (float)gh;
  s_scale = 1.0f;
  s_appear = 1.0f;
  s_use_mask = !carmask::in(u, v);   // 起跟点本身在车身里 ⇒ 目标就在夹爪上，这场别压响应

  // 色度加权门：只在此刻判定"目标本身有颜色"时才开 —— 目标中心小片的饱和度要明显高于窗四周。
  // 无彩色目标（白/黑方块）就自动不开门，保持原来的纯灰度行为。
  s_sat_on = false;
  if (s_chrp) {
    float core = 0;
    for (int dy = -1; dy <= 1; dy++)
      for (int dx = -1; dx <= 1; dx++) core += sample_chr(s_cx + (float)dx * s_obj * 0.25f, s_cy + (float)dy * s_obj * 0.25f);
    core /= 9.0f;
    float surr = 0;
    const float rr = s_obj * K_WIN * 0.5f * 0.85f;      // 窗内靠边一圈
    for (int a = 0; a < 8; a++) {
      const float an = (float)a * 0.7853982f;
      surr += sample_chr(s_cx + cosf(an) * rr, s_cy + sinf(an) * rr);
    }
    surr /= 8.0f;
    if (core - surr > SAT_MIN_DIFF) { s_sat_on = true; s_sat_ref = core; }
  }
  // 首帧：用当前窗直接"训练"滤波器（MOSSE 的第一帧就是直接用输入初始化 num/den）
  extract(gray, s_cx, s_cy, s_obj * K_WIN);
  seed_capture();          // ★ 顺便把"目标本来的样子"存下来（在 fft2 就地破坏 s_in 之前）
  fft2(s_in, false);
  memset(s_den, 0, sizeof(float) * 2 * NN);
  for (int k = 0; k < NN; k++) {
    const float fr = s_in[k * 2], fi = s_in[k * 2 + 1];
    const float gr = s_g[k * 2], gi = s_g[k * 2 + 1];
    s_num[k * 2]     = gr * fr + gi * fi;   // conj(G)·F
    s_num[k * 2 + 1] = gi * fr - gr * fi;
    s_den[k * 2]     = fr * fr + fi * fi;
    s_den[k * 2 + 1] = 0.0f;
  }
  if (psr) *psr = 0.0f;
  return true;
}

// ---------------- 单帧更新 ----------------
// 内部：按给定源边长 W 做一次"取窗→相关→响应分析"，输出峰值相对窗心的源像素偏移、PSR，
// 以及**峰值处的种子外观相似度**（在外观门里当判据用，所以必须按峰值位置量、不能用窗心）。
static bool correlate(const uint8_t* g, float cx, float cy, float W,
                      float* peak_val, float* off_x, float* off_y, float* psr, float* appear) {
  extract(g, cx, cy, W);
  fft2(s_in, false);
  for (int k = 0; k < NN; k++) {                       // R = H ⊙ F, H = num/(den+eps)
    const float dr = s_den[k * 2] + EPS;
    const float hr = s_num[k * 2] / dr, hi = s_num[k * 2 + 1] / dr;
    const float fr = s_in[k * 2], fi = s_in[k * 2 + 1];
    s_resp[k * 2]     = hr * fr - hi * fi;
    s_resp[k * 2 + 1] = hr * fi + hi * fr;
  }
  fft2(s_resp, true);                                   // 响应（取实部）
  // ★ 车身 mask：把落在**车/机械臂/夹爪**上的响应**压死**（-1e30）⇒ 那部分画面等于不存在，
  // 峰值永远落不进去，也就永远锁不到随车固定的夹爪上。这是唯一能挡住它的手段：夹爪的 PSR/外观分
  // 都可以很高（它确实是块稳定的好模板），只有"那块画面本来就是车自己"这条空间先验是硬的。
  // 敢用"压死"而不是"惩罚"，前提是 mask 只覆盖**目标到不了的地方** —— 所以 mask 只留 v≥0.63 那条带
  //（合爪点 v≈0.57、容差到 0.605，都在它上面）；早先那版把整团臂架也压上，结果方块在对准阶段从
  // 臂架格里穿过时被一起压掉，直接跟丢。
  // 安全阀：若窗里可用格太少（目标真跑到车身里了），这次就不压，免得把目标一起挡死、凭空丢。
  if (s_use_mask) {
    const float step = W / (float)N;
    const float ox = cx - W * 0.5f + step * 0.5f;
    const float oy = cy - W * 0.5f + step * 0.5f;
    int free_n = 0;
    for (int j = 0; j < N; j++) {
      const float vv = (oy + (float)j * step) / (float)s_gh;
      for (int i = 0; i < N; i++) {
        const float uu = (ox + (float)i * step) / (float)s_gw;
        if (!carmask::in(uu, vv)) free_n++;
      }
    }
    if (free_n >= NN / 4) {
      for (int j = 0; j < N; j++) {
        const float vv = (oy + (float)j * step) / (float)s_gh;
        for (int i = 0; i < N; i++) {
          const float uu = (ox + (float)i * step) / (float)s_gw;
          if (carmask::in(uu, vv)) s_resp[(j * N + i) * 2] = -1e30f;
        }
      }
    }
  }
  // ★ 色度加权门：只在"起跟时判定目标本身有颜色"时开。把**饱和度不像目标**的格子的正向响应压低 ——
  //   典型收益：地面反射的深色臂影在灰度里和深色方块几乎一样（真机实测：锁上去后车转 35° u 纹丝不动），
  //   但它的饱和度远低于红方块 ⇒ 被压下去。找目标仍靠 luma，色度只做**否决**。
  if (s_sat_on && s_chrp) {
    const float stp = W / (float)N;
    const float ox2 = cx - W * 0.5f + stp * 0.5f;
    const float oy2 = cy - W * 0.5f + stp * 0.5f;
    for (int j = 0; j < N; j++) {
      const float yy = oy2 + (float)j * stp;
      for (int i = 0; i < N; i++) {
        const float xx = ox2 + (float)i * stp;
        if (fabsf(sample_chr(xx, yy) - s_sat_ref) > SAT_TOL) {
          float* rv = &s_resp[(j * N + i) * 2];
          if (*rv > 0.0f) *rv *= SAT_PENALTY;   // 只压正向：负响应压了会抬高、反而可能变成峰
        }
      }
    }
  }
  int bi = 0, bj = 0; float bv = -1e30f;
  for (int j = 0; j < N; j++)
    for (int i = 0; i < N; i++) {
      const float v = s_resp[(j * N + i) * 2];
      if (v > bv) { bv = v; bi = i; bj = j; }
    }
  // 旁瓣统计（排除峰值周围 11×11，PSR 标准做法）
  double sum = 0, sq = 0; int cnt = 0;
  for (int j = 0; j < N; j++)
    for (int i = 0; i < N; i++) {
      int dx = i - bi; if (dx > N / 2) dx -= N; if (dx < -N / 2) dx += N;
      int dy = j - bj; if (dy > N / 2) dy -= N; if (dy < -N / 2) dy += N;
      if (dx >= -5 && dx <= 5 && dy >= -5 && dy <= 5) continue;
      const double v = s_resp[(j * N + i) * 2];
      if (v <= -1e29) continue;        // 被车身 mask 压死的格不参与旁瓣统计（否则会把 PSR 抬高）
      sum += v; sq += v * v; cnt++;
    }
  if (cnt < 16) return false;
  const double mu = sum / cnt;
  const double var = sq / cnt - mu * mu;
  const double sd = var > 1e-12 ? sqrt(var) : 1e-6;
  *psr = (float)((bv - mu) / sd);
  // 亚像素：峰值邻域抛物线拟合
  int iL = (bi + N - 1) % N, iR = (bi + 1) % N, jU = (bj + N - 1) % N, jD = (bj + 1) % N;
  const float vL = s_resp[(bj * N + iL) * 2], vR = s_resp[(bj * N + iR) * 2];
  const float vU = s_resp[(jU * N + bi) * 2], vD = s_resp[(jD * N + bi) * 2];
  float sx = 0, sy = 0;
  const float dx_ = 2.0f * bv - vL - vR;
  if (fabsf(dx_) > 1e-6f) sx = (vR - vL) / (2.0f * dx_);
  const float dy_ = 2.0f * bv - vU - vD;
  if (fabsf(dy_) > 1e-6f) sy = (vD - vU) / (2.0f * dy_);
  if (sx < -0.5f) sx = -0.5f; if (sx > 0.5f) sx = 0.5f;
  if (sy < -0.5f) sy = -0.5f; if (sy > 0.5f) sy = 0.5f;
  // 峰值网格坐标 → 相对窗中心的源像素偏移（解缠绕：超过 N/2 视为负）
  int di = bi - N / 2; if (di > N / 2) di -= N; if (di < -N / 2) di += N;
  int dj = bj - N / 2; if (dj > N / 2) dj -= N; if (dj < -N / 2) dj += N;
  const float step = W / (float)N;
  *off_x = ((float)di + sx) * step;
  *off_y = ((float)dj + sy) * step;
  *peak_val = bv;
  // ★ 外观：在**峰值位置**重新取窗再与种子比。目标常不在窗心（预测偏了更是如此），用窗心会量错。
  extract(g, cx + *off_x, cy + *off_y, W);
  *appear = seed_ncc();
  return true;
}

Upd update(const uint8_t* gray, const uint8_t* chr, int gw, int gh, float pred_u, float pred_v, bool wide, float need_psr, bool size_fix) {
  Upd r = { false, pred_u, pred_v, 0.0f, s_scale, s_obj, 0.0f, s_appear };
  if (!s_on || !gray || gw != s_gw || gh != s_gh || s_obj <= 0.0f) return r;
  s_chrp = chr; s_chrw = gw; s_chrh = gh;   // 本帧的饱和度平面（色度门用）

  const float pcx = pred_u * (float)gw, pcy = pred_v * (float)gh;
  const float* cand = wide ? SC_WIDE : SC_NORMAL;
  const int nc = wide ? 7 : 3;
  const float amin = wide ? APPEAR_MIN_REACQ : APPEAR_MIN;   // 重捕档的外观门更松（见 APPEAR_MIN_REACQ）

  float best_psr = -1e30f, best_W = 0, best_off_x = 0, best_off_y = 0, best_scale = s_scale, best_appear = 0.0f;
  // 另记一份"**通过了外观门**的最优解"。重捕档里 PSR 最高的往往正是随车固定的夹爪（它的 PSR 天生最高），
  // 但它跟种子外观不像 —— 只看 PSR 就永远抓回夹爪（真机实测：重捕后 u/v 又冻在夹爪上）。
  float ok_psr = -1e30f, ok_W = 0, ok_off_x = 0, ok_off_y = 0, ok_scale = s_scale, ok_appear = 0.0f;
  for (int c = 0; c < nc; c++) {
    const float W = s_obj * K_WIN * cand[c];
    float val = 0, offx = 0, offy = 0, psr = 0, appear = 0;
    if (!correlate(gray, pcx, pcy, W, &val, &offx, &offy, &psr, &appear)) continue;
    if (psr > best_psr) {
      best_psr = psr; best_W = W; best_off_x = offx; best_off_y = offy;
      best_scale = cand[c]; best_appear = appear;
    }
    if (appear >= amin && psr > ok_psr) {
      ok_psr = psr; ok_W = W; ok_off_x = offx; ok_off_y = offy;
      ok_scale = cand[c]; ok_appear = appear;
    }
  }
  if (best_psr < -1e29f) return r;
  // 有"像种子"的候选就用它（哪怕 PSR 略低）——这正是把重捕从夹爪上拉回目标的关键。
  if (ok_psr > -1e29f) {
    best_psr = ok_psr; best_W = ok_W; best_off_x = ok_off_x; best_off_y = ok_off_y;
    best_scale = ok_scale; best_appear = ok_appear;
  }
  r.psr = best_psr;
  r.appear = best_appear;
  s_appear = best_appear;
  // 诊断：峰值离窗心多远（占搜索半径比例）。接近/超过 1 ⇒ 目标已贴窗边（窗太小 或 单步位移太大），
  // 位置会被窗边截断 ⇒ 大误差。**在 PSR 门之前**算，好让被拒的帧也能暴露这个症状。
  {
    const float rad = 0.5f * (K_WIN - 1.0f) * s_obj;
    r.obj_px = s_obj;
    r.off_frac = rad > 0.5f
        ? sqrtf(best_off_x * best_off_x + best_off_y * best_off_y) / rad : 0.0f;
  }
  // ★ PSR 门：不达标的帧**一律不采纳位置、也不训练滤波器** —— 防"一帧坏匹配把 s_obj 缩小、窗口越缩越小"。
  if (best_psr < need_psr) {
    // 但**重捕档要允许只修正目标尺寸**，否则会死锁：逼近中目标变大 ⇒ s_obj 偏小 ⇒ 目标在网格里过大 ⇒
    // 匹配不上 ⇒ PSR 不达标 ⇒ 尺寸不许更新 ⇒ 永远修不回来（真机实测：倍率掉到 0.28、全图重捕也找不到）。
    // 这里只动尺寸（不动位置、不训练），下一帧就能拿对的尺寸重新匹配。
    // 尺寸修正只属于"丢失重捕"：预期宽窗（自己动作后的恢复期）PSR 失败时目标位置本就该在原地，
    // 此时按 best_scale 放缩会让 s_obj 两帧翻 4 倍（实测 39→109px），把后续跟踪全带歪。
    if (wide && size_fix) {
      float f = best_scale;
      if (f < 0.5f) f = 0.5f; else if (f > 2.0f) f = 2.0f;
      s_obj *= f;
      if (s_obj < 20.0f) s_obj = 20.0f;
      const float max_obj = (float)(gw < gh ? gw : gh) * 0.6f;
      if (s_obj > max_obj) s_obj = max_obj;
      s_scale *= f;
    }
    return r;
  }

  // 新位置 + 尺度（尺度平滑：一次跳变不超 ±20%，防单帧误判把窗带飞）
  // 先存旧值：外观门判"这不是目标了"时要把位置**退回上一次**（不采纳），否则窗会跟着错的东西一路走。
  const float old_cx = s_cx, old_cy = s_cy, old_obj = s_obj, old_scale = s_scale;
  s_cx = pcx + best_off_x;
  s_cy = pcy + best_off_y;
  if (s_cx < 0) s_cx = 0; if (s_cx > gw - 1) s_cx = (float)(gw - 1);
  if (s_cy < 0) s_cy = 0; if (s_cy > gh - 1) s_cy = (float)(gh - 1);
  // 尺度：正常档按"每帧最多 ±20%"平滑（防单帧误判把窗带飞）；**重捕档允许更大跳变** —— 它的候选本来
  // 就跨 0.5~4.0，锁回来了就该把找到的尺寸采用下来，否则下一帧又用错尺寸、再次丢。
  {
    float f = best_scale;
    const float lo = wide ? 0.5f : 0.8f, hi = wide ? 2.0f : 1.25f;
    if (f < lo) f = lo; else if (f > hi) f = hi;
    s_scale *= f;
    s_obj *= f;
    if (s_obj < 20.0f) s_obj = 20.0f;   // 目标尺寸下限：远距离框小 ⇒ s_obj 小 ⇒ 窗小 ⇒ 搜索半径不够、一步就出窗。20px 保证半径≥2.5×20=50px
    const float max_obj = (float)(gw < gh ? gw : gh) * 0.6f;
    if (s_obj > max_obj) s_obj = max_obj;
  }
  // ★ 尺寸跳变否决：目标本体是刚体，自己动作引起的尺寸变化每帧有限(±25%)。宽窗帧里峰值滑到
  // 旁边的大块纹理（臂架/箱体）时 best_scale 会翻倍 —— 尺寸跳变就是"不是同一个刚体"的指纹
  // （实测 38→76→152px 三帧滑上臂架，外观 0.5~0.6 的门拦不住）。丢失重捕档除外（它本来就要
  // 跨尺度找回）。按未跟上报，位置/尺寸/滤波器全部不动。
  if (!size_fix && s_prev_obj > 1.0f && s_obj > s_prev_obj * 1.6f) {
    s_cx = old_cx; s_cy = old_cy; s_obj = old_obj; s_scale = old_scale;
    return r;
  }

  // 学习门①（防漂移）：峰值离窗心太远（> 半径的 0.6）说明"目标已经贴到窗边"——多半是锁到了别的物体。
  bool learn_ok = (fabsf(best_off_x) < best_W * 0.3f) && (fabsf(best_off_y) < best_W * 0.3f);

  // ★ 外观门（**种子外观锚定**，真机实测定的）：门① 只保证"峰在窗心"，防不住"整条窗一起滑走"——
  // 锁到随车固定的夹爪时 conf/PSR 反而更高（夹爪是"完美模板"），门①过得去、置信度也高。
  // 唯一能发现"已经不是那个物体了"的参照是**起跟那一帧的外观**（r.appear 已按峰值位置算好）。
  //   真机实测：正常跟随稳定在 **0.59~1.00**；锁到夹爪后掉到 **0.30~0.45**。阈值取 0.50 卡在中间。
  // 不达标的帧**整个不采纳**（位置/尺度退回上一次、也不训练）：只"禁学"不够 —— 滤波器虽不再学错东西，
  // 位置却照样报，于是闭环冻在错的位置上一直前进（真机实测 u/v 冻结、连走 5 步）。
  // **重捕档同样过这道门**（上面候选选择已优先挑"像种子"的解）：否则重捕第一件事就是把夹爪抓回来。
  if (r.appear < amin) {
    s_cx = old_cx; s_cy = old_cy; s_obj = old_obj; s_scale = old_scale;
    return r;                                  // r.ok 仍 false ⇒ 计一次 miss
  }

  // 很像 ⇒ 把锚定往当前外观慢慢带（这样"正常逼近"不会被误判成"已经不是那个物体了"）。
  // 必须按**选中位置**重抽窗：correlate 里的 s_in 留着的是最后一个候选的窗，不一定是胜出的那个。
  if (!wide && r.appear >= APPEAR_LEARN) {
    extract(gray, s_cx, s_cy, s_obj * K_WIN);
    seed_blend();
  }

  // 在线更新滤波器：用**当前位置**的窗（相关滤波每帧都要重抽）
  if (!wide && learn_ok) {
    extract(gray, s_cx, s_cy, s_obj * K_WIN);
    fft2(s_in, false);
    for (int k = 0; k < NN; k++) {
      const float fr = s_in[k * 2], fi = s_in[k * 2 + 1];
      const float gr = s_g[k * 2], gi = s_g[k * 2 + 1];
      const float nr = gr * fr + gi * fi, ni = gi * fr - gr * fi;   // conj(G)·F
      const float dr = fr * fr + fi * fi;
      s_num[k * 2]     = ETA * nr + (1.0f - ETA) * s_num[k * 2];
      s_num[k * 2 + 1] = ETA * ni + (1.0f - ETA) * s_num[k * 2 + 1];
      s_den[k * 2]     = ETA * dr + (1.0f - ETA) * s_den[k * 2];
    }
  }

  r.ok = true;
  s_prev_obj = s_obj;   // 只有"确认采纳"的帧才更新尺寸基准（被外观门/尺寸否决回滚的不算）
  r.obj_px = s_obj;
  r.u = s_cx / (float)gw;
  r.v = s_cy / (float)gh;
  r.scale = s_scale;
  return r;
}

}  // namespace dcf
