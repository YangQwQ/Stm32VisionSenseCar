// ground_proj.cpp — 屏幕像素 → 地面坐标（单应矩阵）投影，namespace `ground`。
// 相机固定俯视车前地面：屏幕归一化 (u,v) → 车头系地面 cm（x=车右+, y=车前+）。
// 求解用 Householder QR：正规方程法在 ESP32 单精度 FPU 下条件数平方放大会崩
// （曾算出 H=(480,-567)）；QR 不放大条件数，double 软件模拟也够。

// ================== 实测标定点（改镜头/移相机后重测此表） ==================
// 每行一个坐标对：(屏幕归一化 u, v) → (车头系地面 x右+, y前+ cm)。
// 屏幕归一化：左上(0,0)、右下(1,1)；纵轴越往下越近（v 大→近），横轴 u 小→车左。
// 锚点参考：屏幕中心 (0.5,0.5)→车前正对 (0,52)；下方中线 (0.5,0.75)→车前偏左 (-8,21)。
// 增删点保持 ≥4 且尽量覆盖屏幕区域；加测点直接往表里加即可。
static const double CAL[][4] = {
  {0.75, 0.125,  78,  85},
  {0.50, 0.25,   40, 134},
  {0.75, 0.25,   45,  55},
  {0.25, 0.375, -50, 242},
  {0.25, 0.5,   -45, 112},
  {0.50, 0.5,    0,   52},
  {0.75, 0.5,    16,  26},
  {1.0,  0.5,    23,  13},
  {0.0,  0.75,  -105, 90},
  {0.25, 0.75,  -38,  48},
  {0.50, 0.75,  -8,   21},
  {0.75, 0.75,   1.5, 12},
  {1.0,  0.75,   7,    6},
  {0.0,  1.0,   -64,  36},
  {0.25, 1.0,   -34,  23},
  {0.50, 1.0,   -15,  14}
};
#define CAL_N (int)(sizeof(CAL) / sizeof(CAL[0]))

#include "ground_proj.h"
#include <math.h>  // sqrt/fabs

// 当前生效的单应 + 归一化参数（QR 求解后为动态值）
static double H[8];
static double SU_MU, SU_S, SV_MU, SV_S, SX_MU, SX_S, SY_MU, SY_S;
static bool s_ready = false;

// Householder QR 求解超定最小二乘 Ah≈b（m=2N 方程, n=8 未知）→ h 写回 H。
// A 按列主元反射逐步上三角化，b 同步施加反射，最后回代。返回 false=奇异/数值失败。
static bool qr_fit(const double A[2 * CAL_N][8], const double b[2 * CAL_N], double h[8]) {
  const int m = 2 * CAL_N, n = 8;
  double R[32][8];   // m×n 工作副本（CAL_N≤16 时 m≤32）
  double qb[32];
  for (int i = 0; i < m; i++) { for (int j = 0; j < n; j++) R[i][j] = A[i][j]; qb[i] = b[i]; }
  for (int k = 0; k < n; k++) {
    double n2 = 0;
    for (int i = k; i < m; i++) n2 += R[i][k] * R[i][k];
    if (n2 < 1e-30) return false;
    double alpha = (R[k][k] >= 0 ? -1 : 1) * sqrt(n2);
    R[k][k] -= alpha;                              // v = x - alpha·e1（存 R[k..][k]），归一化
    double vn = 0;
    for (int i = k; i < m; i++) vn += R[i][k] * R[i][k];
    vn = sqrt(vn);
    if (vn < 1e-15) { R[k][k] = alpha; continue; } // 该列已正交，跳过
    for (int i = k; i < m; i++) R[i][k] /= vn;
    for (int j = k + 1; j < n; j++) {              // 反射作用到右侧列与 b
      double dot = 0;
      for (int i = k; i < m; i++) dot += R[i][k] * R[i][j];
      for (int i = k; i < m; i++) R[i][j] -= 2 * dot * R[i][k];
    }
    double db = 0;
    for (int i = k; i < m; i++) db += R[i][k] * qb[i];
    for (int i = k; i < m; i++) qb[i] -= 2 * db * R[i][k];
    R[k][k] = alpha;                               // 存 R 对角
  }
  for (int i = n - 1; i >= 0; i--) {               // 回代解 R h = qb（R 上三角在 R[0..n-1][0..n-1]）
    double s = qb[i];
    for (int j = i + 1; j < n; j++) s -= R[i][j] * h[j];
    if (fabs(R[i][i]) < 1e-15) return false;
    h[i] = s / R[i][i];
  }
  return true;
}

// 启动：QR 求解归一化单应；回验校准点误差 ≤5cm 则启用，否则禁用像素观测。
bool ground::init(void) {
  // 归一化参数（均值 + 平均绝对偏差）
  double um = 0, vm = 0, xm = 0, ym = 0;
  for (int i = 0; i < CAL_N; i++) { um += CAL[i][0]; vm += CAL[i][1]; xm += CAL[i][2]; ym += CAL[i][3]; }
  um /= CAL_N; vm /= CAL_N; xm /= CAL_N; ym /= CAL_N;
  double us = 0, vs = 0, xs = 0, ys = 0;
  for (int i = 0; i < CAL_N; i++) {
    us += fabs(CAL[i][0] - um); vs += fabs(CAL[i][1] - vm);
    xs += fabs(CAL[i][2] - xm); ys += fabs(CAL[i][3] - ym);
  }
  us = us / CAL_N; vs = vs / CAL_N; xs = xs / CAL_N; ys = ys / CAL_N;
  if (us < 1e-9) us = 1; if (vs < 1e-9) vs = 1; if (xs < 1e-9) xs = 1; if (ys < 1e-9) ys = 1;

  double A[32][8], b[32];
  for (int i = 0; i < CAL_N; i++) {
    double u = (CAL[i][0] - um) / us, v = (CAL[i][1] - vm) / vs;
    double x = (CAL[i][2] - xm) / xs, y = (CAL[i][3] - ym) / ys;
    A[2 * i][0] = u; A[2 * i][1] = v; A[2 * i][2] = 1; A[2 * i][3] = 0;
    A[2 * i][4] = 0; A[2 * i][5] = 0; A[2 * i][6] = -u * x; A[2 * i][7] = -v * x; b[2 * i] = x;
    A[2 * i + 1][0] = 0; A[2 * i + 1][1] = 0; A[2 * i + 1][2] = 0; A[2 * i + 1][3] = u;
    A[2 * i + 1][4] = v; A[2 * i + 1][5] = 1; A[2 * i + 1][6] = -u * y; A[2 * i + 1][7] = -v * y; b[2 * i + 1] = y;
  }

  if (!qr_fit(A, b, H)) {
    Serial.println("[ground] 单应QR求解失败，像素观测禁用（px/py 观测将回退 rel_deg）");
    s_ready = false;
    return false;
  }
  SU_MU = um; SU_S = us; SV_MU = vm; SV_S = vs; SX_MU = xm; SX_S = xs; SY_MU = ym; SY_S = ys;
  s_ready = true;
  float maxerr = 0;
  for (int i = 0; i < CAL_N; i++) {
    float ex, ey;
    screen_to_world((float)CAL[i][0], (float)CAL[i][1], &ex, &ey);
    float e = sqrtf((ex - (float)CAL[i][2]) * (ex - (float)CAL[i][2]) +
                    (ey - (float)CAL[i][3]) * (ey - (float)CAL[i][3]));
    if (e > maxerr) maxerr = e;
  }
  if (maxerr <= 5.f) { Serial.printf("[ground] 单应QR求解成功 回验最大误差=%.1fcm\n", maxerr); return true; }
  Serial.printf("[ground] 单应QR回验超差(%.1fcm)，像素观测禁用\n", maxerr);
  s_ready = false;
  return false;
}

bool ground::ready() { return s_ready; }

// 屏幕归一化像素 (u,v) → 车头系地面 (x右+, y前+) cm。
// 单应只在标定区域内可信，区域外分母趋零会剧烈外推（AI 报错/越界像素时可达数千 cm）。
// 故限制：输入须在 0..1，输出须在可接受地面范围，否则返回 false（上层回退 rel_deg）。
bool ground::screen_to_world(float u, float v, float* x, float* y) {
  if (!s_ready) return false;
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