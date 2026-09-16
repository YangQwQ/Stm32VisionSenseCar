// ground_proj.cpp — 屏幕像素 → 地面坐标（单应矩阵）投影，namespace `ground`。
// 相机固定俯视车前地面：屏幕归一化 (u,v) → 车头系地面 cm（x=车右+, y=车前+）。
// 求解用 Householder QR：正规方程法在 ESP32 单精度 FPU 下条件数平方放大会崩
// （曾算出 H=(480,-567)）；QR 不放大条件数，double 软件模拟也够。

#include "Calibration.h"   // 屏幕→地面单应标定点（Calibration.h 集中，加测点改那里即可）
#include "src/ai/ground_proj.h"
#include "src/core/board_log.h"
#include <math.h>  // sqrt/fabs

// 当前生效的单应 + 归一化参数（QR 求解后为动态值）
static double H[8];
static double SU_MU, SU_S, SV_MU, SV_S, SX_MU, SX_S, SY_MU, SY_S;
static bool s_ready = false;

// Householder QR 求解超定最小二乘 Ah≈b（m=2N 方程, n=8 未知）→ h 写回 H。
// A 按列主元反射逐步上三角化，b 同步施加反射，最后回代。返回 false=奇异/数值失败。
static bool qr_fit(const double A[2 * kGroundCalN][8], const double b[2 * kGroundCalN], double h[8]) {
  const int m = 2 * kGroundCalN, n = 8;
  double R[32][8];   // m×n 工作副本（kGroundCalN≤16 时 m≤32）
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
  for (int i = 0; i < kGroundCalN; i++) { um += kGroundCal[i][0]; vm += kGroundCal[i][1]; xm += kGroundCal[i][2]; ym += kGroundCal[i][3]; }
  um /= kGroundCalN; vm /= kGroundCalN; xm /= kGroundCalN; ym /= kGroundCalN;
  double us = 0, vs = 0, xs = 0, ys = 0;
  for (int i = 0; i < kGroundCalN; i++) {
    us += fabs(kGroundCal[i][0] - um); vs += fabs(kGroundCal[i][1] - vm);
    xs += fabs(kGroundCal[i][2] - xm); ys += fabs(kGroundCal[i][3] - ym);
  }
  us = us / kGroundCalN; vs = vs / kGroundCalN; xs = xs / kGroundCalN; ys = ys / kGroundCalN;
  if (us < 1e-9) us = 1; if (vs < 1e-9) vs = 1; if (xs < 1e-9) xs = 1; if (ys < 1e-9) ys = 1;

  double A[32][8], b[32];
  for (int i = 0; i < kGroundCalN; i++) {
    double u = (kGroundCal[i][0] - um) / us, v = (kGroundCal[i][1] - vm) / vs;
    double x = (kGroundCal[i][2] - xm) / xs, y = (kGroundCal[i][3] - ym) / ys;
    A[2 * i][0] = u; A[2 * i][1] = v; A[2 * i][2] = 1; A[2 * i][3] = 0;
    A[2 * i][4] = 0; A[2 * i][5] = 0; A[2 * i][6] = -u * x; A[2 * i][7] = -v * x; b[2 * i] = x;
    A[2 * i + 1][0] = 0; A[2 * i + 1][1] = 0; A[2 * i + 1][2] = 0; A[2 * i + 1][3] = u;
    A[2 * i + 1][4] = v; A[2 * i + 1][5] = 1; A[2 * i + 1][6] = -u * y; A[2 * i + 1][7] = -v * y; b[2 * i + 1] = y;
  }

  if (!qr_fit(A, b, H)) {
    blog::logf(blog::CAM, "单应QR求解失败，像素观测禁用（px/py 观测将回退 rel_deg）");
    s_ready = false;
    return false;
  }
  SU_MU = um; SU_S = us; SV_MU = vm; SV_S = vs; SX_MU = xm; SX_S = xs; SY_MU = ym; SY_S = ys;
  s_ready = true;
  float maxerr = 0;
  for (int i = 0; i < kGroundCalN; i++) {
    float ex, ey;
    screen_to_world((float)kGroundCal[i][0], (float)kGroundCal[i][1], &ex, &ey);
    float e = sqrtf((ex - (float)kGroundCal[i][2]) * (ex - (float)kGroundCal[i][2]) +
                    (ey - (float)kGroundCal[i][3]) * (ey - (float)kGroundCal[i][3]));
    if (e > maxerr) maxerr = e;
  }
  if (maxerr <= 5.f) { blog::logf(blog::CAM, "单应QR求解成功 回验最大误差=%.1fcm", maxerr); return true; }
  blog::logf(blog::CAM, "单应QR回验超差(%.1fcm)，像素观测禁用", maxerr);
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