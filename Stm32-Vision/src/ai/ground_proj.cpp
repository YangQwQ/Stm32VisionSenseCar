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
  static double R[2 * kGroundCalN][8];   // m×n 工作副本，尺寸随标定点数推导（勿写死 16×2）
  static double qb[2 * kGroundCalN];     // 静态：不压 loopTask 栈（见 ground::init 注释）
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

  // 大工作数组不放栈：loopTask 栈 8K，嵌套 init→qr_fit 两套 [2N][8] 易溢出（见历史 panic）。
  // 静态放 .bss（仅 init 期用一次，无并发），省下 ≈6KB 栈。
  static double A[2 * kGroundCalN][8], b[2 * kGroundCalN];
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
  // QR 成功即启用像素观测。回验误差仅作精度日志、不再因此禁用：相机略斜/远点像素点击误差
  // 会让个别标定点残差 >5cm，但整体最短二乘仍可用；禁用会让 AI 退回只有方位的 rel_deg，
  // 反而损失距离。screen_to_world 自带 0..1 与地面范围护栏防外推，误差大只致坐标偏差、不崩溃。
  if (maxerr <= 5.f) { blog::logf(blog::CAM, "单应QR求解成功 回验最大误差=%.1fcm", maxerr); return true; }
  blog::logf(blog::CAM, "单应QR回验超差(%.1fcm)，仍启用像素观测（坐标偏差上限≈该值）", maxerr);
  return true;
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

// 齐次坐标 3×3 矩阵求逆(伴随矩阵/Frob 形式, 纯解析, 无迭代)。
// 返回 det; 若 |det| 过小(奇异)返回 0, 调用方判失败。
static double inv3(const float a[9], float r[9]) {
  double d = a[0] * ((double)a[4] * a[8] - (double)a[7] * a[5])
           - a[1] * ((double)a[3] * a[8] - (double)a[6] * a[5])
           + a[2] * ((double)a[3] * a[7] - (double)a[6] * a[4]);
  if (fabs(d) < 1e-12) return 0;
  r[0] = (float)(((double)a[4] * a[8] - (double)a[7] * a[5]) / d);
  r[1] = (float)(((double)a[2] * a[7] - (double)a[1] * a[8]) / d);
  r[2] = (float)(((double)a[1] * a[5] - (double)a[2] * a[4]) / d);
  r[3] = (float)(((double)a[6] * a[5] - (double)a[3] * a[8]) / d);
  r[4] = (float)(((double)a[0] * a[8] - (double)a[2] * a[6]) / d);
  r[5] = (float)(((double)a[2] * a[3] - (double)a[0] * a[5]) / d);
  r[6] = (float)(((double)a[3] * a[7] - (double)a[6] * a[4]) / d);
  r[7] = (float)(((double)a[6] * a[1] - (double)a[0] * a[7]) / d);
  r[8] = (float)(((double)a[0] * a[4] - (double)a[1] * a[3]) / d);
  return d;
}

// 车头系地面 (x右+, y前+) cm → 屏幕归一化像素 (u,v)。
// 单应对称可逆: 正向 A·(un,vn,1)∝(xn,yn,1), 反解即 A⁻¹·(xn,yn,1)。归一化 (mean/std) 亦然。
// 精度: 记忆坐标本就是无里程计的推算值+单应前瞻误差(几 cm), 投影回屏幕也就对应几十像素的
// 位置指示 —— 只用于"去画面那个大概位置看一眼目标在不在", 不做导航, 这点误差可接受。
bool ground::world_to_screen(float x, float y, float* u, float* v) {
  if (!s_ready) return false;
  float xn = (x - SX_MU) / SX_S, yn = (y - SY_MU) / SY_S;   // 归一化到标定区
  float A[9] = { H[0], H[1], H[2], H[3], H[4], H[5], H[6], H[7], 1.0f };  // 行优先 3×3
  float Ai[9];
  if (inv3(A, Ai) == 0) return false;
  // (un, vn, 1) ∝ Ai·(xn, yn, 1), 除 w 得非齐次
  double w = Ai[6] * xn + Ai[7] * yn + Ai[8];
  if (fabs(w) < 1e-9) return false;
  double un = (Ai[0] * xn + Ai[1] * yn + Ai[2]) / w;
  double vn = (Ai[3] * xn + Ai[4] * yn + Ai[5]) / w;
  float uu = (float)(un * SU_S + SU_MU);        // 反归一化回 0..1 屏幕
  float vv = (float)(vn * SV_S + SV_MU);
  if (uu < -0.05f || uu > 1.05f || vv < -0.05f || vv > 1.05f) return false;  // 标定区外不指示
  *u = uu; *v = vv;
  return true;
}