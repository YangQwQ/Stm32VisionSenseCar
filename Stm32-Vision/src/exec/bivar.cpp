#include "src/exec/bivar.h"
#include <math.h>  // fabs/hypotf

// 机械臂"夹心坐标"双向散点反距离加权(IDW)插值。
// 数据源 Calibration.h 的 kArmPts（只读）由 arm_set() 提供。FK 用 pwm 距、IK 用 x/h 距做加权。
// 可达域判定用标定点各轴范围（正向/反向包围盒）：索引点在已标定区间内即视为可达
// （网格很密，IDW 插值可靠）；区间外才判不可达。不用"平均最近邻×K"——那会把网格角区
// 内、距最近点仅 1~2cm 的合法点误判为超范围。

namespace bivar {

static const ArmSet* S = nullptr;
static bool s_ready = false;
// 各轴 min/max（标定覆盖范围），arm_init 计算一次，供可达域判定
static float s_r_min = 0, s_r_max = 0, s_l_min = 0, s_l_max = 0;
static float s_x_min = 0, s_x_max = 0, s_h_min = 0, s_h_max = 0;

bool arm_init(void) {
  S = arm_set();
  if (!S || !S->pts || S->n < 3) { s_ready = false; return false; }
  s_x_min = s_h_min = 1e30f;
  s_x_max = s_h_max = -1e30f;
  for (uint8_t i = 0; i < S->n; i++) {
    if (S->pts[i].r < s_r_min) s_r_min = S->pts[i].r;
    if (S->pts[i].r > s_r_max) s_r_max = S->pts[i].r;
    if (S->pts[i].l < s_l_min) s_l_min = S->pts[i].l;
    if (S->pts[i].l > s_l_max) s_l_max = S->pts[i].l;
    if (S->pts[i].x < s_x_min) s_x_min = S->pts[i].x;
    if (S->pts[i].x > s_x_max) s_x_max = S->pts[i].x;
    if (S->pts[i].h < s_h_min) s_h_min = S->pts[i].h;
    if (S->pts[i].h > s_h_max) s_h_max = S->pts[i].h;
  }
  s_ready = true;
  return true;
}

bool arm_ready(void) { return s_ready; }

bool arm_fk(float reach_pwm, float lift_pwm, float* x, float* h) {
  if (!s_ready) return false;
  // 越标定 PWM 区间也夹回区间再插值、不拒绝（饱和钳制）：让反馈/状态在边界能读到"夹到边界
  // 的真实位置"。否则过程顶破数据范围时 FK 返回 false 使反馈失锚 → 边界抖动（logic/target 来回跳）。
  if (reach_pwm < s_r_min) reach_pwm = s_r_min; else if (reach_pwm > s_r_max) reach_pwm = s_r_max;
  if (lift_pwm  < s_l_min) lift_pwm  = s_l_min; else if (lift_pwm  > s_l_max) lift_pwm  = s_l_max;
  // IDW：以 pwm 距离平方的倒数加权，最近点占主导，测点处精确穿过
  float sw = 0, sx = 0, sh = 0;
  const float eps = 1e-3f;
  for (uint8_t i = 0; i < S->n; i++) {
    float d = hypotf(S->pts[i].r - reach_pwm, S->pts[i].l - lift_pwm);
    float w = 1.f / (d * d + eps);
    sw += w; sx += w * S->pts[i].x; sh += w * S->pts[i].h;
  }
  *x = sx / sw; *h = sh / sw;
  return true;
}

bool arm_ik(float x, float h, float* reach_pwm, float* lift_pwm) {
  if (!s_ready) return false;
  // 反可达域：目标夹心在标定 x/h 区间内即可插值；区间外（含测点稀疏角区外）判不可达。
  // 超界由调用方(direct_exec)按物理舵机限位再做第二道拦截。
  if (x < s_x_min || x > s_x_max ||
      h < s_h_min || h > s_h_max) return false;
  // IDW 反插：在 x/h 空间加权求 pwm（单一权重作用于两个输出轴）
  float sw = 0, sr = 0, sl = 0;
  const float eps = 1e-3f;
  for (uint8_t i = 0; i < S->n; i++) {
    float d = hypotf(S->pts[i].x - x, S->pts[i].h - h);
    float w = 1.f / (d * d + eps);
    sw += w; sr += w * S->pts[i].r; sl += w * S->pts[i].l;
  }
  *reach_pwm = sr / sw; *lift_pwm = sl / sw;
  return true;
}

bool arm_clamp(float* x, float* h) {
  if (!s_ready || !x || !h) return false;
  bool changed = false;
  if (*x < s_x_min) { *x = s_x_min; changed = true; }
  else if (*x > s_x_max) { *x = s_x_max; changed = true; }
  if (*h < s_h_min) { *h = s_h_min; changed = true; }
  else if (*h > s_h_max) { *h = s_h_max; changed = true; }
  return changed;
}

}  // namespace bivar