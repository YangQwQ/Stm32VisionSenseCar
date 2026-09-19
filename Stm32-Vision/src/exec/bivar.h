#pragma once
#include <Arduino.h>

// ================= 通用散点标定插值骨架（namespace `bivar`） =================
// 约定（含 ground_proj 在内共用）：标定点表存文件头部 → setup 期 init（置 ready 标志
// → 返回布尔的查询接口）。机械臂用它做"双向"：FK=(pwm,pwm)→(x,h)、IK=(x,h)→(pwm,pwm)，
// 均为散点反距离加权插值；ground_proj 用同骨架但保留其单应 QR 内核。
//
// 机械臂坐标（车头系）：夹心 x=车头前向 cm、h=离地 cm。标定点为散点（PWM 对夹心坐标），
// 无需规则网格，超出实测范围（距最近点过远）判不可达返回 false。
namespace bivar {

// 机械臂标定散点：手感内可线性移动的实测点（数据在 Calibration.h 的 kArmPts，只读）
typedef struct {
  float r, l;   // 舵机 PWM：r=移爪 reach / l=抬落 lift
  float x, h;   // 夹心坐标：x=车头前向 cm / h=离地 cm
} CalPt;

typedef struct {
  uint8_t n;          // 标定点数
  const CalPt* pts;   // 散点数组
} ArmSet;

// 数据源接入点：返回标定散点集（实现于 Calibration.cpp）
const ArmSet* arm_set(void);

// setup 期调用一次：校验点数足够、计算特征尺寸（平均最近邻间距）供范围判定，置 ready。
bool arm_init(void);
bool arm_ready(void);

// 正向 FK：(reach_pwm, lift_pwm) → (x, h)。PWM 越标定区间时夹回区间内插值（饱和钳制，
// 返回夹到边界的位姿，不拒绝），供反馈/状态在边界稳定读值、避免抖动。返回 false 仅当未 ready。
bool arm_fk(float reach_pwm, float lift_pwm, float* x, float* h);

// 反向 IK：(x, h) → (reach_pwm, lift_pwm)。在 x/h 空间按测点加权插值，超范围返回 false。
bool arm_ik(float x, float h, float* reach_pwm, float* lift_pwm);

// 把目标夹心 (x,h) 夹到标定可达盒（各轴 min..max）内，返回是否有改动。
// 配合 IK 把"撞边界时直接拒绝"改成"移到盒内最近的合法点再反解"。
bool arm_clamp(float* x, float* h);

}  // namespace bivar