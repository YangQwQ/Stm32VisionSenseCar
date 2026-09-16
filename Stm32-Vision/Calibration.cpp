#include "Calibration.h"

// 供应 bivar 的机械臂散点数据源：把集中式散点表包装成 bivar::ArmSet。
// (原 arm_cal.cpp 迁来：数据本体在 Calibration.h 顶部 kArmPts/kArmPtN)
const bivar::ArmSet* bivar::arm_set(void) {
  static const bivar::ArmSet s = { (uint8_t)kArmPtN, kArmPts };
  return &s;
}