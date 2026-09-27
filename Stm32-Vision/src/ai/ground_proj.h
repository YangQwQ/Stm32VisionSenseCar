#pragma once
#include <Arduino.h>

// 屏幕 → 地面坐标换算(单应投影)。相机固定俯视车前地面: 屏幕归一化 (u,v) → 车头系地面 cm
// (x=车右+, y=车前+), 由标定点最小二乘拟合 3×3 单应。标定点见 Calibration.h, 加测点改那里。
namespace ground {

// 启动拟合(QR 求解 + 回验)并打印诊断, setup 期调用一次。失败则禁用像素观测(ready()=false),
// 上层 px/py 观测一律解算不成。
bool init();

// 单应是否就绪。就绪后 px/py 像素观测方可解算；未就绪时 observe 的位置一律记不成。
bool ready();

// 屏幕归一化像素 (u,v) → 车头系地面 (x右+, y前+) cm。
// 输入越界(非 0..1)或输出超可接受范围(单应外推分母趋零会爆炸)时返回 false。
bool screen_to_world(float u, float v, float* x, float* y);

// 车头系地面 (x右+, y前+) cm → 屏幕归一化像素 (u,v)。单应正变换的可逆映射。
// 仅作指示型定位(去画面大概位置看一眼), 不用于导航 —— 屏幕像素误差不敏感。越界/数值失败返回 false。
bool world_to_screen(float x, float y, float* u, float* v);

}  // namespace ground