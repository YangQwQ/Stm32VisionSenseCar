#pragma once
#include <Arduino.h>

// 屏幕 → 地面坐标换算模块（单应投影，namespace `ground`）。
// 相机固定俯视车前地面：屏幕归一化像素 (u,v) → 车头系地面坐标 cm（x=车右+, y=车前+），
// 由实测标定点最小二乘拟合 3×3 单应（8 自由度），天然吸收相机安装偏差。
// 标定点作为"一个个坐标"放在 ground_proj.cpp 头部区域，加测点直接往表里加即可。
namespace ground {

// 启动拟合（QR 求解 + 回验，误差 ≤5cm 才启用），并打印诊断。setup 期调用一次。
// 失败/超差时禁用像素观测（ready()=false），上层 px/py 观测自动回退 rel_deg。
bool init();

// 单应是否就绪。就绪后 px/py 像素观测方可解算；未就绪时上层应回退 rel_deg 兜底。
bool ready();

// 屏幕归一化像素 (u,v) → 车头系地面 (x右+, y前+) cm。
// 输入越界(非 0..1)或输出超可接受范围（单应外推分母趋零会爆炸）时返回 false。
bool screen_to_world(float u, float v, float* x, float* y);

}  // namespace ground