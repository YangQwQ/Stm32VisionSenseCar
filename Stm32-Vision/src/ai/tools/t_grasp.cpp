#include "src/ai/tools/tool.h"

#include <Arduino.h>
#include <string.h>

// grasp 通道：**板端本地自动夹取**（板端自己 arm low → 转正对准 u → 边前进边对准到 v → 合爪）。
// 与 arm.grasp 的区别：arm.grasp 只做"合爪+抬臂"（车不动，靠 AI 先前把车对准、把距离凑近）；
// 本工具把整套逼近/对准/合爪交给板端闭环（纯画面、不经云端、每步 ~0.5s），
// ⇒ **目标清晰可见、未被遮挡时应优先用它**（比 AI 一步步微操快得多、也稳）。
// x,y = 目标在画面上的归一化位置（0~1，一般是 observe/look 看到的目标坐标）；w,h = 可选框宽高（0=缺省框）。
// ⚠️ 目标被遮挡（夹爪/车身挡住、在画面边缘、或太远太小）时别用它 —— 追踪靠画面，看不见就跟不住。
const char* ai::parse_grasp(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  if (!root["grasp"].is<JsonObjectConst>()) return nullptr;
  JsonObjectConst src = root["grasp"].as<JsonObjectConst>();
  const float x = src["x"] | -1.0f, y = src["y"] | -1.0f;
  if (!(x >= 0.0f && x <= 1.0f) || !(y >= 0.0f && y <= 1.0f)) {
    snprintf(err, cap, "AI grasp 需要 x/y = 目标在画面上的归一化位置(0~1)；收到 (%.3f,%.3f)", (double)x, (double)y);
    return err;
  }
  JsonObject g = dst["grasp"].to<JsonObject>();
  g["x"] = x; g["y"] = y;
  const float w = src["w"] | 0.0f, h = src["h"] | 0.0f;
  if (w > 0.0f && w <= 1.0f) g["w"] = w;
  if (h > 0.0f && h <= 1.0f) g["h"] = h;
  const char* nm = src["name"] | "";
  if (nm[0]) g["name"] = nm;
  return nullptr;
}
