#include "src/ai/tools/tool.h"

#include <Arduino.h>
#include <string.h>

// arm 通道(机械臂): low/raise/fold/grasp/clip/release/pose 单选; 固定抬臂用 raise, 显式坐标用 pose。
// ⚠️ 这张动作名表是**唯一一份** —— 旧代码里同样的名单在 `ai_validate.cpp` 与 `ai_round.cpp` 各存了一份,
//    加一个动作必须两处同时改, 漏一处就是"校验通过但落地静默忽略"。
const char* ai::parse_arm(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  if (!root["arm"].is<JsonObjectConst>()) return nullptr;
  JsonObjectConst src = root["arm"].as<JsonObjectConst>();
  const char* act = src["type"] | "";
  static const char* acts[] = {"low", "raise", "fold", "grasp", "clip", "release", "pose", nullptr};
  bool good = false;
  for (int i = 0; acts[i]; i++) if (!strcmp(act, acts[i])) { good = true; break; }
  if (!good) {
    snprintf(err, cap,
             "AI arm 非法 type=%s(可用 low/raise/fold/grasp/clip/release/pose; 持续抬落/伸缩已移除)", act);
    return err;
  }
  JsonObject a = dst["arm"].to<JsonObject>();
  a["type"] = act; a["act"] = act;   // act 供 exec::act("arm") 与日志摘要复用旧键
  if (!strcmp(act, "pose")) {
    float px = src["x"] | 0.0f;
    float ph = src["h"] | 0.0f;
    if (src["x"].is<float>() || src["x"].is<int>()) a["x"] = constrain(px, 0.0f, 20.0f);
    if (src["h"].is<float>() || src["h"].is<int>()) a["h"] = constrain(ph, -2.0f, 25.0f);
  }
  return nullptr;
}
