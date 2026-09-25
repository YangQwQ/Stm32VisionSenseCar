#include "src/ai/tools/tool.h"
#include "src/ai/ai_validate.h"   // AI_MOVE_MAX_CM / AI_SPIN_MAX_DEG

#include <Arduino.h>
#include <string.h>

// move 通道(轮子): throttle=直行 / spin=原地转 / approach=自动靠近, 三选一。
// ⚠️ 本函数是 `move` 的**唯一**规范化出口 —— 返回后程序不再改写它的任何键。
const char* ai::parse_move(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  if (!root["move"].is<JsonObjectConst>()) return nullptr;
  JsonObjectConst src = root["move"].as<JsonObjectConst>();
  const char* mt = src["type"] | "";
  if (strcmp(mt, "throttle") && strcmp(mt, "spin") && strcmp(mt, "approach")) {
    snprintf(err, cap, "AI move 通道非法 type=%s(须 throttle/spin/approach)", mt);
    return err;
  }
  JsonObject m = dst["move"].to<JsonObject>();
  m["type"] = mt;
  if (!strcmp(mt, "throttle")) {
    float th = src["throttle"] | 0.0f;
    float st = src["steering"] | 0.0f;
    m["throttle"] = constrain(th, -1.0f, 1.0f);
    m["steering"] = constrain(st, -1.0f, 1.0f);
    int dc = src["distance_cm"] | 0;
    if (src["distance_cm"].is<int>() && dc) m["distance_cm"] = constrain(dc, 0, AI_MOVE_MAX_CM);
  } else if (!strcmp(mt, "spin")) {
    int sd = src["dir"] | 0;
    int ss = src["speed"] | 800;   // 原地旋转默认转速(全系统统一档)
    m["dir"] = constrain(sd, -1, 1);
    m["speed"] = constrain(ss, 0, 1000);
    // 定角取绝对值、为 0 不写键; 方向由 dir 定。
    // ⚠️ 不写 `angle_deg` 的**原意是"持续旋转"**(靠后续指令或时限收尾), 所以"不写键"是对的。
    //    但 `round_land` 里另有一段"缺键就补默认角"的改写 —— 那才是坏味道(等于改写模型意图),
    //    待阶段 1 后续批次连同 `distance_cm` 的默认补写一起删; 删时这句注释一并改掉。
    if (src["angle_deg"].is<int>()) {
      int ad = src["angle_deg"] | 0;
      int mag = ad < 0 ? -ad : ad;
      if (mag > 0) m["angle_deg"] = constrain(mag, 1, AI_SPIN_MAX_DEG);
    }
  } else {  // approach
    const char* tgt = src["target"] | "";
    if (tgt[0]) m["target"] = tgt;
  }
  return nullptr;
}
