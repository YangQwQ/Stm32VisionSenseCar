#include "src/ai/tools/tool.h"
#include "src/ai/ai_validate.h"   // AI_MOVE_MAX_CM / AI_SPIN_MAX_DEG
#include "src/ai/ai_client.h"     // ai::logf(补默认值时留一条日志)

#include <Arduino.h>
#include <math.h>
#include <string.h>

// 未给 distance_cm/angle_deg 时补有限段长/角度(而非放开成"持续"): 持续动作不累积位移与车向 ⇒
// 记忆里车位置/朝向失同步, 之后喂回的物体坐标全偏; 补有限值后走多远、转多少始终已知。
#define AI_MOVE_DEFAULT_CM 8
#define AI_SPIN_DEFAULT_DEG 30

// move 通道(轮子): throttle=直行 / spin=原地转 / approach=自动靠近, 三选一。
// ⚠️ 本函数是 `move` 的**唯一**规范化出口 —— 返回后程序不再改写它的任何键(默认值补全也归这里)。
//    注意语义: 不写 distance_cm/angle_deg 的原意是**持续动作**, 故补默认值是**改写模型意图**
//    (已知坏味道); 若改成"不补写、位姿估计仍按估算值累积", 须连 car_update_pose 一起动。
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
    if (src["distance_cm"].is<int>() && dc) {
      m["distance_cm"] = constrain(dc, 0, AI_MOVE_MAX_CM);
    } else if (fabsf(th) > 0.001f) {
      m["distance_cm"] = AI_MOVE_DEFAULT_CM;
      ai::logf("[ai] move 未给 distance_cm, 补为 %dcm", AI_MOVE_DEFAULT_CM);
    }
  } else if (!strcmp(mt, "spin")) {
    int sd = src["dir"] | 0;
    int ss = src["speed"] | 800;   // 原地旋转默认转速(全系统统一档)
    m["dir"] = constrain(sd, -1, 1);
    m["speed"] = constrain(ss, 0, 1000);
    // 定角取绝对值、为 0 不写键; 方向由 dir 定。
    if (src["angle_deg"].is<int>()) {
      int ad = src["angle_deg"] | 0;
      int mag = ad < 0 ? -ad : ad;
      if (mag > 0) m["angle_deg"] = constrain(mag, 1, AI_SPIN_MAX_DEG);
    }
    if (!m["angle_deg"].is<int>() && (m["dir"] | 0) != 0) {
      m["angle_deg"] = AI_SPIN_DEFAULT_DEG;
      ai::logf("[ai] spin 未给 angle_deg, 补为 %d°", AI_SPIN_DEFAULT_DEG);
    }
  } else {  // approach
    const char* tgt = src["target"] | "";
    if (tgt[0]) m["target"] = tgt;
  }
  return nullptr;
}
