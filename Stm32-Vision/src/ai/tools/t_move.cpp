#include "src/ai/tools/tool.h"
#include "src/ai/ai_validate.h"   // AI_MOVE_MAX_CM / AI_SPIN_MAX_DEG
#include "src/ai/ai_client.h"     // ai::logf(补默认值时留一条日志)

#include <Arduino.h>
#include <string.h>

// 未给 value 时补有限段长/角度(而非放开成"持续"): 持续动作不累积位移与车向 ⇒
// 记忆里车位置/朝向失同步, 之后喂回的物体坐标全偏; 补有限值后走多远、转多少始终已知。
#define AI_MOVE_DEFAULT_CM 8
#define AI_SPIN_DEFAULT_DEG 30

// move 通道(轮子): forward/backward=直行 / spin_left/spin_right=原地转 / approach=自动靠近, 五选一。
// 方向写进 type 而不另立 dir 字段: 旧写法把直行与转动的方向词混在一个枚举里, 模型可能搭出
// `type=throttle + dir=left` 这种空转组合; 合成 type 后非法组合不存在。
// ⚠️ 本函数是 `move` 的**唯一**规范化出口 —— 返回后程序不再改写它的任何键(默认值补全也归这里)。
//    注意语义: 不写 value 的原意是**持续动作**, 故补默认值是**改写模型意图**
//    (已知坏味道); 若改成"不补写、位姿估计仍按估算值累积", 须连 car_update_pose 一起动。
// 落地仍走内部规范形(throttle/steering/distance_cm 与 dir/speed/angle_deg), 与手动指令同一套。
const char* ai::parse_move(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  if (!root["move"].is<JsonObjectConst>()) return nullptr;
  JsonObjectConst src = root["move"].as<JsonObjectConst>();
  const char* mt = src["type"] | "";
  bool fwd = !strcmp(mt, "forward"),  bwd = !strcmp(mt, "backward");
  bool sl  = !strcmp(mt, "spin_left"), sr = !strcmp(mt, "spin_right");
  if (!fwd && !bwd && !sl && !sr && strcmp(mt, "approach")) {
    snprintf(err, cap, "AI move 通道非法 type=%s(须 forward/backward/spin_left/spin_right/approach)", mt);
    return err;
  }
  int v = src["value"] | 0;
  int mag = v < 0 ? -v : v;   // 负值只当方向写错, 取绝对值
  JsonObject m = dst["move"].to<JsonObject>();
  if (fwd || bwd) {
    m["type"] = "throttle";
    m["throttle"] = fwd ? 1.0f : -1.0f;
    m["steering"] = 0.0f;
    m["distance_cm"] = mag > 0 ? constrain(mag, 1, AI_MOVE_MAX_CM) : AI_MOVE_DEFAULT_CM;
    if (mag == 0) ai::logf("[ai] move 未给 value, 补为 %dcm", AI_MOVE_DEFAULT_CM);
  } else if (sl || sr) {
    m["type"] = "spin";
    m["dir"] = sl ? -1 : 1;   // 内部规范形: 左=-1 / 右=+1
    m["speed"] = 800;         // 全系统统一转速档(不暴露给 AI)
    m["angle_deg"] = mag > 0 ? constrain(mag, 1, AI_SPIN_MAX_DEG) : AI_SPIN_DEFAULT_DEG;
    if (mag == 0) ai::logf("[ai] spin 未给 value, 补为 %d°", AI_SPIN_DEFAULT_DEG);
  } else {  // approach
    m["type"] = "approach";
    const char* tgt = src["target"] | "";
    if (tgt[0]) m["target"] = tgt;
  }
  return nullptr;
}
