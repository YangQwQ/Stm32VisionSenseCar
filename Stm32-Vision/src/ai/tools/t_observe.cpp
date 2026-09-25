#include "src/ai/tools/tool.h"

// observe: AI 的空间观测(name/px/py/visible), 透传给 worker 更新物体记忆表。
// 纯数据搬运, 没有"非法值"可言 —— 坐标能不能用、要不要入表、要不要报"记不进去", 全在记忆一侧
// (`ai_mem` 的 `mem_observe*`), 那里才有单应/可信度/过期的判据。故本函数不做任何校验或钳制。
const char* ai::parse_observe(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  if (root["observe"].is<JsonObjectConst>())
    dst["observe"] = root["observe"].as<JsonObjectConst>();
  return nullptr;
}
