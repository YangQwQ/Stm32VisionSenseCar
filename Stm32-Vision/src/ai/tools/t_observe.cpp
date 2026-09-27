#include "src/ai/tools/tool.h"

// observe / delete: 物体记忆的写入口与删除口, 都只做形状检查、透传给 worker 处理。
// 纯数据搬运, 没有"非法值"可言 —— 坐标能不能用、要不要入表、名字在不在记忆里、要不要回告,
// 全在记忆一侧(`ai_mem` 的 `mem_observe_xy` / `mem_forget`), 那里才有单应/可信度/时效的判据。
const char* ai::parse_observe(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  if (root["observe"].is<JsonArrayConst>())
    dst["observe"] = root["observe"].as<JsonArrayConst>();
  return nullptr;
}

// delete: 删除已记忆的物体(纯字符串名字数组; 同名判定与 observe 入库时同一套)。
const char* ai::parse_delete(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  if (root["delete"].is<JsonArrayConst>())
    dst["delete"] = root["delete"].as<JsonArrayConst>();
  return nullptr;
}