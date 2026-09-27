#include "src/ai/tools/tool.h"

// zoom 通道: **旧写法**, 已并入 look 工具(见 ai_prompt.cpp 的 look.zoom)。
// 保留本键只为兜底: 模型偶尔仍会发 {"zoom":true} 时校验不报错, 实际由 ai_round 的 land_car 打印一句
// "已并入 look, 本次未生效"。故本函数无值可非法, 也无默认值可补。
const char* ai::parse_zoom(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  if (root["zoom"].is<bool>()) dst["zoom"] = root["zoom"].as<bool>();
  return nullptr;
}
