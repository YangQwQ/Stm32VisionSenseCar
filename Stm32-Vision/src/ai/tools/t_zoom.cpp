#include "src/ai/tools/tool.h"

// zoom 通道: true=要一张放大图(单次, 发完自动回全幅), false=显式回全幅。
// 只有这一个布尔键 —— 框的位置/倍数不在这里给, 由程序按目标位置决定(见 `magnify` 与 `ai_round` 的
// round_prepare)。故本函数无值可非法, 也无默认值可补。
// ⚠️ 阶段 2 会把散在 5 处的放大镜执行点(声明 / 裁图 / carry_image 复刻 / 兜底回全幅 / 坐标反算)
//    连同回执一起收进本文件, 届时 `run` 才非空。
const char* ai::parse_zoom(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  if (root["zoom"].is<bool>()) dst["zoom"] = root["zoom"].as<bool>();
  return nullptr;
}
