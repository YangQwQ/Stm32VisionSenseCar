#include "src/ai/tools/tool.h"

#include <Arduino.h>
#include <string.h>

// 提示词"其它"段的键: 顶层元数据 + light 通道 + 带图请求回显。
// ⚠️ 这几项归一个文件是按**提示词分段**归的, 不是按"有没有副作用"——`light` 是真硬件动作
//    (哪吒灯命令字节), 与 reason/goal/done 这类纯文案键性质完全不同。阶段 3 生成提示词时
//    它们会落在同一段里, 这才是它们同处一处的唯一理由。

// reason: 模型自述本轮意图, 只进日志/历史环/回执, 无校验点。
const char* ai::parse_reason(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  const char* reason = root["reason"] | "";
  if (reason[0]) dst["reason"] = reason;
  return nullptr;
}

// done: 任务完结标记(等同 finish)。只认 true —— 显式给 false 不写键, 免得把"写了 false"与"没写"混同。
const char* ai::parse_done(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  if (root["done"].is<bool>() && root["done"].as<bool>()) dst["done"] = true;
  return nullptr;
}

// goal: 任务终态(finish=完成 / abort=请求中止 / fail=执行失败)。
// 非法值**明拒而不放过** —— 放过就会被下游当成"没写 goal", 任务在该结束的轮次继续空跑。
const char* ai::parse_goal(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  const char* goal = root["goal"] | "";
  if (!goal[0]) return nullptr;
  if (strcmp(goal, "finish") && strcmp(goal, "abort") && strcmp(goal, "fail")) {
    snprintf(err, cap, "AI 非法 goal=%s(可用 finish/abort/fail)", goal);
    return err;
  }
  dst["goal"] = goal;
  return nullptr;
}

// carry_image: 下轮额外携带哪张图("zoom"/"full"/"image1~3")。
// 不校验取值: 分派端(`land_carry_image`)对认不出的值退化为"什么都不带", 无副作用。
const char* ai::parse_carry_image(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  const char* cimg = root["carry_image"] | "";
  if (cimg[0]) dst["carry_image"] = cimg;
  return nullptr;
}

// light 通道: kind 合法, on 必须显式给 bool(缺省拒, 防 AI 漏写 on 把灯误关)。
const char* ai::parse_light(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  if (!root["light"].is<JsonObjectConst>()) return nullptr;
  JsonObjectConst src = root["light"].as<JsonObjectConst>();
  const char* kind = src["kind"] | "";
  if (strcmp(kind, "front") && strcmp(kind, "vibe") && strcmp(kind, "back")) {
    snprintf(err, cap, "AI light 非法 kind=%s", kind);
    return err;
  }
  // ⚠️ 这条错**不走 err 缓冲**, 直接返回静态字面量(长度固定, 没必要拷一遍)。调用方只许当只读串用。
  if (!src["on"].is<bool>())
    return "AI light 缺 on(bool): 必须显式给 on:true/false(缺省会把灯误关)";
  dst["light"]["kind"] = kind;
  dst["light"]["on"] = src["on"].as<bool>();
  return nullptr;
}
