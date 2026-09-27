#include "src/ai/tools/tool.h"

#include <Arduino.h>
#include <string.h>

// car 的 light 通道 + goal 工具的两个键(set/finish)。
// ⚠️ light 是真硬件动作(哪吒灯命令字节), 与 set/finish 这类纯文案键性质完全不同; 同处一文件只是
//    因为它们都"形状简单、无数组", 与 t_observe/t_tasks 的列表型键分开。

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

// goal.set: 更新当前任务的最终目标(纯字符串, 原样透传; 是否变化由落地侧比对)。
const char* ai::parse_set(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  const char* g = root["set"] | "";
  if (g[0]) dst["set"] = g;
  return nullptr;
}

// goal.finish: 任务终态(done=完成 / fail=执行失败 / wait=中止并等待用户输入)。
// 非法值**明拒而不放过** —— 放过就会被下游当成"没写 finish", 任务在该结束的轮次继续空跑。
const char* ai::parse_finish(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  const char* g = root["finish"] | "";
  if (!g[0]) return nullptr;
  if (strcmp(g, "done") && strcmp(g, "fail") && strcmp(g, "wait")) {
    snprintf(err, cap, "AI 非法 finish=%s(可用 done/fail/wait)", g);
    return err;
  }
  dst["finish"] = g;
  return nullptr;
}