#include "src/ai/tools/tool.h"

// task 工具的三个键(纯元数据, 不含硬件动作): 全部原样透传, 处理在 `do_task`
// (写入 `RoundCtx` 的任务笔记与步骤表, 每轮再渲染回喂模型)。故这里只做形状检查、不做取值校验。

// task.note: 单行笔记, 累进任务上下文。
const char* ai::parse_note(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  const char* v = root["note"] | "";
  if (v[0]) dst["note"] = v;
  return nullptr;
}

// task.todo: 整表重写, 每项就是任务名(**纯字符串**数组; 每项完成与否由 task.done 标记)。
const char* ai::parse_todo(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  if (root["todo"].is<JsonArrayConst>()) dst["todo"] = root["todo"].as<JsonArrayConst>();
  return nullptr;
}

// task.done: 标记若干项完成, 数组里是任务编号(**纯数字**, 首项为 1; 见 do_task 的读法)。
const char* ai::parse_done(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  if (root["done"].is<JsonArrayConst>()) dst["done"] = root["done"].as<JsonArrayConst>();
  return nullptr;
}