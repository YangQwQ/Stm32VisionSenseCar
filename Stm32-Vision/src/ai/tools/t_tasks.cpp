#include "src/ai/tools/tool.h"

// 任务记账四键(纯元数据, 不含硬件动作): 全部原样透传, 处理在 `round_land` 的任务记账段
// (写入 `RoundCtx` 的步骤表 `tasks[]`, 每轮再渲染回喂模型)。故这里只做形状检查、不做取值校验。
// ⚠️ 阶段 5 的任务状态镜像(`ai_task`)会从这里取发布点。

// task_goal: 把用户的新意图提升为当前任务目标(AI 显式标记)。
const char* ai::parse_task_goal(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  const char* tg = root["task_goal"] | "";
  if (tg[0]) dst["task_goal"] = tg;
  return nullptr;
}

// task_note: 单行笔记, 累进任务上下文。
const char* ai::parse_task_note(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  const char* tn = root["task_note"] | "";
  if (tn[0]) dst["task_note"] = tn;
  return nullptr;
}

// tasks: 整表重写, 每项就是任务名(**纯字符串**数组; 每项完成与否由 task_done 标记)。
const char* ai::parse_tasks(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  if (root["tasks"].is<JsonArrayConst>()) dst["tasks"] = root["tasks"].as<JsonArrayConst>();
  return nullptr;
}

// task_done: 标记若干项完成, 数组里是任务编号(**纯数字**, 首项为 1; 见 round_land 的读法)。
const char* ai::parse_task_done(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap) {
  (void)err; (void)cap;
  if (root["task_done"].is<JsonArrayConst>())
    dst["task_done"] = root["task_done"].as<JsonArrayConst>();
  return nullptr;
}
