#include "src/ai/tools/tool.h"
#include "src/core/board_log.h"

#include <string.h>

// 工具表(**顺序 = 执行顺序**, 理由见 tool.h 头注)。改这张表之前先读 tool.h 的头注, 以及
// `ai_round.cpp` 里三处带时序契约的注释: `land_carry_image` / approach 块 / 夹取前特写那次取帧。
//
// 排列依据(照 `round_land` 的**实际次序**抄, 不照提示词次序):
//   observe        写记忆, 不碰硬件
//   carry_image    取帧, ⚠️ 必须在**所有**动作之前(否则 AI 收到车还在动的画面, 白等一轮)
//   move           approach 自己会挪车, 故排在其它动作之前
//   zoom           读的是"本轮动作之前"的画面
//   任务记账四键    纯元数据
//   arm            含"夹取前特写"那次取帧(故排在 move 之后, 与 move 同轮时先等车停稳)
//   light          灯
//   尾部三个元数据  reason/done/goal, 无硬件动作; 排尾部是为尽量贴近旧校验次序
//
// 阶段 1 的校验批次只填 `parse`; `run`/`feedback`/`logfmt` 留 nullptr, 由后续批次补上。
// `doc`(模型读到的文案)阶段 3 才逐条搬 —— 现在全为 nullptr 是**有意的**, 见 tool.h。
namespace {
const ai::ToolSpec kTools[] = {
  // key            doc      grp  parse                    run      feedback  logfmt
  { "observe",     nullptr,   0,  ai::parse_observe,       nullptr, nullptr, nullptr },
  { "carry_image", nullptr,   0,  ai::parse_carry_image,   nullptr, nullptr, nullptr },
  { "move",        nullptr,   0,  ai::parse_move,          nullptr, nullptr, nullptr },
  { "zoom",        nullptr,   0,  ai::parse_zoom,          nullptr, nullptr, nullptr },
  { "task_note",   nullptr,   0,  ai::parse_task_note,     nullptr, nullptr, nullptr },
  { "tasks",       nullptr,   0,  ai::parse_tasks,         nullptr, nullptr, nullptr },
  { "task_done",   nullptr,   0,  ai::parse_task_done,     nullptr, nullptr, nullptr },
  { "task_goal",   nullptr,   0,  ai::parse_task_goal,     nullptr, nullptr, nullptr },
  { "arm",         nullptr,   0,  ai::parse_arm,           nullptr, nullptr, nullptr },
  { "light",       nullptr,   0,  ai::parse_light,         nullptr, nullptr, nullptr },
  { "reason",      nullptr,   0,  ai::parse_reason,        nullptr, nullptr, nullptr },
  { "done",        nullptr,   0,  ai::parse_done,          nullptr, nullptr, nullptr },
  { "goal",        nullptr,   0,  ai::parse_goal,          nullptr, nullptr, nullptr },
};
const int kToolN = (int)(sizeof(kTools) / sizeof(kTools[0]));
}  // namespace

const ai::ToolSpec* ai::tools(int* n) {
  if (n) *n = kToolN;
  return kTools;
}

const ai::ToolSpec* ai::tool_by_key(const char* key) {
  if (!key) return nullptr;
  for (int i = 0; i < kToolN; i++)
    if (kTools[i].key && !strcmp(kTools[i].key, key)) return &kTools[i];
  return nullptr;
}

// 表完整性自检(启动时一次): key 非空且唯一、parse 非空。
// **故意只打日志、不 assert** —— 这是编译期常量表, 出错只可能是手误; 让板子照常起来能刷 OTA,
// 比 panic 在车里强(板子固定在车上、串口够不着, 见 CLAUDE.md 的"PC 侧工具")。
void ai::tools_selfcheck() {
  int bad = 0;
  for (int i = 0; i < kToolN; i++) {
    if (!kTools[i].key || !kTools[i].key[0]) {
      blog::logf(blog::AI, "工具表[%d]: key 为空", i);
      bad++;
      continue;
    }
    if (!kTools[i].parse) {
      blog::logf(blog::AI, "工具表[%s]: parse 为空", kTools[i].key);
      bad++;
    }
    for (int j = i + 1; j < kToolN; j++)
      if (kTools[j].key && !strcmp(kTools[i].key, kTools[j].key)) {
        blog::logf(blog::AI, "工具表: key 重复 %s([%d]/[%d])", kTools[i].key, i, j);
        bad++;
      }
  }
  if (!bad) blog::logf(blog::AI, "工具表就绪: %d 项", kToolN);
}
