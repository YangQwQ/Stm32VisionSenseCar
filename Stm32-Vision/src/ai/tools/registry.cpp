#include "src/ai/tools/tool.h"
#include "src/core/board_log.h"

#include <string.h>

// 工具表(顺序只决定**校验顺序**, 理由见 tool.h 头注)。落地顺序在 `ai_round.cpp`:
// car 内部各通道由 `land_car` 排序, 工具之间由 `dispatch_calls` 排序(mem → task → say → car → look → compact → goal)。
//
// 键按**归属工具**分组排列: mem={observe,delete} / car={move,arm,light} / task={note,todo,done} / goal={set,finish}。
// 每个工具的参数文档都会过一遍全表(缺席的键直接跳过), 故键名必须全局唯一。
//
// 阶段 1 的校验批次只填 `parse`; `run`/`feedback`/`logfmt` 留 nullptr。
// `doc`(模型读到的文案)阶段 3 才逐条搬 —— 现在全为 nullptr 是**有意的**, 见 tool.h。
namespace {
const ai::ToolSpec kTools[] = {
  // key      doc      grp  parse                  run      feedback  logfmt
  { "observe", nullptr,  0,  ai::parse_observe,    nullptr, nullptr, nullptr },
  { "delete",  nullptr,  0,  ai::parse_delete,     nullptr, nullptr, nullptr },
  { "move",    nullptr,  0,  ai::parse_move,       nullptr, nullptr, nullptr },
  { "arm",     nullptr,  0,  ai::parse_arm,        nullptr, nullptr, nullptr },
  { "light",   nullptr,  0,  ai::parse_light,      nullptr, nullptr, nullptr },
  { "note",    nullptr,  0,  ai::parse_note,       nullptr, nullptr, nullptr },
  { "todo",    nullptr,  0,  ai::parse_todo,       nullptr, nullptr, nullptr },
  { "done",    nullptr,  0,  ai::parse_done,       nullptr, nullptr, nullptr },
  { "auto_grasp", nullptr, 0, ai::parse_grasp,     nullptr, nullptr, nullptr },
  { "set",     nullptr,  0,  ai::parse_set,        nullptr, nullptr, nullptr },
  { "finish",  nullptr,  0,  ai::parse_finish,     nullptr, nullptr, nullptr },
};
const int kToolN = (int)(sizeof(kTools) / sizeof(kTools[0]));
}  // namespace

const ai::ToolSpec* ai::tools(int* n) {
  if (n) *n = kToolN;
  return kTools;
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
