#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

// AI 输出校验入口: 剥代码块 → 解 JSON → 拒旧版扁平格式 → 按工具注册表逐项规范化。
// ⚠️ **改词表不在这里**: 各通道的白名单/非法值/数值钳制在 `src/ai/tools/` 下按工具分文件
//    (表见 `tools/registry.cpp`, 为什么这么分见 `tools/tool.h` 头注)。本头文件现在只剩入口声明
//    与下面两个硬钳制宏。
namespace ai {

// AI 单步 move 距离硬上限(硬钳制, 覆盖提示词的"≤30cm"建议)。
#define AI_MOVE_MAX_CM 40
// AI 单步 spin 角度硬上限: 大幅盲转会把画面参照全丢, 且无里程计下角度误差随幅度放大。
#define AI_SPIN_MAX_DEG 180

// 向 char 缓冲安全追加一段(空串跳过; 首个不清分隔、后续加 "; ")。返回 false 表示缓冲已满。
bool safe_append(char* buf, size_t cap, bool* first, const char* part);

// 校验并规范化 AI 输出到 out{type,params}(外加旧终态键 done/goal)。返回 nullptr 通过; 否则返回错误字符串。
const char* validate_cmd(const char* content, JsonDocument& out, char* err_buf, size_t err_cap);

}  // namespace ai