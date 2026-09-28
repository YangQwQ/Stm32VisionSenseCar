#include "src/ai/ai_validate.h"
#include "src/ai/ai_alloc.h"
#include "src/ai/tools/tool.h"   // 工具表: 逐通道的白名单/钳制/默认值都在 tools/ 下

#include <string.h>

// ---------------- 输出校验(防误动作) ----------------
// 向 char 缓冲安全追加一段(空串跳过; 首个不加分隔、后续加 "; "); 返回 false=缓冲已满。
bool ai::safe_append(char* buf, size_t cap, bool* first, const char* part) {
  size_t used = strlen(buf);
  const char* sep = *first ? "" : "; ";
  size_t need = strlen(sep) + strlen(part);
  if (used + need + 1 > cap) return false;   // 放不下: 跳过本段, 不硬塞
  snprintf(buf + used, cap - used, "%s%s", sep, part);
  *first = false;
  return true;
}

// 校验并规范化 AI 输出到 out。返回 nullptr 通过; 否则返回错误字符串。
//
// 本文件只剩**整份文档级**的三件事: 剥代码块、解 JSON、拒旧版扁平 `type` 格式。
// 逐通道的白名单 / 非法值 / 数值钳制全部按工具登记在 `tools/` 里 —— 加一个动作或改一个键只动那个
// 工具文件, 不必再来这里。旧写法让同一个动作的真相散在四处(提示词/校验/分派/执行), 漏一处
// **不报错、只是静默失效**, 已踩三次(见 memory `ai-cmd-param-whitelist-copy` 与 tool.h 头注)。
//
// ⚠️ 返回的指针**不保证**指向 `err_buf`: 工具允许直接返回静态字面量(见 t_meta.cpp 的 parse_light)。
//    调用方只许把它当只读 C 串用, 不要假设可写、也不要就地再 snprintf 一遍。
const char* ai::validate_cmd(const char* content, JsonDocument& out, char* err_buf, size_t err_cap) {
  // 剥代码块/首尾空白(Arduino String 无 find/left, 用 indexOf/substring)
  String c = content;
  c.trim();
  if (c.startsWith("```")) {
    int e = c.indexOf('\n');
    if (e >= 0) c = c.substring(e + 1);
    c.trim();
    if (c.endsWith("```")) c = c.substring(0, c.length() - 3);
    c.trim();
  }

  JsonDocument doc(&g_js_alloc);  // PSRAM 池, 避免内部堆碎片
  if (deserializeJson(doc, c)) {
    // 不再 c.substring(0,80) 出第二份副本: err_buf 本身就截断, 交给 snprintf 的精度即可。
    snprintf(err_buf, err_cap, "AI 返回非 JSON: %.80s", c.c_str());
    return err_buf;
  }
  // 顶层必须是 JSON 对象: 模型偶尔给出裸数组/字符串/数字(如 "[]" 或 "ok")。非对象时 `doc["type"]`
  // 恒空、全表 parse 也都查无此键 → 会被当成"空动作"静默放过, 让 AI 以为指令生效而空转。
  // 用 Const 形式判定(理由见 tool.h: `JsonVariantConst::is<JsonObject>()` 恒 false 且照样编译过)。
  JsonVariantConst root = doc;
  if (!root.is<JsonObjectConst>()) {
    snprintf(err_buf, err_cap, "AI 返回的 JSON 不是对象(应为 {\"工具名\":{...}}): %.80s", c.c_str());
    return err_buf;
  }
  // 顶层出现旧单指令格式 type → 拒并给处方: 改用分组 JSON(car 的 move/arm/light 通道)。
  if (doc["type"]) {
    snprintf(err_buf, err_cap,
             "AI 输出了旧版单指令格式(type=%s); 请改用分组 JSON: car 的 move/arm/light 通道(缺席=不动)",
             doc["type"].as<const char*>());
    return err_buf;
  }
  // 参数校验: 按工具表逐项跑(**表序 = 校验顺序**, 见 tools/registry.cpp)。car/mem/task/goal 的参数文档
  // 都过一遍全表, 缺席的键各自的 parse 直接返回 nullptr。故同一次返回里有多个非法键时, 报出的是表里靠前的那个。
  int tn = 0;
  const ai::ToolSpec* ts = ai::tools(&tn);
  for (int i = 0; i < tn; i++) {
    if (!ts[i].parse) continue;   // tools_selfcheck 已就键完整性告警; 这里跳过而不是崩
    const char* e = ts[i].parse(root, out, err_buf, err_cap);
    if (e) return e;
  }
  return nullptr;
}
