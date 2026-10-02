#pragma once
#include <ArduinoJson.h>
#include <stdint.h>
#include <stddef.h>

// AI 工具表: 每个工具(动作通道 / 元数据键)**唯一一份定义**, 校验 / 落地 / 回执 / 日志四个阶段共用。
//
// 为什么要有这张表: 加一个动作原本要同步改四处 —— 提示词文案(`ai_prompt.cpp`) / 校验白名单
// (`ai_validate.cpp`) / 落地分派(`ai_round.cpp` 的 `round_land`) / 执行(`exec::act`)。四处各写一套,
// 漏一处**不报错、只是静默失效**(已踩三次, 见 memory `ai-cmd-param-whitelist-copy`: light 的
// kind/on、zoom 的 px/py/scale/reset 都是这么丢的)。有表之后"改词表"塌缩成"改一个文件"。
//
// ⚠️ **表序现在只决定校验顺序**: `validate_cmd` 按表序逐项跑 parse, 故"同一份参数里有多个非法键"时
//    报出的是表里靠前的那一个(错误文本本身不变, 这是表序唯一还会影响的东西); 单个非法键的行为完全一致。
//    **落地顺序不在这张表里**: car 内部各通道的次序在 `ai_round.cpp` 的 `land_car`, 工具之间的次序在
//    `dispatch_calls`(mem → task → say → car → look → compact → goal)。
//    ⚠️ 提示词里的呈现顺序也不是表序(键名/取值/用法现写在 `ai_prompt.cpp` 的 `tools_schema()` 工具描述里)。
namespace ai {

struct RoundCtx;

// 工具的执行结果。Acted 供"本轮到底有没有真动作"的兜底判定(空转计数)与日志用。
enum class ToolR : uint8_t { None, Acted, Rejected };

struct ToolSpec {
  const char* key;   // JSON 顶层键, 如 "move" / "observe"
  // 喂给模型的说明行(不含组标题)。⚠️ 这是**模型读到的字**, 文案归用户; 阶段 1/2 只建表不搬文案,
  // 阶段 3 再逐条搬(逐字移动, 不改写)。此字段在阶段 3 之前为 nullptr。
  const char* doc;
  uint8_t group;     // 提示词分段(阶段 3 才用; 现在统一 0, 阶段 3 按提示词的实际分段填)

  // ① 校验 + 规范化: 从 root 读**自己的键**、把规范化结果写回 dst 的**同名键**。
  //    收整份文档而非自己的子值, 是因为键的形状不统一(通道是对象、set/finish 是字符串、
  //    observe/todo 是数组), 统一签名才装得进同一张表。
  //    ⚠️ **默认值补全归这里**(现 `parse_move` 补 `distance_cm`/`angle_deg`)—— 本函数返回后程序
  //    不再改写指令。注意语义: 不写 `distance_cm` 的原意是**持续动作**(靠后续指令或时限收尾), 故补
  //    默认值是**改写模型意图**(已知坏味道, 理由见 `t_move.cpp`); 若要改成"不补写、位姿估计仍按
  //    估算值累积", 须连 `car_update_pose` 一起动。
  //    返回 nullptr=通过; 否则返回错误串(直接回给模型, 让它自己改)。
  //    ⚠️ 返回的指针**不保证**指向 `err` —— 允许直接返回静态字面量(现 `parse_light` 就有一条),
  //    调用方只许当只读 C 串用。用不到的 `err`/`cap` 记得 `(void)` 掉。
  const char* (*parse)(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap);

  // ② 落地: 执行(本地处理 或 `exec::act`)并写回执片段。收的是**整份规范化文档**而非自己的子对象:
  //    校验阶段本就要拿整份文档跑全表, 落地沿用同一形参可免去二次切片。这是本方案
  //    已接受的折中 —— 换取逐通道迁移时行为可逐字对照, 不做插件式注册/动态分配/虚函数。
  //    阶段 1 的校验批次里先为 nullptr, 由后续批次填入。
  ToolR (*run)(RoundCtx& c, JsonDocument& cmdD);

  // ③ 文案: 给模型看的回执 / 给日志看的摘要。同上, 后续批次填。
  // ⚠️⚠️ 这两个形参是 **const** JsonDocument ⇒ `cmdD["x"]` 拿到的是 `JsonVariantConst`。在它上面做
  //     **类型检查只能用 Const 形式**(`is<JsonObjectConst>()` / `is<JsonArrayConst>()`):
  //     ArduinoJson v7 里 `Converter<JsonObject>::fromJson` 收的是**非 const** `JsonVariant`, 所以
  //     `JsonVariantConst::is<JsonObject>()` 会落到库注释写着 "unsupported types" 的那个重载,
  //     **恒返回 false 且照样编译过** —— 不报错、不生效, 正是本表要消灭的那类失效。
  //     取值型的 `as<JsonObjectConst>()` / `| 默认值` 不受影响; `as<JsonObject>()` 则是编译错误(会响)。
  void (*feedback)(const RoundCtx& c, const JsonDocument& cmdD, char* buf, size_t cap);
  void (*logfmt)(const JsonDocument& cmdD, char* buf, size_t cap);
};

// ---- 各工具的 parse 实现 ----
// 键 → 工具归属: car={move,arm,light} / mem={observe,delete} / task={note,todo,done} / goal={set,finish}。
// validate_cmd 会对**任一工具**的参数文档依次跑全表(缺席的键各自的 parse 直接返回 nullptr), 故键名必须全局唯一。
const char* parse_observe(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap);   // t_observe.cpp
const char* parse_delete(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap);    // t_observe.cpp
const char* parse_move(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap);      // t_move.cpp
const char* parse_arm(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap);       // t_arm.cpp
const char* parse_light(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap);     // t_meta.cpp
const char* parse_set(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap);       // t_meta.cpp
const char* parse_finish(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap);    // t_meta.cpp
const char* parse_note(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap);      // t_tasks.cpp
const char* parse_todo(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap);      // t_tasks.cpp
const char* parse_done(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap);      // t_tasks.cpp
const char* parse_grasp(JsonVariantConst root, JsonDocument& dst, char* err, size_t cap);     // t_grasp.cpp

// registry.cpp 里的有序表。顺序只决定**校验顺序**(validate_cmd 按表序逐项跑 parse), 落地顺序在 ai_round.cpp
// 的 dispatch_calls(mem → task → say → car → grasp → look → compact → goal)与 land_car。返回数组首址并写回项数。
// 数组是文件级 `const` 常量, 只在启动时构造一次, 无动态分配。
const ToolSpec* tools(int* n);

// 请求体尾部的工具声明原文: `"tools":[...],"tool_choice":"auto"`(ai_prompt.cpp, 静态只读字面量)。
// agent 循环的七个工具(car/mem/task/goal/look/say/compact)在这里对模型声明; 行为与判据仍在系统提示词里。
const char* tools_schema();
// 表完整性自检: key 非空且唯一、parse 非空。异常打一条日志(廉价保险, 不做断言)。init 时调一次。
void tools_selfcheck();

}  // namespace ai
