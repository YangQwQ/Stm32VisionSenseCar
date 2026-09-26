#pragma once
#include <Arduino.h>     // size_t / millis
#include <ArduinoJson.h>

// 空间记忆 + 车姿态。g_mem 等仅本模块内可见。
namespace ai {

// 角度→弧度换算系数(供 ai_nav 的 navigate_to 使用)。
const float AI_PI = 3.14159265358979f;

// 新鲜线(轮): 超过这几轮没再看到, 该物体算"曾看到过、现已不可见"(供遮挡类提示判断, 见 mem_have_lost)。
const int AI_GRASP_STALE = 3;

// 近场线(车头系 前 cm): 与 approach 停距、提示词同口径; 一带 cm 高报且抖, 记忆行只给档位、不报坐标。
const float AI_NEAR_FWD_CM = 20.0f;

// 车自身位姿(全局坐标 x,y cm + 车向角 heading, 命名空间共享: ai_mem 维护 / navigate_to 读用)。
extern float s_car_x, s_car_y;
extern int16_t s_car_heading;

// 新任务起点: 车位置=原点、车头=0°, 清空物体记忆。
void mem_reset();
// 按定距/定角近似累积车姿态(move→平移, spin→转向)。
void car_update_pose(const char* type, const JsonObjectConst& p);
// AI 观测(rel_deg 相对车头, 正=右、负=左): 换算车头系后入表。
// 返回 false = 这次"可见"的观测没能记入(缺距离/像素越界), 调用方应回告 AI 防它以为已记住。
bool mem_observe(const char* name, bool visible, float rel_deg, float dist_cm);
// AI 观测(屏幕归一化像素 px,py): 单应解算车头系坐标后入表。返回值含义同上。
bool mem_observe_xy(const char* name, bool visible, float px, float py);
// 生成喂给 AI 的空间记忆文本: 小车全局 (x,y) + 朝向, 再按新鲜度(最近更新的在前)列出各物体。
// ⚠️ 物体位置是车头系(前/右 cm), 与小车的全局坐标不同系。
void mem_feed(char* buf, size_t cap);
// 记忆里有没有物体(不论多旧): 与 mem_feed 同源, 免得程序说"没有目标"而记忆行里明明列着。
bool mem_have_any();
// 是否存在"曾被观测、但现已不可见"的物体(超新鲜线 AI_GRASP_STALE)。
// 与 mem_have_any 是两种情形: 这个答"有物体, 只是最近看不到", 供遮挡类提示判断。
bool mem_have_lost();
// 在物体记忆表里定位目标全局坐标。
bool mem_find(const char* name, float* tx, float* ty);
// 删除一个物体记忆(按名匹配, 判据与入库/定位同一套)。返回是否真删掉了(名字不在记忆里则 false)。
bool mem_forget(const char* name);
// 每轮结束: 所有物体未观测则过期轮数 +1。
void mem_tick_stale();

}  // namespace ai