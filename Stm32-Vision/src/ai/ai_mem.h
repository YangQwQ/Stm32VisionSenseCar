#pragma once
#include <Arduino.h>     // size_t / millis
#include <ArduinoJson.h>

// 空间记忆 + 车姿态。g_mem 等仅本模块内可见。
namespace ai {

// 角度→弧度换算系数(供 ai_nav 的 navigate_to 使用)。
const float AI_PI = 3.14159265358979f;

// 夹取依据的时效(轮): 目标必须在这几轮内被画面看到过
const int AI_GRASP_STALE = 3;

// 近场与"已到车头/爪后"的距离口径(车头系 前 cm), 集中一处供各处同源。
// 近场线: 与 approach 停距、提示词同口径; 一带 cm 高报且抖, 记忆行只给档位、不报坐标。
const float AI_NEAR_FWD_CM = 20.0f;
// 已到车头/爪后(前 cm): 越过低姿爪口 ⇒ 方块在臂下。
const float AI_ARM_UNDER_CM = 7.0f;

// 记忆坐标"还算不算数"的运动容差(路程 cm / 转角 °): 在此之内那条坐标仍按"看到的位置"用。
// 用路程而非净位移 —— 往返会抵消位移, 但两段各自都累积了推算误差。
const float AI_GRASP_MOVE_TOL_CM = 3.0f;
const float AI_GRASP_TURN_TOL_DEG = 6.0f;

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
// 生成喂给 AI 的空间记忆文本(当前车头局部系精确坐标)。
void mem_feed(char* buf, size_t cap);
// 记忆里有没有可用的物体: 过滤条件与 mem_feed 同源, 免得程序说"没有目标"而记忆行里明明列着。
bool mem_have_any();
// 是否存在"曾被观测、但现已不可见(丢失)"的物体(超新鲜线 AI_GRASP_STALE 但记录仍在)。
// 与 mem_have_any 是两种情形: 这个答"有物体, 只是最近看不到", 供遮挡类提示判断。
bool mem_have_lost();
// 在物体记忆表里定位目标全局坐标。
bool mem_find(const char* name, float* tx, float* ty);
// 每轮结束: 所有物体未观测则过期轮数 +1。
void mem_tick_stale();

}  // namespace ai