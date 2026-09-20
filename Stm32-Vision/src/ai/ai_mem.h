#pragma once
#include <Arduino.h>     // size_t / millis
#include <ArduinoJson.h>

// 空间记忆 + 车姿态(拆分自 ai_client)。g_mem 等仅本模块内可见。
namespace ai {

// 角度→弧度换算系数(内部连接性, 各 TU 自持一份; 供 ai_client.cpp 的 navigate_to 使用)。
const float AI_PI = 3.14159265358979f;

// 车自身位姿(全局坐标 x,y cm + 车向角 heading, 命名空间共享: ai_mem 维护 / navigate_to 读用)。
extern float s_car_x, s_car_y;
extern int16_t s_car_heading;

// 新任务起点: 车位置=原点、车头=0°, 清空物体记忆(原 ai_worker 任务开头那几行)。
void mem_reset();
// 按定距/定角近似累积车姿态(move→平移, spin→转向)。
void car_update_pose(const char* type, const JsonObjectConst& p);
// AI 观测(rel_deg 相对车头, 正=右、负=左): 换算车头系后入表。
void mem_observe(const char* name, bool visible, float rel_deg, float dist_cm);
// AI 观测(屏幕归一化像素 px,py): 单应解算车头系坐标后入表。
bool mem_observe_xy(const char* name, bool visible, float px, float py);
// 生成喂给 AI 的空间记忆文本(当前车头局部系精确坐标)。
void mem_feed(char* buf, size_t cap);
// 在物体记忆表里定位目标全局坐标。
bool mem_find(const char* name, float* tx, float* ty);
// 每轮结束: 所有物体未观测则过期轮数 +1。
void mem_tick_stale();

}  // namespace ai