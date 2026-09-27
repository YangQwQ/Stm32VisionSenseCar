#pragma once
#include <Arduino.h>     // size_t / millis
#include <ArduinoJson.h>

// 空间记忆 + 车姿态。g_mem 等仅本模块内可见。
namespace ai {

// 角度→弧度换算系数(供 ai_nav 的 navigate_to 使用)。
const float AI_PI = 3.14159265358979f;

// 车自身位姿(全局坐标 x,y cm + 车向角 heading, 命名空间共享: ai_mem 维护 / navigate_to 读用)。
extern float s_car_x, s_car_y;
extern int16_t s_car_heading;

// 观测解算用的车位姿: 必须是**拍那张图时**的位姿, 不是 observe 落地时的实时位姿 ——
// 车若在"看图"与"报坐标"之间动过, 用实时位姿解算会把整条记忆平移一个位移量。
struct CarPose {
  float x = 0, y = 0;
  int16_t hd = 0;
};

// 新任务起点: 车位置=原点、车头=0°, 清空物体记忆。
void mem_reset();
// 按定距/定角近似累积车姿态(move→平移, spin→转向)。
void car_update_pose(const char* type, const JsonObjectConst& p);
// AI 观测(屏幕归一化像素 px,py): 单应解算车头系坐标后入表。pose=拍图时的车位姿(缺省=当前实时位姿)。
// 返回 false = 这次"可见"的观测没能记入(像素越界/解算失败), 调用方应回告 AI 防它以为已记住。
// out_right/out_fwd 非空时回带解算出的车头系读数(右+/前+, cm), 供调用方当场回执观测结果。
bool mem_observe_xy(const char* name, bool visible, float px, float py,
                    float* out_right = nullptr, float* out_fwd = nullptr,
                    const CarPose* pose = nullptr);
// 生成喂给 AI 的空间记忆文本: 小车全局 (x,y) + 朝向, 再按新鲜度(最近更新的在前)列出各物体。
// ⚠️ 物体位置是车头系(前/右 cm), 与小车的全局坐标不同系。⚠️ 近场坐标高报且抖, 只作参考。
void mem_feed(char* buf, size_t cap);
// 在物体记忆表里定位目标全局坐标。
bool mem_find(const char* name, float* tx, float* ty);
// 删除一个物体记忆(按名匹配, 判据与入库/定位同一套)。返回是否真删掉了(名字不在记忆里则 false)。
bool mem_forget(const char* name);
// 每轮结束: 所有物体未观测则过期轮数 +1。
void mem_tick_stale();

}  // namespace ai