#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

// 执行器直驱层：替代原执行板（STM32），在本板软件 I2C 直接驱动哪吒扩展板。
// 覆盖 move/stop/arm/light/reset，并本地合成状态文本供 App / AI 参考。
namespace exec {

// 舵机回中 150 + 电机 0（幂等；上电与「回正」共用）。
void init();
// 回正：四舵机回 150 + 电机 0。
void reset();
// 连续机械臂动作步进（约 20ms 一次节拍），无动作时为空操作。
void update_tick();

// 统一执行入口：move / stop / arm / light / reset。返回是否被识别。
bool act(const char* type, const JsonObjectConst& params);
// 调试直驱单舵机：logical 0=转向 / 1=左(前后移爪) / 2=右(抬落) / 3=前(夹爪)。
// 直接写原始 pwm（50..250），不过标定限位，用于探机械极限/标定。返回 false=参数非法。
bool set_servo(uint8_t logical, uint16_t pwm);
// 二连杆 IK：给末端目标位姿(x=轴前方cm, h=地面以上cm)，联动解算并下发左右两舵机 pwm。目标不可达返回 false。
bool arm_pose(float x, float h);
// 该指令是否为"持续型"（持续直驱需要配 stop 收尾）。
bool is_continuous(const char* type, const JsonObjectConst& params);
// 本地合成状态文本，写不进内容即返回 false。
bool read_state(char* buf, size_t cap);

}  // namespace exec