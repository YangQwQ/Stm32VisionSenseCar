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
// 低姿夹取准备位：夹心降到 Calibration.h 的 ARM_LOW_X_CM/ARM_LOW_H_CM（实测过的固定低姿）。
// 降到这儿后靠车的前后移动把目标送进两指之间（机械臂不能左右移动）。不可达返回 false。
bool arm_low();
// 固定抬臂位：夹心移到 Calibration.h 的 ARM_RAISE_X_CM/ARM_RAISE_H_CM（固定高位）。
// 一次离散定位（S 形缓动），非持续步进，不会在边界来回抽搐。不可达返回 false。
bool arm_raise();
// 夹心当前位姿（对当前命令的 pwm 做正运动学回读，与状态行同源）。供上层判"爪是否真的在目标身上"。
// 标定未就绪返回 false（此时调用方应按"位置未知"处理，不要据此拒绝动作）。
bool arm_pos(float* x, float* h);
// 该指令是否为"持续型"（持续直驱需要配 stop 收尾）。
bool is_continuous(const char* type, const JsonObjectConst& params);
// 当前是否有轮子/原地旋转在动（定距 move / spin 的到段等待用）。返回是否在转。
bool wheels_moving();
// 机械臂（移爪/抬落两个舵机，含 grasp 的"合爪后自动抬臂"）当前是否还在动。
// 供 AI 出帧前等机械臂动作到位再取帧：离散定位走 S 形缓动、持续步进逐拍、grasp 合爪后按定时抬臂，
// 三者都不被 wheels_moving() 覆盖 —— 不查它就可能在臂没停时抓帧，导致"夹起前后画面几乎一样"。
bool arm_moving();
// 给 AI 的持续 move 设一个行驶时限（ms，>0 到期自动停轮；<=0 关闭不限时）。
// 无里程计兜底：AI 决策间隔可达数秒，避免持续 move 在两次决策间一直冲撞墙。
void set_move_cap_ms(int ms);
// 夹爪当前是否处于夹紧态（供 get_state 同步手机按钮）。
bool grip_closing();
// 某路灯当前是否开启（kind=front/vibe/back；供 get_state 同步手机按钮）。
bool light_on(const char* kind);
// 本地合成状态文本，写不进内容即返回 false。
bool read_state(char* buf, size_t cap);

}  // namespace exec