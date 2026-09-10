#pragma once
#include <Arduino.h>

// 哪吒扩展板直驱（调试用）：把机械臂舵机 I2C 从执行板 STM32 搬到本板软件 I2C，
// 用于隔离「执行板 STM32 是否导致机械臂抽搐」。协议与执行板 NeZha_I2c.c/NeZha.c 一致：
//   从机地址 0x80；写命令先发 [0x80,0x00,cmd]，再发 [0x80,cmd,hi,lo]；速率 ≤200kHz。
// 接线：哪吒 SCL ← 本板 GPIO47，哪吒 SDA ← 本板 GPIO14；执行板 PB6/PB7 需脱离总线。
namespace nezha {

// 初始化（幂等，惰性）：把两脚设为开漏输出并拉高。
void init();

// 直驱单个舵机：channel=1..4，pwm=50(0°)..250(180°)。返回 false 表示参数越界。
bool set_servo(uint8_t channel, uint16_t pwm);

// 直驱单轮电机：channel=1..4（哪吒 M1..M4），a=正向 PWM、b=反向 PWM，范围 0..1000
//（a>0 正转、b>0 反转、都 0 停）。返回 false 表示参数越界。
bool set_motor(uint8_t channel, uint16_t a, uint16_t b);

// 直驱灯光：kind = front(前灯) / vibe(氛围灯) / back(尾灯=左右一起)。返回是否被识别。
bool led(const char* kind, bool on);

}  // namespace nezha