#ifndef _ACKERMANNDRIVE_H_
#define _ACKERMANNDRIVE_H_

#include "stm32f10x.h"

/* 转向舵机 PWM 位置（阿克曼，arc_servo {120,150,190}） */
#define STEER_CENTER_PWM    150
#define STEER_LEFT_PWM      120
#define STEER_RIGHT_PWM     190
/* 转向满打对应的最大转向角（度，用于里程计 yaw 模型，联调标定） */
#define STEER_MAX_DEG       30
/* 轴距（cm，里程计 yaw 模型，联调标定） */
#define WHEELBASE_CM        15

/* dir 取值（§5.5） */
#define DIR_FWD             0x01
#define DIR_BACK            0x02
#define TURN_LEFT           0x01
#define TURN_RIGHT          0x02

/* 速度闭环（增量式 PID，20ms 节拍）：
 *   speed 0~1000 为"目标轮速指令域"，反馈用 Odom_SpeedUnits()（比例由 Odom.h
 *   的 SPD_FULLSCALE_ENC 定标）。SPD_LOOP_EN=0 退化为开环直通（原行为），
 *   便于 A/B 对比。KD 默认 0（微分易放大编码器噪声，需要再开）。 */
#define SPD_LOOP_EN         1
#define SPD_KP              1.0f
#define SPD_KI              0.05f
#define SPD_KD              0.0f
/* 积分限幅（±1000，防饱和后难退） */
#define SPD_INTG_MAX        1000

void AckermannDrive_Init(void);

/* 停止四轮，保持当前转向 */
void AckermannDrive_Stop(void);
/* 转向回中 */
void AckermannDrive_SteerCenter(void);

/* 持续直行：dir=DIR_FWD/DIR_BACK，speed=0~1000 */
void AckermannDrive_Straight(uint8_t dir, uint16_t speed);
/* 转向打角（dir=TURN_LEFT/RIGHT）并以前进驱动，speed=0~1000 */
void AckermannDrive_Turn(uint8_t dir, uint16_t speed);
/* 转向指定角度（度，相对中值偏移，dir 决定方向）并驱动 */
void AckermannDrive_TurnAngle(uint8_t dir, uint8_t angle_deg, uint16_t speed);
/* 只打角（停轮时用），返回实际 PWM */
uint16_t AckermannDrive_SteerTo(uint8_t dir, uint8_t angle_deg);

/* 当前转向舵机 PWM */
uint16_t AckermannDrive_GetSteerPwm(void);
/* 当前转向角（弧度，相对前进方向，左负右正），供里程计 yaw 模型 */
float AckermannDrive_SteerRad(void);

/* 每 20ms（Odom_Sample 之后）调用：实测轮速比对目标 → PID → 刷新底盘 PWM */
void AckermannDrive_Regulate(void);

#endif
