#ifndef _ROBOTARMACT_H_
#define _ROBOTARMACT_H_

#include "stm32f10x.h"

/* 机械臂指定距离→舵机 PWM 步进换算（无编码器，联调标定） */
#define ARM_CNT_PER_CM      3
/* 夹取/松开的动作步进数（到目标即停） */
#define ARM_GRIP_STEPS      20

typedef enum
{
	ARM_IDLE = 0,
	ARM_LIFT,      /* 持续/指定距离 升降 */
	ARM_REACH,     /* 持续/指定距离 前后移爪 */
	ARM_GRIP       /* 夹取/松开 */
} ArmActKind;

void RobotArmAct_Init(void);
void RobotArmAct_Stop(void);
uint8_t RobotArmAct_Busy(void);
uint8_t RobotArmAct_Kind(void);

/* 持续升降：dir=0x01 抬 / 0x02 落 (§5.5 机械臂 0x01) */
void RobotArmAct_StartLift(uint8_t dir, uint8_t speed);
/* 升降指定距离：dir 同 / dist_cm (§5.5 机械臂 0x02) */
void RobotArmAct_LiftBy(uint8_t dir, uint16_t dist_cm, uint8_t speed);
/* 持续前后移爪：dir=0x01 前伸 / 0x02 后缩 (0x03) */
void RobotArmAct_StartReach(uint8_t dir, uint8_t speed);
/* 前后移爪指定距离 (0x04) */
void RobotArmAct_ReachBy(uint8_t dir, uint16_t dist_cm, uint8_t speed);
/* 夹取/松开：act=0x01 夹 / 0x02 松 (0x05) */
void RobotArmAct_StartGrip(uint8_t act);

/* 每 20~25ms 调用：执行一步并用目标距离步进数判定到位 */
void RobotArmAct_Step(void);

#endif