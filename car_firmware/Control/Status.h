#ifndef _STATUS_H_
#define _STATUS_H_

#include "stm32f10x.h"

/* §5.5 状态字段 */
#define CAR_STATE_STOP      0
#define CAR_STATE_MOVE      1
#define CAR_STATE_TURN      2
#define ARM_STATE_IDLE      0
#define ARM_STATE_LIFT      1
#define ARM_STATE_REACH     2
#define GRIP_OPEN           0
#define GRIP_CLOSED         1

typedef struct
{
	uint8_t  car_state;
	uint8_t  car_speed;
	uint8_t  car_flag;
	uint8_t  arm_state;
	uint8_t  arm_grip;
	uint8_t  arm_flag;
} SysStatus_t;

extern SysStatus_t g_status;

void Status_Init(void);
/* 发送小车子系统状态帧（DEV=0x01, CMD=0x0A）*/
void Status_SendCar(void);
/* 发送机械臂子系统状态帧（DEV=0x02, CMD=0x0A）*/
void Status_SendArm(void);

#endif