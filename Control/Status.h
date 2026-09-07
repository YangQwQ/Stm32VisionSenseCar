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

/* 0x0A 状态帧 flag 位（P1-1 / P1-3：完成标志与夹爪细化，扩展 flag 不新增 CMD） */
#define CAR_FLAG_SLIP       0x01   /* bit0 打滑/堵转 */
#define CAR_FLAG_DONE       0x02   /* bit1 小车定距/定角指令已完成 */
#define ARM_FLAG_DONE       0x01   /* bit0 机械臂定距(升降/移爪)指令已完成 */
#define ARM_FLAG_GRIP_DONE  0x02   /* bit1 夹取/松开动作已完成 */
#define ARM_FLAG_HAS_LOAD   0x04   /* bit2 是否夹到东西（需夹爪到位/电流判定，待标定） */

typedef struct
{
	uint8_t  car_state;
	uint8_t  car_speed;
	uint8_t  car_flag;       /* 自动由状态/完成标志合成（见 Status_SendCar） */
	uint8_t  arm_state;
	uint8_t  arm_grip;
	uint8_t  arm_flag;       /* 自动合成（见 Status_SendArm） */

	/* P1-1：指令完成标志（到达目标后置 1，新指令时清 0，随状态帧周期上报） */
	uint8_t  car_done;
	uint8_t  arm_done;
	/* P1-3：夹爪动作完成 / 是否夹到东西 */
	uint8_t  arm_grip_done;
	uint8_t  arm_has_load;
} SysStatus_t;

extern SysStatus_t g_status;

void Status_Init(void);
/* 发送小车子系统状态帧（DEV=0x01, CMD=0x0A）*/
void Status_SendCar(void);
/* 发送机械臂子系统状态帧（DEV=0x02, CMD=0x0A）*/
void Status_SendArm(void);

#endif