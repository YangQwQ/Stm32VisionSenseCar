#include "Status.h"
#include "UartFrame.h"
#include "Odom.h"
#include <string.h>

SysStatus_t g_status;

/* 上次已发送的数据，用于"数据无变化则不上报" */
static uint8_t last_car[5];
static uint8_t last_arm[5];

static int PayloadChanged(const uint8_t *newp, uint8_t *last, uint8_t len)
{
	int i;
	for (i = 0; i < len; i++)
		if (newp[i] != last[i])
		{
			memcpy(last, newp, len);
			return 1;
		}
	return 0;
}

void Status_Init(void)
{
	g_status.car_state = CAR_STATE_STOP;
	g_status.car_speed = 0;
	g_status.car_flag  = 0;
	g_status.arm_state = ARM_STATE_IDLE;
	g_status.arm_grip  = GRIP_OPEN;
	g_status.arm_flag  = 0;

	g_status.car_done       = 0;
	g_status.arm_done       = 0;
	g_status.arm_grip_done  = 0;
	g_status.arm_has_load   = 0;
}

void Status_SendCar(void)
{
	uint8_t p[6];
	int32_t dist = Odom_GetDistCm();
	uint16_t param = (dist > 0) ? (uint16_t)dist : (uint16_t)(-dist);

	/* flag 合成：打滑/堵转 + 小车指令完成标志（P1-1） */
	g_status.car_flag = 0;
	if (Odom_ErrFlag() & 1)  g_status.car_flag |= CAR_FLAG_SLIP;
	if (g_status.car_done)   g_status.car_flag |= CAR_FLAG_DONE;

	p[0] = g_status.car_state;
	p[1] = g_status.car_speed;
	p[2] = (uint8_t)(param & 0xFF);          /* 低字节在前 */
	p[3] = (uint8_t)((param >> 8) & 0xFF);
	p[4] = g_status.car_flag;

	/* 数据与上次相同则不上报，避免重复发送 */
	if (!PayloadChanged(p, last_car, 5)) return;

	/* 状态帧：DEV=0x01, CMD=0x0A, PAYLOAD=state speed param(2) flag */
	UartFrame_Send(UART_DEV_CAR, UART_CMD_STATUS, p, 5);
}

void Status_SendArm(void)
{
	uint8_t p[6];

	/* flag 合成：机械臂定距完成 + 夹爪动作完成 + 是否夹到东西（P1-3） */
	g_status.arm_flag = 0;
	if (g_status.arm_done)       g_status.arm_flag |= ARM_FLAG_DONE;
	if (g_status.arm_grip_done)  g_status.arm_flag |= ARM_FLAG_GRIP_DONE;
	if (g_status.arm_has_load)   g_status.arm_flag |= ARM_FLAG_HAS_LOAD;

	p[0] = g_status.arm_state;
	p[1] = g_status.arm_grip;
	p[2] = 0;
	p[3] = 0;
	p[4] = g_status.arm_flag;

	/* 数据与上次相同则不上报，避免重复发送 */
	if (!PayloadChanged(p, last_arm, 5)) return;

	UartFrame_Send(UART_DEV_ARM, UART_CMD_STATUS, p, 5);
}