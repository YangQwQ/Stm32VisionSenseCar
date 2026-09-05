#include "Status.h"
#include "UartFrame.h"
#include "Odom.h"

SysStatus_t g_status;

void Status_Init(void)
{
	g_status.car_state = CAR_STATE_STOP;
	g_status.car_speed = 0;
	g_status.car_flag  = 0;
	g_status.arm_state = ARM_STATE_IDLE;
	g_status.arm_grip  = GRIP_OPEN;
	g_status.arm_flag  = 0;
}

void Status_SendCar(void)
{
	uint8_t p[6];
	int32_t dist = Odom_GetDistCm();
	uint16_t param = (dist > 0) ? (uint16_t)dist : (uint16_t)(-dist);

	p[0] = g_status.car_state;
	p[1] = g_status.car_speed;
	p[2] = (uint8_t)(param & 0xFF);          /* 低字节在前 */
	p[3] = (uint8_t)((param >> 8) & 0xFF);
	p[4] = g_status.car_flag;

	/* 状态帧：DEV=0x01, CMD=0x0A, PAYLOAD=state speed param(2) flag */
	UartFrame_Send(UART_DEV_CAR, UART_CMD_STATUS, p, 5);
}

void Status_SendArm(void)
{
	uint8_t p[6];

	p[0] = g_status.arm_state;
	p[1] = g_status.arm_grip;
	p[2] = 0;
	p[3] = 0;
	p[4] = g_status.arm_flag;

	UartFrame_Send(UART_DEV_ARM, UART_CMD_STATUS, p, 5);
}