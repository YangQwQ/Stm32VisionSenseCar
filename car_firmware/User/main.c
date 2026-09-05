#include "stm32f10x.h"
#include "LED.h"
#include "NeZha.h"
#include "Usart.h"
#include "Board_Timer.h"
#include "Vehicle_Chassis.h"
#include "RobotArm.h"
#include "UartFrame.h"
#include "Dispatch.h"
#include "Odom.h"
#include "RobotArmAct.h"
#include "Status.h"
#include "AckermannDrive.h"

static uint32_t Time;

int main(void)
{
	__disable_irq();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	LED_Init();
	Usart1_Init();                     /* 115200 + RXNE 中断 */
	NeZha_Init();
	NeZha_Encoder1_Init();
	NeZha_Encoder2_Init();
	NeZha_Encoder3_Init();
	NeZha_Encoder4_Init();
	RobotArm_Init();                   /* Servo2/3/4 机械臂复位 */
	Vehicle_Chassis_Init();            /* 电机初始化 + 阿克曼舵机回中 */
	AckermannDrive_Init();
	Board_Timer_Init();                /* TIM2 5ms 时基 */

	UartFrame_Reset();
	Odom_Init();
	RobotArmAct_Init();
	Dispatch_Init();
	Status_Init();

	Time = 0;
	__enable_irq();

	while (1)
	{
		if (Board_Timer_Flag_Get())     /* 5ms 节拍 */
		{
			Time++;

			/* 5ms：收包组帧 → 译码分发 */
			{
				uint8_t n = Usart1_RxAvailable();
				while (n--) UartFrame_Feed(Usart1_RxRead());
				while (1)
				{
					UartFrame_t f;
					if (!UartFrame_Get(&f)) break;
					Dispatch_Process(&f);
				}
			}

			/* 20ms：里程计采样 + 到位判定 */
			if (Time % 4 == 0)
			{
				Odom_Sample();
				Dispatch_Periodic();
			}

			/* 25ms：机械臂步进 */
			if (Time % 5 == 0)
			{
				RobotArmAct_Step();
			}

			/* 100ms：周期状态上报 */
			if (Time % 20 == 0)
			{
				Status_SendCar();
				Status_SendArm();
			}

			/* 1s：运行指示灯 */
			if (Time % 200 == 0) LED1_Turn();
		}
	}
}