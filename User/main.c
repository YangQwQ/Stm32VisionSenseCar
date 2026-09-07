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

/* PS2 直控自测开关：
 *   0 = 正常主链路：UART（大脑板）指令驱动小车/机械臂。
 *   1 = PS2 手柄直驱自测：不经 UART，用于不接大脑板时手动确认电机方向/
 *       舵机/编码器参数。Odom 采样与 0x0A 状态帧仍周期上报，可接 USB-TTL
 *       到 USART1 边摇手柄边读里程计帧来标定 ENC_CNT_PER_CM 等。
 *   改此值后需在 Keil 里重新全量编译。 */
#define PS2_SELFTEST  0

static uint32_t Time;

#if PS2_SELFTEST
#include "ps2.h"

/* 自测直驱速度（驱动域 0~1000）与机械臂速度（协议字节 0~255） */
#define PS2_CAR_PWM   500
#define PS2_ARM_SPD   200

/* 手柄按键 → 执行器 映射（自测模式）：
 *   十字键上/下     ：直行前/后
 *   十字键左/右     ：持续左/右转（打角 + 前进）
 *   L1/L2           ：机械臂抬/落（Servo4）
 *   R1/R2           ：移爪前伸/后缩（Servo2）
 *   CROSS / SQUARE  ：夹 / 松（Servo3，单次动作，按下触发一次）
 *   松开对应按键即停止/回中。PS2 接收器接线：CS=PA4 SCK=PA5 DI=PA6 DO=PA7
 *   （见 Hardware/ps2.c PS2_GPIO_Init，与例程一致，板子不同则改那里）。 */
static void Ps2SelfTest_Drive(void)
{
	static uint8_t car = CAR_STATE_STOP;
	static uint8_t arm_prev = 0;            /* 上一拍机械臂持续动作：0无 1升降 2移爪 */
	static uint8_t cross_prev = 0, square_prev = 0;
	uint8_t act;

	ps2_key_serch();                     /* 读手柄并刷新按键状态 */

	/* 小车：十字键持续驱动，全松开则停 */
	if      (ps2_get_key_state(PSB_PAD_UP))    { AckermannDrive_Straight(DIR_FWD,  PS2_CAR_PWM); car = CAR_STATE_MOVE; }
	else if (ps2_get_key_state(PSB_PAD_DOWN))  { AckermannDrive_Straight(DIR_BACK, PS2_CAR_PWM); car = CAR_STATE_MOVE; }
	else if (ps2_get_key_state(PSB_PAD_LEFT))  { AckermannDrive_Turn(TURN_LEFT,  PS2_CAR_PWM); car = CAR_STATE_TURN; }
	else if (ps2_get_key_state(PSB_PAD_RIGHT)) { AckermannDrive_Turn(TURN_RIGHT, PS2_CAR_PWM); car = CAR_STATE_TURN; }
	else                                       { AckermannDrive_Stop(); car = CAR_STATE_STOP; }

	/* 机械臂持续动作（升降/移爪）：按住运行，松开才停（沿触发，避免误停夹爪） */
	if      (ps2_get_key_state(PSB_L1)) { RobotArmAct_StartLift(0x01, PS2_ARM_SPD); arm_prev = 1; }   /* 抬 */
	else if (ps2_get_key_state(PSB_L2)) { RobotArmAct_StartLift(0x02, PS2_ARM_SPD); arm_prev = 1; }   /* 落 */
	else if (ps2_get_key_state(PSB_R1)) { RobotArmAct_StartReach(0x01, PS2_ARM_SPD); arm_prev = 2; }  /* 前伸 */
	else if (ps2_get_key_state(PSB_R2)) { RobotArmAct_StartReach(0x02, PS2_ARM_SPD); arm_prev = 2; }  /* 后缩 */
	else if (arm_prev)                  { RobotArmAct_Stop(); arm_prev = 0; }

	/* 夹爪：按下沿触发一次（保持不会反复重启；不受上面 Stop 影响） */
	act = 0;
	if (ps2_get_key_state(PSB_CROSS) && !cross_prev)  act = 0x01;   /* 夹 */
	if (ps2_get_key_state(PSB_SQUARE) && !square_prev) act = 0x02;  /* 松 */
	cross_prev  = ps2_get_key_state(PSB_CROSS);
	square_prev = ps2_get_key_state(PSB_SQUARE);
	if (act) RobotArmAct_StartGrip(act);

	/* 供 0x0A 状态帧：car_state / car_speed（状态帧 speed 为单字节） */
	g_status.car_state = car;
	g_status.car_speed = (car == CAR_STATE_STOP) ? 0u : (uint8_t)(PS2_CAR_PWM / 4u);
}
#endif /* PS2_SELFTEST */

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
#if PS2_SELFTEST
	PS2_Init();                      /* 手柄接收器（PA4-7），自测模式 */
#endif
	Board_Timer_Init();              /* TIM2 5ms 时基 */

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

#if !PS2_SELFTEST
			/* 5ms：收包组帧 → 译码分发（UART 指令主链路） */
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
#else
			/* 50ms：PS2 手柄直驱自测（软时序较慢，勿加到 5ms 级） */
			if (Time % 10 == 0) Ps2SelfTest_Drive();
#endif

			/* 20ms：里程计采样 + 到位判定 */
			if (Time % 4 == 0)
			{
				Odom_Sample();
#if !PS2_SELFTEST
				Dispatch_Periodic();
#endif
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