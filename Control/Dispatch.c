#include "Dispatch.h"
#include "AckermannDrive.h"
#include "RobotArmAct.h"
#include "Odom.h"
#include "Relay.h"
#include "Status.h"

/* 小车 CMD（§5.5） */
#define CAR_CMD_MOVE_CONT      0x01
#define CAR_CMD_MOVE_DIST      0x02
#define CAR_CMD_TURN_CONT      0x03
#define CAR_CMD_TURN_ANGLE     0x04
#define CAR_CMD_STOP           0x05
#define CAR_CMD_QUERY          0x06

/* 机械臂 CMD（§5.5） */
#define ARM_CMD_LIFT_CONT      0x01
#define ARM_CMD_LIFT_DIST      0x02
#define ARM_CMD_REACH_CONT     0x03
#define ARM_CMD_REACH_DIST     0x04
#define ARM_CMD_GRIP           0x05
#define ARM_CMD_QUERY          0x06

/* stop scope */
#define STOP_ALL               0x00
#define STOP_WHEELS            0x01
#define STOP_ARM               0x02

typedef enum { CAR_IDLE, CAR_MOVE, CAR_TURN } CarMove;
static CarMove car_move;
static uint8_t car_dir, car_speed;

/* P1-3：机械臂最近下发的待完成动作类型（用于区分"定距完成"与"夹爪完成"） */
static uint8_t arm_pending;   /* 0=无 1=定距(升降/移爪) 2=夹爪 */

static uint16_t PayloadU16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

/* 协议 speed 是单字节(0~255)，而驱动层按 0~1000 吃 PWM：输入边界缩放成档位 */
static uint16_t CarPwm(uint8_t sp) { return ((uint16_t)sp * 1000u) / 255u; }

void Dispatch_Init(void)
{
	car_move = CAR_IDLE;
	car_dir = 0; car_speed = 0;
	arm_pending = 0;
	Relay_Stop();
	g_status.car_state = CAR_STATE_STOP;
	g_status.arm_state = ARM_STATE_IDLE;
}

static void HandleCarCmd(const UartFrame_t *f)
{
	uint8_t  dir, sp;
	uint16_t dist, ang;

	switch (f->b[1])
	{
	case CAR_CMD_MOVE_CONT:                       /* 持续移动 dir speed */
		dir = f->b[2]; sp = f->b[3];
		car_dir = dir; car_speed = sp;
		AckermannDrive_Straight(dir, CarPwm(sp));
		Relay_Stop();
		car_move = CAR_MOVE;
		break;

	case CAR_CMD_MOVE_DIST:                       /* dir dist_cm(2) speed */
		dir = f->b[2];
		dist = PayloadU16(&f->b[3]);
		sp = f->b[5];
		car_dir = dir; car_speed = sp;
		g_status.car_done = 0;                    /* 进入执行：清除上次完成标志 */
		AckermannDrive_Straight(dir, CarPwm(sp));
		Relay_Start(RLY_DIST, (int32_t)dist * 10);   /* 0.1cm */
		car_move = CAR_MOVE;
		break;

	case CAR_CMD_TURN_CONT:                       /* dir speed */
		dir = f->b[2]; sp = f->b[3];
		car_dir = dir; car_speed = sp;
		AckermannDrive_Turn(dir, CarPwm(sp));
		Relay_Stop();
		car_move = CAR_TURN;
		break;

	case CAR_CMD_TURN_ANGLE:                      /* dir angle_deg(2) speed */
		dir = f->b[2];
		ang = PayloadU16(&f->b[3]);
		sp = f->b[5];
		car_dir = dir; car_speed = sp;
		g_status.car_done = 0;                    /* 进入执行：清除上次完成标志 */
		AckermannDrive_TurnAngle(dir, (uint8_t)ang, CarPwm(sp));
		Relay_Start(RLY_YAW, (int32_t)ang * 10);     /* 0.1° */
		car_move = CAR_TURN;
		break;

	case CAR_CMD_STOP:                            /* scope */
		switch (f->b[2])
		{
		case STOP_ALL:
		case STOP_WHEELS:
			AckermannDrive_Stop();
			Relay_Stop();
			car_move = CAR_IDLE;
			car_speed = 0;
			g_status.car_done = 0;
			break;
		}
		if (f->b[2] == STOP_ALL || f->b[2] == STOP_ARM)
		{
			RobotArmAct_Stop();
			g_status.arm_state = ARM_STATE_IDLE;
			arm_pending = 0;                      /* 中断动作：清除待完成标记 */
		}
		break;

	case CAR_CMD_QUERY:
		Status_SendCar();
		break;
	default:
		break;
	}
}

static void HandleArmCmd(const UartFrame_t *f)
{
	uint8_t dir, sp, act;
	uint16_t dist;

	switch (f->b[1])
	{
	case ARM_CMD_LIFT_CONT:                      /* dir speed */
		dir = f->b[2]; sp = f->b[3];
		RobotArmAct_StartLift(dir, sp);
		g_status.arm_state = ARM_STATE_LIFT;
		break;

	case ARM_CMD_LIFT_DIST:                      /* dir dist_cm(2) speed */
		dir = f->b[2];
		dist = PayloadU16(&f->b[3]);
		sp = f->b[5];
		arm_pending = 1;                          /* 待完成：定距 */
		g_status.arm_done = 0;
		RobotArmAct_LiftBy(dir, dist, sp);
		g_status.arm_state = ARM_STATE_LIFT;
		break;

	case ARM_CMD_REACH_CONT:                     /* dir speed */
		dir = f->b[2]; sp = f->b[3];
		RobotArmAct_StartReach(dir, sp);
		g_status.arm_state = ARM_STATE_REACH;
		break;

	case ARM_CMD_REACH_DIST:                     /* dir dist_cm(2) speed */
		dir = f->b[2];
		dist = PayloadU16(&f->b[3]);
		sp = f->b[5];
		arm_pending = 1;                          /* 待完成：定距 */
		g_status.arm_done = 0;
		RobotArmAct_ReachBy(dir, dist, sp);
		g_status.arm_state = ARM_STATE_REACH;
		break;

	case ARM_CMD_GRIP:                           /* act */
		act = f->b[2];
		arm_pending = 2;                          /* 待完成：夹爪动作 */
		g_status.arm_grip_done = 0;
		RobotArmAct_StartGrip(act);
		g_status.arm_grip = (act == 0x01) ? GRIP_CLOSED : GRIP_OPEN;
		g_status.arm_state = ARM_STATE_REACH;
		break;

	case ARM_CMD_QUERY:
		Status_SendArm();
		break;
	default:
		break;
	}
}

void Dispatch_Process(const UartFrame_t *frame)
{
	if (frame->b[0] == UART_DEV_CAR)
	{
		HandleCarCmd(frame);
	}
	else if (frame->b[0] == UART_DEV_ARM)
	{
		HandleArmCmd(frame);
	}
}

void Dispatch_Periodic(void)
{
	/* 到位判定：指定距离/角度已达成 → 停车并上报完成 */
	if (Relay_Active() && Relay_Check())
	{
		AckermannDrive_Stop();
		Relay_Stop();
		car_move = CAR_IDLE;
		car_speed = 0;
		g_status.car_done = 1;                    /* P1-1：定距/定角已达成 */
		Status_SendCar();                     /* 上报"已完成" */
	}

	/* 机械臂指定距离/夹爪到位 → 状态复位并置完成标志 */
	if (!RobotArmAct_Busy() && g_status.arm_state != ARM_STATE_IDLE)
	{
		g_status.arm_state = ARM_STATE_IDLE;
		if (arm_pending == 2)
		{
			g_status.arm_grip_done = 1;         /* P1-3：夹取/松开动作完成 */
			arm_pending = 0;
		}
		else if (arm_pending == 1)
		{
			g_status.arm_done = 1;              /* P1-1：升降/移爪定距完成 */
			arm_pending = 0;
		}
	}

	/* 刷新小车运动状态 */
	if (car_move == CAR_MOVE)      g_status.car_state = CAR_STATE_MOVE;
	else if (car_move == CAR_TURN) g_status.car_state = CAR_STATE_TURN;
	else                           g_status.car_state = CAR_STATE_STOP;
	g_status.car_speed = car_speed;
}