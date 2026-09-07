#include "RobotArmAct.h"
#include "RobotArm.h"

static ArmActKind kind;
static uint8_t    dir;
static uint8_t    speed;
static int16_t    step_accum;    /* 已执行步进数 */
static int16_t    target_step;   /* 到位目标步进数（0=持续） */
static uint8_t    busy;

static uint8_t ArmStepPerTick(void)
{
	/* speed 放大步进：0~1000 映射到 1~5 */
	return (uint8_t)(1 + (uint16_t)speed / 300u);
}

void RobotArmAct_Init(void)
{
	kind = ARM_IDLE;
	busy = 0;
	step_accum = 0;
	target_step = 0;
}

void RobotArmAct_Stop(void)
{
	/* 仅停止动作，不复位舵机位置 */
	kind = ARM_IDLE;
	busy = 0;
	step_accum = 0;
	target_step = 0;
}

uint8_t RobotArmAct_Busy(void) { return busy; }
uint8_t RobotArmAct_Kind(void) { return (uint8_t)kind; }

void RobotArmAct_StartLift(uint8_t d, uint8_t sp)
{
	dir = d; speed = sp;
	kind = ARM_LIFT;
	target_step = 0;       /* 持续 */
	step_accum = 1;        /* 首拍先执行一步 */
	busy = 1;
}

void RobotArmAct_LiftBy(uint8_t d, uint16_t dist_cm, uint8_t sp)
{
	dir = d; speed = sp;
	kind = ARM_LIFT;
	target_step = (int16_t)(dist_cm * ARM_CNT_PER_CM);
	step_accum = 1;
	busy = 1;
}

void RobotArmAct_StartReach(uint8_t d, uint8_t sp)
{
	dir = d; speed = sp;
	kind = ARM_REACH;
	target_step = 0;
	step_accum = 1;
	busy = 1;
}

void RobotArmAct_ReachBy(uint8_t d, uint16_t dist_cm, uint8_t sp)
{
	dir = d; speed = sp;
	kind = ARM_REACH;
	target_step = (int16_t)(dist_cm * ARM_CNT_PER_CM);
	step_accum = 1;
	busy = 1;
}

void RobotArmAct_StartGrip(uint8_t act)
{
	dir = act;            /* 0x01 夹 / 0x02 松 */
	kind = ARM_GRIP;
	target_step = ARM_GRIP_STEPS;
	step_accum = 1;
	busy = 1;
}

static void DoStep(void)
{
	uint8_t unit = ArmStepPerTick();

	switch (kind)
	{
	case ARM_LIFT:
		if (dir == 0x01) RobotArm_RaiseHand(unit);
		else             RobotArm_DropHand(unit);
		break;
	case ARM_REACH:
		if (dir == 0x01) RobotArm_StretchHand(unit);
		else             RobotArm_ShrinkHand(unit);
		break;
	case ARM_GRIP:
		if (dir == 0x01) RobotArm_ShakeHand(unit);   /* 夹取 */
		else             RobotArm_LetHand(unit);     /* 松开 */
		break;
	default:
		break;
	}
}

void RobotArmAct_Step(void)
{
	int16_t unit;

	if (!busy) return;

	DoStep();

	if (target_step > 0)
	{
		unit = (int16_t)ArmStepPerTick();
		step_accum += unit;
		if (step_accum >= target_step)
		{
			busy = 0;
			kind = ARM_IDLE;
		}
	}
	/* target_step==0：持续动作，由 stop(scope=arm) 停止 */
}