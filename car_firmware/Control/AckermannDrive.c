#include "AckermannDrive.h"
#include "Vehicle_Chassis.h"
#include "NeZha.h"

static uint16_t steer_pwm = STEER_CENTER_PWM;

#define CLAMP(v, lo, hi)  (((v) < (lo)) ? (lo) : (((v) > (hi)) ? (hi) : (v)))

void AckermannDrive_Init(void)
{
	steer_pwm = STEER_CENTER_PWM;
	Arc_ServoPwm_Set(STEER_CENTER_PWM);
	AckermannDrive_Stop();
}

void AckermannDrive_Stop(void)
{
	/* Vehicle_Chassis 停止：四轮 PWM=0 */
	NeZha_Motor1_SetPwm(0, 0);
	NeZha_Motor2_SetPwm(0, 0);
	NeZha_Motor3_SetPwm(0, 0);
	NeZha_Motor4_SetPwm(0, 0);
}

void AckermannDrive_SteerCenter(void)
{
	steer_pwm = STEER_CENTER_PWM;
	Arc_ServoPwm_Set(steer_pwm);
}

void AckermannDrive_Straight(uint8_t dir, uint16_t speed)
{
	AckermannDrive_SteerCenter();               /* 直行前轮回中 */
	if (dir == DIR_FWD)
	{
		Vehicle_Chassis_Forward(speed);
	}
	else
	{
		Vehicle_Chassis_Backward(speed);
	}
}

void AckermannDrive_Turn(uint8_t dir, uint16_t speed)
{
	if (dir == TURN_LEFT)
	{
		steer_pwm = STEER_LEFT_PWM;
	}
	else
	{
		steer_pwm = STEER_RIGHT_PWM;
	}
	Arc_ServoPwm_Set(steer_pwm);
	Vehicle_Chassis_Forward(speed);             /* 打角 + 前进，剥成转弯 */
}

uint16_t AckermannDrive_SteerTo(uint8_t dir, uint8_t angle_deg)
{
	int16_t offset, target;

	/* angle_deg 相对中值偏移量，映射到 PWM（联调标定：SERVO_CNT_PER_DEG）*/
	/* 先给经验映射：满量程 (RIGHT-LEFT)=70 PWM 对应 STEER_MAX_DEG*2=60° */
	offset = (int16_t)((int32_t)angle_deg * (STEER_RIGHT_PWM - STEER_LEFT_PWM) / (2 * STEER_MAX_DEG));

	if (dir == TURN_LEFT)
	{
		target = (int16_t)STEER_CENTER_PWM - offset;
	}
	else
	{
		target = (int16_t)STEER_CENTER_PWM + offset;
	}

	steer_pwm = (uint16_t)CLAMP(target, STEER_LEFT_PWM, STEER_RIGHT_PWM);
	Arc_ServoPwm_Set(steer_pwm);
	return steer_pwm;
}

void AckermannDrive_TurnAngle(uint8_t dir, uint8_t angle_deg, uint16_t speed)
{
	AckermannDrive_SteerTo(dir, angle_deg);
	Vehicle_Chassis_Forward(speed);
}

uint16_t AckermannDrive_GetSteerPwm(void)
{
	return steer_pwm;
}

float AckermannDrive_SteerRad(void)
{
	float ratio = (float)((int)steer_pwm - STEER_CENTER_PWM) /
	              (float)(STEER_RIGHT_PWM - STEER_CENTER_PWM);
	/* 右正左负，映射到最大转向角 */
	return ratio * STEER_MAX_DEG * 3.14159265f / 180.0f;
}