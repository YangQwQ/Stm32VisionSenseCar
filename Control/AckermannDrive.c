#include "AckermannDrive.h"
#include "Vehicle_Chassis.h"
#include "NeZha.h"
#include "Odom.h"

#define CLAMP(v, lo, hi)  (((v) < (lo)) ? (lo) : (((v) > (hi)) ? (hi) : (v)))

static uint16_t steer_pwm = STEER_CENTER_PWM;

/* ---- 速度闭环状态：当前运动模式 + 目标轮速（指令域 0~1000） ---- */
typedef enum { DRV_IDLE, DRV_FWD, DRV_BACK, DRV_TURN_L, DRV_TURN_R } DriveMode;
static DriveMode drive_mode   = DRV_IDLE;
static uint16_t  drive_target = 0;     /* 目标轮速（指令域 0~1000） */
static int32_t   pid_intg     = 0;     /* 积分项（±SPD_INTG_MAX） */
static int32_t   pid_err_prev = 0;     /* 上一拍误差（微分用） */

/* 按当前模式把 PWM 施加到四轮（IDLE 不发车，停止统一走 AckermannDrive_Stop） */
static void ApplyChassis(DriveMode m, uint16_t pwm)
{
	switch (m)
	{
	case DRV_FWD:    Vehicle_Chassis_Forward(pwm);  break;
	case DRV_BACK:   Vehicle_Chassis_Backward(pwm); break;
	case DRV_TURN_L:
	case DRV_TURN_R: Vehicle_Chassis_Forward(pwm);  break;   /* 打角已在转向入口设好 */
	default:         break;
	}
}

/* 进入/维持某运动：运动模式或目标速度任一变化才重设（清 PID 并立即以目标 PWM
 * 起步，保证响应）；相同运动重复调用（自测每拍、UART 续发）不重置，避免打断闭环。 */
static void AckermannDrive_Go(DriveMode m, uint16_t sp)
{
	uint16_t s = sp > 1000u ? 1000u : sp;   /* sp 无符号，只需上限钳位 */

	if (m == drive_mode && s == drive_target) return;

	drive_mode   = m;
	drive_target = s;
	pid_intg     = 0;
	pid_err_prev = 0;
	ApplyChassis(drive_mode, drive_target);
}

void AckermannDrive_Init(void)
{
	steer_pwm = STEER_CENTER_PWM;
	Arc_ServoPwm_Set(STEER_CENTER_PWM);
	AckermannDrive_Stop();
}

void AckermannDrive_Stop(void)
{
	/* 四轮 PWM=0（保持当前转向） */
	NeZha_Motor1_SetPwm(0, 0);
	NeZha_Motor2_SetPwm(0, 0);
	NeZha_Motor3_SetPwm(0, 0);
	NeZha_Motor4_SetPwm(0, 0);
	drive_mode   = DRV_IDLE;
	drive_target = 0;
	pid_intg     = 0;
	pid_err_prev = 0;
}

void AckermannDrive_SteerCenter(void)
{
	steer_pwm = STEER_CENTER_PWM;
	Arc_ServoPwm_Set(steer_pwm);
}

void AckermannDrive_Straight(uint8_t dir, uint16_t speed)
{
	AckermannDrive_SteerCenter();               /* 直行前轮回中 */
	AckermannDrive_Go((dir == DIR_FWD) ? DRV_FWD : DRV_BACK, speed);
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
	AckermannDrive_Go((dir == TURN_LEFT) ? DRV_TURN_L : DRV_TURN_R, speed);
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
	AckermannDrive_Go((dir == TURN_LEFT) ? DRV_TURN_L : DRV_TURN_R, speed);
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

/* 每 20ms（Odom_Sample 之后）调用：实测轮速比对目标 → PID → 刷新底盘 PWM。
 * SPD_LOOP_EN=0 时直通目标（开环原行为）。 */
void AckermannDrive_Regulate(void)
{
	int32_t e, out;
	float   o;
	int16_t actual;

	if (drive_mode == DRV_IDLE || drive_target == 0) return;
	actual = Odom_SpeedUnits();            /* 0~1000，已取幅值 */

#if !SPD_LOOP_EN
	ApplyChassis(drive_mode, drive_target);   /* 开环直通 */
	return;
#endif

	e = (int32_t)drive_target - actual;
	pid_intg += e;
	if (pid_intg > SPD_INTG_MAX)      pid_intg = SPD_INTG_MAX;
	else if (pid_intg < -SPD_INTG_MAX) pid_intg = -SPD_INTG_MAX;

	o = SPD_KP * (float)e + SPD_KI * (float)pid_intg + SPD_KD * (float)(e - pid_err_prev);
	pid_err_prev = e;

	out = (int32_t)o;
	out = CLAMP(out, 0, 1000);         /* 只正向驱动，不反向制动（无主动刹车） */
	ApplyChassis(drive_mode, (uint16_t)out);
}
