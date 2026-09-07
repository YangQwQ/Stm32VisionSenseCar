#include "Odom.h"
#include "NeZha.h"
#include "AckermannDrive.h"
#include <math.h>

static Odom_t o;

void Odom_Init(void)
{
	Odom_Reset();
}

void Odom_Reset(void)
{
	o.dist_mm = 0;
	o.yaw_deg_x10 = 0;
	o.speed = 0;
}

/* 从 4 路编码器增量估计驱动距离：取四轮均值（阿克曼后轮为主） */
static int32_t DriveDelta(void)
{
	int32_t sum = (int32_t)NeZha_Encoder1_Read()
	            + (int32_t)NeZha_Encoder2_Read()
	            + (int32_t)NeZha_Encoder3_Read()
	            + (int32_t)NeZha_Encoder4_Read();
	return sum / 4;
}

void Odom_Sample(void)
{
	int32_t delta_enc = DriveDelta();
	int32_t delta_mm;      /* 本次行进距离，单位 0.1mm */
	float steer_rad;
	float d_yaw_rad;
	int32_t dy;

	o.speed = (int16_t)delta_enc;

	/* 距离累计 */
	delta_mm = delta_enc * 10 / ENC_CNT_PER_CM;
	o.dist_mm += delta_mm;

	/* yaw 累计：dψ(rad) ≈ Δ距离(cm)/轴距(cm) * tan(转向角) */
	steer_rad = AckermannDrive_SteerRad();
	d_yaw_rad = (float)delta_mm / 100.0f / (float)WHEELBASE_CM * tanf(steer_rad);
	dy = (int32_t)(d_yaw_rad * 1800.0f / 3.14159265f);   /* 0.1° */
	o.yaw_deg_x10 += dy;
}

int32_t Odom_GetDistCm(void)      { return o.dist_mm / 10; }
int32_t Odom_GetDistMm(void)      { return o.dist_mm; }
int32_t Odom_GetYawDegX10(void)   { return o.yaw_deg_x10; }
int16_t Odom_GetSpeed(void)       { return o.speed; }

/* 实测轮速 → 0~1000 指令域（取幅值，反向为负也按幅值参与控制） */
int16_t Odom_SpeedUnits(void)
{
	int32_t s = o.speed;

	if (s < 0) s = -s;
	s = s * 1000L / SPD_FULLSCALE_ENC;
	if (s > 1000) s = 1000;
	return (int16_t)s;
}

/* 打滑/堵转检测：与目标速度比对（占位，联调标定） */
uint16_t Odom_ErrFlag(void)
{
	return 0;
}
