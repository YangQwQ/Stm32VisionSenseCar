#ifndef _ODOM_H_
#define _ODOM_H_

#include "stm32f10x.h"

/* 每厘米编码器计数（轮径/减速比/PPR 综合，联调标定，先给经验占位） */
#define ENC_CNT_PER_CM   100

typedef struct
{
	int32_t dist_mm;        /* 累计行进距离（0.1cm 分辨率：单位 0.1mm） */
	int32_t yaw_deg_x10;    /* 累计车身转角，单位 0.1°（左负右正） */
	int16_t speed;          /* 最近一次采样速度（编码器 delta，供状态帧） */
} Odom_t;

void Odom_Init(void);
void Odom_Reset(void);

/* 每 20ms 调用：采样 4 路编码器并累计 */
void Odom_Sample(void);

int32_t Odom_GetDistCm(void);      /* 返回整厘米 */
int32_t Odom_GetDistMm(void);      /* 0.1cm */
int32_t Odom_GetYawDegX10(void);   /* 0.1° */
int16_t Odom_GetSpeed(void);
uint16_t Odom_ErrFlag(void);       /* 打滑/堵转检测位（联调标定）*/

#endif