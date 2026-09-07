#ifndef _RELAY_H_
#define _RELAY_H_

#include "stm32f10x.h"

typedef enum
{
	RLY_NONE = 0,
	RLY_DIST,     /* 距离目标（0.1cm） */
	RLY_YAW       /* 角度目标（0.1°） */
} RelayKind;

void Relay_Start(RelayKind kind, int32_t magnitude);
void Relay_Stop(void);
uint8_t Relay_Active(void);
/* 比对里程计是否达到目标；达到则自动停用并返回 1 */
uint8_t Relay_Check(void);

#endif
