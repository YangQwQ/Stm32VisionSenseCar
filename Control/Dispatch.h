#ifndef _DISPATCH_H_
#define _DISPATCH_H_

#include "stm32f10x.h"
#include "UartFrame.h"

void Dispatch_Init(void);
/* 处理一帧已验证的指令（§5.5 译码 + 分发） */
void Dispatch_Process(const UartFrame_t *frame);
/* 周期（每 20ms）调用：到位判定 + 状态刷新 */
void Dispatch_Periodic(void);

#endif
