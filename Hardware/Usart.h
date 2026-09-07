#ifndef _USART_H_
#define _USART_H_

#include "stm32f10x.h"

#define UART1_RX_BUF_SIZE   128

void Usart1_Init(void);

/* 发送单个字节（阻塞，等 TXE） */
void Usart1_SendByte(uint8_t Byte);
/* 发送多个字节 */
void Usart1_SendBytes(const uint8_t *buf, uint16_t len);

/* 接收环形缓冲：可用字节数 */
uint8_t Usart1_RxAvailable(void);
/* 取出一个字节（调用前先 RxAvailable>0） */
uint8_t Usart1_RxRead(void);
/* 清空接收缓冲 */
void Usart1_RxFlush(void);

/* USART1 接收中断处理（RXNE），定义于 Usart.c */
void USART1_IRQHandler(void);

#endif