#ifndef _UARTFRAME_H_
#define _UARTFRAME_H_

#include "stm32f10x.h"

/* §5.4 帧格式：AA 55 LEN DEV CMD [PAYLOAD] CRC16(2B,低在前) */
#define UART_FRAME_HEAD1    0xAA
#define UART_FRAME_HEAD2    0x55
#define UART_FRAME_MAX_LEN  32        /* LEN 上限（DEV+CMD+PAYLOAD）*/

/* DEV 值 */
#define UART_DEV_CAR        0x01
#define UART_DEV_ARM        0x02

/* 状态帧指令码（§5.5 状态帧） */
#define UART_CMD_STATUS     0x0A

typedef struct
{
	uint8_t len;                     /* LEN = DEV+CMD+PAYLOAD */
	uint8_t b[UART_FRAME_MAX_LEN];   /* b[0]=DEV  b[1]=CMD  b[2..]=PAYLOAD */
	uint8_t valid;                   /* 本帧有效（CRC 通过）*/
} UartFrame_t;

void UartFrame_Reset(void);
/* 喂入一个接收字节（USART 接收字节流调用） */
void UartFrame_Feed(uint8_t byte);
/* 取出一帧就绪帧，读取后自动清 valid；返回 1 有帧 */
uint8_t UartFrame_Get(UartFrame_t *frame);
/* 累计 CRC 错误帧计数（丢弃帧数） */
uint16_t UartFrame_ErrCount(void);

/* CRC-16/MODBUS：poly 反射 0xA001，初值 0xFFFF，低位在前（可供测试） */
uint16_t UartCrc16(const uint8_t *data, uint16_t len);

/* 发送一帧：AA 55 LEN DEV CMD PAYLOAD CRC16(低在前) */
void UartFrame_Send(uint8_t dev, uint8_t cmd, const uint8_t *payload, uint8_t plen);

#endif