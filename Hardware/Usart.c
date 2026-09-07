#include "stm32f10x.h"
#include "Usart.h"

/*****************************************************************************
 * USART1：大脑板（ESP32-S3）<-> 执行板（本 STM32）
 *  - 波特率 115200，8N1，全双工
 *  - RXNE 中断接收，字节写入环形缓冲，供 UartFrame 拉取
 *****************************************************************************/

#define UART1_RX_MAX        UART1_RX_BUF_SIZE

static volatile uint8_t  rx_buf[UART1_RX_BUF_SIZE];
static volatile uint8_t  rx_head = 0;      /* 写指针（中断内） */
static volatile uint8_t  rx_tail = 0;      /* 读指针（主循环） */
static volatile uint8_t  rx_count = 0;     /* 可用字节数 */

void Usart1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

	/* TX = PA9 */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* RX = PA10 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate            = 115200;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	/* RXNE 接收中断 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART1, ENABLE);

	rx_head = 0; rx_tail = 0; rx_count = 0;
}

void Usart1_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Usart1_SendBytes(const uint8_t *buf, uint16_t len)
{
	uint16_t i;
	for (i = 0; i < len; i++)
	{
		Usart1_SendByte(buf[i]);
	}
}

uint8_t Usart1_RxAvailable(void)
{
	/* 中断内只写 rx_head/rx_count，主程序读取，使用关中断保证原子 */
	uint8_t n;
	__disable_irq();
	n = rx_count;
	__enable_irq();
	return n;
}

uint8_t Usart1_RxRead(void)
{
	uint8_t b;
	__disable_irq();
	b = rx_buf[rx_tail];
	rx_tail = (rx_tail + 1) % UART1_RX_BUF_SIZE;
	if (rx_count > 0) rx_count--;
	__enable_irq();
	return b;
}

void Usart1_RxFlush(void)
{
	__disable_irq();
	rx_head = 0; rx_tail = 0; rx_count = 0;
	__enable_irq();
}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		uint8_t b = (uint8_t)(USART1->DR & 0xFF);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

		if (rx_count < UART1_RX_BUF_SIZE)
		{
			rx_buf[rx_head] = b;
			rx_head = (rx_head + 1) % UART1_RX_BUF_SIZE;
			rx_count++;
		}
		/* 缓冲满则丢弃新字节，避免覆盖未读数据 */
	}
}