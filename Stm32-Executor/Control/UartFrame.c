#include "UartFrame.h"
#include "Usart.h"

/* 接收状态机状态 */
typedef enum
{
	RX_IDLE,
	RX_HEAD2,
	RX_LEN,
	RX_BODY,
	RX_CRC_L,
	RX_CRC_H
} RxState;

static UartFrame_t rx_cur;       /* 正在组帧 */
static uint8_t     rx_state;
static uint8_t     rx_len;       /* 期望 LEN */
static uint8_t     rx_idx;       /* BODY 已收字节数 */
static uint8_t     rx_crc_l;
static uint8_t     rx_crc_h;
static uint16_t    err_cnt;

/* 就绪帧队列：主循环轮询时可能一次收多帧，用环形队列避免覆盖 */
#define FRAME_QUEUE  8
static UartFrame_t q_buf[FRAME_QUEUE];
static uint8_t q_head = 0, q_tail = 0, q_cnt = 0;

/* CRC-16/MODBUS：反射多项式 0xA001，初值 0xFFFF */
static uint16_t Crc16Byte(uint16_t crc, uint8_t byte)
{
	uint8_t i;
	crc ^= byte;
	for (i = 0; i < 8; i++)
	{
		crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
	}
	return crc;
}

uint16_t UartCrc16(const uint8_t *data, uint16_t len)
{
	uint16_t crc = 0xFFFF;
	uint16_t i;
	for (i = 0; i < len; i++)
	{
		crc = Crc16Byte(crc, data[i]);
	}
	return crc;
}

void UartFrame_Reset(void)
{
	rx_state = RX_IDLE;
	rx_len = 0; rx_idx = 0; rx_crc_l = 0; rx_crc_h = 0;
	err_cnt = 0;
	q_head = 0; q_tail = 0; q_cnt = 0;
}

void UartFrame_Feed(uint8_t byte)
{
	switch (rx_state)
	{
	case RX_IDLE:
		if (byte == UART_FRAME_HEAD1) rx_state = RX_HEAD2;
		break;

	case RX_HEAD2:
		if (byte == UART_FRAME_HEAD2) rx_state = RX_LEN;
		else if (byte == UART_FRAME_HEAD1) rx_state = RX_HEAD2;  /* AA..AA55 容错 */
		else rx_state = RX_IDLE;
		break;

	case RX_LEN:
		rx_len = byte;
		if (rx_len < 2 || rx_len > UART_FRAME_MAX_LEN) { rx_state = RX_IDLE; break; }
		rx_idx = 0;
		rx_state = RX_BODY;
		break;

	case RX_BODY:
		if (rx_idx < UART_FRAME_MAX_LEN) rx_cur.b[rx_idx] = byte;
		rx_idx++;
		if (rx_idx >= rx_len) rx_state = RX_CRC_L;
		break;

	case RX_CRC_L:
		rx_crc_l = byte;
		rx_state = RX_CRC_H;
		break;

	case RX_CRC_H:
	{
		uint16_t calc, exp;
		uint8_t crc_data[UART_FRAME_MAX_LEN + 1];
		uint8_t i;

		rx_crc_h = byte;
		rx_cur.len = rx_len;

		/* CRC 输入 = LEN 字节 + (DEV+CMD+PAYLOAD)，即“帧头之后到 PAYLOAD 末尾” */
		crc_data[0] = rx_len;
		for (i = 0; i < rx_len; i++) crc_data[1 + i] = rx_cur.b[i];
		calc = UartCrc16(crc_data, (uint16_t)(1 + rx_len));
		exp  = ((uint16_t)rx_crc_h << 8) | (uint16_t)rx_crc_l;

		if (calc == exp)
		{
			q_buf[q_head] = rx_cur;
			q_buf[q_head].valid = 1;
			q_head = (q_head + 1) % FRAME_QUEUE;
			if (q_cnt < FRAME_QUEUE)
			{
				q_cnt++;
			}
			else /* 队列满：覆盖最旧，保留最新 */
			{
				q_tail = (q_tail + 1) % FRAME_QUEUE;
			}
		}
		else
		{
			err_cnt++;
		}
		rx_state = RX_IDLE;
		break;
	}
	default:
		rx_state = RX_IDLE;
		break;
	}
}

uint8_t UartFrame_Get(UartFrame_t *frame)
{
	if (q_cnt > 0)
	{
		*frame = q_buf[q_tail];
		q_tail = (q_tail + 1) % FRAME_QUEUE;
		q_cnt--;
		return 1;
	}
	return 0;
}

uint16_t UartFrame_ErrCount(void)
{
	return err_cnt;
}

void UartFrame_Send(uint8_t dev, uint8_t cmd, const uint8_t *payload, uint8_t plen)
{
	uint8_t l = 2 + plen;          /* DEV+CMD+PAYLOAD */
	uint8_t crc_in[2 + UART_FRAME_MAX_LEN];
	uint16_t crc;
	uint8_t frame_buf[4 + UART_FRAME_MAX_LEN + 2];
	uint8_t n = 0, i;

	crc_in[0] = l;
	crc_in[1] = dev;
	crc_in[2] = cmd;
	for (i = 0; i < plen; i++) crc_in[3 + i] = payload[i];
	crc = UartCrc16(crc_in, (uint16_t)(3 + plen));

	frame_buf[n++] = UART_FRAME_HEAD1;
	frame_buf[n++] = UART_FRAME_HEAD2;
	frame_buf[n++] = l;
	frame_buf[n++] = dev;
	frame_buf[n++] = cmd;
	for (i = 0; i < plen; i++) frame_buf[n++] = payload[i];
	frame_buf[n++] = (uint8_t)(crc & 0xFF);
	frame_buf[n++] = (uint8_t)((crc >> 8) & 0xFF);

	Usart1_SendBytes(frame_buf, n);
}
