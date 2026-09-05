/*
 * host 单测：UartFrame 帧协议（§5.4 FF帧格式 + CRC-16/MODBUS + 接收状态机）
 * 在 PC 上用 MinGW gcc 编译运行，无需硬件。
 * 编译：见 tools/host_test/run_host_tests.ps1
 */
#include <stdio.h>
#include <string.h>
#include "UartFrame.h"

/* UartFrame.c 通过 Usart.h 引用，Send 时才用到；宿主侧只接不传 */
void Usart1_SendBytes(const uint8_t *buf, uint16_t len)
{
	(void)buf; (void)len;
}

/* 用固件同款 CRC 构造一帧（与 make_frame 一致：CRC 输入=LEN..PAYLOAD，低在前） */
static size_t BuildFrame(uint8_t *out, uint8_t dev, uint8_t cmd,
                         const uint8_t *payload, uint8_t plen)
{
	uint8_t body[UART_FRAME_MAX_LEN + 1];
	uint16_t crc;
	size_t n = 0;

	body[0] = (uint8_t)(2 + plen);        /* LEN */
	body[1] = dev;
	body[2] = cmd;
	if (plen) memcpy(&body[3], payload, plen);
	crc = UartCrc16(body, (uint16_t)(3 + plen));

	out[n++] = 0xAA;
	out[n++] = 0x55;
	out[n++] = body[0];
	out[n++] = dev;
	out[n++] = cmd;
	if (plen) { memcpy(&out[n], payload, plen); n += plen; }
	out[n++] = (uint8_t)(crc & 0xFF);
	out[n++] = (uint8_t)((crc >> 8) & 0xFF);
	return n;
}

static int total = 0, failed = 0;

#define CHECK(cond, msg) do { total++; if (cond) { printf("  ok  - %s\n", msg); } \
	else { failed++; printf("FAIL  - %s\n", msg); } } while (0)

static void feed_all(const uint8_t *buf, size_t len)
{
	size_t i;
	for (i = 0; i < len; i++) UartFrame_Feed(buf[i]);
}

static void test_crc_vector(void)
{
	CHECK(UartCrc16((const uint8_t *)"123456789", 9) == 0x4B37,
	      "CRC-16/MODBUS 标准测试向量 0x4B37");
}

static void test_receive(const char *name, uint8_t dev, uint8_t cmd,
                         const uint8_t *payload, uint8_t plen)
{
	uint8_t frame[64], dev2, cmd2;
	size_t len;
	UartFrame_t f;

	UartFrame_Reset();
	len = BuildFrame(frame, dev, cmd, payload, plen);
	feed_all(frame, len);

	if (!UartFrame_Get(&f))
	{
		CHECK(0, "应收到一帧");
		return;
	}
	dev2 = f.b[0];
	cmd2 = f.b[1];
	CHECK(f.valid && f.len == (2 + plen), name);
	CHECK(dev2 == dev && cmd2 == cmd, "dev/cmd 正确");
	if (plen && f.len > 2)
	{
		CHECK(memcmp(&f.b[2], payload, plen) == 0, "payload 一致");
	}
}

static void test_edge(void)
{
	uint8_t frame[64], bad[64];
	size_t len, i;
	UartFrame_t f;

	/* 1) 坏 CRC 应丢弃 */
	UartFrame_Reset();
	len = BuildFrame(frame, 0x01, 0x01, (const uint8_t[]){0x01, 0xC8}, 2);
	memcpy(bad, frame, len);
	bad[len - 1] ^= 0xFF;               /* 改 CRC 高字节 */
	feed_all(bad, len);
	CHECK(!UartFrame_Get(&f), "坏 CRC 帧被丢弃");

	/* 2) 前置垃圾字节应能重新同步 */
	UartFrame_Reset();
	feed_all((const uint8_t[]){0x00, 0x11, 0xAA}, 3);     /* 垃圾 + AA */
	len = BuildFrame(frame, 0x01, 0x06, NULL, 0);         /* 状态查询 */
	feed_all(frame, len);
	CHECK(UartFrame_Get(&f) && f.b[0] == 0x01 && f.b[1] == 0x06,
	      "前置垃圾+断帧后重新同步");

	/* 3) AA..AA55 容错 */
	UartFrame_Reset();
	feed_all((const uint8_t[]){0xAA, 0xAA}, 2);           /* 两个 AA */
	len = BuildFrame(frame, 0x02, 0x05, (const uint8_t[]){0x01}, 1);
	feed_all(&frame[1], len - 1);                         /* 从 55 起喂剩余全部 */
	CHECK(UartFrame_Get(&f) && f.b[0] == 0x02, "AA..AA55 容错");

	/* 4) LEN 越界应回退 IDLE */
	UartFrame_Reset();
	feed_all((const uint8_t[]){0xAA, 0x55, 0xFF, 0x01, 0x02}, 5); /* LEN=255 超限 */
	CHECK(!UartFrame_Get(&f), "LEN 越界被拒");
	for (i = 0; i < 2000; i++) {} /* 抑制未用告警 */

	/* 5) 连续多帧串接（图传/状态流下背靠背） */
	UartFrame_Reset();
	{
		uint8_t f1[64], f2[64], combo[160];
		size_t l1, l2, n = 0, k;
		l1 = BuildFrame(f1, 0x01, 0x02, (const uint8_t[]){0x01, 0x1E, 0x00, 0xC8}, 4);
		l2 = BuildFrame(f2, 0x02, 0x05, (const uint8_t[]){0x01}, 1);
		memcpy(&combo[n], f1, l1); n += l1;
		memcpy(&combo[n], f2, l2); n += l2;
		for (k = 0; k < n; k++) UartFrame_Feed(combo[k]);
		CHECK(UartFrame_Get(&f) && f.b[0] == 0x01, "串接第 1 帧");
		CHECK(UartFrame_Get(&f) && f.b[0] == 0x02, "串接第 2 帧");
	}
}

int main(void)
{
	printf("== test_uartframe ==\n");
	test_crc_vector();
	test_receive("move 持续移动",        0x01, 0x01, (const uint8_t[]){0x01, 0xC8}, 2);
	test_receive("move 指定距离",        0x01, 0x02, (const uint8_t[]){0x01, 0x1E, 0x00, 0xC8}, 4);
	test_receive("turn 指定角度",        0x01, 0x04, (const uint8_t[]){0x01, 0x1E, 0x00, 0x96}, 4);
	test_receive("stop all",            0x01, 0x05, (const uint8_t[]){0x00}, 1);
	test_receive("car 状态查询",        0x01, 0x06, NULL, 0);
	test_receive("arm grip",            0x02, 0x05, (const uint8_t[]){0x01}, 1);
	test_edge();

	printf("---- %d/%d passed ----\n", total - failed, total);
	return failed ? 1 : 0;
}