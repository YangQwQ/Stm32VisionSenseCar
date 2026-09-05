/*
 * host 单测：Dispatch 指令分发（§5.5 dev/cmd -> 各执行器调用映射）
 * 用桩替换驱动模块，验证 Dispatch_Process/Dispatch_Periodic 传参正确。
 */
#include <stdio.h>
#include <string.h>
#include "Dispatch.h"
#include "stubs.h"
#include "Relay.h"      /* RLY_DIST / RLY_YAW 枚举 */
#include "Status.h"     /* g_status / GRIP_CLOSED */

static int total = 0, failed = 0;
#define CHECK(cond, msg) do { total++; if (cond) printf("  ok  - %s\n", msg); \
	else { failed++; printf("FAIL  - %s\n", msg); } } while (0)

static void Build(UartFrame_t *f, uint8_t dev, uint8_t cmd,
                  const uint8_t *p, uint8_t plen)
{
	memset(f, 0, sizeof(*f));
	f->b[0] = dev;
	f->b[1] = cmd;
	if (plen) memcpy(&f->b[2], p, plen);
	f->len = 2 + plen;
	f->valid = 1;
}

/* 校验从 g_log[from] 起的 n 条操作序列 */
static int ExpectSeq(int from, const int *ops, int n)
{
	if (g_log_cnt - from < n) return 0;
	for (int i = 0; i < n; i++)
		if (g_log[from + i].op != ops[i]) return 0;
	return 1;
}

#if 0 /* 便于扩展的 Seq 打印 */
static void DumpLog(void)
{
	for (int i = 0; i < g_log_cnt; i++)
		printf("  [%d] op=%d (%d,%d,%d)\n", i, g_log[i].op, g_log[i].a, g_log[i].b, g_log[i].c);
}
#endif

int main(void)
{
	UartFrame_t f;
	Dispatch_Init();

	printf("== test_dispatch ==\n");

	/* 1) 持续移动 forward */
	LogClear();
	Build(&f, 0x01, 0x01, (const uint8_t[]){0x01, 200}, 2);
	Dispatch_Process(&f);
	{
		int seq[] = { OP_STRAIGHT };
		CHECK(ExpectSeq(0, seq, 1) && g_log[0].a == 0x01 && g_log[0].b == 200,
		      "move 持续移动 -> Straight(dir=1,speed=200)");
	}

	/* 2) 指定距离移动 30cm */
	LogClear();
	Build(&f, 0x01, 0x02, (const uint8_t[]){0x01, 0x1E, 0x00, 200}, 4);
	Dispatch_Process(&f);
	{
		int seq[] = { OP_STRAIGHT, OP_RELAYSTART };
		CHECK(ExpectSeq(0, seq, 2) &&
		      g_log[1].a == RLY_DIST && g_log[1].b == 300,   /* 30cm * 10 = 300(0.1cm) */
		      "move 指定距离 -> Straight + Relay(RLY_DIST,30cm)");
	}

	/* 3) 转向指定角度 30° */
	LogClear();
	Build(&f, 0x01, 0x04, (const uint8_t[]){0x01, 0x1E, 0x00, 150}, 4);
	Dispatch_Process(&f);
	{
		int seq[] = { OP_TURNANGLE, OP_RELAYSTART };
		CHECK(ExpectSeq(0, seq, 2) &&
		      g_log[0].a == 0x01 && g_log[0].b == 30 && g_log[0].c == 150 &&
		      g_log[1].a == RLY_YAW && g_log[1].b == 300,
		      "turn 指定角度 -> TurnAngle(dir=1,30,150) + Relay(RLY_YAW,30°)");
	}

	/* 4) stop all -> 车停 + 臂停 */
	LogClear();
	Build(&f, 0x01, 0x05, (const uint8_t[]){0x00}, 1);
	Dispatch_Process(&f);
	{
		int seq[] = { OP_CARSTOP, OP_RELAYSTOP, OP_ARMSTOP };
		CHECK(ExpectSeq(0, seq, 3), "stop(scope=all) -> 车停 + 状态复位 + 臂停");
	}

	/* 5) car 状态查询 -> 发小车状态帧 */
	LogClear();
	Build(&f, 0x01, 0x06, NULL, 0);
	Dispatch_Process(&f);
	{
		int seq[] = { OP_SENTCAR };
		CHECK(ExpectSeq(0, seq, 1), "car 状态查询 -> Status_SendCar");
	}

	/* 6) arm grip(夹取) -> 设置 grip */
	LogClear();
	Build(&f, 0x02, 0x05, (const uint8_t[]){0x01}, 1);
	Dispatch_Process(&f);
	{
		int seq[] = { OP_STARTGRIP };
		CHECK(ExpectSeq(0, seq, 1) && g_log[0].a == 0x01,
		      "arm grip(1) -> StartGrip(act=1)");
		CHECK(g_status.arm_grip == GRIP_CLOSED, "grip 状态置为夹持");
	}

	/* 7) Periodic：Relay 到位 -> 停车 + 上报完成 */
	LogClear();
	g_relay_active = 1; g_relay_check = 1; g_arm_busy = 0;
	Dispatch_Periodic();
	{
		int seq[] = { OP_CARSTOP, OP_RELAYSTOP, OP_SENTCAR };
		CHECK(ExpectSeq(0, seq, 3), "周期:Relay 到位 -> 自动停 + 上报");
	}

	/* 8) Periodic：Relay 未激活 -> 不误停 */
	LogClear();
	g_relay_active = 0; g_relay_check = 0; g_arm_busy = 1;
	Dispatch_Periodic();
	CHECK(g_log_cnt == 0, "周期:Relay 未激活且臂忙 -> 无动作");

	/* 9) arm 指定距离升降 */
	LogClear();
	Build(&f, 0x02, 0x02, (const uint8_t[]){0x01, 0x0A, 0x00, 100}, 4);
	Dispatch_Process(&f);
	{
		int seq[] = { OP_LIFTSBY };
		CHECK(ExpectSeq(0, seq, 1) && g_log[0].a == 0x01 && g_log[0].b == 10,
		      "arm 升降指定距离 -> LiftBy(dir=1,10cm)");
	}

	printf("---- %d/%d passed ----\n", total - failed, total);
	return failed ? 1 : 0;
}