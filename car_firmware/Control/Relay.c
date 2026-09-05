#include "Relay.h"
#include "Odom.h"

static RelayKind kind;
static int32_t mag;            /* 目标量（0.1cm 或 0.1°）*/
static int32_t start_mm;
static int32_t start_yaw;
static uint8_t active;

void Relay_Stop(void)
{
	active = 0;
	kind = RLY_NONE;
}

void Relay_Start(RelayKind k, int32_t magnitude)
{
	kind = k;
	mag = magnitude > 0 ? magnitude : -magnitude;
	start_mm  = Odom_GetDistMm();
	start_yaw = Odom_GetYawDegX10();
	active = 1;
}

uint8_t Relay_Active(void)
{
	return active;
}

uint8_t Relay_Check(void)
{
	int32_t got;

	if (!active) return 0;

	if (kind == RLY_DIST)
	{
		got = Odom_GetDistMm() - start_mm;
	}
	else
	{
		got = Odom_GetYawDegX10() - start_yaw;
	}
	if (got < 0) got = -got;

	if (got >= mag)
	{
		active = 0;
		return 1;
	}
	return 0;
}