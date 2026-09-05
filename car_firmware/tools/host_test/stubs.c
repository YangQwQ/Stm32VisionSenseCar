/* 宿主测试桩：按真实头文件签名提供 Dispatch 所依赖模块的桩实现，记录调用 */
#include <string.h>
#include "stubs.h"
#include "AckermannDrive.h"
#include "Relay.h"
#include "RobotArmAct.h"
#include "Status.h"

LogRec g_log[64];
int    g_log_cnt = 0;
int    g_relay_active = 0;
int    g_relay_check = 0;
int    g_arm_busy = 0;

void LogClear(void) { g_log_cnt = 0; }
static void Push(int op, int a, int b, int c)
{
	if (g_log_cnt < 64) {
		g_log[g_log_cnt].op = op;
		g_log[g_log_cnt].a = a;
		g_log[g_log_cnt].b = b;
		g_log[g_log_cnt].c = c;
		g_log_cnt++;
	}
}

/* ---- AckermannDrive ---- */
void AckermannDrive_Init(void) {}
void AckermannDrive_Stop(void)                         { Push(OP_CARSTOP, 0, 0, 0); }
void AckermannDrive_SteerCenter(void)                  {}
void AckermannDrive_Straight(uint8_t dir, uint16_t sp) { Push(OP_STRAIGHT, dir, sp, 0); }
void AckermannDrive_Turn(uint8_t dir, uint16_t sp)     { Push(OP_TURN, dir, sp, 0); }
void AckermannDrive_TurnAngle(uint8_t dir, uint8_t deg, uint16_t sp){ Push(OP_TURNANGLE, dir, deg, sp); }
uint16_t AckermannDrive_SteerTo(uint8_t dir, uint8_t deg){ Push(OP_TURNANGLE, dir, deg, 0); return 150; }
uint16_t AckermannDrive_GetSteerPwm(void)              { return 150; }
float AckermannDrive_SteerRad(void)                    { return 0.0f; }

/* ---- Relay ---- */
void Relay_Start(RelayKind k, int32_t mag)  { Push(OP_RELAYSTART, (int)k, (int)mag, 0); }
void Relay_Stop(void)                       { Push(OP_RELAYSTOP, 0, 0, 0); }
uint8_t Relay_Active(void)                  { return (uint8_t)g_relay_active; }
uint8_t Relay_Check(void)                   { return (uint8_t)g_relay_check; }

/* ---- RobotArmAct ---- */
void RobotArmAct_Init(void) {}
void RobotArmAct_Stop(void)                                  { Push(OP_ARMSTOP, 0, 0, 0); }
uint8_t RobotArmAct_Busy(void)                               { return (uint8_t)g_arm_busy; }
uint8_t RobotArmAct_Kind(void)                               { return 0; }
void RobotArmAct_StartLift(uint8_t d, uint8_t sp)            { Push(OP_STARTLIFT, d, sp, 0); }
void RobotArmAct_LiftBy(uint8_t d, uint16_t dist, uint8_t sp){ Push(OP_LIFTSBY, d, dist, sp); }
void RobotArmAct_StartReach(uint8_t d, uint8_t sp)           { Push(OP_STARTREACH, d, sp, 0); }
void RobotArmAct_ReachBy(uint8_t d, uint16_t dist, uint8_t sp){ Push(OP_REACHBY, d, dist, sp); }
void RobotArmAct_StartGrip(uint8_t act)                      { Push(OP_STARTGRIP, act, 0, 0); }

/* ---- Status ---- */
SysStatus_t g_status;
void Status_Init(void) {}
void Status_SendCar(void) { Push(OP_SENTCAR, 0, 0, 0); }
void Status_SendArm(void) { Push(OP_SENTARM, 0, 0, 0); }