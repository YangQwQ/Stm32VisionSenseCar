#ifndef _STUBS_H_
#define _STUBS_H_

/* 记录的操作码 */
enum {
	OP_NONE = 0,
	OP_STRAIGHT,        /* AckermannDrive_Straight(dir,speed) */
	OP_TURN,            /* AckermannDrive_Turn(dir,speed) */
	OP_TURNANGLE,       /* AckermannDrive_TurnAngle(dir,angle,speed) */
	OP_CARSTOP,         /* AckermannDrive_Stop */
	OP_RELAYSTOP,       /* Relay_Stop */
	OP_RELAYSTART,      /* Relay_Start(kind,mag) */
	OP_ARMSTOP,         /* RobotArmAct_Stop */
	OP_STARTLIFT,       /* RobotArmAct_StartLift(dir,speed) */
	OP_LIFTSBY,         /* RobotArmAct_LiftBy(dir,dist,speed) */
	OP_STARTREACH,      /* RobotArmAct_StartReach(dir,speed) */
	OP_REACHBY,         /* RobotArmAct_ReachBy(dir,dist,speed) */
	OP_STARTGRIP,       /* RobotArmAct_StartGrip(act) */
	OP_SENTCAR,         /* Status_SendCar */
	OP_SENTARM,         /* Status_SendArm */
};

typedef struct {
	int op;
	int a, b, c;        /* 依 op 解释 */
} LogRec;

extern LogRec g_log[64];
extern int    g_log_cnt;

/* 可配置的桩返回值 */
extern int g_relay_active;
extern int g_relay_check;
extern int g_arm_busy;

void LogClear(void);

#endif