//###########################################################################
//
// FILE:    motor.h
//
// TITLE:   KOGURYO Mouse motor header file.
//
//###########################################################################
// $Release Date: 2006.10.2 $
//###########################################################################

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "Val.h"
#include "Struct.h"

interrupt void MotorTimer1ISR(void);

extern void InitMotorVar(volatile MotorVal *pMotor);
extern void MoveStop(_iq19 q19RDis, _iq17 q17RVel, _iq19 q19LDis, _iq17 q17LVel);
extern void MoveToMove(int16 AccVel, int16 DecVel, int16 Dis);
extern void EdgeCheck(volatile EdgeVariable *pEdge, volatile SensorVariable *pSensor);
extern void DiagEdgeCheck(volatile EdgeVariable *pEdge, volatile SensorVariable *pSensor);
extern void PositionAdjPIDCtl(void);
extern void BlockStraight(TurnDebugVar Debug);
extern void BackTurn(TurnDebugVar Debug);
extern void SmoothTurn(TurnDebugVar Debug);
extern void Diag180Turn(TurnDebugVar Debug);
extern void Diag45_135TurnIn(TurnDebugVar Debug);
extern void Diag45_135TurnOut(TurnDebugVar Debug);
extern void CobraTurn45_135Out(TurnDebugVar Debug);
extern void Diag90Turn(TurnDebugVar Debug);
extern void DiagBlockRun(TurnDebugVar Debug);
extern Uint16 WallCheck(volatile SensorVariable *pRSide, volatile SensorVariable *pLSide);


#define MOTOR_ON	{GpioDataRegs.GPASET.bit.GPIO4 = 1;}
#define MOTOR_OFF	{GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;}
#endif
