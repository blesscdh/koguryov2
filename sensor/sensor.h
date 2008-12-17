//###########################################################################
//
// FILE:    sensor.h
//
// TITLE:   KOGURYO Mouse sensor header file.
//
//###########################################################################
// $Release Date: 2006.10.2 $
//###########################################################################
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "Val.h"
#include "Struct.h"

interrupt void SensorTimer0ISR(void);
extern void FrontSensorValueToDisTable(Uint16 i, _iq19 Value);
extern void ResetSensorVariable(volatile SensorVariable *p);
extern void FrontSensorSet(void);
extern void FrontSensorValueCall(void);
extern void SiedSensorSet(void);
extern void SideSensorValueCall(void);
extern void PositionAdjustDiffVal(_iq30 DecelRate, _iq30 AccelRate);
#endif
