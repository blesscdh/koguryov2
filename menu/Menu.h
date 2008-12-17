//###########################################################################
//
// FILE:    motor.c
//
// TITLE:   KOGURYO Mouse motor c  file.
//
//###########################################################################
// $Release Date: 2006.10.2 $
//###########################################################################
#ifndef __MENU_H__
#define __MENU_H__

extern void MenuButton(void);
extern void InitpMode(void);
extern void SetpPathFuntion(void);
extern void SensorValueVeiw(void);
extern void PositionVeiw(void);
extern void StraightTest(void);
extern void TurnAdjust(void);
extern void SearchMaze(void);
extern void SearchMazeLoop(void);
extern void FullRun(void);
extern void SecondRun(Uint16 Speed, Uint16 TurnSpeed);
extern void S800Run(void);
extern void S1100Run(void);
extern void S1220Run(void);
extern void S1300Run(void);
extern void InitSearchStartVar(void);
extern void SearchTypeSelect(void);
extern void Buzz(Uint16 Hz,Uint16 Time);
extern void SpeedEdit(void);
extern void TurnDataSet(void);
extern void TurnTableLoad(void);
extern void TurnTableLoadView(void);
extern 	void _2ndRunTableCall(Uint16 TurnSpeed);
extern void CallUserVar(void);
extern void BurnUserVar(void);
extern void Click(void);
extern void TestView(void);
extern void InitGyroRefVolt(void);
#endif
