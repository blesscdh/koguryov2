//###########################################################################
//
// FILE:    Val.h
//
// TITLE:   KOGURYO Mouse Grobal variable header.
//
//###########################################################################
// $Release Date: 2006.10.2 $
//###########################################################################

#include "struct.h"

#ifndef	__VARIABLE_H__
#define __VARIABLE_H__

#ifdef	__VARIABLE__
	#undef	__VARIABLE__
	#define VARIABLE_EXT
#else
	#define	VARIABLE_EXT extern
#endif

#define VARIABLE_OPT volatile

VARIABLE_EXT VARIABLE_OPT Uint16	gMenuCnt
									,gSensorChanel
									
									,gMoveState
									,gStopCount
									,gPosAdjF
									,gAngleAdjF
									,gRPosWallF
									,gLPosWallF
									,gBackTurnFrontAdjState
									,gDiagDirectAdjState
									,gFrontSensorPull
									,gDiagTurnOutAdjF
									,gDiagTurnOutREdgeAdjF
									,gDiagTurnOutLEdgeAdjF

									,gEdgeDiffAdjustFlag

									,gUserSpeed
									,gUserSpeedBackup
									,gUserTurnSpeed
									,gUserTurnSpeedBackup
									,gDiagSpeedLimit
									,gDirectSpeedLimit
									,gBattVolt

									,gUserDiagVelLimitEdit
									,gUserDirectVelLimitEdit
									,gUserDiagAccelEdit
									,gUserDirectAccelEdit;


//===================================================================
//============== algorithm variable====================================
VARIABLE_EXT VARIABLE_OPT Uint16	gMouseDir
									,gMouseYetDir
									,gMousePosition
									,gMouseYetPosition
									,gPathBufferHead
									,gPathWeightState
									,gMazeMap[256]
									,gMazeMapBackUp[256]
									,gMazeMapBackUp0[256]
									,gMazeMapBackUp1[256]
									,gMazeMapBackUp2[256]
									,gNextBlockDir
									,gBlockToBlock
									,gSearchEndState
									,gBlockRunException
									,gAlgoState
									,gSecondRunGoal
									,gFisrtBlockDiagF
									,gJapanAlgo;

VARIABLE_EXT VARIABLE_OPT int32		gUserAccel;
VARIABLE_EXT VARIABLE_OPT int32		gUserDirectAccel;
VARIABLE_EXT VARIABLE_OPT int32		gUserDiagAccel;

VARIABLE_EXT VARIABLE_OPT int32 	gUserTimeCnt;
VARIABLE_EXT VARIABLE_OPT int32 	gRunTime;

VARIABLE_EXT VARIABLE_OPT Uint32 	gDiffAdjCnt;



VARIABLE_EXT volatile int32 test[1500];


extern volatile SensorVariable *pRFS;
extern volatile SensorVariable *pLFS;
extern volatile SensorVariable *pRDS;
extern volatile SensorVariable *pLDS;
extern volatile SensorVariable *pRSS;
extern volatile SensorVariable *pLSS;


extern int16 gMoveTable[4];
extern int16 gHeadTable[4];

								

VARIABLE_EXT VARIABLE_OPT _iq30		q30PosAdjAccelDiff;
VARIABLE_EXT VARIABLE_OPT _iq30		q30PosAdjDecelDiff;

VARIABLE_EXT VARIABLE_OPT _iq26		q26PosAdjAccelRef;
VARIABLE_EXT VARIABLE_OPT _iq26		q26PosAdjDecelRef;

VARIABLE_EXT VARIABLE_OPT _iq18		q18PositionErrSum;
VARIABLE_EXT VARIABLE_OPT _iq18		q18PositionErr[4];
VARIABLE_EXT VARIABLE_OPT _iq18		q18PosProportionTerm;
VARIABLE_EXT VARIABLE_OPT _iq18		q18PosDerivativeTerm;
VARIABLE_EXT VARIABLE_OPT _iq18		q18PosIntegralTerm;
VARIABLE_EXT VARIABLE_OPT _iq18		q18PosPidOutTerm;

VARIABLE_EXT VARIABLE_OPT _iq18		q18AngleErrSum;
VARIABLE_EXT VARIABLE_OPT _iq18		q18AngleErr[4];
VARIABLE_EXT VARIABLE_OPT _iq18		q18AngleProportionTerm;
VARIABLE_EXT VARIABLE_OPT _iq18		q18AngleDerivativeTerm;
VARIABLE_EXT VARIABLE_OPT _iq18		q18AngleIntegralTerm;
VARIABLE_EXT VARIABLE_OPT _iq18		q18AnglePidOutTerm;


VARIABLE_EXT VARIABLE_OPT _iq19		q19RightPos;
VARIABLE_EXT VARIABLE_OPT _iq19		q19LeftPos;
VARIABLE_EXT VARIABLE_OPT _iq19		q19Position;

VARIABLE_EXT VARIABLE_OPT _iq19 	q19RFSSection[26];
VARIABLE_EXT VARIABLE_OPT _iq19 	q19LFSSection[26];

VARIABLE_EXT VARIABLE_OPT _iq23 	q23LFSSectionDiff[25];
VARIABLE_EXT VARIABLE_OPT _iq23 	q23RFSSectionDiff[25];

VARIABLE_EXT VARIABLE_OPT _iq13 	q13GyroAngle;


VARIABLE_EXT void(*pPathFuntion[30])(TurnDebugVar Debug);

#define TR_LED_ON	{GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;}//led01 io31
#define TL_LED_ON	{GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;}//led02 io27
#define BR_LED_ON	{GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;}//led03 io33
#define BL_LED_ON	{GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;}//led00 io10

#define TR_LED_OFF	{GpioDataRegs.GPASET.bit.GPIO31 = 1;}//led01 io31
#define TL_LED_OFF	{GpioDataRegs.GPASET.bit.GPIO27 = 1;}//led02 io27
#define BR_LED_OFF	{GpioDataRegs.GPBSET.bit.GPIO33 = 1;}//led03 io33
#define BL_LED_OFF	{GpioDataRegs.GPASET.bit.GPIO10 = 1;}//led00 io10

#define LED_OFF		{GpioDataRegs.GPADAT.all |= 0x88000400; GpioDataRegs.GPBDAT.all |= 0x02;}

#define SEN_OFF		{GpioDataRegs.GPADAT.all &= 0xfffff41f;}// 5,6,7,8,9,11 / 0100 0001 1111

#define POS_ADJ_ON	{gPosAdjF = ON;}
#define POS_ADJ_OFF {gPosAdjF = OFF;}
#define ANGLE_ADJ_ON {gAngleAdjF = ON;}
#define ANGLE_ADJ_OFF {gAngleAdjF = OFF;}


#define ONEBLOCK	((int16)180)
#define ONEDIAGBLOCK _IQ17(127.279)

#define QUP(x,y)		(((int32)(x)) << y)
#define QDW(x,y)		(((int32)(x)) >> y)

#define RFS		Sen[3]
#define LFS		Sen[0]
#define RSS		Sen[1]
#define LSS		Sen[4]
#define RDS		Sen[5]
#define LDS		Sen[2]

#define RSEDGE	RSideEdge
#define LSEDGE	LSideEdge
#define RDEDGE	RDiagEdge
#define LDEDGE	LDiagEdge

#define SW_T		(GpioDataRegs.GPADAT.bit.GPIO13)
#define SW_R		(GpioDataRegs.GPADAT.bit.GPIO14)
#define SW_L		(GpioDataRegs.GPADAT.bit.GPIO15)

#define SWT_BELL	{Buzz(3000,20);Buzz(2700,20);}
#define SWR_BELL	{Buzz(3400,20);Buzz(3200,20);}
#define SWL_BELL	{Buzz(4000,20);Buzz(3700,20);}


//=========================================================
//=================== algorithm ============================
//벽 방향   
#define DIR_N		0x01//wall
#define DIR_E		0x02//wall
#define DIR_S		0x04//wall
#define DIR_W		0x08//wall

//블락 투 블락.
#define TURN2STRT			0x00
#define TURN2TURN			0x01
#define TURN2BTURN			0x02

#define LASTPATH	(Uint16)(77)


//마우스의 방향..
#define M_N			0
#define M_E			1
#define M_S			2
#define M_W			3

	
#define F	0
#define R	1
#define B   2
#define L	3

#define BLANK	

enum TurnSpeedTable
{
	SMOOTH680 = 0,
	SMOOTH800,		// 1
	SMOOTHFIRSTDIAG,// 2
	SMOOTH1100,		// 3
	SMOOTH1220,		// 4
	SMOOTH1300,		// 5
	SMOOTH1350		// 6
};

#define ALL_SPI_ROM_PAGE_CNT					511//0 - 511//512 page

#define SIDE_SENSOR_ADDRESS						0
#define R_FRONT_SENSOR_SECTION_ADDRESS 			1
#define L_FRONT_SENSOR_SECTION_ADDRESS 			2
#define R_FRONT_SENSOR_SECTION_DIFF_ADDRESS 	3
#define L_FRONT_SENSOR_SECTION_DIFF_ADDRESS 	4
#define MAP_BACKUP_ADDRESS						5
#define MAP_BACKUP0_ADDRESS						6
#define MAP_BACKUP1_ADDRESS						7
#define MAP_BACKUP2_ADDRESS						8
#define USER_VARIABLE_ADDRESS					9

enum TurnStoreAddress
{
	//////////////////////////////////////////
	//=========== 680 ========================
	TURN680_TABLE   = 10,
	TURN800_TABLE   = 30,
	TURNFIRST_TABLE = 50,
	TURN1100_TABLE  = 70,
	TURN1220_TABLE  = 90,
	TURN1300_TABLE  = 110,	
	TURN1350_TABLE  = 130
	
};


#define GARBAGDATA	((Uint16)(0xfff))

enum EnumSearchType
{
	NC,
	GO_GOAL,
	GO_START,
	GO_01GOAL,
	GO_01START
};
VARIABLE_EXT VARIABLE_OPT enum EnumSearchType gSearchType;


enum EnumMouseHead
{
	DirectRunState,
	DiagonalRunState
};
VARIABLE_EXT VARIABLE_OPT enum EnumMouseHead gMouseHead;

enum EnumTurnState
{
	STRAIGHT,	//FF	0
	R90,		//FRF	1
	BACKTURN,	//B		2
	L90,		//FLF	3
	R180,		//FRRF	4	
	L180,		//FLLF	5
	R135IN,		//FRR	6
	L135IN,		//FLL	7
	R45IN,		//FR	8
	L45IN,		//FL	9
	R135OUT,	//RRF	10
	L135OUT,	//LLF	11
	R45OUT,		//RF	12
	L45OUT,		//LF	13
	RD90,		//RR	14
	LD90,		//LL	15
	RCbr45OUT,	//R		16
	LCbr45OUT,	//L		17
	RCbr135OUT,	//RR	18
	LCbr135OUT,	//LL	19
	RDRUN,		//RL	20
	LDRUN,		//LR	21
	NMATCH		//not match	22
};
VARIABLE_EXT VARIABLE_OPT enum EnumTurnState gTurnState;

//#define SMOOTHTEST 1
//#define BACKTURNTEST 1
#define TURNHOLD	1
#endif

