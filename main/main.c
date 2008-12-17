//###########################################################################
//
// FILE:   MAIN.c
//
// TITLE:  KOGURYO Mouse main c file.
//
//###########################################################################
// $Release Date: 2006.10.2 $
//###########################################################################
#define __VARIABLE__
#define __STRUCT__

#include "DSP280x_Device.h"     // DSP280x Headerfile Include File
#include "DSP280x_Examples.h"   // DSP280x Examples Include File
#include <math.h>

int16	gMoveTable[4] = {0x01,0x10,-0x01,-0x10};
int16	gHeadTable[4] = {DIR_N, DIR_E, DIR_S, DIR_W};

volatile SensorVariable *pRFS = &Sen[3];
volatile SensorVariable *pLFS = &Sen[0];
volatile SensorVariable *pRDS = &Sen[5];
volatile SensorVariable *pLDS = &Sen[2];
volatile SensorVariable *pRSS = &Sen[1];
volatile SensorVariable *pLSS = &Sen[4];


void main(void)
{
	// Step 1. Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the DSP280x_SysCtrl.c file.
	InitSysCtrl();

	// Step 2. Initalize GPIO: 
	// This example function is found in the DSP280x_Gpio.c file and
	// illustrates how to set the GPIO to it's default state.
	InitGpio();//Mouse port init


	// Step 3. Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts 
	DINT;

	
	MemCopy(&RamfuncsLoadStart0, &RamfuncsLoadEnd0, &RamfuncsRunStart0);
	MemCopy(&RamfuncsLoadStart1, &RamfuncsLoadEnd1, &RamfuncsRunStart1);

	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.  
	// This function is found in the DSP280x_PieCtrl.c file.
	InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

	// Initialize the PIE vector table with pointers to the shell Interrupt 
	// Service Routines (ISR).	
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in DSP280x_DefaultIsr.c.
	// This function is found in DSP280x_PieVect.c.
	InitPieVectTable();
	
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;//TINT0
	//PieCtrlRegs.PIEIER1.bit.INTx6 = 1;//ADCINT
	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;//SEQ1INT

	IER |= (M_INT1 | M_INT13);//PIE1, TINT1 Enable

	
	// The	RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
	// symbols are created by the linker. Refer to the F2808.cmd file. 

	// Call Flash Initialization to setup flash waitstates
	// This function must reside in RAM
	InitFlash();

	ResetSystem();

	InitSystem();

	//EnableInterrupts();
	// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM

	VFDPrintf("MAZE");
	Click();
	
	
#ifdef SMOOTHTEST
	
	while((SW_R == HIGH) && (SW_L == HIGH))
		BLANK;

	if(SW_L == LOW)
		TurnDataSet();
	else
		Click();
#else
	while(SW_R == HIGH)
		BLANK;
	
	Click();

#endif
	
	TxPrintf("\n===KOGURYO MOUSE by HHY===\n");
	//VFDPrintf("MAZE");
	VFDPrintf("    ");


	while(TRUE)
	{		
		//TxPrintf("%d  %f  %d  %f  %d  %f\n",pLSS->u16Value, _IQ19toF(pLSS->q19LPFOutData), pRSS->u16Value, _IQ19toF(pRSS->q19LPFOutData), GyroVar.u16Value, _IQ20toF(GyroVar.q20LPFOutData));  
		//TxPrintf("%f\n",_IQ19toF(pLSS->q19LPFOutData));
		//TxPrintf("%d  %f\n",GyroVar.u16Value, _IQ20toF(GyroVar.q20LPFOutData));

		//VFD_Printf("%4x",Sen[0].u16Value);
		//TxPrintf("%d  %d\n",RightMotor.i16QepValue,LeftMotor.i16QepValue);
		//TxPrintf("%f  %f  %f  %f\n",_IQ27toF(RightMotor.q27TickDistance),_IQ19toF(RightMotor.q19DistanceSum),
		//								_IQ27toF(LeftMotor.q27TickDistance),_IQ19toF(LeftMotor.q19DistanceSum));

		
		//TxPrintf("%f  %f  %f  %f\n",_IQ18toF(RightMotor.q18CurrentVelocity[0])
		//							 	,_IQ18toF(RightMotor.q18CurrentVelAvr)
		//							 	,_IQ18toF(LeftMotor.q18CurrentVelocity[0])
		//							 	,_IQ18toF(LeftMotor.q18CurrentVelAvr));
		//TxPrintf("%f  %f  %f  %d\n",_IQ18toF(RightMotor.q18CurrentVelAvr),_IQ17toF(RightMotor.q17ProportionalTerm)
		//								,_IQ18toF(RightMotor.q18ErrVelocity[0]),RightMotor.i16QepValue);
		//TxPrintf("%d\n",RightMotor.i16QepValue);


		//TxPrintf("%f  %f  %d  %d\n",_IQ18toF(RightMotor.q18CurrentVelocity[0])
		//							 	,_IQ18toF(RightMotor.q18CurrentVelAvr)
		//								,RightMotor.u16QepSample
		//								,RightMotor.i16QepValue);

		//TxPrintf("%f  %f  %f  %f\n",_IQ18toF(RightMotor.q18CurrentVelAvr),_IQ18toF(LeftMotor.q18CurrentVelAvr),_IQ19toF(RightMotor.q19DistanceSum),_IQ19toF(LeftMotor.q19DistanceSum));
		//TxPrintf("%f  %f  %f  %f\n",_IQ19toF(pRDS->q19Position),_IQ19toF(pRSS->q19Position),_IQ19toF(pLDS->q19Position),_IQ19toF(pLSS->q19Position));
		//TxPrintf("%f\n",_IQ19toF(q19Position));
		//VFDPrintf("%f",_IQ19toF(q19Position));
		//VFDPrintf("%f",_IQ20toF(q20PosPidOutTerm));
		
		//Delay(0x10000);
		MenuButton();
		BattCheck();
		
	}

}

void InitSystem(void)
{
	//Timer
	InitCpuTimers();
	ConfigCpuTimer(&CpuTimer0, 100, 50);// sensor int
	ConfigCpuTimer(&CpuTimer1, 100, 500);// motor int

	InitAdc();
	InitSci();	
	InitSpi();
	InitEQep(&LeftQepRegs);//left motor qep
	InitEQep(&RightQepRegs);//right motor qep
	VfdInit();
	InitpMode();
	SetpPathFuntion();
	InitMotorVar(&RightMotor);
	InitMotorVar(&LeftMotor);

	InitEPWM(&RightPwmRegs);
	InitEPWM(&LeftPwmRegs);

	FrontSensorValueCall();
	SideSensorValueCall();
	TurnTableLoad();
	PositionAdjustDiffVal(_IQ30(0.3),_IQ30(0.3));
	InitAlgorithmVariable();
	CallUserVar();
	//StartCpuTimer0();//주석풀지 말것.모터 인터럽트에서 자동시작.
	StartCpuTimer1();//motor int start

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;
	
}
void ResetVariable(void)
{
	Uint16 i = 0;
	
	gSensorChanel = 0;
	gMoveState = OFF;
	gStopCount = 0;
	gUserSpeed = 0;
	gUserAccel = 0;
	
	//Sensor Variable reset
	for(i = 0; i < 6; i++)
		ResetSensorVariable(&Sen[i]);

	//GyroVar variable reset
	memset((void *)&GyroVar,0,sizeof(GyroVariable));

	ResetPositionVariable();
	ResetAngleVariable();
	ResetEdgeVariable();
	
}
void ResetPositionVariable(void)
{
	q19RightPos = 0;
	q19LeftPos = 0;
	q19Position = 0;
	q18PositionErrSum = 0;
	q18PositionErr[0] = 0;
	q18PositionErr[1] = 0;
	q18PositionErr[2] = 0;
	q18PositionErr[3] = 0;
	q18PosProportionTerm = 0;
	q18PosDerivativeTerm = 0;
	q18PosIntegralTerm = 0;
	q18PosPidOutTerm = 0;
	gRPosWallF = OFF;
	gLPosWallF = OFF;
	
	gBackTurnFrontAdjState = OFF;
	gDiagDirectAdjState = OFF;
	gFrontSensorPull = OFF;
	gDiagTurnOutAdjF = OFF;
	gDiagTurnOutREdgeAdjF = OFF;
	gDiagTurnOutLEdgeAdjF = OFF;
	POS_ADJ_OFF;
}
void ResetAngleVariable(void)
{
	q18AngleErrSum = 0;
	q18AngleErr[0] = 0;
	q18AngleErr[1] = 0;
	q18AngleErr[2] = 0;
	q18AngleErr[3] = 0;
	q18AngleProportionTerm = 0;
	q18AngleDerivativeTerm = 0;
	q18AngleIntegralTerm = 0;
	q18AnglePidOutTerm = 0;

	ANGLE_ADJ_OFF;
}

void ResetEdgeVariable(void)
{
	memset((void *)&RSideEdge, 0, sizeof(EdgeVariable));
	memset((void *)&LSideEdge, 0, sizeof(EdgeVariable));
	memset((void *)&RDiagEdge, 0, sizeof(EdgeVariable));
	memset((void *)&LDiagEdge, 0, sizeof(EdgeVariable));

	gDiffAdjCnt = 0;
	gEdgeDiffAdjustFlag = OFF;
}


void ResetSystem(void)
{
	MOTOR_OFF;
	LED_OFF;
	SEN_OFF;
	POS_ADJ_OFF;
	ANGLE_ADJ_OFF;
	
	ResetVariable();
}
void Delay(Uint32 Cnt)
{
	while(Cnt--)
	{
		asm("	nop");
		asm("	nop");
	}
}
void BattCheck(void)
{
	static Uint16 BattBuf[5] = {1350,1350,1350,1350,1350};
	static Uint16 BattSum = 6700;

	BattSum -= BattBuf[4];

	BattBuf[4] = BattBuf[3];
	BattBuf[3] = BattBuf[2];
	BattBuf[2] = BattBuf[1];
	BattBuf[1] = BattBuf[0];
	BattBuf[0] = gBattVolt;

	BattSum += gBattVolt;

	if(BattSum < 6700)
	{
		VFDPrintf("L Bt");
		while(TRUE);
	}
	else
		;
}

