//###########################################################################
//
// FILE:    menu.c
//
// TITLE:   KOGURYO Mouse menu c  file.
//
//###########################################################################
// $Release Date: 2006.10.2 $
//###########################################################################

#include "DSP280x_Device.h"     // DSP280x Headerfile Include File
#include "DSP280x_Examples.h"   // DSP280x Examples Include File

Uint16 	fSwichTop = OFF;
Uint16 	fSwichRight = OFF;
Uint16 	ModeSelect = OFF;
Uint16 	Mode0 = 0;
Uint16 	Mode1 = 0;
Uint16 	Mode0Yet = 0;
Uint16 	Mode1Yet = 0;

const char MenuString[3][7][5] = {"RUN ","SCH ","800 ","1100","1220","1300","JAP ",
				  			      "INIT","Fset","Sset","Tadj","Tset","St T","SpdE",
							      "SHOW","SenP","SenV","T  V","N  C","N  C","N  C"};

void (*Mode[3][6])(void);

void MenuButton(void)
{
	if(SW_T == LOW && !fSwichTop)
	{
		fSwichTop = ON;
		gMenuCnt = 0;
	}
	else if(SW_R == LOW && !fSwichRight)
	{
		fSwichRight = ON;
		gMenuCnt = 0;
	}

	else if(!ModeSelect && fSwichTop && (gMenuCnt > 1000))//top push long time --> select
	{
		Click();
		ModeSelect = ON;		 
	}

	else if((SW_T == HIGH) && (SW_R == HIGH))
	{
		if(!ModeSelect && fSwichTop && gMenuCnt > 40)//top push
		{
			fSwichTop = OFF;
			SWT_BELL;
			Mode0++;
			if(Mode0 >= 3) Mode0 = 0;
			else ;
		}
		else if(!ModeSelect && fSwichRight && gMenuCnt > 40)//right push
		{
			fSwichRight = OFF;
			SWR_BELL;
			Mode1++;
			if(Mode1 >= 7) Mode1 = 1;
			else ;
		}
		else if(ModeSelect)//mode select
		{
			ModeSelect = OFF;
			fSwichTop = fSwichRight = OFF;
			
			if(!Mode1) VFDPrintf("err!");
			else 
			{	if(*Mode[Mode0][Mode1-1] == NULL)
					VFDPrintf("-NC-");
				else 
					Mode[Mode0][Mode1 - 1]();
			}
		}
		else
			fSwichTop = fSwichRight = OFF;
	}
	
	if(Mode0Yet != Mode0)  
	{
		VFDPrintf((char *)MenuString[Mode0][0]);
		VFDPrintf((char *)MenuString[Mode0][Mode1]);
		Mode0Yet = Mode0;
	}
	if(Mode1Yet != Mode1)	
	{
		VFDPrintf((char *)MenuString[Mode0][Mode1]);
 		Mode1Yet = Mode1;
	}
		

}
void InitpMode(void)
{	
	Mode[0][0] = FullRun;
	Mode[0][1] = S800Run;
	Mode[0][2] = S1100Run;
	Mode[0][3] = S1220Run;
	Mode[0][4] = S1300Run;
	Mode[0][5] = SearchMazeLoop;
	
	Mode[1][0] = FrontSensorSet;
	Mode[1][1] = SiedSensorSet;
	Mode[1][2] = TurnAdjust;
	Mode[1][3] = TurnDataSet;
	Mode[1][4] = StraightTest;
	Mode[1][5] = SpeedEdit;
	
	Mode[2][0] = PositionVeiw;
	Mode[2][1] = SensorValueVeiw;
	Mode[2][2] = TurnTableLoadView;
	Mode[2][3] = TestView;
	Mode[2][4] = NULL;
	Mode[2][5] = NULL;
	
}

void SetpPathFuntion(void)
{
	pPathFuntion[0] = BlockStraight;
	pPathFuntion[1] = SmoothTurn;
	pPathFuntion[2] = BackTurn;
	pPathFuntion[3] = SmoothTurn;
	pPathFuntion[4] = Diag180Turn;
	pPathFuntion[5] = Diag180Turn;
	pPathFuntion[6] = Diag45_135TurnIn;
	pPathFuntion[7] = Diag45_135TurnIn;
	pPathFuntion[8] = Diag45_135TurnIn;
	pPathFuntion[9] = Diag45_135TurnIn;
	pPathFuntion[10] = Diag45_135TurnOut;
	pPathFuntion[11] = Diag45_135TurnOut;
	pPathFuntion[12] = Diag45_135TurnOut;
	pPathFuntion[13] = Diag45_135TurnOut;
	pPathFuntion[14] = Diag90Turn;
	pPathFuntion[15] = Diag90Turn;
	pPathFuntion[16] = CobraTurn45_135Out;
	pPathFuntion[17] = CobraTurn45_135Out;
	pPathFuntion[18] = CobraTurn45_135Out;
	pPathFuntion[19] = CobraTurn45_135Out;
	pPathFuntion[20] = DiagBlockRun;
	pPathFuntion[21] = DiagBlockRun;
	pPathFuntion[22] = NULL;
	pPathFuntion[23] = NULL;
	pPathFuntion[24] = NULL;
	pPathFuntion[25] = NULL;
	pPathFuntion[26] = NULL;
	pPathFuntion[27] = NULL;
	pPathFuntion[28] = NULL;
	pPathFuntion[29] = NULL;
	
}
void SensorValueVeiw(void)
{
	while(TRUE)
	{

		VFDPrintf("LFS=");
		while(SW_R == HIGH);
		Click();
		while(SW_R == HIGH)
		{
			VFDPrintf("%f",_IQ19toF(pLFS->q19LPFOutData));
			TxPrintf("LFS :%f\n",_IQ19toF(pLFS->q19LPFOutData));
			Delay(0x100000);
		}
		Click();

		VFDPrintf("LSS=");
		while(SW_R == HIGH);
		Click();
		while(SW_R == HIGH)
		{
			VFDPrintf("%f",_IQ19toF(pLSS->q19LPFOutData));
			TxPrintf("LSS :%f\n",_IQ19toF(pLSS->q19LPFOutData));
			Delay(0x100000);
		}
		Click();

		VFDPrintf("LDS=");
		while(SW_R == HIGH);
		Click();
		while(SW_R == HIGH)
		{
			VFDPrintf("%f",_IQ19toF(pLDS->q19LPFOutData));
			TxPrintf("LDS :%f\n",_IQ19toF(pLDS->q19LPFOutData));
			Delay(0x100000);
		}
		Click();

		VFDPrintf("RDS=");
		while(SW_R == HIGH);
		Click();
		while(SW_R == HIGH)
		{
			VFDPrintf("%f",_IQ19toF(pRDS->q19LPFOutData));
			TxPrintf("RDS :%f\n",_IQ19toF(pRDS->q19LPFOutData));
			Delay(0x100000);
		}
		Click();

		VFDPrintf("RSS=");
		while(SW_R == HIGH);
		Click();
		while(SW_R == HIGH)
		{
			VFDPrintf("%f",_IQ19toF(pRSS->q19LPFOutData));
			TxPrintf("RSS :%f\n",_IQ19toF(pRSS->q19LPFOutData));
			Delay(0x100000);
		}
		Click();

		VFDPrintf("RFS=");
		while(SW_R == HIGH);
		Click();
		while(SW_R == HIGH)
		{
			VFDPrintf("%f",_IQ19toF(pRFS->q19LPFOutData));
			TxPrintf("RFS :%f\n",_IQ19toF(pRFS->q19LPFOutData));
			Delay(0x100000);
		}
		Click();

		VFDPrintf("Gyro");
		while(SW_R == HIGH);
		Click();
		while(SW_R == HIGH)
		{
			VFDPrintf("%f",_IQ20toF(GyroVar.q20LPFOutData));
			TxPrintf("Gyro :%f\n",_IQ20toF(GyroVar.q20LPFOutData));
			Delay(0x100000);
		}
		Click();
	}
	
}
void PositionVeiw(void)
{
	while(TRUE)
	{
		VFDPrintf("LFS=");
		while(SW_R == HIGH);
		Click();
		while(SW_R == HIGH)
		{
			VFDPrintf("%f",_IQ19toF(pLFS->q19Position));
			TxPrintf("LFS :%f\n",_IQ19toF(pLFS->q19Position));
			Delay(0x100000);
		}
		Click();

		VFDPrintf("LSS=");
		while(SW_R == HIGH);
		Click();
		while(SW_R == HIGH)
		{
			VFDPrintf("%f",_IQ19toF(pLSS->q19Position));
			TxPrintf("LSS :%f\n",_IQ19toF(pLSS->q19Position));
			Delay(0x100000);
		}
		Click();

		VFDPrintf("LDS=");
		while(SW_R == HIGH);
		Click();
		while(SW_R == HIGH)
		{
			VFDPrintf("%f",_IQ19toF(pLDS->q19Position));
			TxPrintf("LDS :%f\n",_IQ19toF(pLDS->q19Position));
			Delay(0x100000);
		}
		Click();

		VFDPrintf("RDS=");
		while(SW_R == HIGH);
		Click();
		while(SW_R == HIGH)
		{
			VFDPrintf("%f",_IQ19toF(pRDS->q19Position));
			TxPrintf("RDS :%f\n",_IQ19toF(pRDS->q19Position));
			Delay(0x100000);
		}
		Click();

		VFDPrintf("RSS=");
		while(SW_R == HIGH);
		Click();
		while(SW_R == HIGH)
		{
			VFDPrintf("%f",_IQ19toF(pRSS->q19Position));
			TxPrintf("RSS :%f\n",_IQ19toF(pRSS->q19Position));
			Delay(0x100000);
		}
		Click();

		VFDPrintf("RFS=");
		while(SW_R == HIGH);
		Click();
		while(SW_R == HIGH)
		{
			VFDPrintf("%f",_IQ19toF(pRFS->q19Position));
			TxPrintf("RFS :%f\n",_IQ19toF(pRFS->q19Position));
			Delay(0x100000);
		}
		Click();
	}
}

void StraightTest(void)
{
	Uint16 AccelSpeed;
	TurnDebugVar Debug;

	VFDPrintf("    ");
	InitMotorVar(&RightMotor);
	InitMotorVar(&LeftMotor);
	MOTOR_ON;
	POS_ADJ_ON;
	ANGLE_ADJ_ON;

	RightMotor.i32Accel = LeftMotor.i32Accel = gUserAccel = 8500;
	gUserSpeed = 680;
	AccelSpeed = 680;
	
	while(SW_R == HIGH);
	Click();

	InitGyroRefVolt();
	
	//TxPrintf("... %f...",_IQ12toF(GyroVar.q12AngleRef));
	
	while(TRUE)
	{
		MoveToMove(AccelSpeed, gUserSpeed, ONEBLOCK * 3);

		TxPrintf("\n======Accel=====\n");
		while(RightMotor.q19ErrDistance > _IQ19(5))
		{
			//TxPrintf("%f  %f\n",_IQ18toF(q18AngleErr[0]), _IQ18toF(q18AnglePidOutTerm));
			//TxPrintf("%f\n",_IQ17toF(RightMotor.q17CurrentVelAvr));
			//Delay(0x10000);
		}
		
		BackTurn(Debug);

		/*
		AccelSpeed += 100;
		if(AccelSpeed > 3500)
			AccelSpeed = 3500;

		//TxPrintf("\n======Decel=====\n");
		VFDPrintf("%d",AccelSpeed);
		*/
	}
	
}

void TurnAdjust(void)
{
	int16 TurnState;
	Uint16 SubDis;
	Uint16 TurnType = 0;
	Uint16 YetTurnType = 1;
	Uint16 DebugWallType = 0;
	Uint16 YetDebugWallType = 1;
	
	TurnDebugVar Debug;
	
	InitMotorVar(&RightMotor);
	InitMotorVar(&LeftMotor);

	MOTOR_ON;
	POS_ADJ_OFF;
	
	
	while(SW_R == HIGH)
	{
		VFDPrintf("%4ld",QDW(pLSS->q19Position,19));
		Delay(0x100000);
	}
    Click();	

	while(SW_R == HIGH)
	{
		VFDPrintf("%4ld",QDW(pRSS->q19Position,19));
		Delay(0x100000);
	}
	Click();	

	InitGyroRefVolt();

	gBlockToBlock = OFF;
	gPathBufferHead = 0;
	gBackTurnFrontAdjState = OFF;
	KnowBlockPath[0].Position = 0;
	
	while(TRUE)
	{
		if(SW_R == LOW)
			TurnType++;
		else if(SW_L == LOW)
		{
			if(TurnType == 0)
				TurnState = 'R';
			else
				TurnState = 'L';
			Click();
			break;
		}

		if(TurnType == 2)
			TurnType = 0;

		if(!(TurnType == YetTurnType))
		{
			switch(TurnType)
			{
				case 0:
					VFDPrintf("=R=");
					break;
				case 1:
					VFDPrintf("=L=");
					break;
				default:
					break;
			}
			Click();
		}
		YetTurnType = TurnType;
		Delay(100000);
			
	}

	while(TRUE)
	{
		if(SW_R == LOW)
			DebugWallType++;
		else if(SW_L == LOW)
		{
			if(DebugWallType == 0)
			{
				Debug.Twall0 = ON;
				Debug.Fwall0 = ON;
				Debug.Fwall1 = OFF;
			}
			else
			{
				Debug.Twall0 = ON;
				Debug.Fwall0 = OFF;
				Debug.Fwall1 = OFF;
			}
			Click();
			break;
		}

		if(DebugWallType == 2)
			DebugWallType = 0;

		if(!(DebugWallType == YetDebugWallType))
		{
			switch(DebugWallType)
			{
				case 0:
					VFDPrintf("W on");
					break;
				case 1:
					VFDPrintf("Woff");
					break;
				default:
					break;
			}
			Click();
		}
		YetDebugWallType = DebugWallType;
		Delay(100000);
			
	}


	gUserSpeed = 680;
	gUserTurnSpeed = SMOOTH680;
	_2ndRunTableCall(gUserTurnSpeed);

	RightMotor.i32Accel = LeftMotor.i32Accel = gUserAccel = 10000;
	MoveToMove(gUserSpeed, gUserSpeed, ONEBLOCK * 3);
	
	if(TurnState == 'R')
		KnowBlockPath[0].PathState = R90;
	else if(TurnState == 'L')
		KnowBlockPath[0].PathState = L90;

	if(KnowBlockPath[0].PathState > L45IN)
	{
		POS_ADJ_OFF;
	}
	else
	{
		if(gUserTurnSpeed == SMOOTHFIRSTDIAG)
		{
			POS_ADJ_ON;
		}
		else
		{
			POS_ADJ_ON;
			while(RightMotor.q19DistanceSum < _IQ19(300.0))
				;
			POS_ADJ_OFF;
		}
	}
	
	pPathFuntion[KnowBlockPath[0].PathState](Debug);

	POS_ADJ_OFF;
	
	RightMotor.i32Accel = LeftMotor.i32Accel = 5000;

	SubDis = 0;
	MoveStop(QUP(ONEBLOCK * 2 - SubDis, 19), QUP(gUserSpeed,17),QUP(ONEBLOCK * 2 - SubDis, 19), QUP(gUserSpeed,17)); 
	while(TRUE)
		;

}
void SearchTypeSelect(void)
{	
	Uint16 SearchType = 0;
	Uint16 YetSearchType = 1;
	
	memset((void *)gMazeMap, 0x00, sizeof(gMazeMap));
	memset((void *)gMazeMapBackUp, 0x00, sizeof(gMazeMap));
	memset((void *)gMazeMapBackUp0, 0x00, sizeof(gMazeMap));
	memset((void *)gMazeMapBackUp1, 0x00, sizeof(gMazeMap));
	memset((void *)gMazeMapBackUp2, 0x00, sizeof(gMazeMap));
	
	while (TRUE)
	{
		 if(SW_R == LOW)
			 SearchType++;
		 else if(SW_L == LOW) 
		 {	
		 	if(SearchType == 0)
		 		;
			else if(SearchType >= 1)
			{
				SpiReadRom(MAP_BACKUP_ADDRESS + SearchType - 1, 0, 256, (Uint16 *)gMazeMap);
			}
			
			Click();
			break;
		 } 
		 
		 if(SearchType == 5)
			 SearchType = 0;
		 		 
		if(!(SearchType == YetSearchType))
		{
			 switch(SearchType)
			 {
			 case 0:
				 VFDPrintf("ClR ");
				 break;
			 case 1:
				 VFDPrintf("ReS ");
				 break;
			 case 2:
				 VFDPrintf("ReS0");
				 break;
			 case 3:
				 VFDPrintf("ReS1");
				 break;
			 case 4:
				 VFDPrintf("ReS2");
				 break;
			 default :
				 break;

			 }
			 Click();
		}
		YetSearchType = SearchType;

		Delay(100000);
	}
}

void InitSearchStartVar(void)
{
	gRPosWallF = ON;
	gLPosWallF = ON;
	gBackTurnFrontAdjState = OFF;
	gEdgeDiffAdjustFlag = OFF;
	gDiagDirectAdjState = OFF;
	gFrontSensorPull = OFF;
	gUserSpeed = 680;
	gUserTurnSpeed = SMOOTH680;
	gBlockToBlock = OFF;
	ResetEdgeVariable();
	InitGyroRefVolt();
}


void SearchMaze(void)
{
	Uint16 TurnNumber = 0;
	Uint16 PathFuntionHead = 0;
	TurnDebugVar Debug;
	InitMotorVar(&RightMotor);
	InitMotorVar(&LeftMotor);
	MOTOR_ON;
	
	SearchTypeSelect();
	VFDPrintf("Go!!");

	InitSearchStartVar();
	POS_ADJ_ON;
	ANGLE_ADJ_ON;
	
	gDiagSpeedLimit = 1500;
	gDirectSpeedLimit = 1800;
	gUserDiagAccel = 6000;
	gUserDirectAccel = 6000;
	
	
//첫블록 알고리즘 돌리기...
	
	InitAlgorithm();
	Algorithm(gMazeMap[gMousePosition]);

/*
	for(PathFuntionHead = 0; PathFuntionHead < gPathBufferHead; PathFuntionHead++)
		TxPrintf("%d %d %x\n",KnowBlockPath[PathFuntionHead].PathState, KnowBlockPath[PathFuntionHead].PathCnt,
				 KnowBlockPath[PathFuntionHead].Position);
*/

	while(SW_R == HIGH)
	{
		VFDPrintf("%f",_IQ19toF(pLSS->q19Position));
		Delay(0x100000);
	}		

	Click();

	while(SW_R == HIGH)
	{
		VFDPrintf("%f",_IQ19toF(pRSS->q19Position));
		Delay(0x100000);
	}		

	Click();
	VFDPrintf("GO!!");
	while(SW_R == HIGH);
	Click();
	VFDPrintf("    ");

	//gUserTimerCnt = 0;	
	//gUserTime = 0.0;


	MoveToMove(gUserSpeed, gUserSpeed, ONEBLOCK);
	
	RightMotor.q19DistanceSum = LeftMotor.q19DistanceSum = _IQ19(68.0);

	while((TurnNumber != BACKTURN) || (gSearchEndState != ON) || (gMouseYetPosition != 0))
	{
		PathFuntionHead = gPathBufferHead;
		gPathBufferHead = 0;

		if(PathFuntionHead > 0)
			gAlgoState = OFF;
		else
			gAlgoState = ON;

		while(TRUE)
		{

			
			TurnNumber = KnowBlockPath[gPathBufferHead].PathState;
			
			//VFDPrintf("%d  %x",TurnNumber,KnowBlockPath[gPathBufferHead].Position);
			//아는블록후 모르는 블럭 진입.
			if((gPathBufferHead == (PathFuntionHead -1)) && (gAlgoState == OFF))
			{
				gAlgoState = ON;
				
			}

			//VFDPrintf("%4d",TurnNumber);
			//VFDPrintf("%4d",KnowBlockPath[gPathBufferHead].MouseDir);

			//TxPrintf("%d %2x %d %d\n",KnowBlockPath[gPathBufferHead].PathState,KnowBlockPath[gPathBufferHead].Position,KnowBlockPath[gPathBufferHead].MouseDir,gPathBufferHead);
			
			pPathFuntion[TurnNumber](Debug);
		
			if(gAlgoState == ON)
				break;
			else
				gPathBufferHead++;
			
		}
		
	}
	 
}
void SearchMazeLoop(void)
{
	Uint16 TurnNumber = 0;
	Uint16 PathFuntionHead;
	Uint16 SearchLoopNum;
	TurnDebugVar Debug;

	gJapanAlgo = ON;
	SearchMaze();


	gMouseDir = (gMouseDir + 2) & 0x03;
	gMousePosition = 0x01;
	gJapanAlgo |= 0x10;

	for(SearchLoopNum = 0; SearchLoopNum < 2; SearchLoopNum++)
	{
		InitSearchStartVar();
		POS_ADJ_ON;
		ANGLE_ADJ_ON;
		
		//첫블록 알고리즘 돌리기...

		gSearchType = GO_01GOAL;
		gPathBufferHead = 0;
		gAlgoState = ON;
		gSecondRunGoal= OFF;
		gPathWeightState = OFF;
		gSearchEndState = OFF;
		gBlockRunException = OFF;
		gFisrtBlockDiagF = OFF;
		
		memset((void *)KnowBlockPath, 0x00, sizeof(KnowBlockPath));

		DelSearchQueueList();

		Algorithm(gMazeMap[gMousePosition]);

		MoveToMove(gUserSpeed, gUserSpeed, ONEBLOCK);

		RightMotor.q19DistanceSum = LeftMotor.q19DistanceSum = _IQ19(75.0);

		while((TurnNumber != BACKTURN) || (gSearchEndState != ON) || (gMouseYetPosition != 0x0))
		{
			PathFuntionHead = gPathBufferHead;
			gPathBufferHead = 0;

			if(PathFuntionHead > 0)
				gAlgoState = OFF;
			else
				gAlgoState = ON;

			while(TRUE)
			{
				
				TurnNumber = KnowBlockPath[gPathBufferHead].PathState;
				
				//아는블록후 모르는 블럭 진입.
				if((gPathBufferHead == (PathFuntionHead -1)) && (gAlgoState == OFF))
				{
					gAlgoState = ON;
					
				}
				
				pPathFuntion[TurnNumber](Debug);
			
				if(gAlgoState == ON)
					break;
				else
					gPathBufferHead++;
				
			}
			
		}

		gMouseDir = (gMouseDir + 2) & 0x03;
		gMousePosition = 0x01;
	
	}
}

void SecondRun(Uint16 Speed, Uint16 TurnSpeed)
{

	Uint16	RunCnt = 0;
	Uint16	TurnType = 0;
	TurnDebugVar  Debug;

	Delay(0x300000);
	VFDPrintf("    ");

	InitAlgorithm();
	RunPathMake();
 
	InitMotorVar(&RightMotor);
	InitMotorVar(&LeftMotor);
	MOTOR_ON;

	gAlgoState = OFF;
	InitSearchStartVar();
	
	// 2차 속도 넣을 부분...
	gUserSpeed = Speed;
	gUserTurnSpeed = TurnSpeed;

	_2ndRunTableCall(gUserTurnSpeed);
	
	gDiagSpeedLimit = gUserDiagVelLimitEdit;
	gDirectSpeedLimit = gUserDirectVelLimitEdit;
	gUserDiagAccel = (int32)gUserDiagAccelEdit;
	gUserDirectAccel = (int32)gUserDirectAccelEdit;
	
	POS_ADJ_ON;
	ANGLE_ADJ_ON;
	

	// 2차 주행..
	RunCnt = gPathBufferHead;
	gPathBufferHead = 0;
	
	
	if((KnowBlockPath[0].PathState == STRAIGHT) && (KnowBlockPath[0].PathCnt == 1) && (gUserTurnSpeed >= SMOOTH1100))
	{
		//첫 블럭 대각 경우
		//R45IN - Diag Run, LD90, L45OUT 
		//R135IN - LD90, L45OUT,L135OUT
		//R90 - .
		//가속도 15000 이상으로 첫블럭 1000 턴속도로 진입  가속도 10000이상으로 턴속도 변환..
		
		RightMotor.i32Accel = LeftMotor.i32Accel = 15000;

		gUserSpeedBackup = gUserSpeed;
		gUserSpeed = 1000;
		
		gUserTurnSpeedBackup = gUserTurnSpeed;
		gUserTurnSpeed = SMOOTHFIRSTDIAG;
		
		gFisrtBlockDiagF = ON;
	
	}
	else
		;
		
	gUserTimeCnt = 0;
	//----------------------
	MoveToMove(gUserSpeed, gUserSpeed, ONEBLOCK);
	RightMotor.q19DistanceSum = LeftMotor.q19DistanceSum = _IQ19(68.0);
	while(TRUE)
	{
		TurnType = KnowBlockPath[gPathBufferHead].PathState;

		//골 도착 하면 알고리즘 돌리기
		if(gPathBufferHead == (RunCnt -1))
		{	
			TR_LED_ON;
			TL_LED_ON;
			gAlgoState = ON;
			gSecondRunGoal= ON;
			break;

		}
			
		else
			;
		
		pPathFuntion[TurnType](Debug);

		gPathBufferHead++;

	}

	pPathFuntion[TurnType](Debug);

	
	LED_OFF;

	// 돌아오면서 탐색..아는길 대각 등...
	
	
	gDiagSpeedLimit = 1500;
	gDirectSpeedLimit = 1800;
	gUserDiagAccel = 6000;
	gUserDirectAccel = 6000;
	

	while((TurnType != BACKTURN) || (gSearchEndState != ON) || (gMouseYetPosition != 0))
	{
		RunCnt = gPathBufferHead;
		gPathBufferHead = 0;

		if(RunCnt > 0)
			gAlgoState = OFF;
		else
			gAlgoState = ON;

		while(TRUE)
		{
			TurnType= KnowBlockPath[gPathBufferHead].PathState;
			
			//VFDPrintf("%d  %x",TurnType,KnowBlockPath[gPathBufferHead].Position);
			//아는블록후 모르는 블럭 진입.

			if((gPathBufferHead == (RunCnt -1)) && (gAlgoState == OFF))
			{
				gAlgoState = ON;
				
			}
			
			pPathFuntion[TurnType](Debug);
		
			if(gAlgoState == ON)
				break;
			else
				gPathBufferHead++;
			
		}
		
	}
	
}

void S800Run(void)
{
	while(SW_R == HIGH)
	{
		VFDPrintf("%f",_IQ19toF(pLSS->q19Position));
		Delay(0x100000);
	}		

	Click();

	while(SW_R == HIGH)
	{
		VFDPrintf("%f",_IQ19toF(pRSS->q19Position));
		Delay(0x100000);
	}		

	Click();
	VFDPrintf("GO!!");
	while(SW_R == HIGH);
	Click();
	VFDPrintf("    ");

	while(TRUE)
	{
		SecondRun(800, SMOOTH800);
		Delay(0x1000000);
	}
	
	
}
void S1100Run(void)
{
	while(SW_R == HIGH)
	{
		VFDPrintf("%f",_IQ19toF(pLSS->q19Position));
		Delay(0x100000);
	}		

	Click();

	while(SW_R == HIGH)
	{
		VFDPrintf("%f",_IQ19toF(pRSS->q19Position));
		Delay(0x100000);
	}		

	Click();
	VFDPrintf("GO!!");
	while(SW_R == HIGH);
	Click();
	VFDPrintf("    ");

	while(TRUE)	
	{
		SecondRun(1100, SMOOTH1100);
		Delay(0x1000000);
	}
	
	
}

void S1220Run(void)
{
	while(SW_R == HIGH)
	{
		VFDPrintf("%f",_IQ19toF(pLSS->q19Position));
		Delay(0x100000);
	}		

	Click();

	while(SW_R == HIGH)
	{
		VFDPrintf("%f",_IQ19toF(pRSS->q19Position));
		Delay(0x100000);
	}		

	Click();
	VFDPrintf("GO!!");
	while(SW_R == HIGH);
	Click();
	VFDPrintf("    ");

	while(TRUE)	
	{
		SecondRun(1220, SMOOTH1220);
		Delay(0x1000000);
	}
	
	
}

void S1300Run(void)
{
	while(SW_R == HIGH)
	{
		VFDPrintf("%f",_IQ19toF(pLSS->q19Position));
		Delay(0x100000);
	}		

	Click();

	while(SW_R == HIGH)
	{
		VFDPrintf("%f",_IQ19toF(pRSS->q19Position));
		Delay(0x100000);
	}		

	Click();
	VFDPrintf("GO!!");
	while(SW_R == HIGH);
	Click();
	VFDPrintf("    ");

	while(TRUE)	
	{
		SecondRun(1300, SMOOTH1300);
		Delay(0x1000000);
	}
	
	
}
void FullRun(void)
{
	Uint16	RunSpeed;
	Uint16	SmoothSpeed;
	Uint16 	TurnSpeedType = 0;
	Uint16 	YetTurnSpeedType = 1;

	while (TRUE)
	{
		 if(SW_R == LOW)
			 TurnSpeedType++;
		 else if(SW_L == LOW) 
		 {	
		 	
			switch(TurnSpeedType)
			{
				case 0:
					RunSpeed = 800;
					SmoothSpeed = SMOOTH800;
					break;
				case 1:						
					RunSpeed = 1100;
					SmoothSpeed = SMOOTH1100;
					break;
				case 2:						
					RunSpeed = 1220;
					SmoothSpeed = SMOOTH1220;
					break;
				case 3:						
					RunSpeed = 1300;
					SmoothSpeed = SMOOTH1300;
					break;
				default:
					break;
			}
			
			Click();
			break;
		 } 
		 
		 if(TurnSpeedType == 4)
			 TurnSpeedType = 0;
		 		 
		if(!(TurnSpeedType == YetTurnSpeedType))
		{
			 switch(TurnSpeedType)
			 {
				 case 0:
					 VFDPrintf("S 80");
					 break;
				 case 1:
					 VFDPrintf("S110");
					 break;
				 case 2:
				 	 VFDPrintf("S122");
					 break;
				 case 3:
					 VFDPrintf("S130");
					 break;
					 
				 default :
					 break;

			 }
			 Click();
		}
		YetTurnSpeedType = TurnSpeedType;

		Delay(100000);
	}
	
	SearchMaze();

	SecondRun(RunSpeed, SmoothSpeed);

	while(TRUE)
		SecondRun(1300, SMOOTH1300);
	
}


void SpeedEdit(void)

{
	Uint16		f_sw0 = OFF;
	//Uint16		f_sw1 = OFF;
	Uint16 		f_sw2 = OFF;
	Uint16		Speed;
	Uint16		Speed_yet;
	Uint16 		i;

	CallUserVar();
	
	for(i = 0; i< 4; i++)
	{
		switch(i)
		{
			case 0:
				VFDPrintf("DaV ");
				Speed = gUserDiagVelLimitEdit;
				break;
				
			case 1: 
				VFDPrintf("DiV ");
				Speed = gUserDirectVelLimitEdit;
				break;
				
			case 2:
				VFDPrintf("DaA ");
				Speed = gUserDiagAccelEdit;
				break;
				
			case 3:
				VFDPrintf("DiA ");
				Speed = gUserDirectAccelEdit;
				break;
				
			default:
				break;
		}
		
			
		Speed_yet = 0;
		while(SW_R == HIGH);
		Click();
		
		while(TRUE)
		{
			
			if(SW_R == LOW)//Right Push
			{
				Click();
				break;
			}			
			else if((SW_T == LOW) && !f_sw0)
			{
				f_sw0 = ON;
				gMenuCnt = 0;
				TR_LED_ON;
			}
			else if((SW_L == LOW) && !f_sw2)
			{
				f_sw2 = ON;
				gMenuCnt = 0;
				TL_LED_ON;
			}
			else if((SW_T == LOW) && (gMenuCnt > 1000))
			{	
				TR_LED_ON;
				SWT_BELL;
				Speed += 100;
				LED_OFF;
			}
			else if((SW_L == LOW) && (gMenuCnt > 1000))
			{
				TL_LED_ON;
				SWL_BELL;
				Speed -= 100;
				LED_OFF;
			}
			else if((SW_T == HIGH) && (SW_L == HIGH) && (SW_R == HIGH))
			{
				if(f_sw0 && (gMenuCnt > 40) && (gMenuCnt <= 1000))//top push
				{
					f_sw0 = OFF;
					SWT_BELL;
					Speed += 100;
				}
				else if(f_sw2 && (gMenuCnt > 40) && (gMenuCnt <= 1000))//bot push
				{
					f_sw2 = OFF;
					SWL_BELL;
					Speed -= 100;
				}
				else
				{
					//f_sw0 = f_sw1 = f_sw2 = OFF;
					f_sw0 = f_sw2 = OFF;
				}

				LED_OFF;
			}
			
			if(Speed_yet != Speed)	
				VFDPrintf("%4d",Speed);
	
			Speed_yet = Speed;
		}

		
		switch(i)
		{
			case 0:
				gUserDiagVelLimitEdit = Speed;
				break;
				
			case 1:
				gUserDirectVelLimitEdit = Speed;
				break;
				
			case 2:
				gUserDiagAccelEdit = Speed;
				break;
				
			case 3:
				gUserDirectAccelEdit = Speed;
				break;
				
			default:
				break;
		}
		

		
	}

	BurnUserVar();
	
	TxPrintf("Diag Vel = %d, Direct Vel = %d, Diag Accel = %d, Direct Accel = %d\n" ,gUserDiagVelLimitEdit
																					,gUserDirectVelLimitEdit
														  							,gUserDiagAccelEdit
														  							,gUserDirectAccelEdit);
}

#define	SPACE	0x20
#define TAP		0x09
#define CR		0x0D

const char	sSEN[7][10] = {"&LFS","&RSS","&LDS","&RFS","&LSS","&RDS","NULL"};
const char	sEDGE[5][15] = {"NULL","&RDiagEdge","&LDiagEdge","&RSideEdge","&LSideEdge"};
const char	TurnStr[20][11] = {"STRAIGHT","R90","BACKTURN","L90","R180","L180","R135IN","L135IN","R45IN","L45IN","R135OUT",	
						"L135OUT","R45OUT","L45OUT","RD90","LD90","RCbr45OUT","LCbr45OUT","RCbr135OUT","LCbr135OUT"};

void TurnDataSet(void)
{
//	Uint16	i;
	Uint16  x;
//	Uint16  *pTurnVar;
	Uint16  *pTemp;

	Uint16  fStart = FALSE;
	Uint16  BufIndex = 0;
	Uint16  Buf[15];
	Uint16	InputAsciiVal[300];
	Uint16	Cnt = 0;
	Uint16	UnitStr[20][20];
	Uint16	UnitNum;
	Uint16	StrCnt;
	Uint16	TurnNum;
	Uint16	TurnSpeed;

	volatile TurnInfoVariable TurnProfile;

	
	memset(InputAsciiVal, 0, sizeof(InputAsciiVal));
	memset(UnitStr, 0, sizeof(UnitStr));

	TxPrintf("\n ========= Turn Variable Input ==========\n");

	while(TRUE)
	{
		InputAsciiVal[Cnt++] = SCIx_RxChar();
		if(InputAsciiVal[Cnt-1] == CR)
			break;
		if(Cnt >= 299)
			break;
	}

	Cnt-=2;
	TxPrintf("===Input END===\n");

	TxPrintf("%s\n",InputAsciiVal);

	if((InputAsciiVal[0] != '{') || (InputAsciiVal[Cnt] != '}'))
	{
		TxPrintf("\n ============= Input Error =====================\n");
		return;
	}

	Cnt = 1;
	for(UnitNum = 0;UnitNum < 20;UnitNum++)
	{
		for(StrCnt = 0;StrCnt < 20;)
		{
			if((InputAsciiVal[Cnt] != SPACE) && (InputAsciiVal[Cnt] != TAP))
			{
				UnitStr[UnitNum][StrCnt++] = InputAsciiVal[Cnt];
			}
			Cnt++;
			
			if((InputAsciiVal[Cnt] == ',') || (InputAsciiVal[Cnt] == '}'))
			{
				Cnt++;
				//TxPrintf("%s\n",UnitStr[UnitNum]);
				break;
			}
		}

		if(InputAsciiVal[Cnt] == 0)
			break;
	}
	//UnitStr[0]====================
	if(strcmp((char *)UnitStr[0],"680") == 0)
	{
		TurnSpeed = SMOOTH680;
		TxPrintf("SMOOTH680\n");
	}
	else if(strcmp((char *)UnitStr[0],"800") == 0)
	{
		TurnSpeed = SMOOTH800;
		TxPrintf("SMOOTH800\n");
	}
	else if(strcmp((char *)UnitStr[0], "FIRST") == 0)
	{
		TurnSpeed = SMOOTHFIRSTDIAG;
		TxPrintf("SMOOTH_FIRST_DIAG\n");
	}
	else if(strcmp((char *)UnitStr[0], "1100") == 0)
	{
		TurnSpeed = SMOOTH1100;
		TxPrintf("SMOOTH1100\n");
	}
	else if(strcmp((char *)UnitStr[0], "1220") == 0)
	{
		TurnSpeed = SMOOTH1220;
		TxPrintf("SMOOTH1220\n");
	}
	else if(strcmp((char *)UnitStr[0], "1300") == 0)
	{
		TurnSpeed = SMOOTH1300;
		TxPrintf("SMOOTH1300\n");
	}
	else if(strcmp((char *)UnitStr[0], "1350") == 0)
	{
		TurnSpeed = SMOOTH1350;
		TxPrintf("SMOOTH1350\n");
	}
	
	else
	{
		TxPrintf("Turn Speed Error!!\n");
		return;
	}

	//UnitStr[1]====================
	TurnNum = 0xFF;
	for(Cnt = 0; Cnt < 20; Cnt++)
	{
		if(strcmp((char *)UnitStr[1],TurnStr[Cnt]) == 0)
		{
			TxPrintf("%s\n",TurnStr[Cnt]);
			TurnNum = Cnt;
			break;
		}
	}
	if(TurnNum == 0xFF) 
	{
		TxPrintf("Turn Kind Error!!\n");
		return;
	}


	//UnitStr[2]====================
	for(Cnt = 0; Cnt < 5; Cnt++)
	{
		if(strcmp((char *)UnitStr[2],sEDGE[Cnt]) == 0)
		{
			//TxPrintf("%s\n",sEDGE[Cnt]);

			switch(Cnt)
			{
				case 0 : TurnProfile.pTurnInEdge = NULL;
						break;
				case 1 : TurnProfile.pTurnInEdge = &RDiagEdge;
						break;
				case 2 : TurnProfile.pTurnInEdge = &LDiagEdge;
						break;
				case 3 : TurnProfile.pTurnInEdge = &RSideEdge;
						break;
				case 4 : TurnProfile.pTurnInEdge = &LSideEdge;
						break;
				default :
						break;
			}
		}
	}

	//UnitStr[3]====================		
	//Turn In Sensor
	for(Cnt = 0; Cnt < 7; Cnt++)
	{
		if(strcmp((char *)UnitStr[3],sSEN[Cnt]) == 0)
		{
			//TxPrintf("%s\n",sSEN[Cnt]);
			if(Cnt == 6)
				TurnProfile.pTurnInSensor = NULL;
			else
				TurnProfile.pTurnInSensor = &Sen[Cnt];
			break;
		}
	}

	
	TurnProfile.u16TurnInTime = atoi((char*)UnitStr[4]);
	TurnProfile.u16TurnInErr = atoi((char*)UnitStr[5]);
	TurnProfile.u16AccelTime = atoi((char*)UnitStr[6]);
	TurnProfile.u16TurnTime = atoi((char*)UnitStr[7]);
	TurnProfile.u16TurnOutTime = atoi((char*)UnitStr[8]);

	TurnProfile.i32RightAccel = atol((char*)UnitStr[9]);
	TurnProfile.i32LeftAccel = atol((char*)UnitStr[10]);

	//UnitStr[11]====================		
	fStart = FALSE;
	BufIndex = 0;

	for(Cnt = 0; Cnt < 20; Cnt++)
	{
		if(fStart)
		{
			if(UnitStr[11][Cnt] == ')' || Cnt >= 19)
				break;

			if(UnitStr[11][Cnt] == SPACE || UnitStr[11][Cnt] == TAP)
				;
			else
				Buf[BufIndex++] = UnitStr[11][Cnt];
			
		}
		if(UnitStr[11][Cnt] == '(')
			fStart = TRUE;

	}
	Buf[BufIndex] = 0;
	//TxPrintf("%s\n", Buf);
	TurnProfile.q17RightAccelVel = _IQ17(atof((char*)Buf));

	
	//UnitStr[12]====================		
	fStart = FALSE;
	BufIndex = 0;

	for(Cnt = 0; Cnt < 20; Cnt++)
	{
		if(fStart)
		{
			if(UnitStr[12][Cnt] == ')' || Cnt >= 19)
				break;

			if(UnitStr[12][Cnt] == SPACE || UnitStr[12][Cnt] == TAP)
				;
			else
				Buf[BufIndex++] = UnitStr[12][Cnt];
			
		}
		if(UnitStr[12][Cnt] == '(')
			fStart = TRUE;

	}
	Buf[BufIndex] = 0;
	//TxPrintf("%s\n", Buf);
	TurnProfile.q17LeftAccelVel = _IQ17(atof((char*)Buf));
	
//UnitStr[13]====================		
	fStart = FALSE;
	BufIndex = 0;

	for(Cnt = 0; Cnt < 20; Cnt++)
	{
		if(fStart)
		{
			if(UnitStr[13][Cnt] == ')' || Cnt >= 19)
				break;

			if(UnitStr[13][Cnt] == SPACE || UnitStr[13][Cnt] == TAP)
				;
			else
				Buf[BufIndex++] = UnitStr[13][Cnt];
			
		}
		if(UnitStr[13][Cnt] == '(')
			fStart = TRUE;

	}
	Buf[BufIndex] = 0;
	//TxPrintf("%s\n", Buf);
	TurnProfile.q17RefVel = _IQ17(atof((char*)Buf));	

	TurnProfile.u16EdgeTick0 = atoi((char*)UnitStr[14]);
	TurnProfile.u16EdgeTick1 = atoi((char*)UnitStr[15]);
	TurnProfile.u16EdgeTick2 = atoi((char*)UnitStr[16]);

	//UnitStr[17]====================		
	for(Cnt = 0; Cnt < 7; Cnt++)
	{
		if(strcmp((char *)UnitStr[17],sSEN[Cnt]) == 0)
		{
			//TxPrintf("%s\n",sSEN[Cnt]);
			if(Cnt == 6)
				TurnProfile.pTurnEdgeSen = NULL;
			else
				TurnProfile.pTurnEdgeSen = &Sen[Cnt];
			break;
		}
	}
	
	//UnitStr[18]====================		
	for(Cnt = 0; Cnt < 7; Cnt++)
	{
		if(strcmp((char *)UnitStr[18],sSEN[Cnt]) == 0)
		{
			//TxPrintf("%s\n",sSEN[Cnt]);
			if(Cnt == 6)
				TurnProfile.pTurnFEdgeSen = NULL;
			else
				TurnProfile.pTurnFEdgeSen = &Sen[Cnt];
			break;
		}
	}

	//UnitStr[19]====================		
	TurnProfile.u32GyroAngle = atoi((char*)UnitStr[19]);


/////////////////////////////////////////////////////////////////////////
/////////// store////////////////////////////////////////////////////////
	pTemp = (Uint16*)far_malloc(sizeof(TurnInfoVariable) * 2);

	for(x = 0; x < sizeof(TurnInfoVariable); x++)
		pTemp[x] = (((Uint16*)&TurnProfile)[x] >> 8) & 0xff;

	for(x = 0; x < sizeof(TurnInfoVariable); x++)
		pTemp[x + sizeof(TurnInfoVariable)] = ((Uint16*)&TurnProfile)[x] & 0xff;
		
	SpiWriteRom(TurnSpeed * 20 + 10 + TurnNum, 0, (Uint16)(sizeof(TurnInfoVariable)*2), pTemp);


///////////////////////////////////////////////////////////////////////////////
////////////// read test ////////////////////////////////////////////////////////	
	
	SpiReadRom(TurnSpeed * 20 + 10 + TurnNum, 0, sizeof(TurnInfoVariable)*2, (Uint16*)pTemp);
	
	for(x = 0; x < sizeof(TurnInfoVariable); x++)
		((Uint16*)&TurnProfile)[x] = (pTemp[x] << 8) & 0xff00;

	for(x = 0; x < sizeof(TurnInfoVariable); x++)
		((Uint16*)&TurnProfile)[x] |= pTemp[x + sizeof(TurnInfoVariable)] & 0xff;


	TxPrintf("%lx,", TurnProfile.pTurnInEdge);
	TxPrintf("%lx,", TurnProfile.pTurnInSensor);

	TxPrintf("%d,", TurnProfile.u16TurnInTime);
	TxPrintf("%d,", TurnProfile.u16TurnInErr);
	TxPrintf("%d,", TurnProfile.u16AccelTime);
	TxPrintf("%d,", TurnProfile.u16TurnTime);
	TxPrintf("%d,", TurnProfile.u16TurnOutTime);

	TxPrintf("%ld,", TurnProfile.i32RightAccel);
	TxPrintf("%ld,", TurnProfile.i32LeftAccel);

	
	TxPrintf("%f,", _IQ17toF(TurnProfile.q17RightAccelVel));
	TxPrintf("%f,", _IQ17toF(TurnProfile.q17LeftAccelVel));
	TxPrintf("%f,", _IQ17toF(TurnProfile.q17RefVel));

	TxPrintf("%d,", TurnProfile.u16EdgeTick0);
	TxPrintf("%d,", TurnProfile.u16EdgeTick1);
	TxPrintf("%d,", TurnProfile.u16EdgeTick2);

	TxPrintf("%lx,", TurnProfile.pTurnEdgeSen);
	TxPrintf("%lx,", TurnProfile.pTurnFEdgeSen);
	TxPrintf("%ld\n", TurnProfile.u32GyroAngle);


	far_free(pTemp);
	
}


void _2ndRunTableCall(Uint16 TurnSpeed)
{
	Uint16  i;
	Uint16  x;
	Uint16  *pTemp;

	volatile TurnInfoVariable TurnVar;

	pTemp = (Uint16*)far_malloc(sizeof(TurnInfoVariable) * 2);

	for(i = STRAIGHT; i <= LCbr135OUT; i++)
	{
		SpiReadRom(TurnSpeed * 20 + 10 + i, 0, sizeof(TurnInfoVariable)*2, (Uint16*)pTemp);

		for(x = 0; x < sizeof(TurnInfoVariable); x++)
		  ((Uint16*)&TurnVar)[x] = (pTemp[x] << 8) & 0xff00;

		for(x = 0; x < sizeof(TurnInfoVariable); x++)
		  ((Uint16*)&TurnVar)[x] |= pTemp[x + sizeof(TurnInfoVariable)] & 0xff;

		  memcpy((void *)&TurnTable[3][i], (void *)&TurnVar, sizeof(TurnInfoVariable));
	}

	far_free(pTemp);

}

void TurnTableLoadView(void)
{
	Uint16	i;
	Uint16	x;
	Uint16	y;
	Uint16	*pTemp;

	volatile TurnInfoVariable TurnVar;


	pTemp = (Uint16*)far_malloc(sizeof(TurnInfoVariable) * 2);

	for(y = 0; y < 6 ; y++)
	{
		TxPrintf("\n\n");
		
		for(i = STRAIGHT; i <= LCbr135OUT; i++)
		{
			SpiReadRom(y * 20 + 10 + i, 0, sizeof(TurnInfoVariable)*2, (Uint16*)pTemp);

			for(x = 0; x < sizeof(TurnInfoVariable); x++)
				((Uint16*)&TurnVar)[x] = (pTemp[x] << 8) & 0xff00;
			
			for(x = 0; x < sizeof(TurnInfoVariable); x++)
				((Uint16*)&TurnVar)[x] |= pTemp[x + sizeof(TurnInfoVariable)] & 0xff;

			
			if(y == 0)
				TxPrintf("{680,");
			else if(y == 1)
				TxPrintf("{800,");			
			else if(y == 2)
				TxPrintf("{FIRST,");
			else if(y == 3)
				TxPrintf("{1100,");			
			else if(y == 4)
				TxPrintf("{1220,");
			else if(y == 5)
				TxPrintf("{1300,");
			

			
			TxPrintf("%s,", TurnStr[i]);

			if(TurnVar.pTurnInEdge == &RSideEdge)
				TxPrintf("&RSideEdge,");
			else if(TurnVar.pTurnInEdge == &LSideEdge)
				TxPrintf("&LSideEdge,");
			else if(TurnVar.pTurnInEdge == &RDiagEdge)
				TxPrintf("&RDiagEdge,");
			else if(TurnVar.pTurnInEdge == &LDiagEdge)
				TxPrintf("&LDiagEdge,");
			else if(TurnVar.pTurnInEdge == NULL)
				TxPrintf("NULL,");			

			
			if(TurnVar.pTurnInSensor == &RFS)
				TxPrintf("&RFS,");
			else if(TurnVar.pTurnInSensor == &RSS)
				TxPrintf("&RSS,");
			else if(TurnVar.pTurnInSensor == &RDS)
				TxPrintf("&RDS,");
			else if(TurnVar.pTurnInSensor == &LDS)
				TxPrintf("&LDS,");
			else if(TurnVar.pTurnInSensor == &LSS)
				TxPrintf("&LSS,");
			else if(TurnVar.pTurnInSensor == &LFS)
				TxPrintf("&LFS,");
			else if(TurnVar.pTurnInSensor == NULL)
				TxPrintf("NULL,");
			

			TxPrintf("%d,", TurnVar.u16TurnInTime);
			TxPrintf("%d,", TurnVar.u16TurnInErr);
			TxPrintf("%d,", TurnVar.u16AccelTime);
			TxPrintf("%d,", TurnVar.u16TurnTime);
			TxPrintf("%d,", TurnVar.u16TurnOutTime);

			TxPrintf("%ld,", TurnVar.i32RightAccel);
			TxPrintf("%ld,", TurnVar.i32LeftAccel);

			TxPrintf("_IQ17(%.1f),", _IQ17toF(TurnVar.q17RightAccelVel));
			TxPrintf("_IQ17(%.1f),", _IQ17toF(TurnVar.q17LeftAccelVel));
			TxPrintf("_IQ17(%.1f),", _IQ17toF(TurnVar.q17RefVel));
			
			TxPrintf("%d,", TurnVar.u16EdgeTick0);
			TxPrintf("%d,", TurnVar.u16EdgeTick1);
			TxPrintf("%d,", TurnVar.u16EdgeTick2);

			if(TurnVar.pTurnEdgeSen == &RFS)
				TxPrintf("&RFS,");
			else if(TurnVar.pTurnEdgeSen == &RSS)
				TxPrintf("&RSS,");
			else if(TurnVar.pTurnEdgeSen == &RDS)
				TxPrintf("&RDS,");
			else if(TurnVar.pTurnEdgeSen == &LDS)
				TxPrintf("&LDS,");
			else if(TurnVar.pTurnEdgeSen == &LSS)
				TxPrintf("&LSS,");
			else if(TurnVar.pTurnEdgeSen == &LFS)
				TxPrintf("&LFS,");
			else if(TurnVar.pTurnEdgeSen == NULL)
				TxPrintf("NULL,");
			
			if(TurnVar.pTurnFEdgeSen == &RFS)
				TxPrintf("&RFS,");
			else if(TurnVar.pTurnFEdgeSen == &RSS)
				TxPrintf("&RSS,");
			else if(TurnVar.pTurnFEdgeSen == &RDS)
				TxPrintf("&RDS,");
			else if(TurnVar.pTurnFEdgeSen == &LDS)
				TxPrintf("&LDS,");
			else if(TurnVar.pTurnFEdgeSen == &LSS)
				TxPrintf("&LSS,");
			else if(TurnVar.pTurnFEdgeSen == &LFS)
				TxPrintf("&LFS,");
			else if(TurnVar.pTurnFEdgeSen == NULL)
				TxPrintf("NULL,");
		
			TxPrintf("%ld}\n", TurnVar.u32GyroAngle);
			
		}

	}
	far_free(pTemp);
}


void TurnTableLoad(void)
{
	Uint16	i;
	Uint16	x;
	Uint16	y;
  //	Uint16	*pTurnVar;
	Uint16	*pTemp;

	volatile TurnInfoVariable TurnVar;


	pTemp = (Uint16*)far_malloc(sizeof(TurnInfoVariable) * 2);

	for(y = 0; y < 3 ; y++)
	{
		for(i = STRAIGHT; i <= LCbr135OUT; i++)
		{
			SpiReadRom(y * 20 + 10 + i, 0, sizeof(TurnInfoVariable)*2, (Uint16*)pTemp);

			for(x = 0; x < sizeof(TurnInfoVariable); x++)
				((Uint16*)&TurnVar)[x] = (pTemp[x] << 8) & 0xff00;
			
			for(x = 0; x < sizeof(TurnInfoVariable); x++)
				((Uint16*)&TurnVar)[x] |= pTemp[x + sizeof(TurnInfoVariable)] & 0xff;

			if(y < 3)
				memcpy((void *)&TurnTable[y][i], (void *)&TurnVar, sizeof(TurnInfoVariable));
			else
				;
			
			VFDPrintf("T_%d",y);
		}

	}
	far_free(pTemp);
}



void TurnDataSet123(void)
{
	Uint16	i;
	Uint16  x;
	Uint16  y;
	Uint16  *pTurnVar;
	Uint16  *pTemp;

	volatile TurnInfoVariable TurnVar;
	
	
	pTemp = (Uint16*)far_malloc(sizeof(TurnInfoVariable) * 2);

	for(y = 0; y < 6; y++)
	{
		
		for(i = STRAIGHT; i <= LCbr135OUT; i++)
		{
			pTurnVar = (Uint16*)&TurnTable[y][i];
			for(x = 0; x < sizeof(TurnInfoVariable); x++)
				pTemp[x] = (pTurnVar[x] >> 8) & 0xff;

			for(x = 0; x < sizeof(TurnInfoVariable); x++)
				pTemp[x + sizeof(TurnInfoVariable)] = pTurnVar[x] & 0xff;
			
			SpiWriteRom(y * 20 + 10 + i, 0, (Uint16)(sizeof(TurnInfoVariable)*2), pTemp);
			
		}

		for(i = 0; i < 20; i++)
		{
			TxPrintf("%lx,", TurnTable[y][i].pTurnInEdge);
			TxPrintf("%lx,", TurnTable[y][i].pTurnInSensor);

			TxPrintf("%d,", TurnTable[y][i].u16TurnInTime);
			TxPrintf("%d,", TurnTable[y][i].u16TurnInErr);
			TxPrintf("%d,", TurnTable[y][i].u16AccelTime);
			TxPrintf("%d,", TurnTable[y][i].u16TurnTime);
			TxPrintf("%d,", TurnTable[y][i].u16TurnOutTime);

			TxPrintf("%ld,", TurnTable[y][i].i32RightAccel);
			TxPrintf("%ld,", TurnTable[y][i].i32LeftAccel);

			TxPrintf("%f,", _IQ17toF(TurnTable[y][i].q17RightAccelVel));
			TxPrintf("%f,", _IQ17toF(TurnTable[y][i].q17LeftAccelVel));
			TxPrintf("%f,", _IQ17toF(TurnTable[y][i].q17RefVel));
			
			TxPrintf("%d,", TurnTable[y][i].u16EdgeTick0);
			TxPrintf("%d,", TurnTable[y][i].u16EdgeTick1);
			TxPrintf("%d,", TurnTable[y][i].u16EdgeTick2);

			TxPrintf("%lx,", TurnTable[y][i].pTurnEdgeSen);
			TxPrintf("%lx,", TurnTable[y][i].pTurnFEdgeSen);
			TxPrintf("%ld\n", TurnTable[y][i].u32GyroAngle);
				
		}
		TxPrintf("\n\n");
		

		for(i = STRAIGHT; i <= LCbr135OUT; i++)
		{
			SpiReadRom(y * 20 + 10 + i, 0, sizeof(TurnInfoVariable)*2, (Uint16*)pTemp);

			for(x = 0; x < sizeof(TurnInfoVariable); x++)
				((Uint16*)&TurnVar)[x] = (pTemp[x] << 8) & 0xff00;
			
			for(x = 0; x < sizeof(TurnInfoVariable); x++)
				((Uint16*)&TurnVar)[x] |= pTemp[x + sizeof(TurnInfoVariable)] & 0xff;


			TxPrintf("%lx,", TurnVar.pTurnInEdge);
			TxPrintf("%lx,", TurnVar.pTurnInSensor);

			TxPrintf("%d,", TurnVar.u16TurnInTime);
			TxPrintf("%d,", TurnVar.u16TurnInErr);
			TxPrintf("%d,", TurnVar.u16AccelTime);
			TxPrintf("%d,", TurnVar.u16TurnTime);
			TxPrintf("%d,", TurnVar.u16TurnOutTime);

			TxPrintf("%ld,", TurnVar.i32RightAccel);
			TxPrintf("%ld,", TurnVar.i32LeftAccel);

			TxPrintf("%f,", _IQ17toF(TurnVar.q17RightAccelVel));
			TxPrintf("%f,", _IQ17toF(TurnVar.q17LeftAccelVel));
			TxPrintf("%f,", _IQ17toF(TurnVar.q17RefVel));
			
			TxPrintf("%d,", TurnVar.u16EdgeTick0);
			TxPrintf("%d,", TurnVar.u16EdgeTick1);
			TxPrintf("%d,", TurnVar.u16EdgeTick2);

			TxPrintf("%lx,", TurnVar.pTurnEdgeSen);
			TxPrintf("%lx,", TurnVar.pTurnFEdgeSen);
			TxPrintf("%ld\n", TurnVar.u32GyroAngle);
			
		}

		TxPrintf("\n\n");
	}
	far_free(pTemp);

}
	



void CallUserVar(void)
{
	Uint16 Buf[8];
	
	SpiReadRom(USER_VARIABLE_ADDRESS, 0, 8,Buf);

	gUserDiagVelLimitEdit = 	((Buf[0] & 0xff) << 8) | (Buf[1] & 0xff);
	gUserDirectVelLimitEdit =	((Buf[2] & 0xff) << 8) | (Buf[3] & 0xff);
	gUserDiagAccelEdit =		((Buf[4] & 0xff) << 8) | (Buf[5] & 0xff);
	gUserDirectAccelEdit =		((Buf[6] & 0xff) << 8) | (Buf[7] & 0xff);
		
}
void BurnUserVar(void)
{
	Uint16 Buf[8];
	
	Buf[0] = (gUserDiagVelLimitEdit >> 8) & 0xff;
	Buf[1] = gUserDiagVelLimitEdit & 0xff;

	Buf[2] = (gUserDirectVelLimitEdit >> 8) & 0xff;
	Buf[3] = gUserDirectVelLimitEdit & 0xff;

	Buf[4] = (gUserDiagAccelEdit >> 8) & 0xff;
	Buf[5] = gUserDiagAccelEdit & 0xff;

	Buf[6] = (gUserDirectAccelEdit >> 8) & 0xff;
	Buf[7] = gUserDirectAccelEdit & 0xff;
	
	SpiWriteRom(USER_VARIABLE_ADDRESS, 0, 8, Buf);
	
}
	
void Buzz(Uint16 Hz,Uint16 Time)
{
	Uint16 i;
	for(i = 0; i < Time; i++)
	{
		Delay(Hz);
		GpioDataRegs.GPASET.bit.GPIO12 = 1;
		Delay(Hz);
		GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;
	}
}


void Click(void)
{
	SWR_BELL;
	SWL_BELL;
	SWT_BELL;

	Delay(0x100000);
}

void TestView(void)
{
	Uint16 i;
	
	for(i = 0; i < 1500; i++)
		TxPrintf("%f\n",_IQ19toF(test[i]));
}


void InitGyroRefVolt(void)
{
	int16 i;
	int32 Buff[50];
	
	for(LeftMotor.u16Tick = 0; LeftMotor.u16Tick < 50;)
		Buff[LeftMotor.u16Tick] = QDW(GyroVar.q20LPFOutData, 8);// iq12

	GyroVar.q12AngleRef = 0;
	for(i = 0; i < 50; i++)
		GyroVar.q12AngleRef += Buff[i]; 
		
	GyroVar.q12AngleRef = _IQ12div(GyroVar.q12AngleRef, QUP(50,12));

	
	TxPrintf("\nGYRO Ref Volt : %f\n",_IQ12toF(GyroVar.q12AngleRef));
}

