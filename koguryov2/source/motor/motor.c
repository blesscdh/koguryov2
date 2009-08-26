//###########################################################################
//
// FILE:    motor.c
//
// TITLE:   KOGURYO Mouse motor c  file.
//
//###########################################################################
// $Release Date: 2006.10.2 $
//###########################################################################

#include "DSP280x_Device.h"     // DSP280x Headerfile Include File
#include "DSP280x_Examples.h"   // DSP280x Examples Include File

// ZRO, PRD, CAU, CAD, CBU, CBD bits
#define	AQ_NO_ACTION	0x0
#define	AQ_CLEAR		0x1
#define	AQ_SET			0x2
#define	AQ_TOGGLE		0x3

#define TimeTick		_IQ30(0.0005)
#define PULSE_TO_DIS	_IQ30(0.007737579562423)
#define PULSE_TO_VEL	_IQ26(15.475159124846300)
#define MOUSETREAD		73.0// real 69.0

#define PWM_CONVERT		_IQ30(0.22222222)

#define MAX_I_TERM		_IQ17(5.0)
#define MIN_I_TERM		_IQ17(-5.0)

#define MAX_PID_OUT		_IQ17(8950)
#define MIN_PID_OUT		_IQ17(-8950)

//global IQ	19

#define M_POS_KP	_IQ26(1.0)// 1.0
#define M_POS_KD	_IQ26(0.0)// 15.0
#define M_POS_KI	_IQ26(0.00005)

#define M_ANGLE_KP	_IQ26(0.5)
#define M_ANGLE_KD	_IQ26(2.0)
#define M_ANGLE_KI	_IQ26(0.00001)

#define ANGLE_ADJ_RATIO	_IQ18(3.0)

#define M_CAL_POS_KP_MIN	_IQ26(0.32)
#define M_CAL_POS_KP_DIFF	_IQ26(0.0002)

#define M_CAL_POS_KD_MAX	_IQ26(15.0)
#define M_CAL_POS_KD_DIFF	_IQ26(0.0045)

//Motor Max Velocity	4100mm/s

Uint16 WallTable[4][3] = //[MouseDir][Wall] wall --> Front, Right, Left,Back
{
	{DIR_N,DIR_E,DIR_W},
	{DIR_E,DIR_S,DIR_N},
	{DIR_S,DIR_W,DIR_E},
	{DIR_W,DIR_N,DIR_S}
};

void EdgeCheck(volatile EdgeVariable *pEdge, volatile SensorVariable *pSensor)
{
	_iq19 RaiseDiff;
	_iq19 FallDiff;
	Uint16 EdgeTick;
	Uint16 WallTick;
	
	if(RightMotor.q17NextVelocity > _IQ17(2500.0))
	{
		EdgeTick = 90;
		WallTick = 10;
		RaiseDiff = _IQ19(0.25);
		FallDiff = _IQ19(-0.15);
	}
	else if(RightMotor.q17NextVelocity > _IQ17(1500.0))
	{
		EdgeTick = 160;
		WallTick = 20;
		RaiseDiff = _IQ19(0.27);
		FallDiff = _IQ19(-0.18);
	}

	else if(RightMotor.q17NextVelocity > _IQ17(1000.0))
	{
		EdgeTick = 180;
		WallTick = 20;
		RaiseDiff = _IQ19(0.35);
		FallDiff = _IQ19(-0.25);
	}
	else if(RightMotor.q17NextVelocity > _IQ17(700.0))
	{
		EdgeTick = 200;
		WallTick = 25;
		RaiseDiff = _IQ19(0.35);
		FallDiff = _IQ19(-0.25);
	}
	else
	{
		EdgeTick = 230;
		WallTick = 30;
		RaiseDiff = _IQ19(0.35);
		FallDiff = _IQ19(-0.25);
	}

	
	if(pEdge->u16EdgeState != ON)
	{
		if((pSensor->q19Position > _IQ19(100.0)) && (pSensor->q19LPFOutDataDiff > RaiseDiff) && (pEdge->u16EdgeState != FALL))
		{
			pEdge->u16EdgeState = FALL;
			pEdge->u16LimitCnt = 0;
		}
		else if(pEdge->u16EdgeState == FALL)
		{
			if((pEdge->u16LimitCnt++) < EdgeTick)
			{
				if((pSensor->q19Position > _IQ19(100.0)) && (pSensor->q19LPFOutDataDiff < FallDiff))
					pEdge->u16EdgeState = UP;
				else
					;
			}
			else
			{
				pEdge->u16EdgeState = OFF;
			}
		}
		else if((pEdge->u16EdgeState == UP) && (pSensor->q19Position > _IQ19(510.0)))
		{
			pEdge->u16EdgeState = ON;
			pEdge->u32EdgeCheckTick = gDiffAdjCnt;;
			pEdge->u16DelCnt = 0;
			pEdge->u16EdgeOnTick = 0;
		}
	}
	else
		;

	if(pEdge->u16EdgeState == ON)
	{
		pEdge->u16EdgeOn = ON;
		
		if(pEdge->u16DelCnt++ > 10)// 10
			pEdge->u16EdgeState = OFF;
		else
			;
	}
	
	if(pEdge->u16EdgeOn == ON)
	{
		if(pEdge->u16EdgeOnTick++ > 250)
		{
			pEdge->u16EdgeOnTick = 0;
			pEdge->u16EdgeOn = OFF;
		}
		else
			;

		
	}
	else
		pEdge->u16EdgeOnTick = 0;

	//Wall...............

	if((pEdge->u16WallState == ON) && (pSensor->q19Position >= _IQ19(512.0)))
		pEdge->u16WallFallTick++;

	if(pSensor->q19Position < _IQ19(510))
	{		
		if(pEdge->u16WallCheckTick > WallTick)
			pEdge->u16WallState = ON;
		
		pEdge->u16WallCheckTick++;
		pEdge->u16WallFallTick = 0;
	}
	else if((pEdge->u16WallState == ON) && (pEdge->u16WallFallTick > 200))
	{
		pEdge->u16WallState = OFF;
		pEdge->u16WallCheckTick = 0;
	}
	else
		pEdge->u16WallCheckTick = 0;

}

void DiagEdgeCheck(volatile EdgeVariable *pEdge, volatile SensorVariable *pSensor)
{
	Uint16	EdgeTick;
	Uint16	WallTick;
	_iq19	SensorFallDiff;

	if(RightMotor.q17NextVelocity > _IQ17(2500.0))
	{
		SensorFallDiff = _IQ19(0.05);
		EdgeTick = 90;
		WallTick = 10;
	}
	else if(RightMotor.q17NextVelocity > _IQ17(1500.0))
	{
		SensorFallDiff = _IQ19(0.05);
		EdgeTick = 130;
		WallTick = 10;
	}
	else if(RightMotor.q17NextVelocity > _IQ17(1000.0))
	{
		SensorFallDiff = _IQ19(0.05);
		EdgeTick = 150;
		WallTick = 15;
	}
	else if(RightMotor.q17NextVelocity > _IQ17(700.0))
	{
		SensorFallDiff = _IQ19(0.10);
		EdgeTick = 200;
		WallTick = 23;
	}
	else
	{
		SensorFallDiff = _IQ19(0.11);
		EdgeTick = 250;
		WallTick = 28;
	}

	
	if(pEdge->u16EdgeState != ON)
	{
		if((pSensor->q19Position > _IQ19(50.0)) && (pSensor->q19LPFOutDataDiff > SensorFallDiff) && (pEdge->u16EdgeState != FALL))
		{
			pEdge->u16EdgeState = FALL;
			pEdge->u16LimitCnt = 0;
		}
		else if(pEdge->u16EdgeState == FALL)
		{
			if((pEdge->u16LimitCnt++) < EdgeTick)
			{
				if((pSensor->q19Position > _IQ19(50.0)) && (pSensor->q19LPFOutDataDiff < _IQ19(0.0)))
					pEdge->u16EdgeState = UP;
				else
					;
			}
			else
			{
				pEdge->u16EdgeState = OFF;
			}
		}
		else if((pEdge->u16EdgeState == UP) && (pSensor->q19Position > _IQ19(510.0)))
		{
			pEdge->u16EdgeState = ON;
			pEdge->u16DelCnt = 0;
			pEdge->u16EdgeOnTick = 0;
		}
	}
	else
		;

	if(pEdge->u16EdgeState == ON)
	{		
		pEdge->u16EdgeOn= ON;
		
		if(pEdge->u16DelCnt++ > 10)
			pEdge->u16EdgeState = OFF;
		else
			;
	}
	
	if(pEdge->u16EdgeOn == ON)
	{
		if(pEdge->u16EdgeOnTick++ > 250)
		{
			pEdge->u16EdgeOnTick = 0;
			pEdge->u16EdgeOn = OFF;
		}
		else
			;
	}
	else
		pEdge->u16EdgeOnTick = 0;

	
	//Wall...............

	if((pEdge->u16WallState == ON) && (pSensor->q19Position >= _IQ19(512.0)))
		pEdge->u16WallFallTick++;
	
	if(pSensor->q19Position < _IQ19(510))
	{		
		if(pEdge->u16WallCheckTick > WallTick)
			pEdge->u16WallState = ON;
		
		pEdge->u16WallCheckTick++;
		pEdge->u16WallFallTick = 0;
	}
	else if((pEdge->u16WallState == ON) && (pEdge->u16WallFallTick > 200))
	{
		pEdge->u16WallState = OFF;
		pEdge->u16WallCheckTick = 0;
	}
	else
		pEdge->u16WallCheckTick = 0;
	
}

void PositionAdjPIDCtl(void)
{
	_iq19	DiagAdjRate;
	_iq26 	CalPosKp;
	_iq26   CalPosKd;

	static Uint16 DiffAdjFOn = OFF;

	//static Uint16 i = 0;

	/*
	if((i < 1500) && (RightMotor.q17CurrentVelAvr > _IQ17(650.0)))
	{
		test[i] = pLSS->q19PositionDiff;
		i++;	
	}*/

	// Edge
	
	EdgeCheck(&RSideEdge,pRSS);
	EdgeCheck(&LSideEdge,pLSS);
	DiagEdgeCheck(&RDiagEdge,pRDS);
	DiagEdgeCheck(&LDiagEdge,pLDS);
	

	
	//////////////////////////////////////////////////////////////////////////
//-------------- 보정 벽 체크--------------------------------------------------
	
	if((_IQ19abs(pRSS->q19PositionDiff) < _IQ19(0.32)) && (pRSS->q19Position < _IQ19(370.0)) && (pRDS->q19LPFOutData > pRDS->q19MinVal))
		gRPosWallF = ON;

	//if(_IQ19abs(pRSS->q19PositionDiff) > _IQ19(0.65))//0.65
	//	gRPosWallF = OFF;

	if(pRSS->q19Position > _IQ19(500.0))
		gRPosWallF = OFF;

	if(pRDS->q19Position > _IQ19(500.0))
		gRPosWallF = OFF;

	
	if((_IQ19abs(pLSS->q19PositionDiff) < _IQ19(0.32)) && (pLSS->q19Position < _IQ19(370.0)) && (pLDS->q19LPFOutData > pLDS->q19MinVal))
		gLPosWallF = ON;

	//if(_IQ19abs(pLSS->q19PositionDiff) > _IQ19(0.65))//0.65
	//	gLPosWallF = OFF;

	if(pLSS->q19Position > _IQ19(500.0))
		gLPosWallF = OFF;

	if(pLDS->q19Position > _IQ19(500.0))
		gLPosWallF = OFF;
	
	
	
	//0 보다 크면 왼쪽 벽에 가깝고 0 보다 작으면 오른쪽 벽에 가깝다. 
	//음수면 오른쪽 바퀴 빠르게

	q19LeftPos = _IQ19(256.0) - pLSS->q19Position;
	q19RightPos = pRSS->q19Position - _IQ19(256.0);

	if(gBackTurnFrontAdjState == ON)//백턴시 전방보정..
	{
		q19Position = _IQ19mpy((pLFS->q19Position - pRFS->q19Position),_IQ19(2));
	}
	else if(gEdgeDiffAdjustFlag && RSideEdge.u16EdgeOn && LSideEdge.u16EdgeOn)//가지벽 보정   
	{
		q19Position = _IQ19mpy(QUP(((int32)LSideEdge.u32EdgeCheckTick - (int32)RSideEdge.u32EdgeCheckTick), 19), _IQ19(2.5));

		gDiffAdjCnt = 0;

		gEdgeDiffAdjustFlag = OFF;

		DiffAdjFOn = ON;

		if(_IQ19abs(q19Position) > _IQ19(40.0))
		{
			if(q19Position > _IQ19(0.0))
				q19Position = _IQ19(40.0);
			else
				q19Position = _IQ19(-40.0);
		}

	}
	else if(gDiagDirectAdjState == ON)//대각 보정..
	{
		if((pLFS->q19Position < _IQ19(250.0)) && (pLFS->q19LPFOutDataDiff > _IQ19(0.0)))
		{
			q19LeftPos = _IQ19(250.0) - pLFS->q19Position;

			if(pLFS->q19Position < _IQ19(50.0))
				q19LeftPos = _IQ19mpy(q19LeftPos, _IQ19(1.35));

			else if(pLFS->q19Position < _IQ19(100.0))
				q19LeftPos = _IQ19mpy(q19LeftPos, _IQ19(1.2));

			else if(pLFS->q19Position < _IQ19(200.0))
				q19LeftPos = _IQ19mpy(q19LeftPos, _IQ19(1.0));
			else
				q19LeftPos = _IQ19mpy(q19LeftPos, _IQ19(1.3));			
		}
		else
			q19LeftPos = _IQ19(250.0);

		if((pRFS->q19Position < _IQ19(250.0)) && (pRFS->q19LPFOutDataDiff > _IQ19(0.0)))
		{
			q19RightPos =pRFS->q19Position - _IQ19(250.0);

			if(pRFS->q19Position < _IQ19(50.0))
				q19RightPos = _IQ19mpy(q19RightPos, _IQ19(1.35));

			else if(pRFS->q19Position < _IQ19(100.0))
				q19RightPos = _IQ19mpy(q19RightPos, _IQ19(1.2));

			else if(pRFS->q19Position < _IQ19(200.0))
				q19RightPos = _IQ19mpy(q19RightPos, _IQ19(1.0));
			else
				q19RightPos = _IQ19mpy(q19RightPos, _IQ19(1.3));			
		}
		else
			q19RightPos = _IQ19(250.0);


		//대각 직진 가속 보정비와 대각탈출턴 진입 대각 보정의 비율은 다름
		//대각 탈출턴 진입 보정할때는 각 방향의 전방센서만으로 보정한다
		if((q19LeftPos != _IQ19(250.0)) && (q19RightPos != _IQ19(250.0)) && !gDiagTurnOutREdgeAdjF && !gDiagTurnOutLEdgeAdjF)
		{	
			if(gDiagTurnOutAdjF == ON)
				DiagAdjRate = _IQ19(0.11);
			else
				DiagAdjRate = _IQ19(0.35);
			
			q19Position = _IQ19mpy(q19LeftPos + q19RightPos, DiagAdjRate);// 0.35
		}
		else if((q19LeftPos != _IQ19(250.0)) && ((!gDiagTurnOutREdgeAdjF && !gDiagTurnOutLEdgeAdjF) || (!gDiagTurnOutREdgeAdjF && gDiagTurnOutLEdgeAdjF)))
		{
			if(gDiagTurnOutAdjF == ON)
				DiagAdjRate = _IQ19(0.22);
			else
				DiagAdjRate = _IQ19(0.5);
			
			q19Position = _IQ19mpy(q19LeftPos, DiagAdjRate);
		}
		else if(q19RightPos != _IQ19(250.0) && ((!gDiagTurnOutREdgeAdjF && !gDiagTurnOutLEdgeAdjF) || (gDiagTurnOutREdgeAdjF && !gDiagTurnOutLEdgeAdjF)))
		{
			if(gDiagTurnOutAdjF == ON)
				DiagAdjRate = _IQ19(0.22);
			else
				DiagAdjRate = _IQ19(0.5);
			
			q19Position = _IQ19mpy(q19RightPos, DiagAdjRate);
		}
		else
		{
			q19Position = _IQ19(0.0);
		}

	}

	else if(gRPosWallF && gLPosWallF)//양쪽 벽이 있을 경우....
	{	
		q19Position = (q19RightPos + q19LeftPos) >> 1;
		DiffAdjFOn = OFF;
	}
	else if(gRPosWallF && !gLPosWallF)// 오른쪽 벽 보정...왼쪽 벽이 없을 경우..
	{
		q19Position = q19RightPos; 
		DiffAdjFOn = OFF;
	}
	else if(!gRPosWallF && gLPosWallF)//왼쪽 벽 보정...오른쪽 벽이 없을 경우..
	{
		q19Position = q19LeftPos; 
		DiffAdjFOn = OFF;
	}
	else// 앞벽 밀기 및 가지벽 보정후 당기기.  
	{

		if(DiffAdjFOn == ON)
		{
			//가지벽 보정후 당기기...
			if(q19Position > _IQ19(0.0))
			{
				q19Position -= _IQ19(0.18);
			}
			else if(q19Position < _IQ19(0.0))
			{
				q19Position += _IQ19(0.18);
			}
			
			if(_IQ19abs(q19Position) < _IQ19(0.2))
			{
				q19Position = _IQ19(0.0);
			
				DiffAdjFOn = OFF;
			}

		}
		
		// 앞벽 밀기... 및 당기기...
		else if(gFrontSensorPull == ON)
		{
			if((pLFS->q19Position < _IQ19(250.0)) && (pRFS->q19Position < _IQ19(250.0)))
			{
				if(gUserTurnSpeed == SMOOTH680)
					q19Position = _IQ19mpy((pRFS->q19Position - pLFS->q19Position), _IQ19(2.5));// 4.0
				else
					q19Position = _IQ19mpy((pRFS->q19Position - pLFS->q19Position), _IQ19(4.0));// 4.0
				
			}

			else if(pLFS->q19Position < _IQ19(250.0))
			{
				if(gUserTurnSpeed == SMOOTH680)
					q19Position = _IQ19mpy((_IQ19(250.0) - pLFS->q19Position), _IQ19(2.5));	
				else
					q19Position = _IQ19mpy((_IQ19(250.0) - pLFS->q19Position), _IQ19(4.0));
				
				
			}
			
			else if(pRFS->q19Position < _IQ19(250.0))
			{
				if(gUserTurnSpeed == SMOOTH680)
					q19Position = _IQ19mpy((pRFS->q19Position - _IQ19(250.0)), _IQ19(2.5));	
				else
					q19Position = _IQ19mpy((pRFS->q19Position - _IQ19(250.0)), _IQ19(4.0));
				
			}
			
			else
			{
				;
			}
		}

		if(DiffAdjFOn == OFF)
		{
			if(q19Position > _IQ19(0.0))
			{
				q19Position -= _IQ19(0.35);//0.27
			}
			else if(q19Position < _IQ19(0.0))
			{
				q19Position += _IQ19(0.35);//0.27
			}
			
			if(_IQ19abs(q19Position) < _IQ19(0.5))
			{
				q19Position = _IQ19(0.0);
				
			}
		}
	}

	if(_IQ19abs(q19Position) > _IQ19(256))
	{
		if(q19Position > _IQ19(0))
			q19Position  = _IQ19(256);
		else
			q19Position = _IQ19(-256);
	}

	
	//Angle PID Controller
	//  (-) <---- Gyro ----->(+)
	q18AngleErrSum -= q18AngleErr[3];
	q18AngleErr[3] = q18AngleErr[2];
	q18AngleErr[2] = q18AngleErr[1];
	q18AngleErr[1] = q18AngleErr[0];
	q18AngleErr[0] = QDW(GyroVar.q20LPFOutData - QUP(GyroVar.q12AngleRef,8),2);
	q18AngleErrSum += q18AngleErr[0];

	q13GyroAngle += _IQ13abs(QDW(q18AngleErr[0], 5));//IQ13 정도의 범위면 충분함
	
	q18AngleProportionTerm = _IQ18mpyIQX(M_ANGLE_KP, 26, q18AngleErr[0], 18);
	q18AngleDerivativeTerm = _IQ18mpyIQX(M_ANGLE_KD, 26, ((q18AngleErr[0] - q18AngleErr[3]) + _IQ18mpy((q18AngleErr[1] - q18AngleErr[2]),_IQ18(3))),18);
	q18AngleIntegralTerm = _IQ18mpyIQX(M_ANGLE_KI, 26, q18AngleErrSum, 18);

	q18AnglePidOutTerm = q18AngleProportionTerm + q18AngleDerivativeTerm + q18AngleIntegralTerm;

	//PositionAdj PID Controller
	q18PositionErrSum -= q18PositionErr[3];
	q18PositionErr[3] = q18PositionErr[2];
	q18PositionErr[2] = q18PositionErr[1];
	q18PositionErr[1] = q18PositionErr[0];
	q18PositionErr[0] = QDW(q19Position,1);

	q18PositionErrSum += q18PositionErr[0];

	
	CalPosKp = M_POS_KP - _IQ26mpyIQX(M_CAL_POS_KP_DIFF, 26, RightMotor.q17NextVelocity, 17);

	if(CalPosKp < M_CAL_POS_KP_MIN)
		CalPosKp = M_CAL_POS_KP_MIN;

	CalPosKd = M_POS_KD + _IQ26mpyIQX(M_CAL_POS_KD_DIFF, 26, RightMotor.q17NextVelocity, 17);

	if(CalPosKd > M_CAL_POS_KD_MAX)
		CalPosKd = M_CAL_POS_KD_MAX;
	
	q18PosProportionTerm = _IQ18mpyIQX(CalPosKp, 26, q18PositionErr[0], 18);
	q18PosDerivativeTerm = QUP(_IQ16mpyIQX(CalPosKd, 26, ((q18PositionErr[0] - q18PositionErr[3]) + _IQ18mpy((q18PositionErr[1] - q18PositionErr[2]),_IQ18(3))),18), 2);
	q18PosIntegralTerm = _IQ18mpyIQX(M_POS_KI, 26, q18PositionErrSum, 18);

	//포지션 미분값 제한.
	if(_IQ18abs(q18PosDerivativeTerm) > _IQ18(55))
	{
		if(q18PosDerivativeTerm > 0)
			q18PosDerivativeTerm = _IQ18(55);
		else
			q18PosDerivativeTerm = _IQ18(-55);
	}

	if(gAngleAdjF == OFF)
		q18AnglePidOutTerm = _IQ18(0.0);
	else
		;

#if 0
	if(RightMotor.q17CurrentVelAvr > _IQ17(2000) && LeftMotor.q17CurrentVelAvr > _IQ17(2000))
		q18PosPidOutTerm = q18PosProportionTerm + q18PosDerivativeTerm + q18PosIntegralTerm - (_IQ18mpy(q18AnglePidOutTerm, ANGLE_ADJ_RATIO));
	
	else
		q18PosPidOutTerm = q18PosProportionTerm + q18PosDerivativeTerm + q18PosIntegralTerm;
#endif

	q18PosPidOutTerm = q18PosProportionTerm + q18PosDerivativeTerm + q18PosIntegralTerm - (_IQ18mpy(q18AnglePidOutTerm, ANGLE_ADJ_RATIO));
	

	//position PID 최대값 제한.
	if(_IQ18abs(q18PosPidOutTerm) > _IQ18(256))
	{
		if(q18PosPidOutTerm > _IQ18(0.0))
			q18PosPidOutTerm = _IQ18(256);
		else
			q18PosPidOutTerm = _IQ18(-256);
		
	}	

	if(q18PosPidOutTerm < 0)// 오른쪽 벽에 가깝다.
	{
		RightMotor.q26PosAdjRate = _IQ26mpyIQX(q30PosAdjAccelDiff, 30, (_IQ18(256) - q18PosPidOutTerm), 18) + q26PosAdjAccelRef;
		LeftMotor.q26PosAdjRate = _IQ26mpyIQX(q30PosAdjDecelDiff, 30, (_IQ18(256) + q18PosPidOutTerm), 18) + q26PosAdjDecelRef;
	}
	else// 왼쪽 벽에 가깝다.
	{
		RightMotor.q26PosAdjRate = _IQ26mpyIQX(q30PosAdjDecelDiff, 30, (_IQ18(256) - q18PosPidOutTerm), 18) + q26PosAdjDecelRef;
		LeftMotor.q26PosAdjRate = _IQ26mpyIQX(q30PosAdjAccelDiff, 30, (_IQ18(256) + q18PosPidOutTerm), 18) + q26PosAdjAccelRef;
	}

	if(gPosAdjF == OFF)
	{

#ifndef SMOOTHTEST

		q18PositionErrSum = _IQ18(0);
		q18PositionErr[0] = _IQ18(0);
		q18PositionErr[1] = _IQ18(0);
		q18PositionErr[2] = _IQ18(0);
		q18PositionErr[3] = _IQ18(0);

		q18AngleErrSum = _IQ18(0);
		q18AngleErr[0] = _IQ18(0);
		q18AngleErr[1] = _IQ18(0);
		q18AngleErr[2] = _IQ18(0);
		q18AngleErr[3] = _IQ18(0);
#endif

		RightMotor.q26PosAdjRate = LeftMotor.q26PosAdjRate = _IQ26(1);
		
	}
	else ;
	
	
}
interrupt void MotorTimer1ISR(void)
{
//  static Uint16 TintFlag = OFF;
//	volatile MotorVal *pMotor;
//	static Uint16 PwmDuty = 0;
//	static Uint16 PwmDir = 0;


	//QEP READ	
	RightMotor.u16QepSample = RightQepRegs.QPOSCNT;
	LeftMotor.u16QepSample = LeftQepRegs.QPOSCNT;

	//QEP reset
	RightQepRegs.QEPCTL.bit.SWI = 1;//initialize position counter
	LeftQepRegs.QEPCTL.bit.SWI = 1;//initialize position counter


	//QEP convert singed
	if(RightMotor.u16QepSample > 1000)
		RightMotor.i16QepValue = (int16)(RightMotor.u16QepSample - 2049)* (int16)(-1);
	else 
		RightMotor.i16QepValue = (int16)RightMotor.u16QepSample * (int16)(-1);

	if(LeftMotor.u16QepSample > 1000)
		LeftMotor.i16QepValue = (int16)(LeftMotor.u16QepSample - 2049);
	else
		LeftMotor.i16QepValue = (int16)LeftMotor.u16QepSample;
	
	//QEP convert Distance
	RightMotor.q27TickDistance = _IQ27mpyIQX(((int32)(RightMotor.i16QepValue) << 21),21,PULSE_TO_DIS,30);
	LeftMotor.q27TickDistance  = _IQ27mpyIQX(((int32)(LeftMotor.i16QepValue) << 21),21,PULSE_TO_DIS,30);

	RightMotor.q19DistanceSum += QDW(RightMotor.q27TickDistance, 8);//IQ27 --> IQ19
	LeftMotor.q19DistanceSum  += QDW(LeftMotor.q27TickDistance,  8); //IQ27 --> IQ19


	RightMotor.q19ErrDistance = RightMotor.q19UserDistance - RightMotor.q19DistanceSum;
	LeftMotor.q19ErrDistance = LeftMotor.q19UserDistance - LeftMotor.q19DistanceSum;


	//QEP convert Velocity

	RightMotor.q17CurrentVelocity[1] = RightMotor.q17CurrentVelocity[0];
	RightMotor.q17CurrentVelocity[0] = _IQ17mpyIQX(QUP(RightMotor.i16QepValue, 21),21,PULSE_TO_VEL,26);
	RightMotor.q17CurrentVelAvr = _IQ17mpy(RightMotor.q17CurrentVelocity[0] + RightMotor.q17CurrentVelocity[1], _IQ17(0.5));



	LeftMotor.q17CurrentVelocity[1] = LeftMotor.q17CurrentVelocity[0];
	LeftMotor.q17CurrentVelocity[0] = _IQ17mpyIQX(QUP(LeftMotor.i16QepValue,21),21,PULSE_TO_VEL,26);
	LeftMotor.q17CurrentVelAvr =  _IQ17mpy(LeftMotor.q17CurrentVelocity[0] + LeftMotor.q17CurrentVelocity[1], _IQ17(0.5));


	if((_IQ19abs(RightMotor.q19ErrDistance) <= RightMotor.q19StopDistance) && !(RightMotor.StopFlag))
	{
		RightMotor.q17UserVelocity = RightMotor.q17DecelVelocity;

		if(RightMotor.q17DecelVelocity == 0)
			RightMotor.StopFlag = 1;
		else
			RightMotor.StopFlag = 2;
	}

	if((_IQ19abs(LeftMotor.q19ErrDistance) <= LeftMotor.q19StopDistance) && !(LeftMotor.StopFlag))
	{
		LeftMotor.q17UserVelocity = LeftMotor.q17DecelVelocity;

		if(LeftMotor.q17DecelVelocity == 0)
			LeftMotor.StopFlag = 1;
		else
			LeftMotor.StopFlag = 2;
	}
	

	//속도 가감속 
	if(RightMotor.q17UserVelocity > RightMotor.q17NextVelocity)
	{
		RightMotor.q17NextVelocity += _IQ17mpyIQX(TimeTick, 30, QUP(RightMotor.i32Accel,15), 15);
		if(RightMotor.q17UserVelocity < RightMotor.q17NextVelocity)
			RightMotor.q17NextVelocity = RightMotor.q17UserVelocity;
	}
	else if(RightMotor.q17UserVelocity < RightMotor.q17NextVelocity)
	{
		RightMotor.q17NextVelocity -= _IQ17mpyIQX(TimeTick, 30, QUP(RightMotor.i32Accel,15), 15);
		if(RightMotor.q17UserVelocity > RightMotor.q17NextVelocity)
			RightMotor.q17NextVelocity = RightMotor.q17UserVelocity;
	}

	if(LeftMotor.q17UserVelocity > LeftMotor.q17NextVelocity)
	{
		LeftMotor.q17NextVelocity += _IQ17mpyIQX(TimeTick, 30, QUP(LeftMotor.i32Accel,15), 15);
		if(LeftMotor.q17UserVelocity < LeftMotor.q17NextVelocity)
			LeftMotor.q17NextVelocity = LeftMotor.q17UserVelocity;
	}
	else if(LeftMotor.q17UserVelocity < LeftMotor.q17NextVelocity)
	{
		LeftMotor.q17NextVelocity -= _IQ17mpyIQX(TimeTick, 30, QUP(LeftMotor.i32Accel,15), 15);
		if(LeftMotor.q17UserVelocity > LeftMotor.q17NextVelocity)
			LeftMotor.q17NextVelocity = LeftMotor.q17UserVelocity;
	}	


	//현재 정지중인가 ??
	if((RightMotor.StopFlag == 1) && (LeftMotor.StopFlag == 1) && (RightMotor.q17CurrentVelAvr == 0) && (LeftMotor.q17CurrentVelAvr == 0))
	{
		gStopCount++;
		if(gStopCount > 3)
		{
			gMoveState = ON;
			gStopCount = 0;
		}
	}
	else
	{
		gStopCount = 0;
		gMoveState = OFF;
	}


	PositionAdjPIDCtl();

	RightMotor.q17ErrVelocitySum -= RightMotor.q17ErrVelocity[3];
	RightMotor.q17ErrVelocity[3]  = RightMotor.q17ErrVelocity[2];
	RightMotor.q17ErrVelocity[2]  = RightMotor.q17ErrVelocity[1];
	RightMotor.q17ErrVelocity[1]  = RightMotor.q17ErrVelocity[0];
	RightMotor.q17ErrVelocity[0]  = _IQ17mpyIQX(RightMotor.q17NextVelocity, 17, RightMotor.q26PosAdjRate, 26) - RightMotor.q17CurrentVelAvr;
	RightMotor.q17ErrVelocitySum += RightMotor.q17ErrVelocity[0];

	RightMotor.q17ProportionalTerm = _IQ17mpyIQX(RightMotor.q28Kp, 28, RightMotor.q17ErrVelocity[0], 17);
	RightMotor.q17DerivativeTerm = _IQ17mpyIQX(RightMotor.q28Kd, 28,((RightMotor.q17ErrVelocity[0] - RightMotor.q17ErrVelocity[3]) + _IQ17mpy((RightMotor.q17ErrVelocity[1] - RightMotor.q17ErrVelocity[2]),_IQ17(3))),17);
	RightMotor.q17IntegralTerm = _IQ17mpyIQX(RightMotor.q28Ki, 28, RightMotor.q17ErrVelocitySum, 17);

	if(RightMotor.q17IntegralTerm > MAX_I_TERM)
		RightMotor.q17IntegralTerm = MAX_I_TERM;
	else if(RightMotor.q17IntegralTerm < MIN_I_TERM)
		RightMotor.q17IntegralTerm = MIN_I_TERM;

	RightMotor.q17PidOutTerm += RightMotor.q17ProportionalTerm + RightMotor.q17DerivativeTerm + RightMotor.q17IntegralTerm;

	LeftMotor.q17ErrVelocitySum -= LeftMotor.q17ErrVelocity[3];
	LeftMotor.q17ErrVelocity[3]  = LeftMotor.q17ErrVelocity[2];
	LeftMotor.q17ErrVelocity[2]  = LeftMotor.q17ErrVelocity[1];
	LeftMotor.q17ErrVelocity[1]  = LeftMotor.q17ErrVelocity[0];
	LeftMotor.q17ErrVelocity[0]  = _IQ17mpyIQX(LeftMotor.q17NextVelocity, 17, LeftMotor.q26PosAdjRate, 26) - LeftMotor.q17CurrentVelAvr;
	LeftMotor.q17ErrVelocitySum += LeftMotor.q17ErrVelocity[0];

	LeftMotor.q17ProportionalTerm = _IQ17mpyIQX(LeftMotor.q28Kp, 28, LeftMotor.q17ErrVelocity[0], 17);
	LeftMotor.q17DerivativeTerm = _IQ17mpyIQX(LeftMotor.q28Kd, 28,((LeftMotor.q17ErrVelocity[0] - LeftMotor.q17ErrVelocity[3]) + _IQ17mpy((LeftMotor.q17ErrVelocity[1] - LeftMotor.q17ErrVelocity[2]),_IQ17(3))),17);
	LeftMotor.q17IntegralTerm = _IQ17mpyIQX(LeftMotor.q28Ki, 28, LeftMotor.q17ErrVelocitySum, 17);

	if(LeftMotor.q17IntegralTerm > MAX_I_TERM)
		LeftMotor.q17IntegralTerm = MAX_I_TERM;
	else if(LeftMotor.q17IntegralTerm < MIN_I_TERM)
		LeftMotor.q17IntegralTerm = MIN_I_TERM;

	LeftMotor.q17PidOutTerm += LeftMotor.q17ProportionalTerm + LeftMotor.q17DerivativeTerm + LeftMotor.q17IntegralTerm;

	if(RightMotor.q17PidOutTerm > 0)
	{
		if(RightMotor.q17PidOutTerm > MAX_PID_OUT)
			RightMotor.q17PidOutTerm = MAX_PID_OUT;
		
		RightPwmRegs.AQCTLA.bit.ZRO = AQ_SET;
		RightPwmRegs.AQCTLB.bit.ZRO = AQ_CLEAR;
		RightPwmRegs.CMPA.half.CMPA = (Uint16)((_IQ17mpyIQX(RightMotor.q17PidOutTerm, 17, PWM_CONVERT, 30)) >> 17);
	}
	else
	{
		if(RightMotor.q17PidOutTerm < MIN_PID_OUT)
			RightMotor.q17PidOutTerm = MIN_PID_OUT;
		
		RightPwmRegs.AQCTLA.bit.ZRO = AQ_CLEAR;
		RightPwmRegs.AQCTLB.bit.ZRO = AQ_SET;
		RightPwmRegs.CMPB = (Uint16)((int16)((_IQ17mpyIQX(RightMotor.q17PidOutTerm, 17, PWM_CONVERT, 30)) >> 17) * -1);
	}

	if(LeftMotor.q17PidOutTerm > 0)
	{
		if(LeftMotor.q17PidOutTerm > MAX_PID_OUT)
			LeftMotor.q17PidOutTerm = MAX_PID_OUT;
		
		LeftPwmRegs.AQCTLA.bit.ZRO = AQ_SET;
		LeftPwmRegs.AQCTLB.bit.ZRO = AQ_CLEAR;
		LeftPwmRegs.CMPA.half.CMPA = (Uint16)((_IQ17mpyIQX(LeftMotor.q17PidOutTerm, 17, PWM_CONVERT, 30)) >> 17);
	}
	else
	{
		if(LeftMotor.q17PidOutTerm < MIN_PID_OUT)
			LeftMotor.q17PidOutTerm = MIN_PID_OUT;
		
		LeftPwmRegs.AQCTLA.bit.ZRO = AQ_CLEAR;
		LeftPwmRegs.AQCTLB.bit.ZRO = AQ_SET;
		LeftPwmRegs.CMPB = (Uint16)((int16)((_IQ17mpyIQX(LeftMotor.q17PidOutTerm, 17, PWM_CONVERT, 30)) >> 17) * -1);
	}



	//pid timer 수행후 sensor timer start..
	StartCpuTimer0();// sensor int start -- sensor shoot 
	gMenuCnt++;
	gUserTimeCnt++;
	gDiffAdjCnt++;

	RightMotor.u16Tick++;	
	LeftMotor.u16Tick++;
	
}

/*
	if(TintFlag == OFF)
	{
		TL_LED_ON;
		TintFlag = ON;
	}
	else
	{
		TL_LED_OFF;
		TintFlag = OFF;
	}
*/



void InitMotorVar(volatile MotorVal *pMotor)
{
	memset((void *)pMotor,0x00,sizeof(MotorVal));

	pMotor->q28Kd = _IQ28(6.1);//  6.0
	pMotor->q28Ki = _IQ28(0.0017);// 0.0015
	pMotor->q28Kp = _IQ28(4.5);// 4.4

	pMotor->i32Accel = gUserAccel = 8000;
	pMotor->q17UserVelocity = 0; 
	
}

void MoveStop(_iq19 q19RDis, _iq17 q17RVel, _iq19 q19LDis, _iq17 q17LVel)
{
	_iq19 RDisAbs;
	_iq19 LDisAbs;


	StopCpuTimer1();

	RDisAbs = _IQ19abs(q19RDis);
	LDisAbs = _IQ19abs(q19LDis);

	q18PositionErrSum = 0;
	q18PositionErr[0] = 0;
	q18PositionErr[1] = 0;
	q18PositionErr[2] = 0;
	q18PositionErr[3] = 0;
	q18PosProportionTerm = 0;
	q18PosDerivativeTerm = 0;
	q18PosIntegralTerm = 0;
	q18PosPidOutTerm = 0;

	q19RightPos = 0;
	q19LeftPos = 0;
	q19Position = 0;
	
	RightMotor.q19DistanceSum = LeftMotor.q19DistanceSum = _IQ19(0.0);
	RightMotor.StopFlag = LeftMotor.StopFlag = 0;

	if(q19RDis == 0) RightMotor.StopFlag = 1;
	if(q19LDis == 0) LeftMotor.StopFlag = 1;

	RightMotor.q19StopDistance = (_IQ6div(_IQ6mpyIQX(q17RVel, 17, q17RVel, 17), (RightMotor.i32Accel * 2) << 6)) << 13;
	LeftMotor.q19StopDistance =  (_IQ6div(_IQ6mpyIQX(q17LVel, 17, q17LVel, 17),  (LeftMotor.i32Accel * 2) << 6)) << 13;
	
	if((RightMotor.q19StopDistance * 2) > RDisAbs) 
		RightMotor.q19StopDistance = _IQ19mpy(RDisAbs, _IQ19(0.5));
	if((LeftMotor.q19StopDistance * 2) > LDisAbs)
		LeftMotor.q19StopDistance = _IQ19mpy(LDisAbs, _IQ19(0.5));

	RightMotor.q19UserDistance = q19RDis;
	LeftMotor.q19UserDistance  = q19LDis;
	
	RightMotor.q17UserVelocity = q17RVel;
	LeftMotor.q17UserVelocity  = q17LVel;

	RightMotor.q17DecelVelocity = 0;
	LeftMotor.q17DecelVelocity = 0;

	RightMotor.u16Tick = 0;

	gMoveState = OFF;

	RightMotor.q19ErrDistance = RightMotor.q19UserDistance - RightMotor.q19DistanceSum;
	LeftMotor.q19ErrDistance = LeftMotor.q19UserDistance - LeftMotor.q19DistanceSum;
	
	StartCpuTimer1();


	//TxPrintf("\n%f  %f  %f  %ld\n",_IQ19toF(RightMotor.q19StopDistance),_IQ19toF(RightMotor.q19UserDistance),_IQ17toF(RightMotor.q17UserVelocity),RightMotor.i32Accel);
	//TxPrintf("\n%f  %f  %f  %ld\n",_IQ19toF(LeftMotor.q19StopDistance),_IQ19toF(LeftMotor.q19UserDistance),_IQ17toF(LeftMotor.q17UserVelocity),LeftMotor.i32Accel);
}


void MoveToMove(int16 AccVel, int16 DecVel, int16 Dis)
{	
	_iq19 q19Dis = QUP(Dis,19);	
	_iq19 q19DisAbs;
	_iq19 q19AccDis;
	_iq19 q19StopDis;
	_iq17 q17AccVel = QUP(AccVel,17);
	_iq17 q17DecVel = QUP(DecVel,17);

	StopCpuTimer1();
	
	q19DisAbs = _IQ19abs(q19Dis);

	RightMotor.q19DistanceSum = LeftMotor.q19DistanceSum = 0;
	RightMotor.StopFlag = LeftMotor.StopFlag = 0;


	q19AccDis = (_IQ6div((_IQ6mpyIQX(q17AccVel, 17, q17AccVel, 17) - _IQ6mpyIQX(RightMotor.q17CurrentVelAvr,17,RightMotor.q17CurrentVelAvr,17)),((RightMotor.i32Accel * 2) << 6))) << 13;
	q19StopDis = (_IQ6div((_IQ6mpyIQX(q17AccVel, 17, q17AccVel, 17) - _IQ6mpyIQX(q17DecVel,17,q17DecVel,17)),((RightMotor.i32Accel * 2) << 6))) << 13;

	if((q19AccDis + q19StopDis)	> q19DisAbs)
	{
		q19AccDis = _IQ6div((_IQ6mpyIQX(q17DecVel, 17, q17DecVel, 17) - _IQ6mpyIQX(RightMotor.q17CurrentVelAvr,17,RightMotor.q17CurrentVelAvr,17) + _IQ6mpy((2 * RightMotor.i32Accel) << 6,QDW(q19DisAbs,13))), (4*RightMotor.i32Accel) << 6) << 13; 
		q19StopDis = q19DisAbs - q19AccDis;
		q17AccVel = _IQ6sqrt(_IQ6mpy((2*RightMotor.i32Accel)<<6, QDW(q19AccDis,13)) + _IQ6mpyIQX(RightMotor.q17CurrentVelAvr,17,RightMotor.q17CurrentVelAvr,17)) << 11;
	}
	else
		;

	RightMotor.q19UserDistance = LeftMotor.q19UserDistance = q19Dis;
	RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = q17AccVel;
	RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = q17DecVel;
	RightMotor.q19StopDistance = LeftMotor.q19StopDistance = q19StopDis;


	RightMotor.u16Tick = 0;
	
	RightMotor.q19ErrDistance = RightMotor.q19UserDistance - RightMotor.q19DistanceSum;
	LeftMotor.q19ErrDistance = LeftMotor.q19UserDistance - LeftMotor.q19DistanceSum;
	
	
	//TxPrintf("%f %f %f %f",_IQ19toF(q19Dis),_IQ17toF(q17AccVel),_IQ17toF(q17DecVel),_IQ19toF(q19StopDis));
	
	StartCpuTimer1();
}

//턴 탈출 후 턴별 탈출 거리..
TurnOutDis TurnOutErr[6] = 
{
//	 		90 |		 45 | 		135 | 		180
	{ _IQ19( 0.0), _IQ19( 0.0), _IQ19( 0.0), _IQ19( 0.0)},//[680]
	{ _IQ19( 3.0), _IQ19(52.0), _IQ19(34.0), _IQ19(29.0)},//[800]
	{ _IQ19( 0.0), _IQ19( 0.0), _IQ19( 0.0), _IQ19( 0.0)},//FistDiag
	{ _IQ19(17.0), _IQ19(50.0), _IQ19(41.0), _IQ19(34.0)},//[1100]
	{ _IQ19(30.0), _IQ19(42.0), _IQ19(37.0), _IQ19(27.0)},//[1220]
	{ _IQ19(30.0), _IQ19(56.0), _IQ19(37.0), _IQ19(30.0)},//[1300]	
	};

//턴 진입 시 여유거리.   
TurnInDis TurnInErr[6] = 
{
//			90 |		 	45 |		135 |		180
	{ _IQ19( 70.0), _IQ19( 0.0), _IQ19(  0.0), _IQ19(  0.0)},//[680]
	{ _IQ19( 70.0), _IQ19(70.0), _IQ19( 70.0), _IQ19( 70.0)},//[800]
	{ _IQ19( 75.0), _IQ19(75.0), _IQ19( 75.0), _IQ19( 70.0)},//FistDiag
	{ _IQ19( 70.0), _IQ19(70.0), _IQ19( 70.0), _IQ19( 70.0)},//[1100]
	{ _IQ19( 70.0), _IQ19(70.0), _IQ19( 70.0), _IQ19( 70.0)},//[1220]
	{ _IQ19( 70.0), _IQ19(70.0), _IQ19( 70.0), _IQ19( 70.0)},//[1300]

};

void BlockStraight(TurnDebugVar Debug)
{	
	Uint16	RunBlockCnt;
	Uint16	NextDir;
	Uint16 	NextNextDir;
	Uint16	YetDir;
	Uint16	BlockCnt;
	Uint16	EdgeReset;
	Uint16	AccelVel;
	_iq19	Dis;
	_iq19	DiagOutLength;
	_iq19	StartEdgeCheckLength;
	_iq19	TurnInLinkDis;
	_iq19 	RunLength;
	
	gBackTurnFrontAdjState = OFF;
	gDiagDirectAdjState = OFF;
	gBlockToBlock = TURN2STRT;

	POS_ADJ_ON;
	ANGLE_ADJ_ON;	
	gFrontSensorPull = ON;
	
	ResetEdgeVariable();
	EdgeReset = OFF;

	RunBlockCnt = KnowBlockPath[gPathBufferHead].PathCnt;

	if(gPathBufferHead == 0)
		YetDir = NMATCH;
	else
		YetDir = KnowBlockPath[gPathBufferHead - 1].PathState;

	if(gAlgoState == ON)
		NextDir = NMATCH;
	else
	{
		NextDir = KnowBlockPath[gPathBufferHead + 1].PathState;
		NextNextDir = KnowBlockPath[gPathBufferHead + 2].PathState; 

		//FLx
		//FRx
		//FRR ,180 cobra
		//FLL , 180 cobra

		if((gUserTurnSpeed == SMOOTH680) || (gUserTurnSpeed == SMOOTH800))
		{
			if(((NextDir == R90) || (NextDir == L90)) && ((NextNextDir == R90) || (NextNextDir == L90) || (NextNextDir == LASTPATH)))
			{
				gUserSpeed = 680;
				gUserTurnSpeed = SMOOTH680;
				if(RunBlockCnt == 1)
				{
					RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = \
					RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(680.0);	
				}
				
			}
			else
			{
				gUserSpeed = 800;
				gUserTurnSpeed = SMOOTH800;
				RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = \
				RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(800.0);
			}
		}
		else if(gUserTurnSpeed == SMOOTH1300)
		{
			if((NextDir == R135IN) || (NextDir == L135IN))
			{
				gUserSpeed = 1220;
				if(RunBlockCnt == 1)
				{
					RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = \
					RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(1220.0);	
				}
				else
					;
				
			}
			else
			{
				gUserSpeed = 1300;
				if(RunBlockCnt == 1)
				{
					RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = \
					RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(1300.0);	
				}
				else
					;
				
			}
		}
		else if(gUserTurnSpeed == SMOOTH1220)
		{
			gUserSpeed = 1220;
			if(RunBlockCnt == 1)
			{
				RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = \
				RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(1220.0);	
			}
			else
				;
		
		}
		else if(gUserTurnSpeed == SMOOTH1100)
		{
			gUserSpeed = 1100;
			if(RunBlockCnt == 1)
			{
				RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = \
				RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(1100.0);	
			}
			else
				;
		
		}
	
	}

		
	if(RunBlockCnt > 1)
	{
		AccelVel = gDirectSpeedLimit;

		RunLength = _IQ19mpyIQX(QUP(RunBlockCnt,4),4,QUP(ONEBLOCK,4),4);

		//시작점으로 돌아오는 백턴
		if(NextDir == BACKTURN)
			StartEdgeCheckLength = _IQ19(0.0);
		
		//스타트 직진에 //시작점68mm
		else if(KnowBlockPath[gPathBufferHead].Position == 0x00)
			StartEdgeCheckLength = _IQ19(68.0);

		//일본알고리즘 0x01에서 시작점80mm, 백턴 후 와 같다 
		else if((KnowBlockPath[gPathBufferHead].Position == 0x01) && (gJapanAlgo))
			StartEdgeCheckLength = _IQ19(80.0);
		
		//백턴 후 시작점 80mm
		else if(gBlockRunException == ON)
			StartEdgeCheckLength = _IQ19(80.0);

		//일반적인경우 
		else		
			StartEdgeCheckLength = _IQ19(0.0);

		RunLength -= StartEdgeCheckLength;

		
		//전 턴 탈출했을때 거리 재조정
		if((YetDir == R45OUT) || (YetDir == L45OUT))
			DiagOutLength = TurnOutErr[gUserTurnSpeed].T45OutDis;

		else if((YetDir == R90) || (YetDir == L90))
			DiagOutLength = TurnOutErr[gUserTurnSpeed].T90OutDis;
				
		else if((YetDir == R135OUT) || (YetDir == L135OUT))
			DiagOutLength = TurnOutErr[gUserTurnSpeed].T135OutDis;
		
		else if((YetDir == R180) || (YetDir == L180))
			DiagOutLength = TurnOutErr[gUserTurnSpeed].T180OutDis;

		else
			DiagOutLength = _IQ19(0.0);
	
		RunLength -= DiagOutLength;
		
		//다음턴이 아는길 일때 진입거리 재조정
		if((NextDir == R45IN) || (NextDir == L45IN))
			TurnInLinkDis = TurnInErr[gUserTurnSpeed].T45InDis;

		else if((NextDir == R90) || (NextDir == L90))
			TurnInLinkDis = TurnInErr[gUserTurnSpeed].T90InDis;
		
		else if((NextDir == R135IN) || (NextDir == L135IN))
			TurnInLinkDis = TurnInErr[gUserTurnSpeed].T135InDis;
		
		else if((NextDir == R180) || (NextDir == L180))
			TurnInLinkDis = TurnInErr[gUserTurnSpeed].T180InDis;

		//시작점 돌아오는 백턴 
		else if(NextDir == BACKTURN)
			TurnInLinkDis = _IQ19(180.0);
		
		//벽체크 하고 알고리즘 돌려야 하는 경우   
		else if(NextDir == NMATCH)//gAlgoState == ON 이면 NextDir == NMATH
		{
			//직진 가속후 알고리즘 돌릴경우 탐색속도로..
			gUserSpeed = 680;
			gUserTurnSpeed = SMOOTH680;
			TurnInLinkDis = _IQ19(165.0);
		}

		RunLength -= _IQ19(ONEBLOCK) - TurnInLinkDis;		
		
		RightMotor.i32Accel = LeftMotor.i32Accel = gUserDirectAccel;

		//골로 직진 가속해서 들어올상황...
		//탐색속도로 감속속도 전환..
		if(gSecondRunGoal == ON)
			gUserSpeed = 680;
		else
			;
		
		MoveToMove(AccelVel, gUserSpeed, QDW(RunLength,19));
		
		BlockCnt = 0;
		while((RightMotor.q19DistanceSum < RunLength) && (LeftMotor.q19DistanceSum < RunLength))
		{
			
			Dis = _IQ19mpy(RightMotor.q19DistanceSum + LeftMotor.q19DistanceSum, _IQ19(0.5));
			
			if((Dis > (QUP(ONEBLOCK * BlockCnt, 19) + _IQ19(10.0) - DiagOutLength - StartEdgeCheckLength)) && !EdgeReset)
			{
				EdgeReset = ON;
				ResetEdgeVariable();
				
				if(Dis > _IQ19(180.0))
					gEdgeDiffAdjustFlag = ON;
				else
					gEdgeDiffAdjustFlag = OFF;
			
			}
			
			if((Dis > (QUP(ONEBLOCK * BlockCnt, 19) + _IQ19(160.0) - DiagOutLength - StartEdgeCheckLength)) && EdgeReset)
			{
				gEdgeDiffAdjustFlag = OFF;
				EdgeReset = OFF;
				BlockCnt++;

			}

			if(Dis > (RunLength - _IQ19(120.0)))
				gFrontSensorPull = OFF;
			
		}
		if(NextDir != NMATCH)
			ResetEdgeVariable();
		else
			;
	
		gEdgeDiffAdjustFlag = OFF;
		RightMotor.q19DistanceSum = LeftMotor.q19DistanceSum = TurnInLinkDis;//진입 거리 맞추기 
		
	}
	else// 한 블럭 직진..
	{
		if(gSecondRunGoal == ON)//골 도착 탐색속도로 맞춤   
		{
			gUserSpeed = 680;
			RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = _IQ17(680.0);
			RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(680.0);
		}
			
		else
			;

		//전 턴 탈출했을때 거리 보상  
		if((YetDir == R45OUT) || (YetDir == L45OUT))
		{
			RightMotor.q19DistanceSum += TurnOutErr[gUserTurnSpeed].T45OutDis;
			LeftMotor.q19DistanceSum += TurnOutErr[gUserTurnSpeed].T45OutDis;
		}
		else if((YetDir == R90) || (YetDir == L90))
		{
			RightMotor.q19DistanceSum += TurnOutErr[gUserTurnSpeed].T90OutDis;
			LeftMotor.q19DistanceSum += TurnOutErr[gUserTurnSpeed].T90OutDis;
		}
		else if((YetDir == R135OUT) || (YetDir == L135OUT))
		{
			RightMotor.q19DistanceSum += TurnOutErr[gUserTurnSpeed].T135OutDis;
			LeftMotor.q19DistanceSum += TurnOutErr[gUserTurnSpeed].T135OutDis;
		}
		else if((YetDir == R180) || (YetDir == L180))
		{
			RightMotor.q19DistanceSum += TurnOutErr[gUserTurnSpeed].T180OutDis;
			LeftMotor.q19DistanceSum += TurnOutErr[gUserTurnSpeed].T180OutDis;
		}
		else
			;
				
		//다음턴이 아는길 일때 진입거리 재조정
		if((NextDir == R45IN) || (NextDir == L45IN))
			TurnInLinkDis = TurnInErr[gUserTurnSpeed].T45InDis;
		
		else if((NextDir == R90) || (NextDir == L90))
			TurnInLinkDis = TurnInErr[gUserTurnSpeed].T90InDis;
		
		else if((NextDir == R135IN) || (NextDir == L135IN))
			TurnInLinkDis = TurnInErr[gUserTurnSpeed].T135InDis;
		
		else if((NextDir == R180) || (NextDir == L180))
			TurnInLinkDis = TurnInErr[gUserTurnSpeed].T180InDis;

		//벽체크 하고 알고리즘 돌려야 하는경우.
		else if(NextDir == NMATCH)//gAlgoState == ON 이면 NextDir == Straight
		{
			//알고리즘 돌리는 경우 탐색 속도로...
			gUserSpeed = 680;
			gUserTurnSpeed = SMOOTH680;
			RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = _IQ17(680.0);
			RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(680.0);
			
			TurnInLinkDis = _IQ19(165.0);
		}
		
	}
			
	while((RightMotor.q19DistanceSum < TurnInLinkDis) || (LeftMotor.q19DistanceSum < TurnInLinkDis))
	{
		if((RightMotor.q19DistanceSum > _IQ19(60.0)) || (LeftMotor.q19DistanceSum > _IQ19(60.0)))
			gFrontSensorPull = OFF;
		else
			;
	}
	gFrontSensorPull = OFF;

	if(gAlgoState == ON)
	{
		if(gSecondRunGoal == ON)
		{
			gUserSpeed = 680;
			gUserTurnSpeed = SMOOTH680;

			gSearchType = GO_START;
			gSecondRunGoal = OFF;
			//시간 계산 넣기
			gRunTime = _IQ15mpy(QUP(gUserTimeCnt, 15), _IQ15(0.0005));
		}
		else
			;

		Algorithm(WallCheck(pRDS, pLDS));	
		NextDir = KnowBlockPath[0].PathState;
	
		
	}
	else
	{
		NextDir = KnowBlockPath[gPathBufferHead + 1].PathState;
	}
	
		
	if((NextDir == STRAIGHT) || (NextDir == BACKTURN))
	{
		while((RightMotor.q19DistanceSum < _IQ19(ONEBLOCK)) && (LeftMotor.q19DistanceSum < _IQ19(ONEBLOCK)))
			;
		
		if(NextDir == STRAIGHT)// 1차 주행 에서만 들어온다. 2차에서 직진다음 직진은 없다.
			MoveToMove(gUserSpeed, gUserSpeed, ONEBLOCK);
		else// Next Block -- BackTurn..
			;
	}
	else// Next Block -- TURN & Diag
		;

	gBlockRunException = OFF;

}

///////////////////////////////////////////////////////////////////////////////////////////////
// Funtion : Back Turn
// Edit : 2006.01.14	by H.H.Y
// summary :
//
//	보정모두 끄고 옆 보정 앵글 보정
//	좌,우,전방, 벽 체크
//	앞벽이 있을 경우 측면 보정 후 일정거리에서 속도줄이고 전방수평보정
//	일정거리에서 측면 거리 체크후 정지
//	앞벽이 없을 경우 한블럭완전 가기, 일본 알고리즘일때는 0x01에서 센서에 걸리지 않을 만큼
//	보정끄고 측면 거리로 중심 보정
//	백턴후 맵 저장
//	시작점 일 경우 뒤로 일정거리 당겨서 차체 보정
//	시작점 아닐 경우 알고리즘 돌리고 앞벽 유무에 따라 거리 다르게 세팅
///////////////////////////////////////////////////////////////////////////////////////////////
void BackTurn(TurnDebugVar Debug)
{
	_iq19 	WheelDiff;
	_iq19 	MoveAcrossDistance;
	_iq19 	RightAcrossDistance;
	_iq19 	LeftAcrossDistance;
	_iq19 	RightPosition;
	_iq19	LeftPosition;

	_iq19	OutLength;

	_iq17 	RightAcrossVelocity;
	_iq17 	LeftAcrossVelocity;

	Uint16 	i;
	Uint16	WallInfo;
	Uint16	FrontWallState;
	Uint16	RightWallState;
	Uint16	LeftWallState;
	Uint16	YetDir;



	gBackTurnFrontAdjState = OFF;
	gDiagDirectAdjState = OFF;
	gEdgeDiffAdjustFlag = OFF;
	
	WallInfo = gMazeMap[KnowBlockPath[gPathBufferHead].Position] & 0x0f;
	
	if(WallInfo & WallTable[KnowBlockPath[gPathBufferHead].MouseDir][0]) FrontWallState = ON;
	else FrontWallState = OFF;

	if(WallInfo & WallTable[KnowBlockPath[gPathBufferHead].MouseDir][1]) RightWallState = ON;
	else RightWallState = OFF;

	if(WallInfo & WallTable[KnowBlockPath[gPathBufferHead].MouseDir][2]) LeftWallState = ON;
	else LeftWallState = OFF;

#ifdef BACKTURNTEST
	FrontWallState = ON;
	RightWallState = ON;
	LeftWallState  = ON;
	gSearchEndState = OFF;
#endif

	if(FrontWallState == ON)
	{
		MoveToMove(gUserSpeed,gUserSpeed, ONEBLOCK);

		while(TRUE)
		{
			if((pRFS->q19Position < _IQ19(18)) || (pLFS->q19Position < _IQ19(18)))
			{
				RightMotor.i32Accel = LeftMotor.i32Accel = 5000;
				MoveStop(0,0,0,0);
				break;
			}
			else if((pRFS->q19Position < _IQ19(30)) || (pLFS->q19Position < _IQ19(30)))
			{
				POS_ADJ_OFF;
				gBackTurnFrontAdjState = OFF;
				RightPosition = pRSS->q19Position;
				LeftPosition = pLSS->q19Position;
				
			}
			else if((pRFS->q19Position < _IQ19(70)) || (pLFS->q19Position < _IQ19(70)))//80.0		
			{
				ANGLE_ADJ_OFF;
				gBackTurnFrontAdjState = ON;
				RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = _IQ17(100);
			}

			if(RightMotor.q19DistanceSum > _IQ19(120.0))// 예외적으로 앞벽을 보고 못 멈추는 상황 거리로 정지한다
			{
				POS_ADJ_OFF;
				ANGLE_ADJ_OFF;
				gBackTurnFrontAdjState = OFF;
				RightPosition = _IQ19(256.0);
				LeftPosition = _IQ19(256.0);
				MoveStop(0,0,0,0);
				while(!gMoveState);
				MoveStop(_IQ19(-13.0),_IQ17(-300.0),_IQ19(-13.0),_IQ17(-300.0));

				break;
			}
			
		}
	}
	
	else//Front Wall not 
	{	
		if((gJapanAlgo) && (gSearchEndState == ON))
		{
			RightMotor.i32Accel = LeftMotor.i32Accel = 10000;
			
			YetDir = KnowBlockPath[gPathBufferHead-1].PathState;

			if(YetDir == STRAIGHT)
				OutLength = _IQ19(100);
			
			else if(YetDir == L90)
				OutLength = _IQ19(100) - TurnOutErr[gUserTurnSpeed].T90OutDis;

			else if(YetDir == L45OUT)
				OutLength = _IQ19(100) - TurnOutErr[gUserTurnSpeed].T45OutDis;

			else if(YetDir == L135OUT)
				OutLength = _IQ19(100) - TurnOutErr[gUserTurnSpeed].T135OutDis;

			else if(YetDir == L180)
				OutLength = _IQ19(100) - TurnOutErr[gUserTurnSpeed].T180OutDis;
				
			MoveStop(OutLength, QUP(gUserSpeed,17), OutLength, QUP(gUserSpeed,17));

		}
		else
			MoveStop(QUP(ONEBLOCK,19), QUP(gUserSpeed,17), QUP(ONEBLOCK,19), QUP(gUserSpeed,17));
	}

	while(!gMoveState);
	POS_ADJ_OFF;


	if(!RightWallState && !LeftWallState)//양 벽이 모두 없다면.
		WheelDiff = _IQ19(0.0);
	else if(!RightWallState)//오른벽이 없으면.
		WheelDiff = _IQ19mpy(LeftPosition - _IQ19(256.0), _IQ19(0.18));//0.17578125
	else if(!LeftWallState)//왼벽이 없으면.
		WheelDiff = _IQ19mpy(_IQ19(256.0) - RightPosition, _IQ19(0.18));//0.17578125
	else
		WheelDiff = _IQ19mpy((_IQ19(256.0) - RightPosition) + (LeftPosition - _IQ19(256.0)),_IQ19(0.09));//0.087890625

	MoveAcrossDistance = _IQ19mpy(_IQ19(acos(_IQ19toF(_IQ19div(_IQ19(MOUSETREAD) - _IQ19mpy(_IQ19abs(WheelDiff), _IQ19(0.25)),_IQ19(MOUSETREAD))))),_IQ19(MOUSETREAD));
	
	if(WheelDiff > _IQ19(0.0))//Left Across Move
	{
		RightAcrossDistance =  MoveAcrossDistance;
		RightAcrossVelocity = _IQ17(800.0);
		LeftAcrossDistance = 0.0;
		LeftAcrossVelocity = 0.0;
	}
	else//Right Across Move
	{
		RightAcrossDistance = 0.0;
		RightAcrossVelocity = 0.0;
		LeftAcrossDistance = MoveAcrossDistance;
		LeftAcrossVelocity = _IQ17(800.0);
	
	}

	RightMotor.i32Accel = LeftMotor.i32Accel = 5000;// xxxxxx

	if((_IQ19abs(WheelDiff) > _IQ19(10.0)) && FrontWallState)
	{
		MoveStop(_IQ19mpy(RightAcrossDistance, _IQ19(-1.0)), _IQ17mpy(RightAcrossVelocity, _IQ17(-1.0)), _IQ19mpy(LeftAcrossDistance, _IQ19(-1.0)), _IQ17mpy(LeftAcrossVelocity,_IQ17(-1.0)));
		while(!gMoveState) ;

		MoveStop(_IQ19mpy(LeftAcrossDistance, _IQ19(-1.0)), _IQ17mpy(LeftAcrossVelocity,_IQ17(-1.0)), _IQ19mpy(RightAcrossDistance, _IQ19(-1.0)), _IQ17mpy(RightAcrossVelocity, _IQ17(-1.0)));
		while(!gMoveState) ;

		MoveStop(RightAcrossDistance, RightAcrossVelocity, LeftAcrossDistance, LeftAcrossVelocity);
		while(!gMoveState) ;
		
		MoveStop(LeftAcrossDistance, LeftAcrossVelocity, RightAcrossDistance, RightAcrossVelocity);
		
		while(!gMoveState) ;
	}

	//back turn
	RightMotor.i32Accel = LeftMotor.i32Accel = 3000;
	MoveStop(_IQ19(-107), _IQ17(-800), _IQ19(107), _IQ17(800));//107.0
	
	
	memcpy((void *)gMazeMapBackUp2, (void *)gMazeMapBackUp1, sizeof(gMazeMap));
	memcpy((void *)gMazeMapBackUp1, (void *)gMazeMapBackUp0, sizeof(gMazeMap));
	memcpy((void *)gMazeMapBackUp0, (void *)gMazeMapBackUp, sizeof(gMazeMap));
	memcpy((void *)gMazeMapBackUp, (void *)gMazeMap, sizeof(gMazeMap));
	

#ifndef BACKTURNTEST
	//미로 저장.
	SpiWriteRom(MAP_BACKUP_ADDRESS, 0, 256, (Uint16 *)gMazeMapBackUp);
	SpiWriteRom(MAP_BACKUP0_ADDRESS, 0, 256, (Uint16 *)gMazeMapBackUp0);
	SpiWriteRom(MAP_BACKUP1_ADDRESS, 0, 256, (Uint16 *)gMazeMapBackUp1);
	SpiWriteRom(MAP_BACKUP2_ADDRESS, 0, 256, (Uint16 *)gMazeMapBackUp2);
	//
#endif

	while(!gMoveState);// 백턴 정지 ???? 
	

	if(gSearchEndState == OFF)
	{
		gMousePosition = KnowBlockPath[gPathBufferHead].Position;
	
		TxPrintf("%x  %x\n",gMousePosition, gMouseDir);

#ifndef BACKTURNTEST

		Algorithm(gMazeMap[KnowBlockPath[gPathBufferHead].Position]);

		//VFDPrintf("%x",gPathBufferHead);

		if(gPathBufferHead == 0)
			TxPrintf("=%d=\n",KnowBlockPath[gPathBufferHead].PathState);
		
		for(i = 0; i < gPathBufferHead; i++)
		{
			TxPrintf("%d : %x  %d  %d  %d\n",i, 
										KnowBlockPath[i].Position,
										KnowBlockPath[i].MouseDir,
										KnowBlockPath[i].PathState,
										KnowBlockPath[i].PathCnt);
		}
#endif

		RightMotor.i32Accel = LeftMotor.i32Accel = gUserAccel;		


		//백턴후 한블럭 직진 다음 대각 45 인경우 직진거리가 모자라므로 뒤로 당기고 가속도 올린다.
		if((KnowBlockPath[0].PathState == STRAIGHT) && (KnowBlockPath[0].PathCnt == 1)
			&& ((KnowBlockPath[1].PathState == R45IN) || (KnowBlockPath[1].PathState == L45IN)))
		{
			//MoveStop(_IQ19(-15.0), _IQ17(-100.0),_IQ19(-15.0), _IQ17(-100.0));
			//
			//while(!gMoveState)
			//	;
			RightMotor.i32Accel = LeftMotor.i32Accel = 10000;
		}
		
		
		POS_ADJ_ON;
		ANGLE_ADJ_ON;

		MoveToMove(gUserSpeed, gUserSpeed, ONEBLOCK);
		if(FrontWallState == ON)
		{
			gBlockRunException = ON;
			RightMotor.q19DistanceSum = LeftMotor.q19DistanceSum = _IQ19(80.0);
		}
		else
			gBlockRunException = OFF;

		gAlgoState = ON;
	}
	else//시작점으로 돌아왔을경우
	{
		TR_LED_ON;
		TL_LED_ON;

		gMouseYetPosition = 0;
		
		VFDPrintf("%f",_IQ15toF(gRunTime));

		RightMotor.i32Accel = LeftMotor.i32Accel = gUserAccel;
		if(gJapanAlgo == OFF)
		{
			MoveStop(_IQ19(-20.0), _IQ17(-100.0),_IQ19(-20.0), _IQ17(-100.0));
			while(!gMoveState)
			 	;
		}
		else
			 ;
		
		LED_OFF;
	}

}

volatile TurnInfoVariable *pTurnTable;
volatile TurnInfoVariable *pTurnCbrTable;


//턴 테이블이 커지면서 램이 부족에 허덕임
//0 - 680 /1 - 800 /2 - first diag 로 사용
// 3번 배열은 2차 턴들을 위한 공용으로 사용하며 2차 주행때 해당 속도의 턴테이블을 시리얼롬에서  3번 배열로 옮겨서 사용

Uint16 TurnTableIndex[6] = {0, 1, 2, 3, 3, 3};


volatile TurnInfoVariable TurnTable[4][20];


volatile TurnInfoVariable TurnCbrTable[4] = 
{
	
	//RCobra45OUT				   turninT | turninerr | accelT | turnT | turnout | TRaccel | Laccel | RAccelVelocity | LAccelVelocity | Reference Velocity | edge0 | edge1 | Fedge |		 //RCobra45OUT		
				{      NULL, NULL,	   0,		0,	 102,  100, 	0,	8000,	8000, _IQ17(272.0), _IQ17(1088.0),	_IQ17(680.0),	 0, 	0,	   0, NULL, NULL},	
	
	
	//LCobra45OUT				   turninT | turninerr | accelT | turnT | turnout | TRaccel | Laccel | RAccelVelocity | LAccelVelocity | Reference Velocity | edge0 | edge1 | Fedge |		 //LCobra45OUT
				{	   NULL, NULL,	   0,		0,	 102,   98, 	0,	8000,	8000, _IQ17(1088.0), _IQ17(272.0),	_IQ17(680.0),	 0, 	0,	   0, NULL, NULL},	
	
	
	//RCobra135OUT				   turninT | turninerr | accelT | turnT | turnout | TRaccel | Laccel | RAccelVelocity | LAccelVelocity | Reference Velocity | edge0 | edge1 | Fedge |		 //RCobra145OUT 
				{	   NULL, NULL,	   0,		0,	  90,  505,    23,	7000,	7000, _IQ17(365.0), _IQ17(995.0),	_IQ17(680.0),	 0, 	0,	   0, NULL, NULL},	
	
		
	//LCobra135OUT				   turninT | turninerr | accelT | turnT | turnout | TRaccel | Laccel | RAccelVelocity | LAccelVelocity | Reference Velocity | edge0 | edge1 | Fedge |		 //LCobra145OUT 
				{ 	   NULL, NULL,	   0,		0,	  90,  498,    23,	7000,	7000, _IQ17(995.0), _IQ17(365.0),	_IQ17(680.0),	 0, 	0,	   0, NULL, NULL}
};

#define FRONT_WALL_VALUE	_IQ19(110.0)

void SmoothTurn(TurnDebugVar Debug)
{
#ifdef SMOOTHTEST
	Uint16	Temp = 0;
	_iq13	q13GyroTemp = 0;
#endif
	Uint16	Direction;
	Uint16	MouseDir;
	Uint16	Position;
	Uint16	TurnEdge = OFF;
	Uint16	NextDir;
	Uint16	WallInfo;
	Uint16	FrontWallState0;
	Uint16  FrontWallState1;
	Uint16	TurnWallState;
	Uint16	TurnInTime;
	Uint16  GoalPoleNoneF;
	volatile Uint16 *pEdgeCnt;

	gBackTurnFrontAdjState = OFF;
	gDiagDirectAdjState = OFF;
	gEdgeDiffAdjustFlag = OFF;

	Direction = KnowBlockPath[gPathBufferHead].PathState;
	MouseDir = KnowBlockPath[gPathBufferHead].MouseDir;
	Position = KnowBlockPath[gPathBufferHead].Position;
	
	pTurnTable = (TurnInfoVariable *)&TurnTable[TurnTableIndex[gUserTurnSpeed]][Direction];

	if((Position == 0x77) || (Position == 0x78) || (Position == 0x87) || (Position == 0x88))
		GoalPoleNoneF = ON;
	else
		GoalPoleNoneF = OFF;
	
	//턴 할때 앞벽의 유무 --> 턴에지 종류 판별
	WallInfo = gMazeMap[Position] & 0x0f;
	FrontWallState0 = WallInfo & WallTable[MouseDir][0];

	//SMOOTH1300  턴에지에 필요..
	WallInfo = gMazeMap[Position + gMoveTable[(Direction + MouseDir) & 0x03]] & 0x0f;
	FrontWallState1 = WallInfo & WallTable[MouseDir][0];

	//gMouseYetPosition 과 gMouseYetDir 으로 현재 벽의 방향 정보 생성
	//턴 진입 시점 체크를 위해 전블럭의 좌우 벽정보 필요 벽이 있으면 벽이 사라질 때를 체크하며
	//벽이 없을경우 에지를 체크한다.
	WallInfo = gMazeMap[Position + gMoveTable[(MouseDir + 2) & 0x03]] & 0x0f;

	if(Direction == R90)//Right
	{		
		TurnWallState = WallInfo & WallTable[MouseDir][1];
	}
	
	else//Left
		TurnWallState = WallInfo & WallTable[MouseDir][2];

#ifdef SMOOTHTEST
	FrontWallState0 = Debug.Fwall0;
	FrontWallState1 = Debug.Fwall1;
	TurnWallState = Debug.Twall0;
#endif

#ifdef TURNHOLD
	if(gUserTurnSpeed != SMOOTH680)
		RightMotor.q19DistanceSum = _IQ19(0.0);
#endif

	if(gBlockToBlock != TURN2TURN)
	{
		TurnInTime = pTurnTable->u16TurnInTime + pTurnTable->u16TurnInErr;

		if(TurnWallState == OFF)
		{
			pEdgeCnt = &(pTurnTable->pTurnInEdge->u16EdgeOnTick);
			while(!(pTurnTable->pTurnInEdge->u16EdgeOn))
			{
				if(GoalPoleNoneF == ON)
					break;

				#ifdef TURNHOLD
				if(RightMotor.q19DistanceSum > _IQ19(250.0))
				{
					MOTOR_OFF;
					VFDPrintf("T F!");
					while(TRUE);
				}
				#endif
			}
			
		}
		else
		{
			pEdgeCnt = &(pTurnTable->pTurnInEdge->u16WallFallTick);
			while(pTurnTable->pTurnInSensor->q19Position < _IQ19(512.0))
				;
		}
		
		
	}
	else
	{
		RightMotor.q19DistanceSum = LeftMotor.q19DistanceSum = _IQ19(0.0);
		TurnInTime = pTurnTable->u16TurnInErr;
		pEdgeCnt = &(RightMotor.u16Tick);
		*pEdgeCnt = 0;
	}

	POS_ADJ_OFF;
	
	//Turn in Section	
	for(;(*pEdgeCnt) < TurnInTime;)
	{
		// 골에 폴대가 없을 경우....
		if(GoalPoleNoneF == ON)
		{
			if(RightMotor.q19DistanceSum > _IQ19(185.0))
			{
				//TR_LED_ON;
				//TL_LED_ON;
				break;
			}
			else
				;
		}
		else
			;

		#ifndef SMOOTHTEST
		if((gUserTurnSpeed == SMOOTH680) && ((RFS.q19Position < FRONT_WALL_VALUE) || (LFS.q19Position < FRONT_WALL_VALUE)))
		{
			TR_LED_ON;
			TL_LED_ON;
			break;
		}
			
		
		#endif
	}
	

//TurnIn Time test...진입거리///////////////////////////////////////////////
//
/*
	RightMotor.i32Accel = LeftMotor.i32Accel = 4000;
	MoveStop(QUP(ONEBLOCK,19), QUP(gUserSpeed,17),QUP(ONEBLOCK,19), QUP(gUserSpeed,17));
	while(TRUE) 
		NULL;
*/
////////////////////////////////////////////////////////////////////
//test end---------
	RightMotor.i32Accel = pTurnTable->i32RightAccel;
	LeftMotor.i32Accel  = pTurnTable->i32LeftAccel;
	RightMotor.q17UserVelocity = pTurnTable->q17RightAccelVel;
	LeftMotor.q17UserVelocity  = pTurnTable->q17LeftAccelVel;

	//Turn Accel Section
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16AccelTime;)
		;


	//Turn Uniform Section
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16TurnTime;)
	{
		if(FrontWallState0 == OFF)
		{	
			if(gUserTurnSpeed == SMOOTH680)
			{
				if((RightMotor.u16Tick > (pTurnTable->u16EdgeTick0 - 50)) && !TurnEdge && (pTurnTable->pTurnEdgeSen->q19LPFOutDataDiff > _IQ19(0.4)))
				{
					TurnEdge = ON;
					#ifdef SMOOTHTEST
					Temp = RightMotor.u16Tick;
					#else
					RightMotor.u16Tick = pTurnTable->u16EdgeTick0;
					#endif

				}
				else if(!TurnEdge && !GoalPoleNoneF)
				{
					#ifndef SMOOTHTEST
					if(RightMotor.u16Tick > pTurnTable->u16TurnTime - 10)
						RightMotor.u16Tick -= 15;
					else
						;
					#endif
				}
				
				
			}
			else // gUserTurnSpeed == SMOOTH800 or above
			{
				if((RightMotor.u16Tick > (pTurnTable->u16EdgeTick0 - 50)) && !TurnEdge && (pTurnTable->pTurnEdgeSen->q19Position < _IQ19(510.0)))
				{
					TurnEdge = ON;
					#ifdef SMOOTHTEST
					Temp = RightMotor.u16Tick;
					#else
					RightMotor.u16Tick = pTurnTable->u16EdgeTick0;
					#endif
				}
				else
					;
			}
			
		}
		else// FrontWallState0 == ON
		{
			if((RightMotor.u16Tick > (pTurnTable->u16EdgeTick1 - 50)) && !TurnEdge && (pTurnTable->pTurnFEdgeSen->q19LPFOutDataDiff < _IQ19(0.0)) &&
			(pTurnTable->pTurnFEdgeSen->q19Position < _IQ19(150.0)))
			{
				TurnEdge = ON;
				#ifdef SMOOTHTEST
				Temp = RightMotor.u16Tick;
				#else
				
				if(FrontWallState1 == OFF)
					RightMotor.u16Tick = pTurnTable->u16EdgeTick1;
				else 
					RightMotor.u16Tick = pTurnTable->u16EdgeTick2;	

				#endif
			}
			else 
				;
		}
	}
	//Turn Decel Section

	RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = pTurnTable->q17RefVel;
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16AccelTime;)
		;

	RightMotor.i32Accel = LeftMotor.i32Accel = gUserAccel;
	RightMotor.u16Tick = 0;

	TR_LED_OFF;
	TL_LED_OFF;

#ifdef SMOOTHTEST
	
	for(;RightMotor.u16Tick < pTurnTable->u16TurnOutTime;)
		;

	VFDPrintf("%4d", Temp);

#else

	//첫 블럭 대각 처리..
	if(gFisrtBlockDiagF == ON)
	{
		gUserTurnSpeed = gUserTurnSpeedBackup;
		gUserSpeed = gUserSpeedBackup;
		gFisrtBlockDiagF = OFF;
	}
	else
		;


	if(gAlgoState == ON)
	{
		Algorithm(WallCheck(pRSS, pLSS));
		NextDir = KnowBlockPath[0].PathState;
	}
	else
		NextDir = KnowBlockPath[gPathBufferHead + 1].PathState;

	switch(NextDir)
	{
		case STRAIGHT:
			MoveToMove(gUserSpeed, gUserSpeed, ONEBLOCK);
			gBlockToBlock = TURN2STRT;
			break;

		case R90:
		case L90:
			gBlockToBlock = TURN2TURN;
			break;
			
		case BACKTURN:
			gBlockToBlock = TURN2BTURN;
			break;

		default:
			gBlockToBlock = 0xff;
			break;

	}
	//Turn out Setion
	for(; RightMotor.u16Tick < pTurnTable->u16TurnOutTime;)
	{
		if((gBlockToBlock == TURN2TURN) && (gUserTurnSpeed == SMOOTH680) && ((RFS.q19Position < FRONT_WALL_VALUE) || (LFS.q19Position < FRONT_WALL_VALUE)))
			break;
	}

	POS_ADJ_ON;
	ANGLE_ADJ_ON;
	
	
#endif
		
}

void Diag180Turn(TurnDebugVar Debug)
{

#ifdef SMOOTHTEST
	Uint16	Temp = 0;
#endif
	Uint16	Direction;
	Uint16	Position;
	Uint16	MouseDir;
	Uint16	TurnEdge = OFF;
	//Uint16	NextDir;
	Uint16	WallInfo;
	Uint16	FrontWallState;
	//Uint16	TurnWallState;


	gBackTurnFrontAdjState = OFF;
	gDiagDirectAdjState = OFF;
	gEdgeDiffAdjustFlag = OFF;
	Direction = KnowBlockPath[gPathBufferHead].PathState;
	Position = KnowBlockPath[gPathBufferHead].Position;
	MouseDir = KnowBlockPath[gPathBufferHead].MouseDir;
	//NextDir = KnowBlockPath[gPathBufferHead + 1].PathState;
	pTurnTable = (TurnInfoVariable *)&TurnTable[TurnTableIndex[gUserTurnSpeed]][Direction];

	WallInfo = gMazeMap[Position] & 0x0f;
	FrontWallState = WallInfo & WallTable[MouseDir][0];
	
#ifdef SMOOTHTEST

	FrontWallState = Debug.Fwall0; 
#endif

	//사이드 에지까지 대기
	while(pTurnTable->pTurnInSensor->q19Position < _IQ19(512.0))
		;
	POS_ADJ_OFF;

	
	//Turn In
	for(; pTurnTable->pTurnInEdge->u16WallFallTick < pTurnTable->u16TurnInTime;)
		;

	RightMotor.i32Accel = pTurnTable->i32RightAccel;
	LeftMotor.i32Accel = pTurnTable->i32LeftAccel;
	RightMotor.q17UserVelocity = pTurnTable->q17RightAccelVel;
	LeftMotor.q17UserVelocity = pTurnTable->q17LeftAccelVel;

	//Turn Accel Section
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16AccelTime;)
		;

	//Turn Uniform Section
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16TurnTime;)
	{		
		if(FrontWallState == OFF)
		{
			if((RightMotor.u16Tick > (pTurnTable->u16EdgeTick0 - 50)) && !TurnEdge && (pTurnTable->pTurnEdgeSen->q19Position < _IQ19(512.0)))
			{
				TurnEdge = ON;
				#ifdef SMOOTHTEST
				Temp = RightMotor.u16Tick;
				#else
				RightMotor.u16Tick = pTurnTable->u16EdgeTick0;
				#endif
			}
			else
				;
		}
		else
			;
				
	}
	
	
	//Turn Decel Section
	RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = pTurnTable->q17RefVel;
	
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16AccelTime;)
		;
	
	RightMotor.i32Accel = LeftMotor.i32Accel = gUserAccel;


#ifdef SMOOTHTEST

	VFDPrintf("%4d",Temp);

#else

	POS_ADJ_ON;
	ANGLE_ADJ_ON;
        
        if(gUserTurnSpeed == SMOOTH1300)
                gUserSpeed = 1300;

	MoveToMove(gUserSpeed, gUserSpeed, ONEBLOCK);
	
#endif

}
void Diag45_135TurnIn(TurnDebugVar Debug)
{
#ifdef SMOOTHTEST
	Uint16	Temp = 0;
	Uint16	InEdgeUp = OFF;	
	volatile SensorVariable *pSen;
	//Uint32	GyroAngleCheck = 0;
#endif
	Uint16	Direction;
	Uint16	MouseDir;
	Uint16	Position;
	Uint16	NextDir;
	Uint16	WallInfo;
	Uint16	FrontWallState;
	Uint16	TurnWallState;
	Uint16	TurnEdge = OFF;
	//Uint16	GyroAngleAdjF = OFF;
	volatile Uint16 *pEdgeCnt;

	gBackTurnFrontAdjState = OFF;
	gDiagDirectAdjState = OFF;
	gEdgeDiffAdjustFlag = OFF;
	
	Direction = KnowBlockPath[gPathBufferHead].PathState;
	MouseDir = KnowBlockPath[gPathBufferHead].MouseDir;
	Position = KnowBlockPath[gPathBufferHead].Position;
	NextDir = KnowBlockPath[gPathBufferHead + 1].PathState;
	
	pTurnTable = (TurnInfoVariable *)&TurnTable[TurnTableIndex[gUserTurnSpeed]][Direction];

	//R45IN, L45IN, R135IN, L135IN 턴 할때 앞벽의 유무 --> 턴에지 종류 판별
	WallInfo = gMazeMap[Position] & 0x0f;
	FrontWallState = WallInfo & WallTable[MouseDir][0];
	
	//턴 진입전 벽인지 에지인지 
	WallInfo = gMazeMap[Position + gMoveTable[(MouseDir + 2) & 0x03]] & 0x0f;
	if((Direction & 0x01) == OFF)//Right
		TurnWallState = WallInfo & WallTable[MouseDir][1];
		
	else//Left
		TurnWallState = WallInfo & WallTable[MouseDir][2];

	
#ifdef SMOOTHTEST

	FrontWallState = Debug.Fwall0;
	TurnWallState = Debug.Twall0;
	NextDir = RDRUN;
#endif

#ifdef TURNHOLD
		RightMotor.q19DistanceSum = _IQ19(0.0);
#endif


	// 135 Edge 진입은 없다.	
	if((TurnWallState == OFF) && ((Direction == R45IN) || (Direction == L45IN)))
	{
		pEdgeCnt = &(pTurnTable->pTurnInEdge->u16EdgeOnTick);
		while(!(pTurnTable->pTurnInEdge->u16EdgeOn))
		{
			
			#ifdef TURNHOLD
			if(RightMotor.q19DistanceSum > _IQ19(250.0))
			{
				MOTOR_OFF;
				VFDPrintf("T F!");
				while(TRUE);
			}
			#endif
		}
	}
	else
	{
		pEdgeCnt = &(pTurnTable->pTurnInEdge->u16WallFallTick);
		while(pTurnTable->pTurnInSensor->q19Position < _IQ19(512.0))
			;
	}
	
	POS_ADJ_OFF;
	
	//Turn In
	for(;(*pEdgeCnt) < pTurnTable->u16TurnInTime;)
			;

	RightMotor.i32Accel = pTurnTable->i32RightAccel;
	LeftMotor.i32Accel = pTurnTable->i32LeftAccel;
	RightMotor.q17UserVelocity = pTurnTable->q17RightAccelVel;
	LeftMotor.q17UserVelocity = pTurnTable->q17LeftAccelVel;
	
	//Turn Accel Section
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16AccelTime;)
		;

	//Turn Uniform Section
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16TurnTime;)
	{
		//45도 진입 턴보정...
		// 무조건 자이로 보정
		if((Direction == R45IN) || (Direction == L45IN))
			;//GyroAngleAdjF = ON;
		
		else// if((Direction == R135IN) || (Direction == L135IN))
		{
			//135도 진입 앞벽 없는경우
			if(FrontWallState == OFF)
			{
				if((RightMotor.u16Tick > (pTurnTable->u16EdgeTick0 - 50)) && !TurnEdge && (pTurnTable->pTurnEdgeSen->q19LPFOutDataDiff > _IQ19(0.2)))
				{
					TurnEdge = ON;
					#ifdef SMOOTHTEST
					Temp = RightMotor.u16Tick;
					#else
					RightMotor.u16Tick = pTurnTable->u16EdgeTick0;
					#endif
					
				}
				else
					;
			}
			//135도 진입 앞벽이 있는경우
			else
			{
				if((RightMotor.u16Tick > (pTurnTable->u16EdgeTick1 - 50)) && !TurnEdge && (pTurnTable->pTurnFEdgeSen->q19Position < _IQ19(250.0)) 
					&& (pTurnTable->pTurnFEdgeSen->q19LPFOutDataDiff< _IQ19(0.0)))
				{
					TurnEdge = ON;
					#ifdef SMOOTHTEST
					Temp = RightMotor.u16Tick;
					#else
					RightMotor.u16Tick = pTurnTable->u16EdgeTick1;
					#endif
				}
				else
					;
			}
			
		}
	}
		
	//Turn Decel Section
	RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = pTurnTable->q17RefVel;
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16AccelTime;)
		;
	RightMotor.i32Accel = LeftMotor.i32Accel = gUserAccel;

#ifndef SMOOTHTEST
	gBackTurnFrontAdjState == OFF;
	gEdgeDiffAdjustFlag = OFF;
	gDiagDirectAdjState = ON;
	gDiagTurnOutAdjF = ON;
	gDiagTurnOutREdgeAdjF = OFF;
	gDiagTurnOutLEdgeAdjF = OFF;
	POS_ADJ_ON;
	ANGLE_ADJ_OFF;
	//ANGLE_ADJ_ON;


	//다음턴이 코브라 턴인경우
	//속도를 680으로 낮춘다.
	if((NextDir >= RCbr45OUT) && (NextDir <= LCbr135OUT))
	{
		RightMotor.i32Accel = LeftMotor.i32Accel = 8000;
		gUserSpeed = 680;
		gUserTurnSpeed = SMOOTH680;
		RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = _IQ17(680.0);
		RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(680.0);
	}
	else
		;

	//첫블럭 대각처리
	if(gFisrtBlockDiagF == ON)
	{
		gUserTurnSpeed = gUserTurnSpeedBackup;
		gUserSpeed = gUserSpeedBackup;
		gFisrtBlockDiagF = OFF;
	}
	else
		;

	if(gUserTurnSpeed == SMOOTH1100)
	{
		gUserSpeed = 1100;
		RightMotor.i32Accel = LeftMotor.i32Accel = 10000;
		RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = _IQ17(1100.0);
		RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(1100.0);
	}
	else if(gUserTurnSpeed == SMOOTH1220)
	{
		gUserSpeed = 1220;
		RightMotor.i32Accel = LeftMotor.i32Accel = 10000;
		RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = _IQ17(1220.0);
		RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(1220.0);
	}
	else if(gUserTurnSpeed == SMOOTH1300)
	{
		RightMotor.i32Accel = LeftMotor.i32Accel = 12000;
			
		if((NextDir == R135OUT) || (NextDir == L135OUT))
		{
			gUserSpeed = 1220;
			RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = _IQ17(1220.0);
			RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(1220.0);
		}
		else if((NextDir == RD90) || (NextDir == LD90))
		{
			gUserSpeed = 1220;
			RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = _IQ17(1220.0);
			RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(1220.0);
		}
		else if((NextDir == RDRUN) || (NextDir == LDRUN))
		{
			gUserSpeed = 1300;
		}
		else
		{
			gUserSpeed = 1300;
			RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = _IQ17(1300.0);
			RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(1300.0);
		}
		
		
	}
	
#else

	if((Direction == R45IN) || (Direction == R135IN))
		pSen = pLDS;
	else if((Direction == L45IN) || (Direction == L135IN))
		pSen = pRDS;

	InEdgeUp = OFF;
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16TurnOutTime;)
	{
		if(((pSen->q19Position < _IQ19(200.0)) || ((pSen->q19Position< _IQ19(400.0)) && (pSen->q19LPFOutDataDiff > _IQ19(0.5)))) && !InEdgeUp)
			InEdgeUp = ON;
		else if((InEdgeUp == ON) && ( pSen->q19LPFOutDataDiff < _IQ19(0.0)))
		{
			//TR_LED_ON;
			//TL_LED_ON;
			break;
		}
	}

	VFDPrintf("%4d",Temp);

#endif	

}
void Diag45_135TurnOut(TurnDebugVar Debug)   
{
#ifdef SMOOTHTEST
	Uint16	Temp = 0;
	Uint32  GyroAngleCheck = 0;
#endif
	Uint16	InEdgeUp = OFF;
	//Uint16	TurnEdgeUp = OFF;
	Uint16	TurnEdgeState = OFF;
	Uint16	DiagFWallState;
	Uint16	DiagFSecWallState;
	Uint16	Direction;
	//Uint16	YetDir;
	Uint16	Position;
	Uint16	MouseDir;
	Uint16	NextDir;
	Uint16	WallInfo;
	Uint16 	GyroAngleAdjF = OFF;
	
#ifndef SMOOTHTEST
	POS_ADJ_ON;
	ANGLE_ADJ_OFF;
	gBackTurnFrontAdjState = OFF;
	gEdgeDiffAdjustFlag = OFF;
	gDiagDirectAdjState = ON;
	gDiagTurnOutAdjF = ON;

#endif

	Direction = KnowBlockPath[gPathBufferHead].PathState;
	Position = KnowBlockPath[gPathBufferHead].Position;
	MouseDir = KnowBlockPath[gPathBufferHead].MouseDir;
	NextDir = KnowBlockPath[gPathBufferHead + 1].PathState; 
	pTurnTable = (TurnInfoVariable *)&TurnTable[TurnTableIndex[gUserTurnSpeed]][Direction];

	if((Direction == R45OUT) || (Direction == R135OUT))
	{
		gDiagTurnOutREdgeAdjF = ON;
		gDiagTurnOutLEdgeAdjF = OFF;
	}
	else
	{
		gDiagTurnOutREdgeAdjF = OFF;
		gDiagTurnOutLEdgeAdjF = ON;
	}
		
	WallInfo = gMazeMap[Position] & 0x0f;
	if(Direction == R45OUT)
		DiagFWallState = WallInfo & WallTable[MouseDir][2];
	else if(Direction == L45OUT)	
		DiagFWallState = WallInfo & WallTable[MouseDir][1];
	
	else //if((direction == R135OUT) || (direction == L135OUT))
	{
		DiagFWallState = WallInfo & WallTable[MouseDir][0];
		if(Direction == R135OUT)
			WallInfo = gMazeMap[Position + gMoveTable[(MouseDir + 1) & 0x03]] & 0x0f;
		else
			WallInfo = gMazeMap[Position + gMoveTable[(MouseDir + 3) & 0x03]] & 0x0f;
		
		DiagFSecWallState = WallInfo & WallTable[MouseDir][0];
	}
		
#ifdef SMOOTHTEST

	DiagFWallState = Debug.Fwall0;
	DiagFSecWallState = Debug.Fwall1;
	NextDir = NMATCH;
	
#endif

	InEdgeUp = OFF;

#ifdef TURNHOLD
	RightMotor.q19DistanceSum = _IQ19(0.0);
#endif

	while(TRUE)//턴 진입 시점
	{
		if(((pTurnTable->pTurnInSensor->q19Position < _IQ19(230.0)) || ((pTurnTable->pTurnInSensor->q19Position < _IQ19(450.0)) 
			&& (pTurnTable->pTurnInSensor->q19LPFOutDataDiff > _IQ19(0.2)))) && !InEdgeUp)
			InEdgeUp = ON;
		else if((InEdgeUp == ON) && (pTurnTable->pTurnInSensor->q19LPFOutDataDiff < _IQ19(0.0)))
			break;

#ifdef TURNHOLD	
		if(RightMotor.q19DistanceSum > _IQ19(300.0))
		{
			MOTOR_OFF;
			VFDPrintf("T F!");
			while(TRUE);
		}
		else
			;
#endif
	}

	//POS_ADJ_OFF;
	
	//Turn In
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < (pTurnTable->u16TurnInTime);)
	{
		if((pTurnTable->u16TurnInTime - 20) < RightMotor.u16Tick)
		{
			POS_ADJ_OFF;
		}
		else
			;
	}

	RightMotor.i32Accel = pTurnTable->i32RightAccel;
	LeftMotor.i32Accel = pTurnTable->i32LeftAccel;
	RightMotor.q17UserVelocity = pTurnTable->q17RightAccelVel;
	LeftMotor.q17UserVelocity = pTurnTable->q17LeftAccelVel;

	q13GyroAngle = _IQ13(0.0);
	//Turn Accel Section
	for(RightMotor.u16Tick = LeftMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16AccelTime;)
		;

	//Turn Uniform Section
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16TurnTime;)
	{
		if((Direction == R45OUT) || (Direction == L45OUT))
		{	
		#ifdef SMOOTHTEST
			;
		#else
			
			GyroAngleAdjF = ON;
			
			if(GyroAngleAdjF && (QDW(q13GyroAngle, 13) >= pTurnTable->u32GyroAngle))
			{
				GyroAngleAdjF = OFF;
				break;
			}

		#endif
			
		}
		else //if((direction == R135OUT) || (direction == L135OUT))
		{
			if(DiagFWallState == OFF)
			{
				if((RightMotor.u16Tick > (pTurnTable->u16EdgeTick0 - 50)) && !TurnEdgeState && (pTurnTable->pTurnEdgeSen->q19Position < _IQ19(512.0)))
				{
					TurnEdgeState = ON;
					#ifdef SMOOTHTEST
					Temp = RightMotor.u16Tick;
					#else
					RightMotor.u16Tick = pTurnTable->u16EdgeTick0;
					#endif
				}
				else
					;
				
			}
			else
			{
				if((RightMotor.u16Tick > (pTurnTable->u16EdgeTick1 - 50)) && !TurnEdgeState && (pTurnTable->pTurnEdgeSen->q19Position < _IQ19(480.0))
					&& (pTurnTable->pTurnFEdgeSen->q19LPFOutDataDiff < _IQ19(0.0)))
				{
					
					TurnEdgeState = ON;
					#ifdef SMOOTHTEST	
					Temp = RightMotor.u16Tick;
					#else
					if(DiagFSecWallState == OFF)
						RightMotor.u16Tick = pTurnTable->u16EdgeTick1;
					else
						RightMotor.u16Tick = pTurnTable->u16EdgeTick2;
					#endif	
				}
				else
					;
				
			}
		}

	}

	#ifdef SMOOTHTEST
	GyroAngleCheck = QDW(q13GyroAngle, 13);
	#else
	while(GyroAngleAdjF && (QDW(q13GyroAngle, 13) < pTurnTable->u32GyroAngle))
		;
	#endif
	
	//Turn Decel Section

	RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = pTurnTable->q17RefVel;	
	for(RightMotor.u16Tick = LeftMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16AccelTime;)
		;

	RightMotor.i32Accel = LeftMotor.i32Accel = gUserAccel;

#ifndef SMOOTHTEST

	gBackTurnFrontAdjState = OFF;
	gEdgeDiffAdjustFlag = OFF;
	gDiagDirectAdjState = OFF;
	POS_ADJ_ON;
	ANGLE_ADJ_ON;

	if((NextDir == STRAIGHT) || (NextDir == BACKTURN))
	{
		if(gUserTurnSpeed == SMOOTH1300)
			gUserSpeed = 1300;
		else if(gUserTurnSpeed == SMOOTH1220)
			gUserSpeed = 1220;
		else if(gUserTurnSpeed == SMOOTH1100)
			gUserSpeed = 1100;
		
		MoveToMove(gUserSpeed, gUserSpeed, ONEBLOCK);
	}
	else 
	{
		MOTOR_OFF;
		VFDPrintf("E45O");
		while(TRUE)
			;
	}

#else
	if((Direction == R45OUT) || (Direction == L45OUT))
		VFDPrintf("%4lx",GyroAngleCheck);
	else
		VFDPrintf("%4d",Temp);
#endif

}



void CobraTurn45_135Out(TurnDebugVar Debug)
{
	Uint16 	InEdgeUp = OFF;
	Uint16 	Direction;
	Uint16	NextDir;

#ifndef SMOOTHTEST
	POS_ADJ_ON;
	ANGLE_ADJ_OFF;
	gBackTurnFrontAdjState = OFF;
	gEdgeDiffAdjustFlag = OFF;
	gDiagDirectAdjState = ON;
	gDiagTurnOutAdjF = ON;
	gDiagTurnOutREdgeAdjF = OFF;
	gDiagTurnOutLEdgeAdjF = OFF;
#endif

	
	Direction = KnowBlockPath[gPathBufferHead].PathState;	
	pTurnTable = (TurnInfoVariable *)&TurnTable[TurnTableIndex[gUserTurnSpeed]][Direction];

#ifdef SMOOTHTEST 
	POS_ADJ_OFF;
#endif
		
	InEdgeUp = OFF;
	while(TRUE)//턴 진입 시점
	{
		if(((pTurnTable->pTurnInSensor->q19Position < _IQ19(230.0)) || ((pTurnTable->pTurnInSensor->q19Position < _IQ19(450.0)) 
			&& (pTurnTable->pTurnInSensor->q19LPFOutDataDiff > _IQ19(0.2)))) && !InEdgeUp)
			InEdgeUp = ON;
		else if((InEdgeUp == ON) && (pTurnTable->pTurnInSensor->q19LPFOutDataDiff < _IQ19(0.0)))
			break;
	}
	POS_ADJ_OFF;

	for(RightMotor.u16Tick = LeftMotor.u16Tick = 0; RightMotor.u16Tick < (pTurnTable->u16TurnInTime);)
		;

	RightMotor.i32Accel = pTurnTable->i32RightAccel;
	LeftMotor.i32Accel = pTurnTable->i32LeftAccel;
	RightMotor.q17UserVelocity = pTurnTable->q17RightAccelVel;
	LeftMotor.q17UserVelocity = pTurnTable->q17LeftAccelVel;
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < (pTurnTable->u16AccelTime);)
		;

	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < (pTurnTable->u16TurnTime);)
		;
	
	RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = pTurnTable->q17RefVel;
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < (pTurnTable->u16AccelTime);)
		;

	RightMotor.i32Accel = LeftMotor.i32Accel = gUserAccel;
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < (pTurnTable->u16TurnOutTime);)
		;

//-------------------------------------------------

	pTurnCbrTable = (TurnInfoVariable *)&TurnCbrTable[Direction - 16];

	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < (pTurnCbrTable->u16TurnInTime);)
		;

	RightMotor.i32Accel    = pTurnCbrTable->i32RightAccel;
	LeftMotor.i32Accel     = pTurnCbrTable->i32LeftAccel;
	RightMotor.q17UserVelocity = pTurnCbrTable->q17RightAccelVel;
	LeftMotor.q17UserVelocity = pTurnCbrTable->q17LeftAccelVel;
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < (pTurnCbrTable->u16AccelTime);)
		;

	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < (pTurnCbrTable->u16TurnTime);)
		;
	
	RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = pTurnCbrTable->q17RefVel;
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < (pTurnCbrTable->u16AccelTime);)
		;

	RightMotor.i32Accel = LeftMotor.i32Accel = gUserAccel;
	

#ifndef SMOOTHTEST

	RightMotor.u16Tick = 0;

	Algorithm(WallCheck(pRSS,pLSS));
	NextDir = KnowBlockPath[0].PathState;

	switch(NextDir)
	{
		case STRAIGHT:
			MoveToMove(gUserSpeed, gUserSpeed, ONEBLOCK);
			gBlockToBlock = TURN2STRT;
			break;

		case R90:
		case L90:
			gBlockToBlock = TURN2TURN;
			break;
			
		case BACKTURN:
			gBlockToBlock = TURN2BTURN;
			break;
		default:
			gBlockToBlock = 0xff;
			break;

	}

	for(; RightMotor.u16Tick < (pTurnCbrTable->u16TurnOutTime);)
		;

	gBackTurnFrontAdjState = OFF;
	gEdgeDiffAdjustFlag = OFF;
	gDiagDirectAdjState = OFF;
	POS_ADJ_ON;
	ANGLE_ADJ_ON;

#else

	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < (pTurnCbrTable->u16TurnOutTime);)
		;

#endif
}


void Diag90Turn(TurnDebugVar Debug)
{
#ifdef SMOOTHTEST
	Uint16	Temp = 0;
	volatile SensorVariable *pSen;
#endif
	Uint16	InEdgeUp = OFF;
	Uint16	TurnEdge = OFF;
	Uint16	TurnEdgeUp = OFF;
	Uint16	Direction;
	Uint16	Position;
	Uint16	MouseDir;
	Uint16	FrontWallState;
	Uint16	WallInfo;
	Uint16	NextDir;

#ifndef SMOOTHTEST
	POS_ADJ_ON;
	ANGLE_ADJ_OFF;
	gBackTurnFrontAdjState = OFF;
	gEdgeDiffAdjustFlag = OFF;
	gDiagDirectAdjState = ON;
	gDiagTurnOutAdjF = ON;
#endif
	
	Direction = KnowBlockPath[gPathBufferHead].PathState;
	Position = KnowBlockPath[gPathBufferHead].Position;
	MouseDir = KnowBlockPath[gPathBufferHead].MouseDir;
	NextDir = KnowBlockPath[gPathBufferHead + 1].PathState;
	pTurnTable = (TurnInfoVariable *)&TurnTable[TurnTableIndex[gUserTurnSpeed]][Direction];

	if(Direction == RD90)
	{
		gDiagTurnOutREdgeAdjF = ON;
		gDiagTurnOutLEdgeAdjF = OFF;
	}
	else
	{
		gDiagTurnOutREdgeAdjF = OFF;
		gDiagTurnOutLEdgeAdjF = ON;
	}

	WallInfo = gMazeMap[Position] & 0x0f;
	FrontWallState = WallInfo & WallTable[MouseDir][0];
	
#ifdef SMOOTHTEST

	FrontWallState = Debug.Fwall0;

#endif

	InEdgeUp = OFF;

#ifdef TURNHOLD
	RightMotor.q19DistanceSum = _IQ19(0.0);
#endif

	while(TRUE)//턴 진입 시점
	{
		if(((pTurnTable->pTurnInSensor->q19Position < _IQ19(230.0)) || ((pTurnTable->pTurnInSensor->q19Position < _IQ19(450.0)) 
			&& (pTurnTable->pTurnInSensor->q19LPFOutDataDiff > _IQ19(0.2)))) && !InEdgeUp)
			InEdgeUp = ON;
		else if((InEdgeUp == ON) && (pTurnTable->pTurnInSensor->q19LPFOutDataDiff < _IQ19(0.0)))
			break;
		
#ifdef TURNHOLD
		if(RightMotor.q19DistanceSum > _IQ19(300.0))
		{
			MOTOR_OFF;
			VFDPrintf("T F!");
			while(TRUE);
		}
		else
			;
#endif
	}
	

	//Turn In
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < (pTurnTable->u16TurnInTime);)
	{
		if((pTurnTable->u16TurnInTime - 10) < RightMotor.u16Tick)
		{
			POS_ADJ_OFF;
		}
		else
			;
	}

	RightMotor.i32Accel = pTurnTable->i32RightAccel;
	LeftMotor.i32Accel = pTurnTable->i32LeftAccel;
	RightMotor.q17UserVelocity = pTurnTable->q17RightAccelVel;
	LeftMotor.q17UserVelocity = pTurnTable->q17LeftAccelVel;
	//Turn Accel Section
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16AccelTime;)
		;


	//Turn Uniform Section
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16TurnTime;)
	{
		if(FrontWallState == OFF)
		{
			
			if((pTurnTable->pTurnEdgeSen->q19LPFOutDataDiff > _IQ19(0.2)) && !TurnEdgeUp)
				TurnEdgeUp = ON;
			else if((RightMotor.u16Tick > (pTurnTable->u16EdgeTick0 - 30)) && (TurnEdgeUp == ON) && !TurnEdge 
				&& ( pTurnTable->pTurnEdgeSen->q19LPFOutDataDiff < _IQ19(0.0)))
			{
				TurnEdge = ON;
				#ifdef SMOOTHTEST
				Temp = RightMotor.u16Tick;
				#else
				RightMotor.u16Tick = pTurnTable->u16EdgeTick0;
				#endif
			}
			
			
		}
		else
		{
			if((RightMotor.u16Tick > (pTurnTable->u16EdgeTick1 - 30)) && !TurnEdge && (pTurnTable->pTurnFEdgeSen->q19LPFOutDataDiff < _IQ19(0.0)) 
				&& (pTurnTable->pTurnFEdgeSen->q19Position < _IQ19(200.0)))
			{
				TurnEdge = ON;
				#ifdef SMOOTHTEST
				Temp = RightMotor.u16Tick;
				#else
				RightMotor.u16Tick = pTurnTable->u16EdgeTick1;
				#endif
			}
		}
		
		
	}
	//Turn Decel Section

	RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = pTurnTable->q17RefVel;
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16AccelTime;)
		;
	RightMotor.i32Accel = LeftMotor.i32Accel = gUserAccel;

#ifndef SMOOTHTEST
	gBackTurnFrontAdjState = OFF;
	gEdgeDiffAdjustFlag = OFF;
	gDiagDirectAdjState = ON;
	gDiagTurnOutAdjF = ON;
	gDiagTurnOutREdgeAdjF = OFF;
	gDiagTurnOutLEdgeAdjF = OFF;
	POS_ADJ_ON;
	ANGLE_ADJ_OFF;
	//ANGLE_ADJ_ON;

	//다음턴이 코브라 턴인경우
	//속도를 680으로 낮춘다.
	if((NextDir >= RCbr45OUT) && (NextDir <= LCbr135OUT))
	{
		RightMotor.i32Accel = LeftMotor.i32Accel = 10000;
		gUserSpeed = 680;
		gUserTurnSpeed = SMOOTH680;
		RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = _IQ17(680.0);
		RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(680.0);
	}
	else
		;


	if(gUserTurnSpeed == SMOOTH1300)
	{
		RightMotor.i32Accel = LeftMotor.i32Accel = 12000;
			
		if((NextDir == R135OUT) || (NextDir == L135OUT))
		{
			gUserSpeed = 1220;
			RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = _IQ17(1220.0);
			RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(1220.0);			
		}
		else if((NextDir == RD90) || (NextDir == LD90))
		{
			gUserSpeed = 1220;
			RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = _IQ17(1220.0);
			RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(1220.0);			
		}
		else if((NextDir == RDRUN) || (NextDir == LDRUN))
		{
			gUserSpeed = 1300;
		}
		else
		{
			gUserSpeed = 1300;
			RightMotor.q17UserVelocity = LeftMotor.q17UserVelocity = _IQ17(1300.0);
			RightMotor.q17DecelVelocity = LeftMotor.q17DecelVelocity = _IQ17(1300.0);			
		}
	}

#else

	if(Direction == RD90)
		pSen = pLSS;
	else if(Direction == LD90)
		pSen = pRSS;

	InEdgeUp = OFF;
	for(RightMotor.u16Tick = 0; RightMotor.u16Tick < pTurnTable->u16TurnOutTime;)
	{
		if(((pSen->q19Position < _IQ19(200.0)) || ((pSen->q19Position< _IQ19(400.0)) && (pSen->q19LPFOutDataDiff > _IQ19(0.5)))) && !InEdgeUp)
			InEdgeUp = ON;
		else if((InEdgeUp == ON) && ( pSen->q19LPFOutDataDiff < _IQ19(0.0)))
		{
			TR_LED_ON;
			TL_LED_ON;
			break;
		}
		
	}
	
	VFDPrintf("%4d",Temp);

#endif

}
void DiagBlockRun(TurnDebugVar Debug)
{
	Uint16	RunBlockCnt;
	Uint16	NextDir;
	int16	AccelVel;
	_iq19	RunLength;
	_iq19 	DiagInDis;

	AccelVel = gDiagSpeedLimit;

	POS_ADJ_ON;
	ANGLE_ADJ_ON;
	
	gBackTurnFrontAdjState == OFF;
	gEdgeDiffAdjustFlag = OFF;
	gDiagDirectAdjState = ON;	
	gDiagTurnOutAdjF = OFF;
	gDiagTurnOutREdgeAdjF = OFF;
	gDiagTurnOutLEdgeAdjF = OFF;

	RunBlockCnt = KnowBlockPath[gPathBufferHead].PathCnt;
	NextDir = KnowBlockPath[gPathBufferHead + 1].PathState;

	//다음턴이 코브라 턴인경우
	//속도를 680으로 낮춘다.
	if((NextDir >= RCbr45OUT) && (NextDir <= LCbr135OUT))
	{
		gUserSpeed = 680;
		gUserTurnSpeed = SMOOTH680;
	}
	else
		;

	if(gUserTurnSpeed == SMOOTH1300)
	{
		if((NextDir == R135OUT) || (NextDir == L135OUT))
		{
			gUserSpeed = 1220;
		}
		else if((NextDir == RD90) || (NextDir == LD90))
		{
			gUserSpeed = 1220;
		}
		else
		{
			gUserSpeed = 1300;
		}
	}
	else if(gUserTurnSpeed == SMOOTH1220)
	{
		gUserSpeed = 1220;
	}
	else if(gUserTurnSpeed == SMOOTH1100)
	{
		gUserSpeed = 1100;
	}

	DiagInDis = _IQ19(58.0);
	
	if(RunBlockCnt > 1)
	{
		RunLength = _IQ19mpyIQX(QUP(RunBlockCnt,4),4,ONEDIAGBLOCK,17) - _IQ19mpyIQX(QUP(RunBlockCnt,4),4, _IQ17(0.3),17) - DiagInDis;
		
		RightMotor.i32Accel = LeftMotor.i32Accel = gUserDiagAccel;
		MoveToMove(AccelVel, gUserSpeed, QDW(RunLength,19));

		while((RightMotor.q19DistanceSum < (RunLength - _IQ19(15.0))) && (LeftMotor.q19DistanceSum < (RunLength - _IQ19(15.0))))
			;
		
	}
	else
	{
		RightMotor.i32Accel = LeftMotor.i32Accel = gUserAccel;
		MoveToMove(gUserSpeed,gUserSpeed,60);
		
		while(RightMotor.q19DistanceSum < _IQ19(60.0))
			;
	}

	gDiagDirectAdjState = OFF;
	gDiagTurnOutAdjF = ON;

}

Uint16 WallCheck(volatile SensorVariable *pRSide, volatile SensorVariable *pLSide)
{
	Uint16	WallInfo;
	Uint16	RightWallState = OFF;
	Uint16	LeftWallState  = OFF;
	Uint16	FrontWallState = OFF;

	if(pRSide->q19LPFOutData > pRSide->q19MinVal)
		RightWallState  = ON;

	if(pLSide->q19LPFOutData > pLSide->q19MinVal)
		LeftWallState  = ON;

	if(((pRFS->q19Position < _IQ19(190.0)) && (pLFS->q19Position < _IQ19(190.0))))
		FrontWallState = ON;
	
	switch(gMouseDir)
	{	
	
	case M_N://				W_wall							E_wall						 N_wall
		WallInfo = ((LeftWallState << 3) & 0x08) + ((RightWallState << 1) & 0x02) + (FrontWallState & 0x01);
		break;

	case M_E://				S_wall							E_wall						 N_wall
		WallInfo = ((RightWallState << 2) & 0x04) + ((FrontWallState << 1) & 0x02) + (LeftWallState & 0x01);
		break;

	case M_S://				W_wall							S_wall						 E_wall
		WallInfo = ((RightWallState << 3) & 0x08) + ((FrontWallState << 2) & 0x04) + ((LeftWallState << 1) & 0x02); 
		break;

	case M_W://				W_wall							S_wall						 N_wall
		WallInfo = ((FrontWallState << 3) & 0x08) + ((LeftWallState << 2) & 0x04) + (RightWallState & 0x01);
		break;

	default:
		MOTOR_OFF;
		VFDPrintf("Werr ");
		while(TRUE) 
			;
	}

	return	(WallInfo & 0x0f);
}



