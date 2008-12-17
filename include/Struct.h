#ifndef __STRUCT_H__
#define __STRUCT_H__

#ifdef	__STRUCT__
	#undef	__STRUCT__
	#define STRUCT_EXT
#else
	#define	STRUCT_EXT extern
#endif

#define VARIABLE_OPT volatile



typedef struct 
{
	Uint16	u16Value;
	
	_iq20	q20LPFOutData;
	_iq20	q20LPFInData;
	_iq20	q20AngleVelocity;

	_iq12	q12AngleRef;
}GyroVariable;

STRUCT_EXT volatile GyroVariable GyroVar;

typedef struct
{
	int16  	i16QepValue;
	Uint16 	u16QepSample;
	Uint16 	u16CapCnt;
	Uint16  StopFlag;
	Uint16  u16Tick;

	int32	i32Accel;

	_iq28	q28Kp;
	_iq28	q28Ki;
	_iq28	q28Kd;
	
	_iq27	q27TickDistance;

	_iq26	q26PosAdjRate;
	
	_iq19	q19DistanceSum;
	_iq19	q19UserDistance;
	_iq19	q19ErrDistance;
	_iq19	q19StopDistance;
	
	_iq17	q17CurrentVelocity[2];
	_iq17	q17CurrentVelAvr;
	_iq17	q17UserVelocity;
	_iq17	q17NextVelocity;
	_iq17	q17ErrVelocity[4];
	_iq17	q17DecelVelocity;

	_iq17	q17ErrVelocitySum;
	_iq17	q17ProportionalTerm;
	_iq17	q17DerivativeTerm;	
	_iq17	q17IntegralTerm;
	_iq17	q17PidOutTerm;

	
}MotorVal;

STRUCT_EXT volatile MotorVal RightMotor, LeftMotor;

struct Path
{
	Uint16	Position:8;
	Uint16	TurnDir:4;
	Uint16	MouseDir:4;
	Uint16	PathCnt:8;
	Uint16	PathState:8;
};

STRUCT_EXT volatile struct Path KnowBlockPath[256];

typedef volatile struct 
{
	_iq19	T90OutDis;
	_iq19	T45OutDis;
	_iq19	T135OutDis;
	_iq19	T180OutDis;	
}TurnOutDis;

typedef volatile struct 
{
	_iq19	T90InDis;
	_iq19	T45InDis;
	_iq19	T135InDis;
	_iq19	T180InDis;	
}TurnInDis;


typedef volatile struct 
{
	Uint16	Twall0:1;
	Uint16	Twall1:1;
	Uint16	Fwall0:1;
	Uint16	Fwall1:1;
	Uint16	RSV:12;
}TurnDebugVar;

typedef struct 
{
	_iq19	q19Position;
	_iq19	q19PositionYet;
	_iq19	q19PositionDiff;
	_iq19	q19HighCoefficient;
	_iq19	q19LowCoefficient;
	_iq19	q19MaxVal;
	_iq19	q19MinVal;
	_iq19	q19MidVal;
	_iq19	q19LPFOutDataYet;
	_iq19	q19LPFOutData;
	_iq19	q19LPFInData;
	_iq19	q19LPFOutDataDiff;
	_iq19	q19LPFInDataDiff;
	_iq19	q19LPFInDataDiffYet;
	
	Uint16	u16Value;
		
}SensorVariable;

extern volatile SensorVariable Sen[6];

typedef struct 
{
	Uint32	u32EdgeCheckTick;
	Uint16	u16EdgeState;
	Uint16	u16LimitCnt;
	Uint16	u16DelCnt;
	Uint16  u16WallCheckTick;
	Uint16	u16WallState;
	Uint16	u16EdgeOn;
	Uint16	u16EdgeOnTick;
	Uint16  u16WallFallTick;

}EdgeVariable;

extern volatile EdgeVariable RSideEdge,LSideEdge,RDiagEdge,LDiagEdge;

typedef struct 
{
	volatile EdgeVariable *pTurnInEdge;
	volatile SensorVariable *pTurnInSensor;
	
	Uint16		u16TurnInTime;
	Uint16		u16TurnInErr;
	Uint16		u16AccelTime;
	Uint16		u16TurnTime;
	Uint16		u16TurnOutTime;
	int32		i32RightAccel;
	int32		i32LeftAccel;
	_iq17		q17RightAccelVel;
	_iq17		q17LeftAccelVel;
	_iq17		q17RefVel;

	Uint16		u16EdgeTick0;
	Uint16		u16EdgeTick1;
	Uint16		u16EdgeTick2;

	volatile SensorVariable *pTurnEdgeSen;
	volatile SensorVariable *pTurnFEdgeSen;

	Uint32		u32GyroAngle;
}TurnInfoVariable;

extern volatile TurnInfoVariable TurnTable[4][20];


#endif



