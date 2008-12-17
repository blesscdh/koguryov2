//###########################################################################
//
// FILE:    sensor.c
//
// TITLE:   KOGURYO Mouse sensor c file.
//
//###########################################################################
// $Release Date: 2006.10.2 $
//###########################################################################

#include "DSP280x_Device.h"     // DSP280x Headerfile Include File
#include "DSP280x_Examples.h"   // DSP280x Examples Include File


#define ADC_RESULT			0x7108
													
#define RF_SEN_AD4		9	//ADB1
#define LF_SEN_AD0		0	//ADA0

#define RS_SEN_AD1		1	//ADA1
#define LS_SEN_AD2		2	//ADA2

#define R45_SEN_AD3		8	//ADB0
#define L45_SEN_AD5		10	//ADB2

#define BATT_AD		3	//ADA3
#define GYRO_AD		11	//ADB3


#define RF_SEN_AD4_SEQ_VAL		0x9999
#define LF_SEN_AD0_SEQ_VAL		0x0000

#define RS_SEN_AD1_SEQ_VAL		0x1111
#define LS_SEN_AD2_SEQ_VAL		0x2222

#define R45_SEN_AD3_SEQ_VAL		0x8888
#define L45_SEN_AD5_SEQ_VAL		0xaaaa

//sensor port
#define RF_SEN_SHOOT		11	//shoot_in04
#define LF_SEN_SHOOT		9	//shoot_in00
#define RS_SEN_SHOOT		5	//shoot_in01	
#define LS_SEN_SHOOT		8	//shoot_in02
#define R45_SEN_SHOOT		6	//shoot_in03
#define L45_SEN_SHOOT		7	//shoot_in05


//LOW PASS FILTER 필터 상수 및 계산 값...
#define 	M_PI				3.1415926536 


#define 	SENSOR_F_CUT		20//(20Hz)
#define		SENSOR_F_SAMPLE		2000//(2KHz)
#define		SENSOR_W_CUT		tan(M_PI * SENSOR_F_CUT / SENSOR_F_SAMPLE)

#define		SENSOR_Ka	    	_IQ30(-0.9390625058)//(W_CUT -1) / (W_CUT + 1)
#define		SENSOR_Kb			_IQ30(0.0304687471)// W_CUT / (W_CUT + 1)

#define 	SENSOR_F_CUT_DIFF		50//(50Hz)
#define		SENSOR_F_SAMPLE_DIFF	2000//(2KHz)
#define		SENSOR_W_CUT_DIFF		tan(M_PI * SENSOR_F_CUT_DIFF / SENSOR_F_SAMPLE_DIFF)

#define		SENSOR_Ka_DIFF	    	_IQ30(-0.8540806855)//(W_CUT -1) / (W_CUT + 1)
#define		SENSOR_Kb_DIFF			_IQ30(0.0729596573)// W_CUT / (W_CUT + 1)



#define		GYRO_F_CUT			10.0//cut of FREQ 10Hz
#define		GYRO_F_SAMPLE		20000//(20kHz)
#define		GYRO_W_CUT			tan(M_PI * GYRO_F_CUT / GYRO_F_SAMPLE)
#define		GYRO_Ka	    		_IQ30(-0.9968633318)
#define		GYRO_Kb				_IQ30(0.0015683341)

//IIR LPF
//Y(n) = Kb ((X(n) + X(n-1))) - Ka(Y(n-1))

volatile Uint16 SensorShootSEQ[6] = {LF_SEN_SHOOT
									,RS_SEN_SHOOT
									,L45_SEN_SHOOT
									,RF_SEN_SHOOT
									,LS_SEN_SHOOT
									,R45_SEN_SHOOT};

volatile Uint16 SensorADCSEQ[6] = {LF_SEN_AD0_SEQ_VAL
								  ,RS_SEN_AD1_SEQ_VAL
								  ,L45_SEN_AD5_SEQ_VAL
								  ,RF_SEN_AD4_SEQ_VAL
								  ,LS_SEN_AD2_SEQ_VAL
								  ,R45_SEN_AD3_SEQ_VAL};

interrupt void SensorTimer0ISR(void)	
{
	// Acknowledge this interrupt to receive more interrupts from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

	//==================================================================================//
	//---------------User Program------------------------------------------------------//
	//==================================================================================//

	AdcRegs.ADCCHSELSEQ1.all = SensorADCSEQ[gSensorChanel];
	AdcRegs.ADCCHSELSEQ2.all = SensorADCSEQ[gSensorChanel];
	AdcRegs.ADCCHSELSEQ3.all = SensorADCSEQ[gSensorChanel];
	AdcRegs.ADCCHSELSEQ4.all = SensorADCSEQ[gSensorChanel];

	AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;
	
	GpioDataRegs.GPASET.all = (ON << SensorShootSEQ[gSensorChanel]);

}
interrupt void SEQ1INT_ISR(void)   //SEQ1 ADC
{
	
	static Uint16 ADChanelCnt = 0;
	static Uint32 SenValSum = 0;
	_iq19	Buff;

	// To receive more interrupts from this PIE group, acknowledge this interrupt 
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; 

	//==============================================================================//
	//----------User Program-------------------------------------------------------//
	//==============================================================================//
	
	if(ADChanelCnt == 0)
		GpioDataRegs.GPACLEAR.all = (ON << SensorShootSEQ[gSensorChanel]);
	else
		;
	
	SenValSum += AdcMirror.ADCRESULT0;//ADC  result 12bit 유효값
	SenValSum += AdcMirror.ADCRESULT1;//ADC  result 12bit 유효값
	SenValSum += AdcMirror.ADCRESULT2;//ADC  result 12bit 유효값
	SenValSum += AdcMirror.ADCRESULT3;//ADC  result 12bit 유효값
	SenValSum += AdcMirror.ADCRESULT4;//ADC  result 12bit 유효값
	SenValSum += AdcMirror.ADCRESULT5;//ADC  result 12bit 유효값
	SenValSum += AdcMirror.ADCRESULT6;//ADC  result 12bit 유효값
	SenValSum += AdcMirror.ADCRESULT7;//ADC  result 12bit 유효값
	SenValSum += AdcMirror.ADCRESULT8;//ADC  result 12bit 유효값
	SenValSum += AdcMirror.ADCRESULT9;//ADC  result 12bit 유효값
	SenValSum += AdcMirror.ADCRESULT10;//ADC  result 12bit 유효값
	SenValSum += AdcMirror.ADCRESULT11;//ADC  result 12bit 유효값
	SenValSum += AdcMirror.ADCRESULT12;//ADC  result 12bit 유효값
	SenValSum += AdcMirror.ADCRESULT13;//ADC  result 12bit 유효값
	SenValSum += AdcMirror.ADCRESULT14;//ADC  result 12bit 유효값

	if(ADChanelCnt == 3)
	{
		if(gSensorChanel == 0)
			gBattVolt = AdcMirror.ADCRESULT15;
		else
			GyroVar.u16Value = AdcMirror.ADCRESULT15 >> 2;//ADC  result 12bit 유효값 => 10bit 로 변환
	}
	else
		SenValSum += AdcMirror.ADCRESULT15;//ADC  result 12bit 유효값

	
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;

	ADChanelCnt++;

	//센서 하나당 시퀀스 4번 
	if(ADChanelCnt >= 4)//마지막 시퀀스 
	{
		ADChanelCnt = 0;		
		Sen[gSensorChanel].u16Value = SenValSum >> 7;
		SenValSum = 0;

		//sensor val filtering
		Sen[gSensorChanel].q19LPFOutDataYet = Sen[gSensorChanel].q19LPFOutData;
		Buff = Sen[gSensorChanel].q19LPFOutData = _IQ19mpyIQX(SENSOR_Kb, 30, Sen[gSensorChanel].q19LPFInData + QUP(Sen[gSensorChanel].u16Value, 19), 19) 
										 - _IQ19mpyIQX(SENSOR_Ka, 30, Sen[gSensorChanel].q19LPFOutData, 19);
		Sen[gSensorChanel].q19LPFInData = QUP(Sen[gSensorChanel].u16Value, 19);

		//sensor diff filtering
		Sen[gSensorChanel].q19LPFInDataDiff = Sen[gSensorChanel].q19LPFOutData - Sen[gSensorChanel].q19LPFOutDataYet;
		Sen[gSensorChanel].q19LPFOutDataDiff = _IQ19mpyIQX(SENSOR_Kb_DIFF, 30, Sen[gSensorChanel].q19LPFInDataDiff + Sen[gSensorChanel].q19LPFInDataDiffYet, 19) 
											 - _IQ19mpyIQX(SENSOR_Ka_DIFF, 30, Sen[gSensorChanel].q19LPFOutDataDiff, 19);
		Sen[gSensorChanel].q19LPFInDataDiffYet = Sen[gSensorChanel].q19LPFInDataDiff;


	
		if((gSensorChanel == 3) || (gSensorChanel == 0))//RFS  = 3, LFS = 0.		
			FrontSensorValueToDisTable(gSensorChanel, Buff);
		else
		{
			Sen[gSensorChanel].q19PositionYet = Sen[gSensorChanel].q19Position;
			
			if(Buff >= Sen[gSensorChanel].q19MidVal)
				Sen[gSensorChanel].q19Position = _IQ19mpy(Sen[gSensorChanel].q19HighCoefficient,_IQ19sqrt(Buff - Sen[gSensorChanel].q19MidVal)) + _IQ19(256.0);
			else
				Sen[gSensorChanel].q19Position = _IQ19mpy(Sen[gSensorChanel].q19LowCoefficient,_IQ19sqrt(Buff - Sen[gSensorChanel].q19MinVal)) + _IQ19(512.0);			
	
			if(Sen[gSensorChanel].q19Position < 0)
				Sen[gSensorChanel].q19Position = 0;

			Sen[gSensorChanel].q19PositionDiff = Sen[gSensorChanel].q19Position - Sen[gSensorChanel].q19PositionYet;
			
		}


		//gyro val filtering
					
		GyroVar.q20LPFOutData = _IQ20mpyIQX(GYRO_Kb, 30, GyroVar.q20LPFInData + ((Uint32)(GyroVar.u16Value) << 20) , 20) 
										 - _IQ20mpyIQX(GYRO_Ka, 30, GyroVar.q20LPFOutData, 20);
		GyroVar.q20LPFInData = (Uint32)(GyroVar.u16Value) << 20;

		gSensorChanel++;
	
		if(gSensorChanel >= 6)
		{
			gSensorChanel = 0;
			StopCpuTimer0();//센서 한바퀴 돌고 off
		}
		else
			;
	}
	else
	{
		if(ADChanelCnt == 3)
		{
			if(gSensorChanel == 0)
				AdcRegs.ADCCHSELSEQ4.bit.CONV15 = BATT_AD;
			else
				AdcRegs.ADCCHSELSEQ4.bit.CONV15 = GYRO_AD;
		}
		else
			;		
		AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;
	}

}	  
void FrontSensorValueToDisTable(Uint16 i, _iq19 Value)
{
	volatile long *p[4] = {q19LFSSection,0,0,q19RFSSection};
	volatile long *pDiff[4] = {q23LFSSectionDiff,0,0,q23RFSSectionDiff};
	int16 TableIndex;
	
	
	//광량대 거리 테이블.
	if(Value >= p[i][13])
	{
		if(Value >= p[i][6])
		{
			if(Value >= p[i][3])
			{
				if(Value >= p[i][1])
				{
					if(Value >= p[i][0])
					{// =max
						TableIndex = -1;
					}
					else// =0
					{
						TableIndex = 0;
					}
				}
				else
				{
					if(Value >= p[i][2])
					{// =1
						TableIndex = 1;
					}
					else// =2
					{
						TableIndex = 2;
					}
					
				}
			}
			else// 6<= .< 3
			{
				if(Value >= p[i][5])
				{
					if(Value >= p[i][4])
					{// =3
						TableIndex = 3;
					}
					else
					{// =4
						TableIndex = 4;
					}
					
				}
				else// =5
				{
					TableIndex = 5;
				}
				
			}
		}
		else// < 6
		{
			if(Value >= p[i][10])
			{
				if(Value >= p[i][8])
				{
					if(Value >= p[i][7])
					{// =6	
						TableIndex = 6;
					}
					else// <7
					{// =7
						TableIndex = 7;
					}
				}
				else// < 8
				{
					if(Value >= p[i][9])
					{// =8
						TableIndex = 8;
					}
					else
					{// =9
						TableIndex = 9;
					}
					
				}
			}
			else// < 10
			{
				if(Value >= p[i][12])
				{
					if(Value >= p[i][11])
					{// =10
						TableIndex = 10;
					}
					else
					{// = 11
						TableIndex = 11;
					}
				}
				else
				{// =12
					TableIndex = 12;
				}
			}
		}
	}
	else// < 13
	{
		if(Value >= p[i][19])
		{
			if(Value >= p[i][16])
			{
				if(Value >= p[i][15])
				{
					if(Value >= p[i][14])
					{// =13
						TableIndex = 13;
					}
					else
					{// =14
						TableIndex = 14;
					}
				}
				else
				{// =15
					TableIndex = 15;
				}
			}
			else// < 16
			{
				if(Value >= p[i][18])
				{
					if(Value >= p[i][17])
					{// =16
						TableIndex = 16;
					}
					else
					{// =17
						TableIndex = 17;
					}
				}
				else
				{// =18
					TableIndex = 18;
				}
			}
		}
		else// < 19
		{
			if(Value >= p[i][22])
			{
				if(Value >= p[i][21])
				{
					if(Value >= p[i][20])
					{// =19
						TableIndex = 19;
					}
					else
					{// =20
						TableIndex = 20;
					}
				}
				else
				{// =21
					TableIndex = 21;
				}
			}
			else// < 22
			{
				if(Value >= p[i][24])
				{
					if(Value >= p[i][23])
					{// =22
						TableIndex = 22;
					}
					else
					{// =23
						TableIndex = 23;
					}
				}
				else
				{
					if(Value >= p[i][25])
					{// = 24
						TableIndex = 24;
					}
					else
					{// =min
						TableIndex = 0xff;
					}
				}
			}
			
		}
	}

	if(TableIndex == -1)//max
		Sen[i].q19Position = 0;
	else if(TableIndex == 0xff)//min
		Sen[i].q19Position = _IQ19(250.0);
	else
	{
		Sen[i].q19Position = (((int32)(TableIndex * 10) << 23)  +_IQ23mpyIQX(p[i][TableIndex] - Value, 19, pDiff[i][TableIndex], 23)) >> 4;
	}
		
}


void ResetSensorVariable(volatile SensorVariable *p)
{
	memset((void *)p,0,sizeof(SensorVariable));
}

void FrontSensorSet(void)
{
	Uint16 i,y;
	int16 YetBufHead = -1;
	int16 BufHead = 0;
	Uint16 WriteBuf[110];
	
	InitMotorVar(&RightMotor);
	InitMotorVar(&LeftMotor);
	RightMotor.i32Accel = LeftMotor.i32Accel = 1000.0;
	MoveStop(0, 0, 0, 0);
	
	POS_ADJ_OFF;
	MOTOR_ON;

	while(SW_R == HIGH);
	Delay(0x800000);

	MoveStop(_IQ19(-280), _IQ17(-200), _IQ19(-280), _IQ17(-200));

	while(!gMoveState)
	{
		if( _IQ19abs(RightMotor.q19DistanceSum) < _IQ19(251.0))
		{
			BufHead = (int16)_IQ19int(_IQ19div(_IQ19abs(RightMotor.q19DistanceSum), _IQ19(10.0)));
			if(BufHead != YetBufHead)
			{
				q19RFSSection[BufHead] = pRFS->q19LPFOutData;
				q19LFSSection[BufHead] = pLFS->q19LPFOutData;
			}
				
			YetBufHead = BufHead;
		}
	}

	for(i = 0; i < 25; i++)
	{
		q23RFSSectionDiff[i] = _IQ23div((int32)10 << 23, (q19RFSSection[i] - q19RFSSection[i+1]) << 4);
		q23LFSSectionDiff[i] = _IQ23div((int32)10 << 23, (q19LFSSection[i] - q19LFSSection[i+1]) << 4);
	}
	for(i = 0; i < 26; i++)
	{
		TxPrintf("%d LFS : %f  RFS : %f\t",i,_IQ19toF(q19LFSSection[i]),_IQ19toF(q19RFSSection[i]));
		if(i != 25)
			TxPrintf("LFSdiff: %f  RFSdiff: %f\n",_IQ23toF(q23LFSSectionDiff[i]),_IQ23toF(q23RFSSectionDiff[i]));
		
	}
	
	for(i = 0,y = 0; i < 26 ;i++)
	{
		WriteBuf[y++] = (q19RFSSection[i] >> 0) & 0xff;
		WriteBuf[y++] = (q19RFSSection[i] >> 8) & 0xff;
		WriteBuf[y++] = (q19RFSSection[i] >> 16) & 0xff;
		WriteBuf[y++] = (q19RFSSection[i] >> 24) & 0xff;
	}
	SpiWriteRom(R_FRONT_SENSOR_SECTION_ADDRESS, 0, 26 * 4, WriteBuf);

	for(i = 0,y = 0; i < 26 ;i++)
	{
		WriteBuf[y++] = (q19LFSSection[i] >> 0) & 0xff;
		WriteBuf[y++] = (q19LFSSection[i] >> 8) & 0xff;
		WriteBuf[y++] = (q19LFSSection[i] >> 16) & 0xff;
		WriteBuf[y++] = (q19LFSSection[i] >> 24) & 0xff;
	}
	SpiWriteRom(L_FRONT_SENSOR_SECTION_ADDRESS, 0, 26 * 4, WriteBuf);
	
	for(i = 0, y = 0; i < 25 ;i++)
	{
		WriteBuf[y++] = (q23RFSSectionDiff[i] >> 0) & 0xff;
		WriteBuf[y++] = (q23RFSSectionDiff[i] >> 8) & 0xff;
		WriteBuf[y++] = (q23RFSSectionDiff[i] >> 16) & 0xff;
		WriteBuf[y++] = (q23RFSSectionDiff[i] >> 24) & 0xff;
	}
	SpiWriteRom(R_FRONT_SENSOR_SECTION_DIFF_ADDRESS, 0, 25 * 4, WriteBuf);

	for(i = 0,y = 0; i < 25 ;i++)
	{
		WriteBuf[y++] = (q23LFSSectionDiff[i] >> 0) & 0xff;
		WriteBuf[y++] = (q23LFSSectionDiff[i] >> 8) & 0xff;
		WriteBuf[y++] = (q23LFSSectionDiff[i] >> 16) & 0xff;
		WriteBuf[y++] = (q23LFSSectionDiff[i] >> 24) & 0xff;
	}
	SpiWriteRom(L_FRONT_SENSOR_SECTION_DIFF_ADDRESS, 0, 25 * 4, WriteBuf);
	
	
}
void FrontSensorValueCall(void)
{
	Uint16 i,y;	
	Uint16 ReadBuf[110];

	SpiReadRom(R_FRONT_SENSOR_SECTION_ADDRESS, 0, 26 * 4, ReadBuf);
	for(i = 0,y = 0; i < 26; i++)
	{
		q19RFSSection[i] =  (((int32)ReadBuf[y++] & 0xff) << 0);
		q19RFSSection[i] |= (((int32)ReadBuf[y++] & 0xff) << 8);
		q19RFSSection[i] |= (((int32)ReadBuf[y++] & 0xff) << 16);
		q19RFSSection[i] |= (((int32)ReadBuf[y++] & 0xff) << 24);
		
	}

	SpiReadRom(L_FRONT_SENSOR_SECTION_ADDRESS, 0, 26 * 4, ReadBuf);
	for(i = 0,y = 0; i < 26; i++)
	{
		q19LFSSection[i] =  (((int32)ReadBuf[y++] & 0xff) << 0);
		q19LFSSection[i] |= (((int32)ReadBuf[y++] & 0xff) << 8);
		q19LFSSection[i] |= (((int32)ReadBuf[y++] & 0xff) << 16);
		q19LFSSection[i] |= (((int32)ReadBuf[y++] & 0xff) << 24);
		
	}

	SpiReadRom(R_FRONT_SENSOR_SECTION_DIFF_ADDRESS, 0, 25 * 4, ReadBuf);
	for(i = 0,y = 0; i < 25; i++)
	{
		q23RFSSectionDiff[i] =  (((int32)ReadBuf[y++] & 0xff) << 0);
		q23RFSSectionDiff[i] |= (((int32)ReadBuf[y++] & 0xff) << 8);
		q23RFSSectionDiff[i] |= (((int32)ReadBuf[y++] & 0xff) << 16);
		q23RFSSectionDiff[i] |= (((int32)ReadBuf[y++] & 0xff) << 24);
		
	}

	SpiReadRom(L_FRONT_SENSOR_SECTION_DIFF_ADDRESS, 0, 25 * 4, ReadBuf);
	for(i = 0,y = 0; i < 25; i++)
	{
		q23LFSSectionDiff[i] =  (((int32)ReadBuf[y++] & 0xff) << 0);
		q23LFSSectionDiff[i] |= (((int32)ReadBuf[y++] & 0xff) << 8);
		q23LFSSectionDiff[i] |= (((int32)ReadBuf[y++] & 0xff) << 16);
		q23LFSSectionDiff[i] |= (((int32)ReadBuf[y++] & 0xff) << 24);
		
	}
	TxPrintf("\n=======================================================================\n");
	for(i = 0; i < 26; i++)
	{
		TxPrintf("%d LFS : %f  RFS : %f\t",i,_IQ19toF(q19LFSSection[i]),_IQ19toF(q19RFSSection[i]));
		if(i != 25)
			TxPrintf("LFSdiff: %f  RFSdiff: %f\n",_IQ23toF(q23LFSSectionDiff[i]),_IQ23toF(q23RFSSectionDiff[i]));
		
	}
	
	
}

void SiedSensorSet(void)
{
	Uint16 i;
	volatile SensorVariable *p[4] = {&RDS,&RSS,&LDS,&LSS};
	Uint16 WriteBuf[80];

	memset((void *)WriteBuf, 0, sizeof(WriteBuf));
	
	VFDPrintf("Rmax");
	while(SW_R == HIGH);
	Click();

	p[0]->q19MaxVal = p[0]->q19LPFOutData;
	p[1]->q19MaxVal = p[1]->q19LPFOutData;
	p[2]->q19MinVal = p[2]->q19LPFOutData;
	p[3]->q19MinVal = p[3]->q19LPFOutData;

	VFDPrintf("Lmax");
	while(SW_R == HIGH);
	Click();

	
	p[0]->q19MinVal = p[0]->q19LPFOutData;
	p[1]->q19MinVal = p[1]->q19LPFOutData;
	p[2]->q19MaxVal = p[2]->q19LPFOutData;
	p[3]->q19MaxVal = p[3]->q19LPFOutData;



	VFDPrintf("Mid");
	while(SW_R == HIGH);
	Click();

	p[1]->q19MidVal = p[1]->q19LPFOutData;
	p[0]->q19MidVal = p[0]->q19LPFOutData;
	p[2]->q19MidVal = p[2]->q19LPFOutData;
	p[3]->q19MidVal = p[3]->q19LPFOutData;
	
	for(i = 0; i < 4; i++)
	{
		p[i]->q19HighCoefficient = _IQ19mpy((int32)(-1) << 19,_IQ19div((int32)256 << 19, _IQ19sqrt(p[i]->q19MaxVal - p[i]->q19MidVal))); 
		p[i]->q19LowCoefficient = _IQ19mpy((int32)(-1) << 19,_IQ19div((int32)256 << 19, _IQ19sqrt(p[i]->q19MidVal - p[i]->q19MinVal))); 

		WriteBuf[i*20+0] = (p[i]->q19MaxVal >> 0)  & 0xff;
		WriteBuf[i*20+1] = (p[i]->q19MaxVal >> 8)  & 0xff;
		WriteBuf[i*20+2] = (p[i]->q19MaxVal >> 16) & 0xff;
		WriteBuf[i*20+3] = (p[i]->q19MaxVal >> 24) & 0xff;
		
		WriteBuf[i*20+4] = (p[i]->q19MinVal >> 0)  & 0xff;
		WriteBuf[i*20+5] = (p[i]->q19MinVal >> 8)  & 0xff;
		WriteBuf[i*20+6] = (p[i]->q19MinVal >> 16) & 0xff;
		WriteBuf[i*20+7] = (p[i]->q19MinVal >> 24) & 0xff;

		WriteBuf[i*20+8]  = (p[i]->q19MidVal >> 0)  & 0xff;
		WriteBuf[i*20+9]  = (p[i]->q19MidVal >> 8)  & 0xff;
		WriteBuf[i*20+10] = (p[i]->q19MidVal >> 16) & 0xff;
		WriteBuf[i*20+11] = (p[i]->q19MidVal >> 24) & 0xff;

		WriteBuf[i*20+12] = (p[i]->q19HighCoefficient >> 0)  & 0xff;
		WriteBuf[i*20+13] = (p[i]->q19HighCoefficient >> 8)  & 0xff;
		WriteBuf[i*20+14] = (p[i]->q19HighCoefficient >> 16) & 0xff;
		WriteBuf[i*20+15] = (p[i]->q19HighCoefficient >> 24) & 0xff;
		
		WriteBuf[i*20+16] = (p[i]->q19LowCoefficient >> 0)  & 0xff;
		WriteBuf[i*20+17] = (p[i]->q19LowCoefficient >> 8)  & 0xff;
		WriteBuf[i*20+18] = (p[i]->q19LowCoefficient >> 16) & 0xff;
		WriteBuf[i*20+19] = (p[i]->q19LowCoefficient >> 24) & 0xff;
	}

	SpiWriteRom(SIDE_SENSOR_ADDRESS,0,80,WriteBuf);

}

void SideSensorValueCall(void)
{
	Uint16 i;
	volatile SensorVariable *p[4] = {&RDS,&RSS,&LDS,&LSS};
	Uint16 ReadBuf[80];

	memset((void *)ReadBuf, 0,sizeof(ReadBuf));

	SpiReadRom(SIDE_SENSOR_ADDRESS,0,80,ReadBuf);

	for(i = 0; i < 4;i++)
	{
		p[i]->q19MaxVal =  (((Uint32)ReadBuf[i*20 + 0] & 0xff) << 0);
		p[i]->q19MaxVal |= (((Uint32)ReadBuf[i*20 + 1] & 0xff) << 8); 
		p[i]->q19MaxVal |= (((Uint32)ReadBuf[i*20 + 2] & 0xff) << 16); 
		p[i]->q19MaxVal |= (((Uint32)ReadBuf[i*20 + 3] & 0xff) << 24); 
		
		p[i]->q19MinVal =  (((Uint32)ReadBuf[i*20 + 4] & 0xff) << 0);
		p[i]->q19MinVal |= (((Uint32)ReadBuf[i*20 + 5] & 0xff) << 8); 
		p[i]->q19MinVal |= (((Uint32)ReadBuf[i*20 + 6] & 0xff) << 16); 
		p[i]->q19MinVal |= (((Uint32)ReadBuf[i*20 + 7] & 0xff) << 24); 

		p[i]->q19MidVal =  (((Uint32)ReadBuf[i*20 + 8] & 0xff) << 0);
		p[i]->q19MidVal |= (((Uint32)ReadBuf[i*20 + 9] & 0xff) << 8); 
		p[i]->q19MidVal |= (((Uint32)ReadBuf[i*20 + 10] & 0xff) << 16); 
		p[i]->q19MidVal |= (((Uint32)ReadBuf[i*20 + 11] & 0xff) << 24); 
		
		p[i]->q19HighCoefficient =	(((Uint32)ReadBuf[i*20 + 12] & 0xff) << 0);
		p[i]->q19HighCoefficient |= (((Uint32)ReadBuf[i*20 + 13] & 0xff) << 8); 
		p[i]->q19HighCoefficient |= (((Uint32)ReadBuf[i*20 + 14] & 0xff) << 16); 
		p[i]->q19HighCoefficient |= (((Uint32)ReadBuf[i*20 + 15] & 0xff) << 24); 
		
		p[i]->q19LowCoefficient =  (((Uint32)ReadBuf[i*20 + 16] & 0xff) << 0);
		p[i]->q19LowCoefficient |= (((Uint32)ReadBuf[i*20 + 17] & 0xff) << 8); 
		p[i]->q19LowCoefficient |= (((Uint32)ReadBuf[i*20 + 18] & 0xff) << 16); 
		p[i]->q19LowCoefficient |= (((Uint32)ReadBuf[i*20 + 19] & 0xff) << 24); 
		
	}

	TxPrintf("\n=================================================================\n");
	TxPrintf("RDS max :%4.2f  RDS min :%4.2f RDS mid :%4.2f RDS hc :%4.2f RDS lc :%4.2f\n" ,_IQ19toF(p[0]->q19MaxVal)
																			,_IQ19toF(p[0]->q19MinVal)
																			,_IQ19toF(p[0]->q19MidVal)
																			,_IQ19toF(p[0]->q19HighCoefficient)
																			,_IQ19toF(p[0]->q19LowCoefficient));
	TxPrintf("RSS max :%4.2f  RSS min :%4.2f RSS mid :%4.2f RSS hc :%4.2f RSS lc :%4.2f\n" ,_IQ19toF(p[1]->q19MaxVal)
																			,_IQ19toF(p[1]->q19MinVal)
																			,_IQ19toF(p[1]->q19MidVal)
																			,_IQ19toF(p[1]->q19HighCoefficient)
																			,_IQ19toF(p[1]->q19LowCoefficient));
	TxPrintf("LDS max :%4.2f  LDS min :%4.2f LDS mid :%4.2f LDS hc :%4.2f LDS lc :%4.2f\n" ,_IQ19toF(p[2]->q19MaxVal)
																			,_IQ19toF(p[2]->q19MinVal)
																			,_IQ19toF(p[2]->q19MidVal)
																			,_IQ19toF(p[2]->q19HighCoefficient)
																			,_IQ19toF(p[2]->q19LowCoefficient));
	TxPrintf("LSS max :%4.2f  LSS min :%4.2f LSS mid :%4.2f LSS hc :%4.2f LSS lc :%4.2f\n" ,_IQ19toF(p[3]->q19MaxVal)
																			,_IQ19toF(p[3]->q19MinVal)
																			,_IQ19toF(p[3]->q19MidVal)
																			,_IQ19toF(p[3]->q19HighCoefficient)
																			,_IQ19toF(p[3]->q19LowCoefficient));
	
	
}

void PositionAdjustDiffVal(_iq30 DecelRate, _iq30 AccelRate)
{
	q30PosAdjDecelDiff = (_IQ22div(DecelRate >> 8, (int32)256 << 22)) << 8;
	q30PosAdjAccelDiff = (_IQ22div(AccelRate >> 8, (int32)256 << 22)) << 8;

	q26PosAdjDecelRef = ((int32)1 << 26) - (DecelRate >> 4);
	q26PosAdjAccelRef = ((int32)1 << 26) - (AccelRate >> 4);;

	//TxPrintf("\n%f  %f  %f  %f\n",_IQ30toF(q30PosAdjAccelDiff),_IQ30toF(q30PosAdjDecelDiff),_IQ30toF(q30PosAdjAccelRef),_IQ30toF(q30PosAdjDecelRef));
}






