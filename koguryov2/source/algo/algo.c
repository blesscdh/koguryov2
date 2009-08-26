//###########################################################################
//
// FILE:   algo.c
//
// TITLE:  KOGURYO Mouse algorithm c file.
//
//###########################################################################
// $Release Date: 2006.12.27 $
//###########################################################################
#include "DSP280x_Device.h"     // DSP280x Headerfile Include File
#include "DSP280x_Examples.h"   // DSP280x Examples Include File

#include "algo.h"
struct bit
{ 
	Uint16 Weight:16;
	Uint16 NowVector:4;
	Uint16 YetVector:4;
	Uint16 BlockCnt:6;
	Uint16 BlockState:2;
};

typedef  union
{
	Uint32 ALL_UINT;
	struct bit Divide;
	
}MapVariable;
volatile MapVariable  gMapValue[256];

typedef struct QueueLists
{
	Uint16 Position;
	Uint16 Weight;
	volatile struct QueueLists *Next;
}QueueList;

volatile QueueList *MapSearchQueueHead = NULL;
volatile QueueList *EndPos = NULL;

//Uint16 DirectWeight[15] = {17,12,10,9,8,7,7,7,7,7,7,7,7,7,7};
Uint16 DirectWeight[15] = {14,10,8,7,7,6,6,5,5,5,5,5,5,5,5};

//Uint16 DiagWeight[30] = {22,15,12,10,10,10,10,10,10,10,10,10,10,10,10,
//						 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10};
Uint16 DiagWeight[30] = {11,8,7,6,5,5,5,5,5,5,5,5,5,5,5,
	                     5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};

#define TURN_WEIGHT	((Uint16)23)


const	char	PathState[22][10] = {"Straight",
								 "R90",
								 "BackTurn",
								 "L90",
								 "R180",
								 "L180",
								 "R135IN",
								 "L135IN",
								 "R45IN",
								 "L45IN",
								 "R135OUT",
								 "L135OUT",
								 "R45OUT",
								 "L45OUT",
								 "RD90",
								 "LD90",
								 "Rcbr45Out",
								 "Lcbr45Out",
								 "Rcbr135Out",
								 "Lcbr135Out",
								 "DiagRun",
								 "DiagRun"};
		



#define TURN		1
#define	DIRECT		2
#define DIAGONAL	3

#pragma CODE_SECTION(WriteMazeWeight, "ramfuncs0");
#pragma CODE_SECTION(PopSearchQueueList, "ramfuncs0");
#pragma CODE_SECTION(InsertSearchQueueList, "ramfuncs0");
#pragma CODE_SECTION(DelSearchQueueList, "ramfuncs0");
#pragma CODE_SECTION(KnowBlockPathMake, "ramfuncs1");

void InitAlgorithmVariable(void)
{
	gMouseDir = 0;
	gMouseYetDir = 0;
	gSearchType = NC;
	gMousePosition = 0;
	gMouseYetPosition = 0;
	gPathBufferHead = 0;
	gPathWeightState = OFF;
	gJapanAlgo = OFF;
	gAlgoState = ON;
	gSearchEndState = OFF;
	gBlockToBlock = 0;
	gBlockRunException = OFF;

	memset((void *)gMazeMap, 0x00, sizeof(gMazeMap));
	memset((void *)gMazeMapBackUp, 0x00, sizeof(gMazeMap));
	memset((void *)gMazeMapBackUp0, 0x00, sizeof(gMazeMap));
	memset((void *)gMazeMapBackUp1, 0x00, sizeof(gMazeMap));
	memset((void *)gMazeMapBackUp2, 0x00, sizeof(gMazeMap));
	memset((void *)gMapValue, 0x00, sizeof(gMapValue));

	InitSearchQueueList();
	
}

void InitWeight(void)
{
	Uint16 i = 0;

	for(i = 0; i < 256;i++)
	{
		gMapValue[i].ALL_UINT = 0x00000fff;
	}
	

	switch(gSearchType)
	{
		case NC:
			
			VFDPrintf("Err ");
			MOTOR_OFF;
			while(TRUE)
				;
			
		case GO_GOAL:
		case GO_01GOAL:

			//LSB 무게값 : 16, 현재방향 :4, 전방향 : 4, 블럭수 : 6, 연속블럭종류 : 2, MSB
			gMapValue[0x77].Divide.Weight = 0;
			gMapValue[0x78].Divide.Weight = 0;
			gMapValue[0x88].Divide.Weight = 0;
			gMapValue[0x87].Divide.Weight = 0;

			gMapValue[0x77].Divide.NowVector = DIR_W;
			gMapValue[0x78].Divide.NowVector = DIR_N;
			gMapValue[0x88].Divide.NowVector = DIR_E;
			gMapValue[0x87].Divide.NowVector = DIR_S;

			InsertSearchQueueList(0x77,0x00);
			InsertSearchQueueList(0x87,0x00);
			InsertSearchQueueList(0x88,0x00);
			InsertSearchQueueList(0x78,0x00);
			
			break;
			
		case GO_START://원점으로 돌아가기
			
			gMapValue[0].Divide.Weight = 0;
			gMapValue[0].Divide.NowVector = DIR_N;
			InsertSearchQueueList(0x00,0x00);

			break;

		case GO_01START://0x01로 돌아가기

			gMapValue[0x01].Divide.Weight = 0;
			gMapValue[0x01].Divide.NowVector = DIR_N;
			InsertSearchQueueList(0x01,0x00);
			
			break;
			
		default:	
			VFDPrintf("Err ");
			MOTOR_OFF;
			while(TRUE)
				;
					
	}
}


void InitAlgorithm(void)
{
	Uint16	Cnt;

	if(gJapanAlgo)
		gSearchType = GO_01GOAL;
	else
		gSearchType = GO_GOAL;
	
	gMouseDir = 0;
	gMouseYetDir = 0;
	gMousePosition = 0;
	gPathBufferHead = 0;
	gAlgoState = ON;
	gSecondRunGoal = OFF;
	gPathWeightState = OFF;
	gSearchEndState = OFF;
	gBlockRunException = OFF;
	gFisrtBlockDiagF = OFF;
	
	memset((void *)KnowBlockPath, 0x00, sizeof(KnowBlockPath));

	DelSearchQueueList();

	//외각 벽 저장
	for(Cnt = 0; Cnt < 256; Cnt++)
	{
		if((Cnt & 0xf0) == 0)
			gMazeMap[Cnt] |= DIR_W;
		else if((Cnt & 0xf0) == 0xf0)
			gMazeMap[Cnt] |= DIR_E;

		if((Cnt & 0x0f) == 0)
			gMazeMap[Cnt] |= DIR_S;
		else if((Cnt & 0x0f) == 0x0f)
			gMazeMap[Cnt] |= DIR_N;
	}
	gMazeMap[0x00] |= 0xe;
	
}


void WriteMazeWeight(Uint16 NowPos)
{
	Uint16 TempWall = 0;
	Uint16 x = 0;
	Uint16 NextWeight = 0;
	Uint16 SumWeight = 0;
	Uint16 HeadState = 0;
	int16 NextPos = 0;
	int16 MousePosition = 0;
	

	while(MapSearchQueueHead->Next != NULL)
	{
		MousePosition = PopSearchQueueList();

		if((MousePosition < 0) || (MousePosition > 0xff))
		{
			MOTOR_OFF;
			VFDPrintf("ErrP");
			while(TRUE)
				;
		}
		
		if(!gPathWeightState && (gMapValue[NowPos].Divide.Weight < gMapValue[MousePosition].Divide.Weight))
		{
			DelSearchQueueList();
			break;
		}
		
		TempWall = gMazeMap[MousePosition] & 0x0f;//상위 4bit 주행중 알게되는 미로벽

		for(x = 0; x < 4; x++)//사방 조사//
		{


			if(!(TempWall & gHeadTable[x]))//벽이 없는쪽.
			{
				if((gHeadTable[x] & gMapValue[MousePosition].Divide.NowVector))
				{
					HeadState = DIRECT;
					SumWeight = DirectWeight[gMapValue[MousePosition].Divide.BlockCnt];
				}
				else
				{

					if(gMapValue[MousePosition].Divide.YetVector == gHeadTable[x])
					{
						HeadState = DIAGONAL;
						SumWeight = DiagWeight[gMapValue[MousePosition].Divide.BlockCnt];
						
					}
					else
					{
						HeadState = TURN;
						SumWeight = TURN_WEIGHT;//gTurnWeightVal;// 3;
					}
				}
				
				if((gMapValue[MousePosition].Divide.Weight + SumWeight) < gMapValue[(MousePosition + gMoveTable[x])].Divide.Weight)
				{	
					NextWeight = gMapValue[MousePosition].Divide.Weight + SumWeight;//벽 없는 다음 이동할 블럭의 무게값. 여기서 직진과 대각 일반턴의 무게값 조정..
					NextPos = MousePosition + gMoveTable[x];

					if((NextPos < 0) || (NextPos > 0xff))
					{

						VFDPrintf("ErW0");
						MOTOR_OFF;
						while(TRUE) 
							;

					}
					else
						;
					
					if(HeadState == TURN)
					{
						gMapValue[NextPos].Divide.BlockCnt = 0;
						gMapValue[NextPos].Divide.BlockState = TURN;
					}
					else if(HeadState == DIAGONAL)
					{
						if(gMapValue[MousePosition].Divide.BlockState == DIAGONAL)
						{
							gMapValue[NextPos].Divide.BlockCnt = gMapValue[MousePosition].Divide.BlockCnt + 1;
							gMapValue[NextPos].Divide.BlockState = DIAGONAL;
						}
						else
						{
							gMapValue[NextPos].Divide.BlockCnt = 0;
							gMapValue[NextPos].Divide.BlockState = DIAGONAL;
						
						}
					}
					else if(HeadState == DIRECT)
					{	
						if(gMapValue[MousePosition].Divide.BlockState == DIRECT)
						{
							gMapValue[NextPos].Divide.BlockCnt = gMapValue[MousePosition].Divide.BlockCnt + 1;
							gMapValue[NextPos].Divide.BlockState = DIRECT;

						}
						else
						{
							gMapValue[NextPos].Divide.BlockCnt = 0;
							gMapValue[NextPos].Divide.BlockState = DIRECT;

						}
					}

					gMapValue[NextPos].Divide.YetVector = gMapValue[MousePosition].Divide.NowVector;
					gMapValue[NextPos].Divide.Weight = NextWeight;
					gMapValue[NextPos].Divide.NowVector = gHeadTable[x];
					InsertSearchQueueList(NextPos,NextWeight);

									
				}
				else
					;	

			}
			else
				;

		}
	}	

}

void Algorithm(Uint16 WallInfo)
{
	Uint16		x;
	Uint16		NextWeight;
	Uint16		WeightMin;
	Uint16		TurnDir;
	Uint16		NextTurn;
	Uint16		KnowBlockState;

	Uint16		ExBackUp01; 
	Uint16		ExBackUp02;
	Uint16		ExBackUp11;

	int16 		NextPos;
	
	if(!(gMazeMap[gMousePosition] & 0x10))
	{
	
		//벽 정보 엡데이트
		gMazeMap[gMousePosition] |= (1 << 4);
		gMazeMap[gMousePosition] |= WallInfo & 0x0f;
		KnowBlockState = OFF;

		for(x = 0; x < 4; x++)
		{
			if(WallInfo & gHeadTable[x])
			{
				switch(x) 
				{

				case 0://north
					if(!((gMousePosition & 0x0f) == 0x0f))
						gMazeMap[gMousePosition + gMoveTable[x]] |= DIR_S;
					break;

				case 1://east
					if(!((gMousePosition & 0xf0) == 0xf0))
						gMazeMap[gMousePosition + gMoveTable[x]] |= DIR_W ;
					break;

				case 2://south
					if(!((gMousePosition & 0x0f) == 0x00))
						gMazeMap[gMousePosition+ gMoveTable[x]] |= DIR_N;
					break;

				case 3://west
					if(!((gMousePosition & 0xf0) == 0x00))
						gMazeMap[gMousePosition + gMoveTable[x]] |= DIR_E;
					break;

				default:
					break;
				}


			}
		}
	}
	else
		KnowBlockState = ON;

	
	ExBackUp01 = gMazeMap[0x01];
	ExBackUp02 = gMazeMap[0x02];
	ExBackUp11 = gMazeMap[0x11];

	if((KnowBlockState == OFF) || (gMousePosition == 0x00) || (gMousePosition == 0x77) || (gMousePosition == 0x78) || (gMousePosition == 0x87) || (gMousePosition == 0x88) 
		|| ((gJapanAlgo) && (gMousePosition == 0x01)))
	{
		//일본 대회 알고 때문에 
		//첫블럭(0x01)에서 북쪽 동쪽이 
		//열려 있을때 직진이외의 방향이 나오는것을 막음

		
		if((gJapanAlgo == 0x11) && (gMousePosition == 0x01))
		{
			if(gMouseDir == 0)//마우스 북쪽 향할때
			{
				gMazeMap[0x01] |= DIR_E;
				gMazeMap[0x11] |= DIR_W;
			}
			
			else if(gMouseDir == 1)//마우스 동쪽 향할때
			{
				gMazeMap[0x01] |= DIR_N;
				gMazeMap[0x02] |= DIR_S;
				
			}
		}
		
		InitWeight();
		WriteMazeWeight(gMousePosition);
		
	}

	WeightMin = gMapValue[gMousePosition].Divide.Weight;//현재 위치의 무게값.

	for(x = 0; x < 4; x++)
	{
		if(!((gMazeMap[gMousePosition] & 0x0f) & gHeadTable[x]))
		{
			NextWeight = gMapValue[gMousePosition + gMoveTable[x]].Divide.Weight;//벽이 없는 쪽 블럭의 무게값을 가져온다.

			if(NextWeight < WeightMin)
			{
				WeightMin = NextWeight;//최소무게값 업데이트
				NextPos = gMousePosition + gMoveTable[x];//갈 방향 저장.
				TurnDir = x;//절대 방향
				NextTurn = (TurnDir + 4 - gMouseDir) & 0x03;
			}
			
		}
	}

	if((NextPos < 0) || (NextPos > 0xff))
	{
		VFDPrintf("EAL2");
		MOTOR_OFF;
		while(TRUE)
			;
	}

	
	gMazeMap[0x01] = ExBackUp01;
	gMazeMap[0x02] = ExBackUp02;
	gMazeMap[0x11] = ExBackUp11;

	gMouseYetPosition = gMousePosition;
	gMousePosition = NextPos;//다음 갈 좌표

	gMouseYetDir = gMouseDir;
	gMouseDir = TurnDir;//(0,1,2,3) - N E S W

	gPathBufferHead = 0;
	
	KnowBlockPath[gPathBufferHead].TurnDir = NextTurn;
	KnowBlockPath[gPathBufferHead].PathState = NextTurn;
	KnowBlockPath[gPathBufferHead].Position = gMouseYetPosition;
	KnowBlockPath[gPathBufferHead].MouseDir = gMouseYetDir;
	KnowBlockPath[gPathBufferHead].PathCnt = 1;
	KnowBlockPath[gPathBufferHead + 1].PathState = LASTPATH;
	
	if(!WeightMin)
	{
		switch(gSearchType)
		{
			case NC:
				
				VFDPrintf("EAL0");
				MOTOR_OFF;
				while(TRUE)
					;
				
				
			case GO_GOAL:
		
				gSearchType = GO_START;
				break;

			case GO_01GOAL:

				gSearchType = GO_01START;
				break;
				
			case GO_START:
				
				gSearchEndState = ON;
				break;
				
			case GO_01START:
				
				gSearchEndState = ON;
				gPathBufferHead++;
				
				KnowBlockPath[gPathBufferHead].TurnDir = BACKTURN;
				KnowBlockPath[gPathBufferHead].PathState = BACKTURN;
				KnowBlockPath[gPathBufferHead].Position = gMousePosition;
				KnowBlockPath[gPathBufferHead++].MouseDir = gMouseDir;

				break;
				
			default:	
				VFDPrintf("EAL1");
				MOTOR_OFF;
				
				while(TRUE)
					;
					
		}
	}



	if((gMazeMap[gMousePosition]& 0x10) && (WeightMin != 0) && (NextTurn != BACKTURN))//다음갈 블록이 아는 블록이면 맵 만들기.
		KnowBlockPathMake(gMousePosition, gMouseDir);		
	else
		;

}
void KnowBlockPathMake(Uint16 Position, Uint16 MouseDir)
{
	Uint16 WeightMin;
	Uint16 x;
	Uint16 cnt;
	Uint16 NextWeight;
	Uint16 AbsoluteDir;
	Uint16 NextPos;
	Uint16 NextTurn;
	Uint16 i[4];
	Uint16 BlockCnt;
	Uint16 DiagPos[4];
	Uint16 DiagDir[4];
	Uint16 YetTurnState;
	Uint16 YetMouseHead;
	Uint16 LastPath = OFF;

	gPathBufferHead = 1;
	
	do//아는 블럭까지 패스 찾기
	{
		WeightMin = gMapValue[Position].Divide.Weight;
		for(x = 0; x < 4; x++)
		{
			if(!((gMazeMap[Position] & 0x0f) & gHeadTable[x]))
			{
				NextWeight = gMapValue[Position + gMoveTable[x]].Divide.Weight;
				if(NextWeight < WeightMin)
				{
					WeightMin = NextWeight;//최소무게값 업데이트
					NextPos = Position + gMoveTable[x];//갈 방향 저장.
					AbsoluteDir= x;//절대 방향
					NextTurn = (AbsoluteDir+ 4 - MouseDir) & 0x03;
				}

			}
		}
		KnowBlockPath[gPathBufferHead].Position = Position;//YetPosition
		Position = NextPos;
		KnowBlockPath[gPathBufferHead].MouseDir = MouseDir;//YetMouseDir
		MouseDir = AbsoluteDir;
		KnowBlockPath[gPathBufferHead].TurnDir = NextTurn;
		gPathBufferHead++;

	}while((gMazeMap[Position]& 0x10) && (WeightMin != 0) && (NextTurn != BACKTURN));

	gMousePosition = Position;
	gMouseDir = MouseDir;

	
	if(!WeightMin)
	{
		switch(gSearchType)
		{
			case NC:
				
				VFDPrintf("ErK1");
				MOTOR_OFF;
				
				while(TRUE)
					;
				
			case GO_GOAL:

				gSearchType = GO_START;
				break;

			case GO_01GOAL:
				
				gSearchType = GO_01START;
				break;
				
			case GO_START:
			case GO_01START:
				
				gSearchEndState = ON;
				KnowBlockPath[gPathBufferHead].Position = gMousePosition;
				KnowBlockPath[gPathBufferHead].MouseDir = gMouseDir;
				KnowBlockPath[gPathBufferHead].TurnDir = BACKTURN;
				KnowBlockPath[gPathBufferHead++].PathState = BACKTURN;
				break;
				
			default:	
				VFDPrintf("ErK2");
				MOTOR_OFF;
				
				while(TRUE)
					;
				
				
		}

	}
	
	gMouseHead = DirectRunState;
	x = 0;
	cnt = 0;
	gTurnState = BACKTURN;

	while(TRUE)
	{
		if((x+3) < gPathBufferHead)
		{
			i[3] = KnowBlockPath[x+3].TurnDir;
			i[2] = KnowBlockPath[x+2].TurnDir;
			i[1] = KnowBlockPath[x+1].TurnDir;
			i[0] = KnowBlockPath[x].TurnDir;
			
			DiagPos[3] = KnowBlockPath[x+3].Position;
			DiagPos[2] = KnowBlockPath[x+2].Position;
			DiagPos[1] = KnowBlockPath[x+1].Position;
			DiagPos[0] = KnowBlockPath[x].Position;
			
			DiagDir[3] = KnowBlockPath[x+3].MouseDir;
			DiagDir[2] = KnowBlockPath[x+2].MouseDir;
			DiagDir[1] = KnowBlockPath[x+1].MouseDir;
			DiagDir[0] = KnowBlockPath[x].MouseDir;
		}

		else if((x+2) < gPathBufferHead)
		{
			i[3] = GARBAGDATA;
			i[2] = KnowBlockPath[x+2].TurnDir;
			i[1] = KnowBlockPath[x+1].TurnDir;
			i[0] = KnowBlockPath[x].TurnDir;	

			DiagPos[2] = KnowBlockPath[x+2].Position;
			DiagPos[1] = KnowBlockPath[x+1].Position;
			DiagPos[0] = KnowBlockPath[x].Position;
			
			DiagDir[2] = KnowBlockPath[x+2].MouseDir;
			DiagDir[1] = KnowBlockPath[x+1].MouseDir;
			DiagDir[0] = KnowBlockPath[x].MouseDir;

			if((i[2] != F) && (gPathBufferHead == (x+3)) && (gMouseHead == DirectRunState))
				LastPath = ON;
			
		}
		else if((x+1) < gPathBufferHead)
		{
			
			i[3] = GARBAGDATA;
			i[2] = GARBAGDATA;
			i[1] = KnowBlockPath[x+1].TurnDir;
			i[0] = KnowBlockPath[x].TurnDir;
			DiagPos[1] = KnowBlockPath[x+1].Position;
			DiagPos[0] = KnowBlockPath[x].Position;
			DiagDir[1] = KnowBlockPath[x+1].MouseDir;
			DiagDir[0] = KnowBlockPath[x].MouseDir;

			if(gPathBufferHead == (x+2))
				LastPath = ON;
		}
		else if(x < gPathBufferHead)
		{
			i[3] = GARBAGDATA;
			i[2] = GARBAGDATA;
			i[1] = GARBAGDATA;
			i[0] = KnowBlockPath[x].TurnDir;
			DiagPos[0] = KnowBlockPath[x].Position;
			DiagDir[0] = KnowBlockPath[x].MouseDir;

			if(gPathBufferHead == (x+1))
				LastPath = ON;
		}
		else
		{
			gPathBufferHead = cnt;
			KnowBlockPath[gPathBufferHead].PathState = LASTPATH;
			break;
		}

		YetTurnState = gTurnState;
		YetMouseHead = gMouseHead;
		
		if(gMouseHead == DirectRunState)
		{
			if((i[0] == F) && (LastPath == OFF))//F
			{				
				if(i[1] == R)//FR
				{
					
					if(i[2] == R)//FRR
					{
						if(i[3] == F)//FRRF
						{
							//RIGHT180	ORTH->ORTH
							gMouseHead = DirectRunState;
							gTurnState = R180;
						}
						else//FRR
						{
							//RIGHT135IN	ORTH->DIAG
							gMouseHead = DiagonalRunState;
							gTurnState = R135IN;
						}
						x+=3;
					}
					else if(i[2] == F)//FRF
					{
						//RIGHT90	ORTH->ORTH
						gMouseHead = DirectRunState;
						gTurnState = R90;
						x+=2;
					}
					else//FR
					{
						//RIGHT45IN ORTH->DIAG
						gMouseHead = DiagonalRunState;
						gTurnState = R45IN;
						x+=2;
					}
				}
				else if(i[1] == L)//FL
				{
					if(i[2] == L)//FLL
					{
						if(i[3] == F)//FLLF
						{
							//LEFT180	ORTH->ORTH
							gMouseHead = DirectRunState;
							gTurnState = L180;
							x+=3;
						}
						else if ((i[3] == B) && (gSearchEndState == ON) && (gJapanAlgo))
						{
							gMouseHead = DirectRunState;
							gTurnState = L180;
							x+=3;
						}
						else//FLL
						{
							//LEFT135IN ORTH->DIAG
							gMouseHead = DiagonalRunState;
							gTurnState = L135IN;
							x+=3;
						}
					}
					else if(i[2] == F)//FLF
					{
						//LEFT90	ORTH->ORTH
						gMouseHead = DirectRunState;
						gTurnState = L90;
						x+=2;
					}
					else//FL
					{
						//LEFT45IN	ORTH->DIAG
						gMouseHead = DiagonalRunState;
						gTurnState = L45IN;
						x+=2;
					}
					
				}
				else if(i[1] == F)//FF
				{
					gMouseHead = DirectRunState;
					gTurnState = STRAIGHT;
					
					if(i[2] == F)//FFF
					{
						if(i[3] == F)//FFFF
						{
							//STRAIGHT FFFF
							x+=3;
							BlockCnt = 3;
						}
						else
						{
							//STRAIGHT FFF
							x+=2;
							BlockCnt = 2;
						}

					}
					else
					{
						//STRAIGHT FF
						x+=1;
						BlockCnt = 1;
					}
					//STRAIGHT
					
				}
				
				else
				{
					//F 하나만 남았을 경우
					gMouseHead = DirectRunState;
					gTurnState = STRAIGHT;
					x+=1;
					BlockCnt = 1;
					if(YetTurnState == STRAIGHT)
					{
						KnowBlockPath[cnt-1].Position = DiagPos[0];
						KnowBlockPath[cnt-1].MouseDir = DiagDir[0];

					}
					else
					{
						KnowBlockPath[cnt].Position = DiagPos[0];
						KnowBlockPath[cnt].MouseDir = DiagDir[0];
					}
				}
				
				//패스 찾을때 한블록 직진 삽입..
				if(gTurnState != STRAIGHT)
				{
					if(YetTurnState == STRAIGHT)
					{
						KnowBlockPath[cnt-1].PathCnt++;
						KnowBlockPath[cnt-1].MouseDir = DiagDir[0];

						if(KnowBlockPath[cnt -1].Position == 0x00)
							;
						else if((KnowBlockPath[cnt -1].Position == 0x01) && (gJapanAlgo))
							;							
						else
							KnowBlockPath[cnt-1].Position = DiagPos[0];
						
					}
					else
					{
						KnowBlockPath[cnt].PathState = STRAIGHT;
						KnowBlockPath[cnt].Position = DiagPos[0];
						KnowBlockPath[cnt].MouseDir = DiagDir[0];
						KnowBlockPath[cnt++].PathCnt = 1;
					}

				}
				

			}
			else//일치하는 것이 없을때.
			{
				gMouseHead = DirectRunState;
				gTurnState = NMATCH;
				if(i[0] == F)
				{
					gTurnState = STRAIGHT;
					BlockCnt = 1;
				}
				else
					;
				
				x+=1;
			}	
		
		}
		else//DIAGONAL
		{
			if(i[0] == R)//R
			{
				if(i[1] == R)//RR
				{
					if(i[2] == F)//RRF
					{
						//RIGHT135OUT	DIAG->ORTH
						gMouseHead = DirectRunState;
						gTurnState = R135OUT;
					}
					else if((i[2] == B) && (gSearchEndState == ON))
					{
						gMouseHead = DirectRunState;
						gTurnState = R135OUT;	
					}
					else//RR
					{
						//RIGHTDIAG90	DIAG->DIAG
						if(LastPath == OFF)
						{
							gMouseHead = DiagonalRunState;
							gTurnState = RD90;
						}
						else
						{
							gMouseHead = DirectRunState;
							gTurnState = RCbr135OUT;
						}
					}
					x+=2;
				}
				else if(i[1] == F)//RF
				{
					//RIGHT45OUT	DIAG->ORTH
					gMouseHead = DirectRunState;
					gTurnState = R45OUT;
					x+=1;
				}
				else if(i[1] == L)//RL
				{
					//Diag Straight DIAG->DIAG
					gMouseHead = DiagonalRunState;
					gTurnState = RDRUN;
					x+=1;
				}
				else if((i[1] == B) && (gSearchEndState == ON) && gJapanAlgo)
				{
					gMouseHead = DirectRunState;
					gTurnState = R45OUT;
					x+=1;
				}
				else//R
				{

					//CobraR45OUT
					gMouseHead = DirectRunState;
					gTurnState = RCbr45OUT;
					x+=1;
				}
					
				
			}
			else if(i[0] == L)//L
			{
				if(i[1] == L)//LL
				{
					if(i[2] == F)//LLF
					{
						//LEFT135OUT	DIAG->ORTH
						gMouseHead = DirectRunState;
						gTurnState = L135OUT;
					}
					else if((i[2] == B) && (gSearchEndState == ON))
					{
						gMouseHead = DirectRunState;
						gTurnState = L135OUT;	
					}
					else//LL
					{
						//LEFTDIAG90	DIAG->DIAG
						if(LastPath == OFF)
						{
							gMouseHead = DiagonalRunState;
							gTurnState = LD90;
						}
						else
						{
							gMouseHead = DirectRunState;
							gTurnState = LCbr135OUT;
						}
					}
					x+= 2;
				}
				else if(i[1] == F)//LF
				{
					//LEFT45OUT 	DIAG->ORTH
					gMouseHead = DirectRunState;
					gTurnState = L45OUT;
					x+=1;
				}
				else if(i[1] == R)//LR
				{
					//Diag Straight
					gMouseHead = DiagonalRunState;
					gTurnState = LDRUN;
					x+=1;
				}
				else if((i[1] == B) && (gSearchEndState == ON))
				{
					gMouseHead = DirectRunState;
					gTurnState = L45OUT;
					x+=1;
				}
				else
				{
					//CobraL45OUT
					gMouseHead = DirectRunState;
					gTurnState = LCbr45OUT;
					x+=1;
				}
			}
			else
				;//NULL
		}
		
		if((YetTurnState == STRAIGHT) && (gTurnState == STRAIGHT))
		{
			KnowBlockPath[--cnt].PathCnt += BlockCnt;
			if(KnowBlockPath[cnt].Position == 0x00)
				;
			else if((KnowBlockPath[cnt].Position == 0x01) && (gJapanAlgo))
				;							
			else
				KnowBlockPath[cnt].Position = DiagPos[0];
		}
		else if(gTurnState == STRAIGHT)
		{
			KnowBlockPath[cnt].PathState = gTurnState;
			KnowBlockPath[cnt].PathCnt = BlockCnt;
			KnowBlockPath[cnt].Position = DiagPos[0];
		}
		else
		{
			KnowBlockPath[cnt].PathState = gTurnState;
			
			if(YetMouseHead == DirectRunState)
			{
				if(gTurnState == NMATCH)
				{
					if(i[0] == L)
						KnowBlockPath[cnt].PathState = L90;
					else if(i[0] == R)
						KnowBlockPath[cnt].PathState = R90;
					else if(i[0] == B)
						KnowBlockPath[cnt].PathState = BACKTURN;
					
					KnowBlockPath[cnt].Position = DiagPos[0];
					KnowBlockPath[cnt].MouseDir = DiagDir[0];
				}
				else//대각진입, 90
				{
					if((gTurnState == R180) || (gTurnState == L180) || (gTurnState == R135IN) || (gTurnState == L135IN))
					{
						KnowBlockPath[cnt].Position = DiagPos[2];
						KnowBlockPath[cnt].MouseDir = DiagDir[2];	
					}
					else //if R45IN , L45IN, R90, L90
					{
						KnowBlockPath[cnt].Position = DiagPos[1];
						KnowBlockPath[cnt].MouseDir = DiagDir[1];
					}

					
				}
			}
			else//대각주행.탈출
			{
				//대각 직진 블럭수 세기
				if((gTurnState == RDRUN) || (gTurnState == LDRUN))
				{
					if((YetTurnState == RDRUN) || (YetTurnState == LDRUN))
						KnowBlockPath[--cnt].PathCnt++;
					else
						KnowBlockPath[cnt].PathCnt = 1;
				}
						
				KnowBlockPath[cnt].Position = DiagPos[1];
				KnowBlockPath[cnt].MouseDir = DiagDir[1];
				
			}

		}

		cnt++;
		
	}

}


void InitSearchQueueList(void)
{
	MapSearchQueueHead = (QueueList *)far_malloc(sizeof(QueueList));
	if(MapSearchQueueHead == NULL)
	{
		MOTOR_OFF;
		while(TRUE)
		{
			VFDPrintf("merr");
		}
	}
		
	MapSearchQueueHead->Position = GARBAGDATA;
	MapSearchQueueHead->Weight = GARBAGDATA;
	MapSearchQueueHead->Next = NULL;

}

void RunPathMake(void)
{
	Uint16 cnt;
	Uint16 x;
	Uint16 Goal[4] = {0,0,0,0};
	Uint16 LowWeight = 0x0fff;
	Uint16 GoalPosition = 0;
	Uint16 WeightMin;
	Uint16 NextWeight;
	Uint16 AbsoluteDir;
	Uint16 NextTurn;
	Uint16 NextPos;
	Uint16 MouseDir;
	Uint16 Position;
	//Uint16 PositionBuffer[256];
	//Uint16 temp = 0;

	Uint16 i[4];
	Uint16 BlockCnt;
	//Uint16 KnowBlockDiag;
	Uint16 DiagPos[4];
	Uint16 DiagDir[4];
	Uint16 YetTurnState;
	Uint16 YetMouseHead;
	Uint16 LastPath = OFF;


	memset((void *)gMazeMap,0x00,sizeof(gMazeMap));
	memset((void *)gMazeMapBackUp,0x00,sizeof(gMazeMap));

	SpiReadRom(MAP_BACKUP_ADDRESS, 0, 256, (Uint16 *)gMazeMapBackUp);
	
	for(cnt = 0; cnt < 256; cnt++)
	{
		if((gMazeMapBackUp[cnt] >> 4) & 0x01)
			gMazeMap[cnt] |= gMazeMapBackUp[cnt] & 0x0f;
		else//가보지 않은 블럭
		{
			gMazeMap[cnt] = DIR_N | DIR_E | DIR_S | DIR_W;
			for(x = 0; x < 4; x++)
			{
				switch(x) 
				{

					case 0://north
						if(!((cnt & 0x0f) == 0x0f))
							gMazeMap[cnt + gMoveTable[x]] |= DIR_S;
						break;

					case 1://east
						if(!((cnt & 0xf0) == 0xf0))
							gMazeMap[cnt + gMoveTable[x]] |= DIR_W ;
						break;

					case 2://south
						if(!((cnt & 0x0f) == 0x00))
							gMazeMap[cnt + gMoveTable[x]] |= DIR_N;
						break;

					case 3://west
						if(!((cnt & 0xf0) == 0x00))
							gMazeMap[cnt + gMoveTable[x]] |= DIR_E;
						break;

					default:
						break;
				}
			}
		}
	}

	gSearchType = GO_START;
	gPathWeightState = ON;

	DelSearchQueueList();
	InitWeight();
	WriteMazeWeight(0);
	DelSearchQueueList();
	gPathWeightState = OFF;

	//골 찾기
	cnt = 0;
	if(!(gMazeMap[0x77] & DIR_W) || !(gMazeMap[0x77] & DIR_S))
		Goal[cnt++] = 0x77;

	if(!(gMazeMap[0x78] & DIR_W) || !(gMazeMap[0x78] & DIR_N))
		Goal[cnt++] = 0x78;

	if(!(gMazeMap[0x87] & DIR_E) || !(gMazeMap[0x87] & DIR_S))
		Goal[cnt++] = 0x87;
	
	if(!(gMazeMap[0x88] & DIR_E) || !(gMazeMap[0x88] & DIR_N))
		Goal[cnt++] = 0x88;

	for(cnt = 0; cnt < 4; cnt++)
	{
		if((LowWeight > gMapValue[Goal[cnt]].Divide.Weight) && Goal[cnt])
		{
			LowWeight = gMapValue[Goal[cnt]].Divide.Weight;	
			GoalPosition = Goal[cnt];
		}	
	}

	WeightMin = gMapValue[GoalPosition].Divide.Weight;//Goal's Weight
	Position = GoalPosition;//Goal
	
	//TxPrintf("%x\n",Position);
	TxPrintf("\nGoal = %x\n",Position);

	
	gPathBufferHead = 0;
	MouseDir = 0;//At Goal Mouse Direction
	
	memset((void *)KnowBlockPath, 0x00, sizeof(KnowBlockPath));
	
	while(TRUE)
	{
		for(x = 0; x < 4; x++)
		{
			if(!((gMazeMap[Position] & 0x0f) & gHeadTable[x]))
			{
				NextWeight = gMapValue[Position + gMoveTable[x]].Divide.Weight;
				if(NextWeight < WeightMin)
				{
					WeightMin = NextWeight;//최소무게값 업데이트
					NextPos = Position + gMoveTable[x];//갈 방향 저장.
					AbsoluteDir= x;//절대 방향
					NextTurn = (AbsoluteDir+ 4 - MouseDir) & 0x03;
				}

			}
		}

		KnowBlockPath[gPathBufferHead].Position = Position;//YetPosition
		Position = NextPos;
		MouseDir = AbsoluteDir;
		gPathBufferHead++;

		if(gPathBufferHead > 255) 
		{
			TxPrintf("error\n"); 
			while(TRUE)
			{
				VFDPrintf("E2nd");
			}
			//TRACE("Error\n");
			//break;
		}
		if(NextPos == 0)
		{
			KnowBlockPath[gPathBufferHead++].Position = 0;
			break;
		}
	
	}
	
	
	//for(cnt = 0; cnt < gPathBufferHead; cnt++)
	//	TxPrintf("%x\n",KnowBlockPath[cnt].Position);

	for(cnt = 0; cnt < 256; cnt++)
		gMapValue[cnt].Divide.Weight = 0xffff;

	for(cnt = 0; cnt < gPathBufferHead; cnt++)
	{
		gMapValue[KnowBlockPath[gPathBufferHead -cnt-1].Position].Divide.Weight = 0xfff - cnt; 
	}

	//TxPrintf("OK\n\n\n\n");

	WeightMin = 0xfff;
	gPathBufferHead = 0;
	Position = 0;
	MouseDir = 0;//At Start Mouse Direction
	memset((void *)KnowBlockPath, 0x00, sizeof(KnowBlockPath));
	
	while(TRUE)
	{
		for(x = 0; x < 4; x++)
		{
			if(!((gMazeMap[Position] & 0x0f) & gHeadTable[x]))
			{
				NextWeight = gMapValue[Position + gMoveTable[x]].Divide.Weight;
				if(NextWeight < WeightMin)
				{
					WeightMin = NextWeight;//최소무게값 업데이트
					NextPos = Position + gMoveTable[x];//갈 방향 저장.
					AbsoluteDir= x;//절대 방향
					NextTurn = (AbsoluteDir+ 4 - MouseDir) & 0x03;
				}

			}
		}

		KnowBlockPath[gPathBufferHead].Position = Position;//YetPosition
		Position = NextPos;
		KnowBlockPath[gPathBufferHead].MouseDir = MouseDir;//YetMouseDir
		MouseDir = AbsoluteDir;
		KnowBlockPath[gPathBufferHead].TurnDir = NextTurn;
		gPathBufferHead++;


		if(gPathBufferHead > 255) 
		{
			TxPrintf("error\n"); 
			while(TRUE)
			{
				VFDPrintf("E2nd");
			}
			//TRACE("Error\n");
			//break;
		}
		if(NextPos == GoalPosition) 
		{
			KnowBlockPath[gPathBufferHead].Position = GoalPosition;		
			KnowBlockPath[gPathBufferHead].MouseDir = MouseDir;
			KnowBlockPath[gPathBufferHead++].TurnDir = STRAIGHT;//골은 무조건 직진...
			break;
		}
	
	}
	// 2차 도착후 시작점 돌아오기 탐색턴..위해서..
	gMouseDir = MouseDir;
	gMousePosition = GoalPosition + gMoveTable[MouseDir];

	TxPrintf("%x %x\n",gMouseDir, gMousePosition);
	
	//for(cnt = 0; cnt < gPathBufferHead; cnt++)
	//	TRACE("%x %x %x\n",KnowBlockPath[cnt].Position,KnowBlockPath[cnt].TurnDir,KnowBlockPath[cnt].MouseDir);
	
	gMouseHead = DirectRunState;
	x = 0;
	cnt = 0;
	gTurnState = BACKTURN;

	while(TRUE)//KnowBlockDiag == ON)
	{
		if((x+3) < gPathBufferHead)
		{
			i[3] = KnowBlockPath[x+3].TurnDir;
			i[2] = KnowBlockPath[x+2].TurnDir;
			i[1] = KnowBlockPath[x+1].TurnDir;
			i[0] = KnowBlockPath[x].TurnDir;
			
			DiagPos[3] = KnowBlockPath[x+3].Position;
			DiagPos[2] = KnowBlockPath[x+2].Position;
			DiagPos[1] = KnowBlockPath[x+1].Position;
			DiagPos[0] = KnowBlockPath[x].Position;
			
			DiagDir[3] = KnowBlockPath[x+3].MouseDir;
			DiagDir[2] = KnowBlockPath[x+2].MouseDir;
			DiagDir[1] = KnowBlockPath[x+1].MouseDir;
			DiagDir[0] = KnowBlockPath[x].MouseDir;
		}

		else if((x+2) < gPathBufferHead)
		{
			i[3] = GARBAGDATA;
			i[2] = KnowBlockPath[x+2].TurnDir;
			i[1] = KnowBlockPath[x+1].TurnDir;
			i[0] = KnowBlockPath[x].TurnDir;	

			DiagPos[2] = KnowBlockPath[x+2].Position;
			DiagPos[1] = KnowBlockPath[x+1].Position;
			DiagPos[0] = KnowBlockPath[x].Position;
			
			DiagDir[2] = KnowBlockPath[x+2].MouseDir;
			DiagDir[1] = KnowBlockPath[x+1].MouseDir;
			DiagDir[0] = KnowBlockPath[x].MouseDir;

			if((i[2] != F) && (gPathBufferHead == (x+3)) && (gMouseHead == DirectRunState))
				LastPath = ON;
			
		}
		else if((x+1) < gPathBufferHead)
		{
			
			i[3] = GARBAGDATA;
			i[2] = GARBAGDATA;
			i[1] = KnowBlockPath[x+1].TurnDir;
			i[0] = KnowBlockPath[x].TurnDir;
			DiagPos[1] = KnowBlockPath[x+1].Position;
			DiagPos[0] = KnowBlockPath[x].Position;
			DiagDir[1] = KnowBlockPath[x+1].MouseDir;
			DiagDir[0] = KnowBlockPath[x].MouseDir;

			if(gPathBufferHead == (x+2))
				LastPath = ON;
		}
		else if(x < gPathBufferHead)
		{
			i[3] = GARBAGDATA;
			i[2] = GARBAGDATA;
			i[1] = GARBAGDATA;
			i[0] = KnowBlockPath[x].TurnDir;
			DiagPos[0] = KnowBlockPath[x].Position;
			DiagDir[0] = KnowBlockPath[x].MouseDir;

			if(gPathBufferHead == (x+1))
				LastPath = ON;
		}
		else
		{
			gPathBufferHead = cnt;
			KnowBlockPath[cnt].PathState = STRAIGHT;//마지막 골에서 직진거리 유지..(한블록  일 경우)..
			break;
		}

		YetTurnState = gTurnState;
		YetMouseHead = gMouseHead;
		
		if(gMouseHead == DirectRunState)
		{
			if((i[0] == F) && (LastPath == OFF))//F
			{				
				if(i[1] == R)//FR
				{
					
					if(i[2] == R)//FRR
					{
						if(i[3] == F)//FRRF
						{
							//RIGHT180	ORTH->ORTH
							gMouseHead = DirectRunState;
							gTurnState = R180;
						}
						else//FRR
						{
							//RIGHT135IN	ORTH->DIAG
							gMouseHead = DiagonalRunState;
							gTurnState = R135IN;
						}
						x+=3;
					}
					else if(i[2] == F)//FRF
					{
						//RIGHT90	ORTH->ORTH
						gMouseHead = DirectRunState;
						gTurnState = R90;
						x+=2;
					}
					else//FR
					{
						//RIGHT45IN ORTH->DIAG
						gMouseHead = DiagonalRunState;
						gTurnState = R45IN;
						x+=2;
					}
				}
				else if(i[1] == L)//FL
				{
					if(i[2] == L)//FLL
					{
						if(i[3] == F)//FLLF
						{
							//LEFT180	ORTH->ORTH
							gMouseHead = DirectRunState;
							gTurnState = L180;
							x+=3;
						}
						else//FLL
						{
							//LEFT135IN ORTH->DIAG
							gMouseHead = DiagonalRunState;
							gTurnState = L135IN;
							x+=3;
						}
					}
					else if(i[2] == F)//FLF
					{
						//LEFT90	ORTH->ORTH
						gMouseHead = DirectRunState;
						gTurnState = L90;
						x+=2;
					}
					else//FL
					{
						//LEFT45IN	ORTH->DIAG
						gMouseHead = DiagonalRunState;
						gTurnState = L45IN;
						x+=2;
					}
					
				}
				else if(i[1] == F)//FF
				{
					gMouseHead = DirectRunState;
					gTurnState = STRAIGHT;
					
					if(i[2] == F)//FFF
					{
						if(i[3] == F)//FFFF
						{
							//STRAIGHT FFFF
							x+=3;
							BlockCnt = 3;
						}
						else
						{
							//STRAIGHT FFF
							x+=2;
							BlockCnt = 2;
						}

					}
					else
					{
						//STRAIGHT FF
						x+=1;
						BlockCnt = 1;
					}
					//STRAIGHT
					
				}
				
				else
				{
					//F 하나만 남았을 경우
					gMouseHead = DirectRunState;
					gTurnState = STRAIGHT;
					x+=1;
					BlockCnt = 1;
					if(YetTurnState == STRAIGHT)
					{
						KnowBlockPath[cnt-1].Position = DiagPos[0];
						KnowBlockPath[cnt-1].MouseDir = DiagDir[0];

					}
					else
					{
						KnowBlockPath[cnt].Position = DiagPos[0];
						KnowBlockPath[cnt].MouseDir = DiagDir[0];
					}
				}
				
				//패스 찾을때 한블록 직진 삽입..
				if(gTurnState != STRAIGHT)
				{
					if(YetTurnState == STRAIGHT)
					{
						KnowBlockPath[cnt-1].PathCnt++;
						KnowBlockPath[cnt-1].MouseDir = DiagDir[0];

						if(KnowBlockPath[cnt -1].Position == 0x00)
							;
						else
							KnowBlockPath[cnt-1].Position = DiagPos[0];
						
					}
					else
					{
						KnowBlockPath[cnt].PathState = STRAIGHT;
						KnowBlockPath[cnt].Position = DiagPos[0];
						KnowBlockPath[cnt].MouseDir = DiagDir[0];
						KnowBlockPath[cnt++].PathCnt = 1;
					}

				}
				

			}
			else//일치하는 것이 없을때.
			{
				gMouseHead = DirectRunState;
				gTurnState = NMATCH;
				if(i[0] == F)
				{
					gTurnState = STRAIGHT;
					BlockCnt = 1;
				}
				else
					;
				
				x+=1;
			}	
		
		}
		else//DIAGONAL
		{
			if(i[0] == R)//R
			{
				if(i[1] == R)//RR
				{
					if(i[2] == F)//RRF
					{
						//RIGHT135OUT	DIAG->ORTH
						gMouseHead = DirectRunState;
						gTurnState = R135OUT;
						x+=2;
					}
					else//RR
					{
						//RIGHTDIAG90	DIAG->DIAG
						if(LastPath == OFF)
						{
							gMouseHead = DiagonalRunState;
							gTurnState = RD90;
						}
						else
						{
							gMouseHead = DirectRunState;
							gTurnState = RCbr135OUT;
						}
						x+=2;
					}
				}
				else if(i[1] == F)//RF
				{
					//RIGHT45OUT	DIAG->ORTH
					gMouseHead = DirectRunState;
					gTurnState = R45OUT;
					x+=1;
				}
				else if(i[1] == L)//RL
				{
					//Diag Straight DIAG->DIAG
					gMouseHead = DiagonalRunState;
					gTurnState = RDRUN;
					x+=1;
				}
				else//R
				{

					//CobraR45OUT
					gMouseHead = DirectRunState;
					gTurnState = RCbr45OUT;
					x+=1;
				}
					
				
			}
			else if(i[0] == L)//L
			{
				if(i[1] == L)//LL
				{
					if(i[2] == F)//LLF
					{
						//LEFT135OUT	DIAG->ORTH
						gMouseHead = DirectRunState;
						gTurnState = L135OUT;
					}
					else if((i[2] == B) && (gSearchEndState == ON))
					{
						gMouseHead = DirectRunState;
						gTurnState = L135OUT;	
					}
					else//LL
					{
						//LEFTDIAG90	DIAG->DIAG
						if(LastPath == OFF)
						{
							gMouseHead = DiagonalRunState;
							gTurnState = LD90;
						}
						else
						{
							gMouseHead = DirectRunState;
							gTurnState = LCbr135OUT;
						}
					}
					x+= 2;
				}
				else if(i[1] == F)//LF
				{
					//LEFT45OUT 	DIAG->ORTH
					gMouseHead = DirectRunState;
					gTurnState = L45OUT;
					x+=1;
				}
				else if(i[1] == R)//LR
				{
					//Diag Straight
					gMouseHead = DiagonalRunState;
					gTurnState = LDRUN;
					x+=1;
				}
				else if((i[1] == B) && (gSearchEndState == ON))
				{
					gMouseHead = DirectRunState;
					gTurnState = L45OUT;
					x+=1;
				}
				else
				{
					//CobraL45OUT
					gMouseHead = DirectRunState;
					gTurnState = LCbr45OUT;
					x+=1;
				}
			}
			else
				;
		}
		
		if((YetTurnState == STRAIGHT) && (gTurnState == STRAIGHT))
		{
			KnowBlockPath[--cnt].PathCnt += BlockCnt;
			if(KnowBlockPath[cnt].Position == 0x00)
				;
			else
				KnowBlockPath[cnt].Position = DiagPos[0];
		}
		else if(gTurnState == STRAIGHT)
		{
			KnowBlockPath[cnt].PathState = gTurnState;
			KnowBlockPath[cnt].PathCnt = BlockCnt;
			KnowBlockPath[cnt].Position = DiagPos[0];
		}
		else
		{
			KnowBlockPath[cnt].PathState = gTurnState;
			
			if(YetMouseHead == DirectRunState)
			{
				if(gTurnState == NMATCH)
				{
					if(i[0] == L)
						KnowBlockPath[cnt].PathState = L90;
					else if(i[0] == R)
						KnowBlockPath[cnt].PathState = R90;
					else if(i[0] == B)
						KnowBlockPath[cnt].PathState = BACKTURN;
					
					KnowBlockPath[cnt].Position = DiagPos[0];
					KnowBlockPath[cnt].MouseDir = DiagDir[0];
				}
				else//대각진입, 90
				{
					if((gTurnState == R180) || (gTurnState == L180) || (gTurnState == R135IN) || (gTurnState == L135IN))
					{
						KnowBlockPath[cnt].Position = DiagPos[2];
						KnowBlockPath[cnt].MouseDir = DiagDir[2];	
					}
					else //if R45IN , L135IN, R90, L90
					{
						KnowBlockPath[cnt].Position = DiagPos[1];
						KnowBlockPath[cnt].MouseDir = DiagDir[1];
					}

					
				}
			}
			else//대각주행.탈출
			{
				//대각 직진 블럭수 세기
				if((gTurnState == RDRUN) || (gTurnState == LDRUN))
				{
					if((YetTurnState == RDRUN) || (YetTurnState == LDRUN))
						KnowBlockPath[--cnt].PathCnt++;
					else
						KnowBlockPath[cnt].PathCnt = 1;
				}
						
				KnowBlockPath[cnt].Position = DiagPos[1];
				KnowBlockPath[cnt].MouseDir = DiagDir[1];			
			}

		}

		cnt++;
		
	}

	for(cnt = 0; cnt < gPathBufferHead; cnt++)
	{
		TxPrintf("%d  TURN : %s\tDIR : %d  POS : %x CNT : %d\n", cnt, PathState[KnowBlockPath[cnt].PathState],KnowBlockPath[cnt].MouseDir,KnowBlockPath[cnt].Position,KnowBlockPath[cnt].PathCnt);
	}

	
	memset((void *)gMazeMap,0x00,sizeof(gMazeMap));
	memset((void *)gMazeMapBackUp,0x00,sizeof(gMazeMap));

	SpiReadRom(MAP_BACKUP_ADDRESS, 0, 256, (Uint16 *)gMazeMapBackUp);

	
	for(cnt = 0; cnt < 256; cnt++)
		gMazeMap[cnt] = gMazeMapBackUp[cnt];
	
}



void InsertSearchQueueList(Uint16 pos, Uint16 weight)
{
	volatile QueueList *NextList;
	volatile QueueList *SearchList;
	volatile QueueList *YetSearchList;

	//큐가 비어있을경우 그냥 삽입
	if(MapSearchQueueHead->Next == NULL)
	{
		EndPos = (QueueList *)far_malloc(sizeof(QueueList));
		if(EndPos == NULL)
		{
			MOTOR_OFF;
			while(TRUE)
			{
				VFDPrintf("merr");
			}
		}
		
		EndPos->Position = pos;
		EndPos->Weight = weight;
		EndPos->Next = NULL;

		MapSearchQueueHead->Next = EndPos;
		
	}
	//큐에 리스트가 있을경우 무게값 비교후 삽입.
	else
	{
		YetSearchList = MapSearchQueueHead;
		SearchList = MapSearchQueueHead->Next;

		while(SearchList != NULL)
		{
			if(SearchList->Weight <= weight)
			{
				YetSearchList = SearchList;
				SearchList = SearchList->Next;
				
			}
			else
				break;
		}
			
		NextList = (QueueList *)far_malloc(sizeof(QueueList));
		if(NextList == NULL)
		{
			MOTOR_OFF;
			while(TRUE)
			{
				VFDPrintf("merr");
			}
		}
		
		NextList->Position = pos;
		NextList->Weight = weight;

		if(SearchList == NULL)
		{
			NextList->Next = NULL;
			EndPos->Next = NextList;
			EndPos = NextList;

		}
		else
		{
			NextList->Next = SearchList;
			YetSearchList->Next = NextList;
		}
			
	}
}

int16 PopSearchQueueList(void)
{
	volatile QueueList *PopList;
	int16 temp = GARBAGDATA;

	
	PopList = MapSearchQueueHead->Next;

	if(PopList == NULL)
	{
		temp = -1;
	}
	else
	{
		MapSearchQueueHead->Next = PopList->Next;

		temp = PopList->Position;
		far_free((void *)PopList);
	}
	
	return temp;
}

void DelSearchQueueList(void)
{
	while(PopSearchQueueList() != -1)
		;
}

