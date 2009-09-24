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

void main(void)
{
	char RcvData;

	
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

	
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	MemCopy(&RamfuncsLoadStart1, &RamfuncsLoadEnd1, &RamfuncsRunStart1);
	MemCopy(&Flash28_API_LoadStart, &Flash28_API_LoadEnd, &Flash28_API_RunStart);
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
	
	IER |= M_INT1;
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


	VFDPrintf("=Hi=");

	if(SW_R == LOW)
		UserProgramStart();

	PrintMenu();
	TxPrintf("\nMonitor2808]#");

	while(TRUE)
	{		
		RcvData = SCIx_RxChar();
		SCIx_TxChar(RcvData);

		switch(RcvData)
		{
			case 'M':
			case 'm':
				PrintMenu();
				break;
				
			case 'A':
			case 'a':
				DINT;
				LED_OFF;
				TxPrintf("\n  Delete All Flash Sector.\n");
				DeleteAllFlash();
				EINT;
				break;
				
			case 'O':
			case 'o':
				DINT;
				LED_OFF;
				DeleteSecletFlash();
				EINT;
				break;
				
			case 'D':
			case 'd':
				DINT;
				LED_OFF;
				UserFlashErase_DownloadPrm();
				break;
				
			case 'G':
			case 'g':
				DINT;
				LED_OFF;
				UserProgramStart();
				break;

			case CR:
				break;
				
			default:
				TxPrintf("\n=======Wrong Command!!========\n");
				PrintMenu();
				break;
		}
		
		TxPrintf("\nMonitor2808]#");
	}

}

void InitSystem(void)
{
	InitSci();	
	InitFlashAPI2808();
	VfdInit();
	InitCpuTimers();
	ConfigCpuTimer(&CpuTimer0, 100, 100000);// sensor int
	StartCpuTimer0();
}
void ResetVariable(void)
{


}


void ResetSystem(void)
{
	LED_OFF;
	MOTOR_OFF;
	SEN_OFF;
	ResetVariable();
}
void PrintMenu(void)
{
	TxPrintf("\n");
	TxPrintf("========   TMS320F2808 Monitor V1.0   ========\n");	
	TxPrintf("  M  :	Display Menu.\n");
	TxPrintf("  A  :	Delete All Flash.( Sector B - D )\n");
	TxPrintf("  O  :	Delete Select Sector Flash. ( ex. O B )\n");
	TxPrintf("  D  :	User Sector Flash Erase & DownLoad User Program ( *.Hex )\n");
	TxPrintf("  G  :	Go User Program\n");
	TxPrintf("========   by Hwang Ha-yun Maze 8th   ========\n");	
	TxPrintf("========  blog.naver.com/hhyjjang.do  ========\n");	
	
}
