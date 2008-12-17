// TI File $Revision: /main/1 $
// Checkin $Date: December 1, 2004   11:11:30 $
//###########################################################################
//
// FILE:	DSP280x_Adc.c
//
// TITLE:	DSP280x ADC Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP280x V1.10 $
// $Release Date: April 18, 2005 $
//###########################################################################

#include "DSP280x_Device.h"     // DSP280x Headerfile Include File
#include "DSP280x_Examples.h"   // DSP280x Examples Include File

#define ADC_usDELAY  7000L

//---------------------------------------------------------------------------
// InitAdc: 
//---------------------------------------------------------------------------
// This function initializes ADC to a known state.
//
// PLEASE NOTE, ADC INIT IS DIFFERENT ON 281x vs 280x DEVICES!!!
//

//ADC chanel
#define SEN_AD0		0
#define SEN_AD1		1
#define SEN_AD2		2
#define SEN_AD3		8
#define SEN_AD4		9
#define SEN_AD5		10
#define BATT_AD		3
#define GYRO_AD		11


void InitAdc(void)
{
    //extern void DSP28x_usDelay(Uint32 Count);

    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed 
	// after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the 
	// CPU_CLOCK_SPEED define statement in the DSP280x_Examples.h file must 
	// contain the correct CPU clock period in nanoseconds. 


	/*
	AdcRegs.ADCTRL1.bit.RESET = 0;//not reset
	AdcRegs.ADCTRL1.bit.SUSMOD = 0;//Emulation suspend off
	AdcRegs.ADCTRL1.bit.ACQ_PS = 1;//value +1 times th ADCLK period. 2cycle
	AdcRegs.ADCTRL1.bit.CPS = 0;//ADCCLK = F/1
	AdcRegs.ADCTRL1.bit.CONT_RUN = 1;//Continuous conversion mode
	AdcRegs.ADCTRL1.bit.SEQ_OVRD = 0;//override off
	AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;//Cascaded mode
	*/
	AdcRegs.ADCTRL1.all = 0x0150;
	
	AdcRegs.ADCTRL2.all = 0x0;

	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 1;

	AdcRegs.ADCMAXCONV.all = 15;//16°³
	AdcRegs.ADCREFSEL.all = 0;//internal reference selected


	/*
	AdcRegs.ADCTRL3.bit.ADCBGRFDN = 3;//The bandgap and reference circuitry is powered up
	AdcRegs.ADCTRL3.bit.ADCPWDN = 1;//The analog circuitry inside the core is powered up
	AdcRegs.ADCTRL3.bit.ADCCLKPS = 2;//hspclk/4 = 12.5MHz  T ; 160ns
	AdcRegs.ADCTRL3.bit.SMODE_SEL = 0;//Sequential sampling mode is selected
	*/

	AdcRegs.ADCTRL3.all = 0x00E4;  // Power up bandgap/reference/ADC circuits
	
    DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels

	
}	

//===========================================================================
// End of file.
//===========================================================================
