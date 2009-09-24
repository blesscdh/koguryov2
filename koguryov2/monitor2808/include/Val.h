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

#define SW_T		(GpioDataRegs.GPADAT.bit.GPIO13)
#define SW_R		(GpioDataRegs.GPADAT.bit.GPIO14)
#define SW_L		(GpioDataRegs.GPADAT.bit.GPIO15)


#define LED0_ON	{GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;}
#define LED1_ON	{GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;}
#define LED2_ON	{GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;}
#define LED3_ON	{GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;}
#define LED4_ON	{GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;}


#define LED0_OFF	{GpioDataRegs.GPASET.bit.GPIO10 = 1;}
#define LED1_OFF	{GpioDataRegs.GPASET.bit.GPIO27 = 1;}
#define LED2_OFF	{GpioDataRegs.GPASET.bit.GPIO30 = 1;}
#define LED3_OFF	{GpioDataRegs.GPASET.bit.GPIO31 = 1;}
#define LED4_OFF	{GpioDataRegs.GPBSET.bit.GPIO33 = 1;}


#define FR_GREEN_ON		LED2_ON
#define FL_GREEN_ON		LED1_ON
#define RR_BLUE_ON		LED4_ON
#define RL_BLUE_ON		LED0_ON
#define M_WHITE_ON		LED3_ON

#define FR_GREEN_OFF	LED2_OFF
#define FL_GREEN_OFF	LED1_OFF
#define RR_BLUE_OFF		LED4_OFF
#define RL_BLUE_OFF		LED0_OFF
#define M_WHITE_OFF		LED3_OFF


#define LED_OFF		{GpioDataRegs.GPADAT.all |= 0xc8000400; GpioDataRegs.GPBDAT.all |= 0x02;}
#define SEN_OFF		{GpioDataRegs.GPADAT.all &= 0xfffff41f;}// 5,6,7,8,9,11 / 0100 0001 1111
#define MOTOR_OFF	{GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;}

#define CR		0x0D
#define BELL	0x07

#endif

