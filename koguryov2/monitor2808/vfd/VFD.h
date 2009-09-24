//###########################################################################
//
// FILE:    VFD.h
//
// TITLE:   KOGURYO Mouse VFD header.
//
//###########################################################################
// $Release Date: 2006.10.2 $
//###########################################################################
#ifndef __VFD_H__
#define __VFD_H__

extern void VfdCtlRegInit(Uint16 CrlReg);
extern void VfdInit(void);
extern void VfdDataLoad(char *Buff);
extern 	void VFDPrintf(char *Form, ... );


#endif
