

MEMORY
{
PAGE 0: 

	
	OTP(RX)         : origin = 0x3D7800, length = 0x000400     
	USERFLASH(RX)   : origin = 0x3E8002, length = 0x00BFF0     	/* userprogram text, etc */
	MONITOR(RX)     : origin = 0x3F4000, length = 0x003F80		/* Monitor program */
	BEGIN(RX)       : origin = 0x3E8000, length = 0x000002     

	ROM(RX)         : origin = 0x3FF000, length = 0x000FC0     
	RESET(R)        : origin = 0x3FFFC0, length = 0x000002     
	VECTORS(R)      : origin = 0x3FFFC2, length = 0x00003E     

	_RSIDEEDGE(WRXI): origin = 0x000600, length = 0x00000A
	_LSIDEEDGE(WRXI): origin = 0x00060A, length = 0x00000A
	_RDIAGEDGE(WRXI): origin = 0x000614, length = 0x00000A
	_LDIAGEDGE(WRXI): origin = 0x00061E, length = 0x00000A

	RAMM1(WRXI)		: origin = 0x000628, length = 0x0001D8		/* ramfuncs0 */
	RAML_H(WRXI) 	: origin = 0x0080B4, length = 0x003F4C      /* ramfuncs1, bss, heap, etc */

PAGE 1:

	RAMM0(WRXI)     : origin = 0x000000, length = 0x000600		/*stack*/

	_INF_LED(WRXI)	: origin = 0x008000, length = 0x000B4
	
}

SECTIONS
{

	.cinit				: > USERFLASH	PAGE = 0
	.pinit              : > USERFLASH	PAGE = 0
	.text               : > USERFLASH	PAGE = 0
	codestart           : > BEGIN		PAGE = 0
	ramfuncs0            : LOAD = USERFLASH,
							RUN = RAMM1, 
							LOAD_START(_RamfuncsLoadStart0),
							LOAD_END(_RamfuncsLoadEnd0),
							RUN_START(_RamfuncsRunStart0),
							PAGE = 0

	ramfuncs1           : LOAD = USERFLASH,
							RUN = RAML_H, 
							LOAD_START(_RamfuncsLoadStart1),
							LOAD_END(_RamfuncsLoadEnd1),
							RUN_START(_RamfuncsRunStart1),
							PAGE = 0                         

	.stack				: > RAMM0				PAGE = 1
	.ebss               : > RAML_H 		PAGE = 0
	.esysmem            : > RAML_H		PAGE = 0
	.sysmem			   	: > RAML_H				PAGE = 0

	.econst             : > USERFLASH   PAGE = 0
	.switch             : > USERFLASH   PAGE = 0      

	IQmath              : > USERFLASH   PAGE = 0                  
	IQmathTables        : > ROM         PAGE = 0, TYPE = NOLOAD   

	.reset              : > RESET,      PAGE = 0, TYPE = DSECT
	vectors             : > VECTORS     PAGE = 0, TYPE = DSECT

	INF_LED				: > _INF_LED		PAGE = 1	
	
	RSIDEEDGE			: > _RSIDEEDGE		PAGE = 0
	LSIDEEDGE			: > _LSIDEEDGE		PAGE = 0
	RDIAGEDGE			: > _RDIAGEDGE		PAGE = 0
	LDIAGEDGE			: > _LDIAGEDGE		PAGE = 0


}

