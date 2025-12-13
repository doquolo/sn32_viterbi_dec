#ifndef __SEGMENT_H
#define __SEGMENT_H

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN32F400.h>

/*_____ D E F I N I T I O N S ______________________________________________*/
#define	SEG_A		0x01
#define	SEG_B		0x02
#define	SEG_C		0x04
#define	SEG_D		0x08
#define	SEG_E		0x10
#define	SEG_F		0x20
#define	SEG_G		0x40
#define	SEG_H		0x80

/*_____ M A C R O S ________________________________________________________*/

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern uint8_t segment_buff[4];
extern const uint8_t SEGMENT_TABLE[];
void Digital_DisplayDEC(uint16_t dec);
void Digital_DisplayHEX(uint16_t dec);

void Digital_Scan(void);
#endif	/*__SN32F400_CMP_H*/
