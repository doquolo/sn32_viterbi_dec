/*_____ I N C L U D E S ____________________________________________________*/
#include <SN32F400.h>
#include <SN32F400_Def.h>
#include "..\Driver\GPIO.h"
#include "..\Driver\WDT.h"
#include "..\Driver\UART.h"
#include "..\Driver\Utility.h"
#include "..\Driver\CT16B0.h"
#include "..\Module\KeyScan.h"
#include "..\Module\Segment.h"	
#include "..\Module\Viterbi.c"
/*_____ D E C L A R A T I O N S ____________________________________________*/
void PFPA_Init(void);
void NotPinOut_GPIO_init(void);


/*_____ D E F I N I T I O N S ______________________________________________*/
#ifndef	SN32F407					//Do NOT Remove or Modify!!!
	#error Please install SONiX.SN32F4_DFP.0.0.18.pack or version >= 0.0.18
#endif
#define	PKG						SN32F407				//User SHALL modify the package on demand (SN32F407)

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
uint16_t read_key;
const uint8_t key_table[] = {KEY_1,KEY_2,KEY_3,KEY_4,KEY_5,KEY_6,KEY_7,KEY_8,KEY_9,KEY_10,KEY_11,KEY_12,KEY_13,KEY_14,KEY_15,KEY_16};
const uint8_t key_map[] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p'};
uint8_t mode;
uint8_t uart0_read_cnt = 0;
uint8_t uart1_read_cnt = 0;

uint8_t key1_found = 0;
uint8_t key2_found = 0;
uint8_t data_buffer[4];
uint8_t data_index = 0;
uint8_t process_flag = 0;
uint8_t sequence_type = 0; // 1 - enc, 2 - dec_1, 3 - dec_2
uint8_t uart1_response_count = 0;

uint8_t deint_buffer[4]; // 4 byte
uint8_t dec_buffer[2]; // 2 byte



/*****************************************************************************
* Function		: main
* Description	: uart0 send the received data
* Input			: Nones
* Output		: None
* Return		: None
* Note			: Connect P3.1(UTXD0) and P3.2(URXD0)
*****************************************************************************/
int	main(void)
{
	//User can configure System Clock with Configuration Wizard in system_SN32F400.c
	SystemInit();
	SystemCoreClockUpdate();				//Must call for SN32F400, Please do NOT remove!!!

	//Note: User can refer to ClockSwitch sample code to switch various HCLK if needed.

	PFPA_Init();										//User shall set PFPA if used, do NOT remove!!!

	//1. User SHALL define PKG on demand.
	//2. User SHALL set the status of the GPIO which are NOT pin-out to input pull-up.
	NotPinOut_GPIO_init();

	//--------------------------------------------------------------------------
	//User Code starts HERE!!!
	//enable reset pin function
	SN_SYS0->EXRSTCTRL_b.RESETDIS = 0;
	
	GPIO_Init();								//initial gpio
	
	GulNum = 0;
	for (int i = 0; i < 56; i++)		//clear Rx FIFO
	{
		bUART0_RecvFIFO[i] = 0;
		bUART1_RecvFIFO[i] = 0;
	}
	
	UART0_Init();
	SN_PFPA->UART0 = 0b00001010;		//URXD0:P32		UTXD0:P31
	UART1_Init();
	SN_PFPA->UART1 = 0b00000011;		//URXD1:P10   UTXD1: P18
	
	WDT_Init();									//Set WDT reset overflow time ~ 250ms
	
	while (1)
	{
		__WDT_FEED_VALUE;

		while (uart0_read_cnt != GulNum) {
				uint8_t current_byte = bUART0_RecvFIFO[uart0_read_cnt];
				
				if (!key1_found) {
						// Looking for first key byte (either 0xFF or 0x00)
						if (current_byte == 0xFF) {
								key1_found = 1;
								sequence_type = 1;  // Potentially 0xFF 0xFF sequence
						}
						else if (current_byte == 0x00) {
								key1_found = 1;
								sequence_type = 2;  // Potentially 0x00 0x00 sequence
						}
				}
				else if (!key2_found) {
						// Looking for second key byte (must match first)
						if ((sequence_type == 1 && current_byte == 0xFF) || 
								(sequence_type == 2 && current_byte == 0x00)) {
								key2_found = 1;
								data_index = 0;  // Reset data buffer index
						} else {
								// Not a valid sequence, reset
								key1_found = 0;
								sequence_type = 0;
						}
				}
				else {
						// Both keys found, start collecting data
						data_buffer[data_index] = current_byte;
						data_index++;
						
						if ((data_index >= 4 && sequence_type == 1) || (data_index >= 2 && sequence_type == 2)) {
								// All 4 bytes received, trigger processing
								process_flag = 1;
								
								// Don't reset state yet - keep sequence_type for processing
						}
				}
				
				uart0_read_cnt++;
				
				// Process the data when flag is set
				if (process_flag) {
						process_flag = 0;
						
						// Handle different sequence types
						if (sequence_type == 1) {
							// show dec on display
							segment_buff[0] = (SEG_B|SEG_C|SEG_D|SEG_E|SEG_G);
							segment_buff[1] = (SEG_A|SEG_E|SEG_D|SEG_F|SEG_G);
							segment_buff[2] = (SEG_G|SEG_E|SEG_D);
							// 0xFF 0xFF sequence
							deinterleave(data_buffer, deint_buffer);
							for (int i = 0; i < 4; i++) {
									UART0_SendByte(deint_buffer[i]);
							}
							viterbi_decode_hard(deint_buffer, dec_buffer, 0);
							//viterbi_decode_hard(data_buffer, dec_buffer, 0);
							for (int i = 0; i < 2; i++) {
									UART0_SendByte(dec_buffer[i]);
							}
						}
						else if (sequence_type == 2) {
								// show enc on display
								segment_buff[0] = (SEG_A|SEG_E|SEG_D|SEG_F|SEG_G);
								segment_buff[1] = (SEG_G|SEG_C|SEG_E);
								segment_buff[2] = (SEG_G|SEG_E|SEG_D);
								// 0x00 0x00 sequence
								for (int i = 0; i < 2; i++) {
										UART1_SendByte(data_buffer[i]); 
								}
						}
						
						// Reset state for next sequence
						key1_found = 0;
						key2_found = 0;
						sequence_type = 0;
						data_index = 0;
				}
		}
		
		// foward uart1 received byte to uart0
		while (uart1_read_cnt != GulNum1)		//send the received data
		{
			UART0_SendByte(bUART1_RecvFIFO[uart1_read_cnt]);
			uart1_read_cnt++;
			if(uart1_read_cnt >= UART1_BUFF_SIZE)			//uart1 buff boundary protect
			{
				uart1_read_cnt = 0;
			}
		}
			
		Digital_Scan();
		UT_DelayNx10us(10);	
	}
}
/*****************************************************************************
* Function		: NotPinOut_GPIO_init
* Description	: Set the status of the GPIO which are NOT pin-out to input pull-up. 
* Input				: None
* Output			: None
* Return			: Non
* Note				: 1. User SHALL define PKG on demand.
*****************************************************************************/
void NotPinOut_GPIO_init(void)
{
#if (PKG == SN32F405)
	//set P0.4, P0.6, P0.7 to input pull-up
	SN_GPIO0->CFG = 0x00A008AA;
	//set P1.4 ~ P1.12 to input pull-up
	SN_GPIO1->CFG = 0x000000AA;
	//set P3.8 ~ P3.11 to input pull-up
	SN_GPIO3->CFG = 0x0002AAAA;
#elif (PKG == SN32F403)
	//set P0.4 ~ P0.7 to input pull-up
	SN_GPIO0->CFG = 0x00A000AA;
	//set P1.4 ~ P1.12 to input pull-up
	SN_GPIO1->CFG = 0x000000AA;
	//set P2.5 ~ P2.6, P2.10 to input pull-up
	SN_GPIO2->CFG = 0x000A82AA;
	//set P3.0, P3.8 ~ P3.13 to input pull-up
	SN_GPIO3->CFG = 0x0000AAA8;
#endif
}

/*****************************************************************************
* Function		: HardFault_Handler
* Description	: ISR of Hard fault interrupt
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void HardFault_Handler(void)
{
	NVIC_SystemReset();
}
