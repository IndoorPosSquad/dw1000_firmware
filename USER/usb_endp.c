/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Endpoint routines
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  */


/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "DW1000.h"
#include "string.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8_t int_Receive_Buffer[2];
volatile uint8_t PrevXferComplete = 1;
u32 data[16]={500, 707, 500};
u8 in_buf[64];
u8 out_buf[64];
u8 Buffer[128];
int count;
extern u8 mac[8];
// extern void Pop(u8* data);
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void EP1_IN_Callback(void)
{
	#if defined(RX5) || defined(RX6)
	Pop(in_buf);
	USB_SIL_Write(EP1_IN, (u8*)(in_buf), 64);
	#endif
	#ifdef RX4
	USB_SIL_Write(EP1_IN, (u8*)(data), 64);
	#endif
	// SetEPRxStatus(ENDP1, EP_RX_NAK); // NOT TX DISABLE
	SetEPTxStatus(ENDP1, EP_TX_VALID);
}

// Host to Controller Comm
// +-------+---------------+---------------+---------------+---------------+---------------+---------------+
// | Bytes |               0               |       1       |   Variable    |      N-1      |       N       |
// +-------+---+---+---+---+---+---+---+---+---------------+---------------+---------------+---------------+
// | Bits  | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |               |   Variable    |                               |
// +-------+---+---+---+---+---+---+---+---+---------------+---------------+-------------------------------+
// |       | TYPE  |  CMD  |     FRAG      | Packet Length |    Payload    |          FEC(Optional)        |
// +-------+-------+-----------------------+---------------+---------------+-------------------------------+
// 1. Frame Type
//	00 - Set Mac
//		The payload carries the mac to be set.
//	01 - Message
//		The payload carries the raw message to sent, see raw_write().
//	10 - Distance / Location poll
//	11 - Command
//		CMD - 00
//			Reboot
//		CMD - 01
//			Write Reg
//		CMD - 10
//			Read Reg
//		CMD - 11
//			Set log level?
// 2. Packet Length
//	Total length of all Payloads in a sequence in Unsigned Integer 8.
// 3. FEC(OPTIONAL)
//	CRC16 of the frame.
	
void EP3_OUT_Callback(void)
{
	USB_SIL_Read(EP3_OUT, out_buf);
	// DATA PROCESSING...
	if (out_buf[0] == 0x10) {
		memcpy(mac, out_buf+2, 8);
		set_MAC(mac);
	} else{
		if (out_buf[1] < (u8)(63)) {
			Buffer[0] = 0x90;
			memcpy(Buffer+1, out_buf+2, (u8)(out_buf[1]));
			raw_write(Buffer, (u16*)((u8)(out_buf[1])+1));
		} else if (out_buf[1] < (u8)(125)) {
				// 2 frames
				if (count == 0 && out_buf[0]==0x00) {
					count = 1;
					Buffer[0] = 0x90;
					memcpy(Buffer+1, out_buf+2, 62);
				} else if (count == 1 && out_buf[0]==0x01) {
					count = 0;
					memcpy(Buffer+63, out_buf+2, (u8)(out_buf[1]) - 62);
					raw_write(Buffer, (u16*)((u8)(out_buf[1])+1));
				} else {
					count = 0;
				}
		} else {
			// 3 frames
			if (count == 0 && out_buf[0]==0x00) {
				count = 1;
				Buffer[0] = 0x90;
				memcpy(Buffer+1, out_buf+2, 62);
			} else if (count == 1 && out_buf[0]==0x01) {
				count = 2;
				memcpy(Buffer+63, out_buf+2, 62);
			} else if (count == 2 && out_buf[0]==0x02) {
				count = 0;
				memcpy(Buffer+63, out_buf+2, (u8)(out_buf[1]) - 124);
				raw_write(Buffer, (u16*)((u8)(out_buf[1])+1));
			} else {
				count = 0;
			}
		}
	}
	SetEPRxStatus(ENDP3, EP_RX_VALID);
}

// void EP2_OUT_Callback (void)
// {
	/* Read received data (2 bytes) */  
	// USB_SIL_Read(EP2_OUT, int_Receive_Buffer);
	// incoming data processing
	// SetEPRxStatus(ENDP2, EP_RX_VALID);
// }

void EP2_IN_Callback(void)
{
	PrevXferComplete = 1;
}

	// to send an interrupt
	// if (PrevXferComplete){
	// PrevXferComplete = 0;

	// /* Copy mouse position info in ENDP1 Tx Packet Memory Area*/
	// USB_SIL_Write(EP2_IN, Data_Pointer, Data_Len);

	// /* Enable endpoint for transmission */
	// SetEPTxStatus(ENDP2, EP_TX_VALID);
	// }
	
/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SOF_Callback(void)
{
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
// done
