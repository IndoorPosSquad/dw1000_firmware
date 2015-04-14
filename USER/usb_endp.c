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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8_t int_Receive_Buffer[2];
__IO uint8_t PrevXferComplete = 1;
u32 data[16]={500, 707, 500};
extern void  snddata(void);

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void EP1_IN_Callback(void)
{
	USB_SIL_Write(EP1_IN, (u8*)(data), 64);
	SetEPRxStatus(ENDP1, EP_RX_NAK); // NOT TX DISABLE
	SetEPTxStatus(ENDP1, EP_TX_VALID);
}

void EP1_OUT_Callback(void)
{
  // DataLen = USB_SIL_Write(EP1_OUT, Data_Pointer);
  // SetEPRxStatus(ENDP1, EP_RX_VALID);
}

void EP2_OUT_Callback (void)
{
	/* Read received data (2 bytes) */  
	// USB_SIL_Read(EP2_OUT, int_Receive_Buffer);
	// incoming data processing
	// SetEPRxStatus(ENDP2, EP_RX_VALID);
}

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
