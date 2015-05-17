/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "usb_istr.h"
#include "usb_lib.h"
#include "usb_pwr.h"
#include "DW1000.h"
#include "USART.h"
#include "SPI.h"
#include <stdio.h>
#include "delay.h"

// USB
extern volatile uint8_t PrevXferComplete;
uint8_t int_Send_Buffer[2];

// Common
volatile u8 time_up = 0;
u8 status_flag = IDLE;
u8 distance_flag = IDLE;
extern u8 Rx_Buff[128];
extern u8 Tx_Buff[128];
extern u8 Sequence_Number;
extern u8 mac[8];
u8 usart_buffer[64];
u8 usart_index;
u8 usart_status;

extern u32 Tx_stp_L;
extern u8 Tx_stp_H;
extern u32 Rx_stp_L;
extern u8 Rx_stp_H;

extern u32 Tx_stp_LT[3];
extern u8 Tx_stp_HT[3];
extern u32 Rx_stp_LT[3];
extern u8 Rx_stp_HT[3];
extern u32 LS_DATA[3];

extern u16 std_noise;
extern	u16 fp_ampl1;
extern	u16 fp_ampl2;
extern	u16 fp_ampl3;
extern	u16 cir_mxg;
extern	u16 rxpacc;
extern	double fppl;
extern	double rxl;
extern const u8 broadcast_addr[8];

extern u32 data[16];
/*
0:已完成处理
1：正在接收
2：已完成接收，没有完成处理


*/
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
	* @brief  This function handles Memory Manage exception.
	* @param  None
	* @retval None
*/
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
	* @brief  This function handles Bus Fault exception.
	* @param  None
	* @retval None
*/
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
	* @brief  This function handles Usage Fault exception.
	* @param  None
	* @retval None
*/
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
	* @brief  This function handles SVCall exception.
	* @param  None
	* @retval None
*/
void SVC_Handler(void)
{
}

/**
	* @brief  This function handles Debug Monitor exception.
	* @param  None
	* @retval None
*/
void DebugMon_Handler(void)
{
}

/**
	* @brief  This function handles PendSVC exception.
	* @param  None
	* @retval None
*/
void PendSV_Handler(void)
{
}

/**
	* @brief  This function handles SysTick Handler.
	* @param  None
	* @retval None
*/

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
	* @brief  This function handles PPP interrupt request.
	* @param  None
	* @retval None
*/

void EXTI1_IRQHandler(void)
{
	u32 status;
	u8 tmp;
	u16 size;
	// u16 pl_size;
	static u8 *dst;
	static u8 *src; // need improvement
	u8 *payload;
	int i;
	int for_me = 1;
	EXTI_ClearITPendingBit(EXTI_Line1);
	// enter interrupt
	while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)==0)
	{
		// printf("Int Triggered.\r\n");
		read_status(&status);
		printf("status: %08X\r\n", status);
		if((status&0x00040000)==0x00040000) // LDE Err
		{
			to_IDLE();
			tmp=0x04;
			// Clear Flag
			Write_DW1000(0x0F,0x02,&tmp,1);
			printf("LDE err.\r\n");
			load_LDE();
			RX_mode_enable();
		}
		if((status&0x00000400)==0x00000400) // LDE Success
		{
			printf("LDE Success.\r\n");
			if ((distance_flag == CONFIRM_SENT_LS_REQ)||(distance_flag == SENT_LS_REQ))
			{
				Read_DW1000(0x15,0x00,(u8 *)(&Rx_stp_L),4);
				Read_DW1000(0x15,0x04,&Rx_stp_H,1);
			}
		}

		if((status&0x0000C000)==0x00008000) // CRC err
		{
			tmp=0xF0;
			Write_DW1000(0x0F,0x01,&tmp,1);
			to_IDLE();
			RX_mode_enable();
			printf("CRC Failed.\r\n");
		}
		if((status&0x00006000)==0x00002000)
		{
			tmp=0x20;
			Write_DW1000(0x0F,0x01,&tmp,1);
			printf("We got a weird status: 0x%08X\r\n",status);
			// to_IDLE();
			// RX_mode_enable();
		}

		if((status&0x00000080)==0x00000080) // transmit done
		{
			// printf("Transmit done.\r\n");
			tmp=0x80;
			Write_DW1000(0x0F,0x00,&tmp,1);
			// clear the flag

			// Inform Host

			if(status_flag == SENT_LS_ACK)
			{
				printf("LS ACK\t\tSuccessfully Sent\r\n");
				status_flag = CONFIRM_SENT_LS_ACK;
				data_response(mac, src);
			}
			else if(status_flag == CONFIRM_SENT_LS_ACK)
			{
				printf("LS DATA\t\tSuccessfully Sent\r\n");
				status_flag = SENT_LS_DATA;
				status_flag = IDLE;
				to_IDLE();
				RX_mode_enable();
			}
			// currently to avoid err, cannot work as an anchor and a client at the same time
			else if(distance_flag == SENT_LS_REQ)
			{
				distance_flag = CONFIRM_SENT_LS_REQ;
				// Read Time Stamp
				printf("LS Req\t\tSuccessfully Sent\r\n");
				Read_DW1000(0x17,0x00,(u8 *)(&Tx_stp_L),4);
				Read_DW1000(0x17,0x04,&Tx_stp_H,1);
				printf("0x%8x\r\n",Tx_stp_L);
				printf("0x%2x\r\n",Tx_stp_H);
				to_IDLE();
				RX_mode_enable();
			}
			else if(distance_flag == GOT_LS_DATA)
			{
				// TODO
				// Successfully Sent LS RETURN
				distance_flag = IDLE;
				printf("LS RETURN\t\tSuccessfully Sent\r\n");
			}

		}
		else if(((status&0x00004000)==0x00004000)||((status&0x00002000)==0x00002000)) // receive done
		{
			printf("receive done.\r\n");
			to_IDLE();
			// clear flag
			tmp=0x60;
			Write_DW1000(0x0F,0x01,&tmp,1);

			// LS or Ethernet?
			// to me?
			// inform Host
			// send buffer to host

			raw_read(Rx_Buff, &size);
			printf("raw_read completed.\r\n");
			// parse_rx(Rx_Buff, size, &src, &dst, &payload, &pl_size);

			src = &(Rx_Buff[22-8]);
			dst = &(Rx_Buff[22-16]);
			payload = &(Rx_Buff[22]);
			// pl_size = (u16)(size - 22);

// printf("\r\nGot a Frame:\r\n\
// Frame type: %X\r\n\
// Frame size: %d\r\n\
// Frame Header: %02X %02X\r\n\
// src: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n\
// dst: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n\
// pl_size: %d\r\n\
// first byte of pl: %02X\r\n",
// Rx_Buff[0]>>5, size, Rx_Buff[0], Rx_Buff[1],\
// src[0], src[1], src[2], src[3], src[4], src[5], src[6], src[7],\
// dst[0], dst[1], dst[2], dst[3], dst[4], dst[5], dst[6], dst[7],\
// pl_size, payload[0]);

			printf("Header: %02X\r\n", (u8)(Rx_Buff[0]&0xE0));
			for (i=0;i<8;i++)
			{
				if( (u8)(dst[i]) != (u8)(broadcast_addr[i]) )
				{
					for (i=0;i<8;i++)
					{
						if( (u8)(dst[i]) != (u8)(mac[i]) )
						{
							for_me = 0;
							break;
						}
					}
					break;
				}
			}
			if (for_me == 1)
			if ((u8)(Rx_Buff[0]&0xE0) == 0x80) // LS Frame
			{
				printf("A LS Frame.\r\n");
				if ((payload[0] == 0x00)&&(status_flag == IDLE)) // GOT LS Req
				{
					send_LS_ACK(mac, src);
					status_flag = SENT_LS_ACK;
					printf("\r\n===========Got LS Req===========\r\n");
				}
				else if ((payload[0] == 0x01)) // GOT LS ACK
				//&&((distance_flag == CONFIRM_SENT_LS_REQ)||(distance_flag == SENT_LS_REQ))
				{
					printf("\r\n===========Got LS ACK===========\r\n");

					// while((u32)(status&0x00000400) == (u32)(0))
					// {
						// Delay(50);
						// read_status(&status);
					// }

					// for (i=0;i<10;i++)
					// {
						// Delay();
					// }
					// read_status(&status);
					// printf("status before read: %08X\r\n", status);
					Read_DW1000(0x15,0x00,(u8 *)(&Rx_stp_LT[(int)(src[7]&0x0F) - 1]),4);
					Read_DW1000(0x15,0x04,&Rx_stp_HT[(int)(src[7]&0x0F) - 1],1);
					// printf("0x%8x\r\n",Rx_stp_LT[(int)(src[7]&0x0F) - 1]);
					// printf("0x%2x\r\n",Rx_stp_HT[(int)(src[7]&0x0F) - 1]);

					// printf("0x%8x\r\n",Rx_stp_L);
					// printf("0x%2x\r\n",Rx_stp_H);
					// Read_DW1000(0x15,0x00,(u8*)(&Rx_stp_L),4);
					// Read_DW1000(0x15,0x04,&Rx_stp_H,1);
					// printf("0x%8x\r\n",Rx_stp_L);
					// printf("0x%2x\r\n",Rx_stp_H);
					// Read_DW1000(0x12,0x00,(u8 *)(&std_noise),2);
					// Read_DW1000(0x12,0x02,(u8 *)(&fp_ampl2),2);
					// Read_DW1000(0x12,0x04,(u8 *)(&fp_ampl3),2);
					// Read_DW1000(0x12,0x06,(u8 *)(&cir_mxg),2);
					// Read_DW1000(0x15,0x07,(u8 *)(&fp_ampl1),2);
					// Read_DW1000(0x10,0x02,(u8 *)(&rxpacc),2);
					to_IDLE();
					RX_mode_enable();
				}
				else if (payload[0] == 0x02) // GOT LS DATA
				{
					printf("\r\n===========Got LS DATA===========\r\n");
					distance_flag = GOT_LS_DATA;
					LS_DATA[(int)(src[7]&0x0F) - 1] = *(u32 *)(payload + 1);
					printf("data: %08X\r\n",LS_DATA[(int)(src[7]&0x0F) - 1]);
					distance_measurement((int)(src[7]&0x0F) - 1);
					// quality_measurement();
					// TODO
					// sent_LS_RETURN(mac, src);

					distance_forward((int)(src[7]&0x0F) - 1, data[(int)(src[7]&0x0F) - 1]);

					distance_flag = IDLE;
					to_IDLE();
					RX_mode_enable();
				}
				else if (payload[0] == 0x03) // GOT LS RETURN
				{
					// TODO
					status_flag = IDLE;
				}
				else if (payload[0] == 0x04) // distance forward
				{
					u32 dist = (payload[1] << 24) + (payload[2] << 16) + (payload[3] << 8) + (payload[4]);
					handle_distance_forward((int)(src[7]&0x0F) - 1, dist);
				}
				else
				{
					to_IDLE();
					RX_mode_enable();
				}
			}
			else
			{
				//Here the other data processing
				to_IDLE();
				RX_mode_enable();
			}
		}
		else
		{
			to_IDLE();
			RX_mode_enable();
		}
	}
}


#ifdef TX
void TIM2_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET )
	{
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
		Location_polling();

		if (PrevXferComplete){
		PrevXferComplete = 0;
		int_Send_Buffer[0] = 0xFF;
		int_Send_Buffer[1] = 0x00;
		// /* Copy mouse position info in ENDP1 Tx Packet Memory Area*/
		// USB_SIL_Write(EP2_IN, int_Send_Buffer, 2);
		// /* Enable endpoint for transmission */
		// SetEPTxStatus(ENDP2, EP_TX_VALID);
		UserToPMABufferCopy(int_Send_Buffer, GetEPTxAddr(ENDP2), 2);
		SetEPTxCount(ENDP2, 2);
		SetEPTxValid(ENDP2);
		}
	}
}
#endif

void TIM3_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET )
	{
		TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE);
		TIM_SetCounter(TIM3,0x0000);
		TIM_Cmd(TIM3, DISABLE);
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);
		TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
		time_up = 1;
	}
}


void TIM4_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET )
	{
		TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);
		usart_status=2;
		//计数器TIM4清零,停止工作
		TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE);
		TIM_SetCounter(TIM4,0x0000);
		TIM_Cmd(TIM4, DISABLE);

		usart_handle();

	}
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		if(usart_status==0)
		{
			usart_status=1;

			//开启计数器TIM4
			TIM_ClearFlag(TIM4, TIM_FLAG_Update);
			TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
			TIM_Cmd(TIM4, ENABLE);

			usart_buffer[usart_index++]=USART1->DR;
			if(usart_index==64)
			{
				usart_index=0;
			}
		}
		else if(usart_status==1)
		{
			//计数器TIM4清零,
			TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE);
			TIM_SetCounter(TIM4,0x0000);
			TIM_ClearFlag(TIM4, TIM_FLAG_Update);
			TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);

			usart_buffer[usart_index++]=USART1->DR;
			if(usart_index==64)
			{
				usart_index=0;
			}
		}
	}
}

/*******************************************************************************
* Function Name  : USB_IRQHandler
* Description    : This function handles USB Low Priority interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS) || defined (STM32F37X)
void USB_LP_IRQHandler(void)
#else
void USB_LP_CAN1_RX0_IRQHandler(void)
#endif
{
  USB_Istr();
}

/*******************************************************************************
* Function Name  : USB_FS_WKUP_IRQHandler
* Description    : This function handles USB WakeUp interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
void USB_FS_WKUP_IRQHandler(void)
#else
void USBWakeUp_IRQHandler(void)
#endif
{
  EXTI_ClearITPendingBit(EXTI_Line18);
}

void SysTick_Handler(void)
{
	TimingDelay_Decrement();
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
