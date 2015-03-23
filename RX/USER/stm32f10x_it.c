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
#include "SPI.h"
#include "USART.h"
#include "DW1000.h"
#include <stdio.h>
u8 status_flag=0;
extern u8 usart_buffer[64];
extern u8 usart_index;
extern u8 usart_status;
extern u8 Sequence_Number;
u8 ars_counter;
u8 Receive_buffer[14];
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
void SysTick_Handler(void)
{
}

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
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */
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
 void EXTI1_IRQHandler(void)
{
	u32 status;
	u8 tmp;
	//u8 i;
	EXTI_ClearITPendingBit(EXTI_Line1);
	while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)==0)
{
	Read_DW1000(0x0F,0x00,(u8 *)(&status),4);
	/*if((status&0x00046080)==0x00000000)
	{

		to_IDLE();
		RX_mode_enable();
	} */
/*	Read_DW1000(0x0F,0x00,(u8 *)(&status),4);
	if((status&0x00040000)==0x00040000)
	{
		tmp=0x04;
		Write_DW1000(0x0F,0x02,&tmp,1);
		to_IDLE();
		RX_mode_enable();
	}  	*/
	Read_DW1000(0x0F,0x00,(u8 *)(&status),4);
	if((status&0x00006000)==0x00002000)
	{
		tmp=0x20;
		Write_DW1000(0x0F,0x01,&tmp,1);
		//printf("sth res:0x%8x\r\n",status);
		to_IDLE();
	}

	//while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)==0)
	//{
		Read_DW1000(0x0F,0x00,(u8 *)(&status),4);
		//Read_DW1000(0x0F,0x00,(u8 *)(&status),4);
		//printf("0x%4x",status);
		if((status&0x00000080)==0x00000080)
		{
			//printf("sth send:0x%8x\r\n",status);
			tmp=0x80;
			Write_DW1000(0x0F,0x00,&tmp,1);
			if(status_flag==1)
			{
				//计数器TIM3清零
				status_flag=2;	//定位应答发送成功
				printf("定位应答\t\t发送成功\r\n");
				TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE);
				TIM_SetCounter(TIM3,0x0000);
				TIM_ClearFlag(TIM3, TIM_FLAG_Update);
				TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
				data_response();
			}
			else if(status_flag==2)
			{
				printf("定位数据%d\t\t发送成功\r\n",ars_counter+1);
				status_flag=3;	 //定位数据发送成功
				//计数器TIM3清零
				TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE);
				TIM_SetCounter(TIM3,0x0000);
				TIM_ClearFlag(TIM3, TIM_FLAG_Update);
				TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
			}
		}

		Read_DW1000(0x0F,0x00,(u8 *)(&status),4);
		if((status&0x00004000)==0x00004000)
		{
			//printf("sth done:0x%8x\r\n",status);
			//Read_DW1000(0x10,0x00,&tmp,1);
			//printf("0x%02x\r\n",tmp);
			tmp=0x60;
			Write_DW1000(0x0F,0x01,&tmp,1);
			//printf("%8x\r\n",status) ;
			//to_IDLE();
			//RX_mode_enable();

			Read_DW1000(0x11,0x00,Receive_buffer,14);

			//Read_DW1000(0x0f,0x03,&tmp,1);
			//tmp=((tmp&0x40)==0x00);
			//Write_DW1000(0x0D,0x03,&tmp,1);
			/*for(i=0;i<14;i++)
			{
				printf("%02x",Receive_buffer[i]);
			}
			printf("\r\n")  ;  	 */

			if(((Receive_buffer[0]&0x07)==0x04)&&(status_flag==3)) //如果是ACK
			{
				if(Receive_buffer[2]==Sequence_Number)
				{
					//计数器TIM3清零,停止工作
					TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE);
					TIM_SetCounter(TIM3,0x0000);
					TIM_Cmd(TIM3, DISABLE);
					TIM_ClearFlag(TIM3, TIM_FLAG_Update);
					TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
					status_flag=0; //对端已接收
					ars_counter=0;
					printf("定位数据\t\t对端已接收\r\n");
					printf("==================================\r\n");
					to_IDLE();
					RX_mode_enable();
				}
			}
			else if(((Receive_buffer[0]&0x07)==0x01)&&(status_flag==0)&&(Receive_buffer[9]==0x38))//申请数据
			{

				ACK_send();

				//开启计数器TIM3
				Sequence_Number=Receive_buffer[2];
				status_flag=1;//收到定位申请
				printf("\r\n===========收到定位申请===========\r\n");

			}
			else
			{
				to_IDLE();
				tmp=0x01;
				Write_DW1000(0x0D,0x01,&tmp,1);
			}
		}
		//Read_DW1000(0x0F,0x00,(u8 *)(&status),2);

}
}
//status_flag=0;对端已接收
//status_flag=1;收到定位申请
//status_flag=2;定位应答发送成功
//status_flag=3;定位数据发送成功
void TIM3_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET )
	{
		TIM_ClearITPendingBit(TIM3,TIM_FLAG_Update);
		if(status_flag==1)
		{
			printf("定位应答\t\t发送超时\r\n");
			//计数器TIM3清零,停止工作
			TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE);
			TIM_SetCounter(TIM3,0x0000);
			TIM_Cmd(TIM3, DISABLE);
			TIM_ClearFlag(TIM3, TIM_FLAG_Update);
			TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
			status_flag=0;
			to_IDLE();
			RX_mode_enable();
		}
		else if(status_flag==2)
		{
			//to_IDLE();
			//RX_mode_enable();

			TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE);
			TIM_SetCounter(TIM3,0x0000);
			TIM_Cmd(TIM3, DISABLE);
			TIM_ClearFlag(TIM3, TIM_FLAG_Update);
			TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);

			printf("定位数据%d\t\t发送超时\r\n",ars_counter+1);
			status_flag=0;
			ars_counter=0;
			printf("==================================\r\n");

		}
		else if(status_flag==3)
		{
			//to_IDLE();
			//RX_mode_enable();
			printf("定位数据%d\t\t对端未接收\r\n",ars_counter+1);
			ars_counter++;
			if(ars_counter<3)
			{
				data_response()	;
				status_flag=2;
			}
			else
			{
				TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE);
				TIM_SetCounter(TIM3,0x0000);
				TIM_Cmd(TIM3, DISABLE);
				TIM_ClearFlag(TIM3, TIM_FLAG_Update);
				TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
				status_flag=0;
				//to_IDLE();
				//RX_mode_enable();
				Sequence_Number++;
				ars_counter=0;
				printf("==================================\r\n");
			}
		}
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

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
