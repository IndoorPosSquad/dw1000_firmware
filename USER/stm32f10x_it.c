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

#include "SPI.h"
#include <stdio.h>
#include "delay.h"

#include "USART.h"

#include "CONFIG.h"

// USB

// Common

volatile u8 time_up = 0;

u8 usart_buffer[USART_BUFFER_LEN];
u16 usart_index;
u8 usart_status;

extern int debug_lvl;

// extern void Fifoput(u8* data, int len);
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
void NMI_Handler(void) {
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void) {
	/* Go to infinite loop when Hard Fault exception occurs */
	while(1) {
	}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
*/
void MemManage_Handler(void) {
	/* Go to infinite loop when Memory Manage exception occurs */
	while(1) {
	}
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
*/
void BusFault_Handler(void) {
	/* Go to infinite loop when Bus Fault exception occurs */
	while(1) {
	}
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void) {
	/* Go to infinite loop when Usage Fault exception occurs */
	while(1) {
	}
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
*/
void SVC_Handler(void) {
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void) {
}

/**
	* @brief  This function handles PendSVC exception.
	* @param  None
	* @retval None
*/
void PendSV_Handler(void) {
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

void EXTI0_IRQHandler(void) {
	handle_event();
}

#ifdef TX
void TIM2_IRQHandler(void) {
	if(TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
		#if defined(LOCATION)
		Location_polling();
		#elif defined(ETC)
		ETC_polling();
		#endif
	}
}
#endif

void TIM3_IRQHandler(void) {
	if (TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET) {
		TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
		TIM_SetCounter(TIM3, 0x0000);
		TIM_Cmd(TIM3, DISABLE);
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
		time_up = 1;
	}
}


void TIM4_IRQHandler(void) {
	if (TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);
		usart_status = 2;
		//计数器TIM4清零,停止工作
		TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
		TIM_SetCounter(TIM4, 0x0000);
		TIM_Cmd(TIM4, DISABLE);

		usart_handle();
	}
}

void USART1_IRQHandler(void) {
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		if (usart_status == 0) {
			usart_status = 1;

			//开启计数器TIM4
			TIM_ClearFlag(TIM4, TIM_FLAG_Update);
			TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM4, ENABLE);

			usart_buffer[usart_index++] = USART1->DR;
			if (usart_index == USART_BUFFER_LEN) {
				usart_index = 0;
			}
		} else if (usart_status == 1) {
			//计数器TIM4清零,
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_SetCounter(TIM4, 0x0000);
			TIM_ClearFlag(TIM4, TIM_FLAG_Update);
			TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

			usart_buffer[usart_index++] = USART1->DR;
			if (usart_index == USART_BUFFER_LEN) {
				usart_index = 0;
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

void SysTick_Handler(void) {
	TimingDelay_Decrement();
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
