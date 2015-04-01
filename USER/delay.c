//#include <stm32f10x_lib.h>
#include "stm32f10x.h"
#include "delay.h"

static u32 Delay_Time;
extern volatile u8 time_up;

void SysTick_init(void)
{
	//Systick 配置，1us进入一次中断
	if (SysTick_Config(SystemFrequency / 1000000))	
	{ 
		/* Capture error */ 
		while (1);
	}
	//关闭定时器
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
}

void Delay_us(u32 Time)
{ 
	Delay_Time = Time;	
	//使能Systick定时器
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	while(Delay_Time != 0);
}

void Delay(void)
{
	time_up = 0;
	TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE);
	TIM_SetCounter(TIM3,0x0000);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);					    		
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	while(time_up!=0);
}

void TimingDelay_Decrement(void)
{
	if (Delay_Time != 0x00)
	{ 
		Delay_Time--;
	}
}
