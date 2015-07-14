#ifndef __DELAY_H
#define __DELAY_H
//#include <stm32f10x_lib.h>
//////////////////////////////////////////////////////////////////////////////////
//Mini STM32开发板
//使用SysTick的普通计数模式对延迟进行管理
//包括delay_us,delay_ms
//********************************************************************************
//V1.2修改说明
//修正了中断中调用出现死循环的错误
//防止延时不准确,采用do while结构!
//////////////////////////////////////////////////////////////////////////////////
void SysTick_init(void);
void Delay_us(u32 nus);
void Delay(void);
void TimingDelay_Decrement(void);
void delay(u32 nCount);
#endif
