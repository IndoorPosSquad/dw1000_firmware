#include "stm32f10x.h"
#include "SPI.h"
#include "DW1000.h"


u8 Sequence_Number=0x00;
extern u8 status_flag;
u8 Tx_Buff[14];
u32 Tx_stp_L;
u8 Tx_stp_H;
u32 Rx_stp_L;
u8 Rx_stp_H;
u32 diff;
u8 tmp;
extern u8 Receive_buffer[14];
/*
DW1000初始化
*/
void DW1000_init(void)
{
	u32 tmp;	
	////////////////////工作模式配置////////////////////////
	//AGC_TUNE1	：设置为16 MHz PRF
	tmp=0x00008870;
	Write_DW1000(0x23,0x04,(u8 *)(&tmp),2);
	//AGC_TUNE2	 ：不知道干啥用，技术手册明确规定要写0x2502A907
	tmp=0x2502A907;
	Write_DW1000(0x23,0x0C,(u8 *)(&tmp),4);	
	//DRX_TUNE2：配置为PAC size 8，16 MHz PRF
	tmp=0x311A002D;
	Write_DW1000(0x27,0x08,(u8 *)(&tmp),4);
	//NSTDEV  ：LDE多径干扰消除算法的相关配置
	tmp=0x0000006D;
	Write_DW1000(0x2E,0x0806,(u8 *)(&tmp),1);
	//LDE_CFG2	：将LDE算法配置为适应16MHz PRF环境
	tmp=0x00001607;
	Write_DW1000(0x2E,0x1806,(u8 *)(&tmp),2);
	//TX_POWER	 ：将发送功率配置为16 MHz,智能功率调整模式 
	tmp=0x0E082848;
	Write_DW1000(0x1E,0x00,(u8 *)(&tmp),4);
	//RF_TXCTRL	 ：选择发送通道5
	tmp=0x001E3FE0;
	Write_DW1000(0x28,0x0C,(u8 *)(&tmp),4);
	//TC_PGDELAY：脉冲产生延时设置为适应频道5
	tmp=0x000000C0;
	Write_DW1000(0x2A,0x0B,(u8 *)(&tmp),1);
	//FS_PLLTUNE   ：PPL设置为适应频道5
	tmp=0x000000A6;
	Write_DW1000(0x2B,0x0B,(u8 *)(&tmp),1);
	/////////////////////使用功能配置/////////////////////////
	//local address	：写入本机地址（PAN_ID 和本机短地址）
	tmp=_PAN_ID;
	tmp=(tmp<<16)+_RX_sADDR; 
	Write_DW1000(0x03,0x00,(u8 *)(&tmp),4);  	  
	//re-enable	  auto ack	 Frame Filter ：开启接收自动重启功能、自动应答功能、帧过滤功能
	tmp=0x200011FD;
	Write_DW1000(0x04,0x00,(u8 *)(&tmp),4);
	//  test pin SYNC：用于测试的LED灯引脚初始化，SYNC引脚禁用	
	tmp=0x00101540;
	Write_DW1000(0x26,0x00,(u8 *)(&tmp),2);
	tmp=0x01;
	Write_DW1000(0x36,0x28,(u8 *)(&tmp),1);
	// interrupt   ：中断功能选择（）
	tmp=0x00006080;
	Write_DW1000(0x0E,0x00,(u8 *)(&tmp),4);
	// ack等待
	tmp=3;
	Write_DW1000(0x1A,0x03,(u8 *)(&tmp),1);

	printf("定位芯片配置\t\t完成\r\n");	
}
/*
数据回发
*/
void data_response(void)
{
	

	if(status_flag==2)
	{
		Read_DW1000(0x17,0x00,(u8 *)(&Tx_stp_L),4);
		Read_DW1000(0x15,0x09,(u8 *)(&Rx_stp_L),4);
		Read_DW1000(0x17,0x04,&Tx_stp_H,1);
		Read_DW1000(0x15,0x0d,&Rx_stp_H,1);
		printf("%8x\r\n",Rx_stp_L);
		printf("%2x\r\n",Rx_stp_H);
		printf("%8x\r\n",Tx_stp_L);
		printf("%2x\r\n",Tx_stp_H);
	

		if(Tx_stp_H==Rx_stp_H)
		{
			diff=(Tx_stp_L-Rx_stp_L);
		}
		else if(Rx_stp_H<Tx_stp_H)
		{
				diff=((Tx_stp_H-Rx_stp_H)*0xFFFFFFFF+Tx_stp_L-Rx_stp_L);
		}
		else
		{
			diff=((0xFF-Rx_stp_H+Tx_stp_H+1)*0xFFFFFFFF+Tx_stp_L-Rx_stp_L);
		}
	 }
	printf("%08x\r\n",diff);
	Tx_Buff[0]=0x41;
	Tx_Buff[1]=0x88;
	Tx_Buff[2]=Sequence_Number;
	Tx_Buff[4]=(u8)(_PAN_ID>>8);
   	Tx_Buff[3]=(u8)_PAN_ID;
	Tx_Buff[6]=(u8)(_TX_sADDR>>8);
	Tx_Buff[5]=(u8)_TX_sADDR;
	Tx_Buff[8]=(u8)(_RX_sADDR>>8);
	Tx_Buff[7]=(u8)_RX_sADDR;
   	Tx_Buff[9]=(u8)diff;
	diff>>=8;
	Tx_Buff[10]=(u8)diff;
	diff>>=8;
	Tx_Buff[11]=(u8)diff;
	diff>>=8;
	Tx_Buff[12]=(u8)diff;
	Tx_Buff[13]=0x01;

	to_IDLE();
	tmp=15;
	Write_DW1000(0x08,0x00,&tmp,1);
	Write_DW1000(0x09,0x00,Tx_Buff,14);
	tmp=0x82;
	Write_DW1000(0x0D,0x00,&tmp,1);

	TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE);
	TIM_SetCounter(TIM3,0x0000);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);


}
/*
打开接收模式
*/
void RX_mode_enable(void)
{
	u8 tmp;
	/*Read_DW1000(0x0f,0x03,&tmp,1);
	if(((tmp&0xc0)==0x40)||((tmp&0xc0)==0x80))
	{
		tmp=((tmp&0x80)==0x80);
		Write_DW1000(0x0D,0x03,&tmp,1);
	}	  */
	tmp=0x01;
	Write_DW1000(0x0D,0x01,&tmp,1);	

//	printf("开启接收模式\t\t完成\r\n");
}
/*
返回IDLE状态
*/
void to_IDLE(void)
{
	u8 tmp;
	tmp=0x40;
	Write_DW1000(0x0D,0x00,&tmp,1);	
}
void ACK_send(void)
{
	Tx_Buff[0]=0x44;
	Tx_Buff[1]=0x00;
	Tx_Buff[2]=Receive_buffer[2];

	to_IDLE();
	tmp=5;
	Write_DW1000(0x08,0x00,&tmp,1);
	Write_DW1000(0x09,0x00,Tx_Buff,3);
	tmp=0x82;
	Write_DW1000(0x0D,0x00,&tmp,1);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE);				    		
	TIM_Cmd(TIM3, ENABLE);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);

}
