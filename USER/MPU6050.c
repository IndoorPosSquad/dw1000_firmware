#include "mpu6050.h"

/*---------
±2g 16384 LSB/g
±4g 8192 LSB/g
±8g 4096 LSB/g
±16g 2048 LSB/g
Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
± 250 °/s 131 LSB/°/s
± 500 °/s 65.5 LSB/°/s
± 1000 °/s 32.8 LSB/°/s
± 2000 °/s 16.4 LSB/°/s
-----------*/
/* Private defines -----------------------------------------------------------*/
#define	Factor_2g               16384  
#define	Factor_4g               8192
#define	Factor_8g               4096
#define	Factor_16g              2048
#define	Factor_250_Gyroscope    131
#define	Factor_500_Gyroscope    65.5
#define	Factor_1000_Gyroscope   32.8
#define	Factor_2000_Gyroscope   16.4
//mpu6050 add
#define	SMPLRT_DIV		0x19	 //陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	 //低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	 //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	        0x1C	 //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define FIFO_EN                 0x23
#define	ACCEL_XOUT_H	        0x3B
#define	ACCEL_XOUT_L	        0x3C
#define	ACCEL_YOUT_H	        0x3D
#define	ACCEL_YOUT_L	        0x3E
#define	ACCEL_ZOUT_H	        0x3F
#define	ACCEL_ZOUT_L	        0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	SIGNAL_PATH_RESET	0x68	  //reset
#define	PWR_MGMT_1		0x6B	  //电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	  //IIC地址寄存器(默认数值0x68，只读)
#define	SlaveAddress	        0xD0	  //IIC写入时的地址字节数据，+1为读取

void Delay5us(void)
{		
   u8 i=100; //可以优化
   while(i) 
   { 
     i--; 
   }  
}

void delay5ms(void)
{		
    int i=10000;  
    while(i) 
    { 
     i--; 
    }  
}

void delay500ms(void)
{		
    int i=1000000;  
    while(i) 
    { 
     i--; 
    }  
}


void I2C_Start(void)
{
    SDA_H;               
    SCL_H;                  
    Delay5us();            
    SDA_L; 
    SCL_L;
    Delay5us();                                   
}

void I2C_Stop(void)
{
    SDA_L;         
    Delay5us();                 
    SCL_H;                   
    SDA_H;                 
    Delay5us();             
}

u8 I2C_Slave_ACK(void)//0ACK
{
    u8 s,ack;
    SCL_H; 
    Delay5us();
    s=SDA_read;
    if(s)
      ack=1;
    else
      ack=0;                                   
    SCL_L;                   
    Delay5us(); 
    return ack;
}

void I2C_SendByte(u8 SendByte)
{
    u8 i;
    for(i = 0; i < 8; i++)
    {
        Delay5us();
        if(SendByte&0x80)
          SDA_H;  
        else 
          SDA_L;   
        SendByte<<=1;
        Delay5us();
	SCL_H;
        Delay5us();
        SCL_L;
        Delay5us();
    }   
}

u8 I2C_RecvByte(void)
{
    u8 i;
    u8 ReceiveByte=0;			
    for(i = 0; i < 8; i++)
    {
        ReceiveByte<<=1;      
        SCL_H;
        Delay5us();	
    if(SDA_read)
    {
        ReceiveByte|=0x01;
    }
        SCL_L;
        Delay5us();
    }
    return ReceiveByte;
}

void I2C_nack(void)
{
    SDA_H;
    SCL_H;                    
    Delay5us();                                 
    SCL_L;                    
    Delay5us();               
}

void Single_WriteI2C(u8 REG_Address,u8 REG_data)
{
    I2C_Start();                  
    I2C_SendByte(SlaveAddress);   
    I2C_Slave_ACK();
    I2C_SendByte(REG_Address);    
    I2C_Slave_ACK();
    I2C_SendByte(REG_data);       
    I2C_Slave_ACK();
    I2C_Stop();                   
} 

u8 Single_ReadI2C(u8 REG_Address)
{
    u8 REG_data;
    I2C_Start();                   
    I2C_SendByte(SlaveAddress);
    I2C_Slave_ACK();
    I2C_SendByte(REG_Address);
    I2C_Slave_ACK();
    I2C_Start();                   
    I2C_SendByte(SlaveAddress+1);
    I2C_Slave_ACK();
    REG_data=I2C_RecvByte();       
    I2C_nack();               
    I2C_Stop();                    
    return REG_data;
}

void InitMPU6050(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

		GPIO_InitStructure.GPIO_Pin = SCL | SDA;					//圆点博士:配置使用的I2C口
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //圆点博士:设置I2C口最大允许输出速度
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	  //圆点博士:设置I2C为开漏输出
		GPIO_Init(I2C_PORT, &GPIO_InitStructure);
	
    Single_WriteI2C(PWR_MGMT_1, 0x00);	         //解除休眠状态
    Single_WriteI2C(SMPLRT_DIV, 0x4F);           //Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) 100HZ
    Single_WriteI2C(CONFIG, 0x06);               //EXT_SYNC_SET=0 Input disabled,DLPF_CFG=7 RESERVED
    Single_WriteI2C(GYRO_CONFIG, 0x18);          //± 1000 °/s
    Single_WriteI2C(ACCEL_CONFIG, 0x01);         //± 2g
//    Single_WriteI2C(SIGNAL_PATH_RESET, 0x07);    //reset
	
// 		Single_WriteI2C(PWR_MGMT_1, 0x00);	//解除休眠状态
 	Single_WriteI2C(SMPLRT_DIV, 0x07);
// 	Single_WriteI2C(CONFIG, 0x06);
// 	Single_WriteI2C(GYRO_CONFIG, 0x18);
// 	Single_WriteI2C(ACCEL_CONFIG, 0x01);
	
}
 
u16 GetData_Word(u8 REG_Address)
{
    u8 H,L;
    H=Single_ReadI2C(REG_Address);
    L=Single_ReadI2C(REG_Address+1);
    return (H<<8)+L;   //合成数据
}
    
float GET_MPU6050_TMP(void)
{
    s16 t;
    float tmp;
    t=GetData_Word(TEMP_OUT_H);
    tmp=(float)(t)/340 + 36.53;
    return tmp; 
}
void READ_MPU6050(MPU6050 *p)
{
    s16 BUF[6];
    float ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z;
		s16 Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z;
		InitMPU6050();
    BUF[0]=(s16)GetData_Word(ACCEL_XOUT_H); 
    BUF[1]=(s16)GetData_Word(ACCEL_YOUT_H);
    BUF[2]=(s16)GetData_Word(ACCEL_ZOUT_H);
    BUF[3]=(s16)GetData_Word(GYRO_XOUT_H);					
    BUF[4]=(s16)GetData_Word(GYRO_YOUT_H);
    BUF[5]=(s16)GetData_Word(GYRO_ZOUT_H);
	  ACCEL_X=(float)BUF[0]/Factor_2g;             
    ACCEL_Y=(float)BUF[1]/Factor_2g;
    ACCEL_Z=(float)BUF[2]/Factor_2g;
    GYRO_X=(float)BUF[3]/Factor_1000_Gyroscope;
    GYRO_Y=(float)BUF[4]/Factor_1000_Gyroscope;
    GYRO_Z=(float)BUF[5]/Factor_1000_Gyroscope;
    Accel_X=(s16)(ACCEL_X*100);                  //小数点一位
    Accel_Y=(s16)(ACCEL_Y*100);
    Accel_Z=(s16)(ACCEL_Z*100);
    Gyro_X=(s16)(GYRO_X*100);
    Gyro_Y=(s16)(GYRO_Y*100);
    Gyro_Z=(s16)(GYRO_Z*100);  
		p->Accel_X=(float)Accel_X/100;
		p->Accel_Y=(float)Accel_Y/100;
		p->Accel_Z=(float)Accel_Z/100;
		p->Gyro_X=(float)Gyro_X/100;
		p->Gyro_Y=(float)Gyro_Y/100;
		p->Gyro_Z=(float)Gyro_Z/100;

}

