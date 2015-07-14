#ifndef _MPU6050_H_
#define _MPU6050_H_
#include <stm32f10x.h>
typedef struct 
{
	float Accel_X;
	float Accel_Y;
	float Accel_Z;
	float Gyro_X;
	float Gyro_Y;
	float Gyro_Z;	
}MPU6050;

#define SCL 		GPIO_Pin_10	
#define SDA 		GPIO_Pin_11	
#define I2C_PORT   GPIOB
//
#define SCL_L 		GPIO_ResetBits(I2C_PORT, SCL)
#define SCL_H 		GPIO_SetBits(I2C_PORT, SCL)
#define SDA_L 		GPIO_ResetBits(I2C_PORT, SDA)
#define SDA_H   	GPIO_SetBits(I2C_PORT, SDA)
//
#define SDA_read   	GPIO_ReadInputDataBit(I2C_PORT, SDA)

void READ_MPU6050( MPU6050 *p);
void InitMPU6050(void);
void delay500ms(void);
#endif
