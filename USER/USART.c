#include "stm32f10x.h"
#include "DW1000.h"
#include "MPU6050.h"
#include "USART.h"
#include "SPI.h"
#include "CONFIG.h"
#include "math.h"
#include "utils.h"
#include "delay.h"
#include "solve.h"

extern u8 usart_buffer[USART_BUFFER_LEN];
extern u8 usart_index;
extern u8 usart_status;
// extern u8 ars_max;
extern u8 time_offset;
extern u8 speed_offset;
extern u32 LS_DATA[3];
extern float distance[3];
extern float raw_distance[3];
extern xyz location;

int debug_lvl = DEBUG_LVL;
extern float calib[3];

// macro for parsing command from host
//#define TYPE(buf) (((buf[0]) >> 6) & 0x03)
//#define CMD(buf)  (((buf[0]) >> 4) & 0x03)
//#define FRAG()
//#define PACKET_LENGTH(buf) (buf[1])

// get the first byte of payload
#define DATA1(buf) (buf[2])

/*
  USART1 Init, 115200baudrate 8 bit/1 stop/no check
 */
void USART1_init(u8 dip_config) {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	debug_lvl = (int) ((dip_config & 0x30) >> 4);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 921600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_ClearFlag(USART1, USART_FLAG_TC);

	USART_Cmd(USART1, ENABLE);

	DEBUG2(("USART init do \t, with debug_lvl: %d\r\n", debug_lvl));
}
/*
fputc redirect
*/
int fputc(int ch, FILE *f) {
	USART_SendData(USART1, (unsigned char) ch);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
	return (ch);
}

void usart_handle(void) {
	//u16 i;
	//u32 tmp = 0;
	//u8 tmpp[128];
	//TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	if (usart_status == 2) {
		switch (TYPE(usart_buffer)) {
			//case 0x00: //0x00
			//DEBUG1(("reserved type\n"));
			//break;
		case MESSAGE_TYPE:
			DEBUG1(("message type\n"));
			break;
		case LOCATION_TYPE:
			DEBUG1(("location info\n"));
			// TIM2
			switch (SUBTYPE(usart_buffer)) {
			case LOC_ON:
				DEBUG1(("Open\n"));
				TIM_Cmd(TIM2, ENABLE);
				break;
			case LOC_OFF:
				DEBUG1(("Close\n"));
				TIM_Cmd(TIM2, DISABLE);
				break;
			case LOC_CALIB:
				DEBUG1(("Calibration\n"));
				calibration(CALI_POS_X, CALI_POS_Y, CALI_POS_Z);
				break;
			}
			break;
		case CMD_TYPE:
			DEBUG1(("command type\n"));
			switch (SUBTYPE(usart_buffer)) {
			case CMD_REBOOT:
				DEBUG1(("Reboot cmd\n"));
				break;
			case CMD_WR: //0xD0
				DEBUG1(("write reg cmd\n"));
				break;
			case CMD_RR: //0xE0
				DEBUG1(("read reg cmd\n"));
				break;
			case CMD_LOGLV: //0xF0
				debug_lvl = (int) CMD_LOGLV_PARAM(usart_buffer);
				DEBUG1(("set log level to %d\n", debug_lvl));
				break;
			}
			break;
		}

		usart_status = 0;
		usart_index = 0;
	}
}

void upload_location_info(void) {
	#ifdef USE_MPU6050
	MPU6050 mpu6050_buf;
	#endif

	#ifdef USE_TEMP_VOLT_SENSOR
	u8 temperature;
	#endif

	#ifdef FAKE_SERIAL
	static int count = 0;
	static int step = 1;
	if (count >= 32) {
		step = -1;
	} else if (count <= -32) {
		step = 1;
	}
	count += step;
	#endif

	#ifdef FAKE_SERIAL
	PCout(13) = 1;
	delay(0x3ffff);
	printf("Loc: %.2lf %.2lf %.2lf\n", (float) sin((float)count / 16 * 3.14), (float) cos((float) count / 16 * 3.14), 0.1 + 0.01 * cos((float)count));
	PCout(13) = 0;
	#elif defined(SOLVE_LOCATION)
	DEBUG1(("LS_DT: %d %d %d\r\n", LS_DATA[0], LS_DATA[1], LS_DATA[2]));
	DEBUG1(("Dist:  %.2lf %.2lf %.2lf\r\n", distance[0], distance[1], distance[2]));
	DEBUG1(("Raw:   %.2lf %.2lf %.2lf\r\n", raw_distance[0], raw_distance[1], raw_distance[2]));
	DEBUG1(("Cali:  %.2lf %.2lf %.2lf\r\n", calib[0], calib[1], calib[2]));
	DEBUG1(("Loc: %.2lf %.2lf %.2lf\n", location.x, location.y, location.z));
	DEBUG1(("\r\n"));
	printf(" -1 ");
	printf(" %d ", (int)(location.x * 100));
	printf(" %d ", (int)(location.y * 100));
	printf(" %d ", (int)(location.z * 100));
	printf(" -1 ");
	DEBUG1(("\r\n"));
	#else
	DEBUG1(("Raw:  %.2lf %.2lf %.2lf\r\n", raw_distance[0], raw_distance[1], raw_distance[2]));
	printf("Dist: %.2lf %.2lf %.2lf\r\n", distance[0], distance[1], distance[2]);
	#endif

#ifdef USE_MPU6050
	READ_MPU6050(&mpu6050_buf);
	printf("MPU: %.2lf %.2lf %.2lf\r\n", mpu6050_buf.Accel_X, mpu6050_buf.Accel_Y, mpu6050_buf.Accel_Z);
#endif

#ifdef USE_TEMP_VOLT_SENSOR
	Read_Tmp(&temperature);
	printf("Temp: %d\r\n", temperature);
#endif
}
