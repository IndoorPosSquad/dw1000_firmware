#include <string.h>

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

#ifdef ETC
int upload_range = UPLOAD_RANGE;
#endif

// macro for parsing command from host
//#define TYPE(buf) (((buf[0]) >> 6) & 0x03)
//#define CMD(buf)  (((buf[0]) >> 4) & 0x03)
//#define FRAG()
//#define PACKET_LENGTH(buf) (buf[1])

// get the first byte of payload
#define DATA1(buf) (buf[2])

/*
  USART1 Init, 921600baudrate 8 bit/1 stop/no check
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

	USART_InitStructure.USART_BaudRate = 115200;
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
	int i;
	//u32 tmp = 0;
	//u8 tmpp[128];
	//TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	if (usart_status == 2) {
		switch (TYPE(usart_buffer)) {
			//case 0x00: //0x00
			//DEBUG1(("reserved type\n"));
			//break;
		case MESSAGE_TYPE:
			DEBUG1(("message\r\n"));
			//DEBUG1(("LEN: %d\r\n", PACKET_LEN(usart_buffer)));
			//DEBUG1(("SEQ: %d\r\n", PACKET_SEQ(usart_buffer)));
			//for (i = 0; i < PACKET_LEN(usart_buffer); i += 1) {
			//	DEBUG1(("%02X\r\n", PACKET_PLD(usart_buffer)[i]));
			//}
			transfer_message(PACKET_LEN(usart_buffer),
					 PACKET_SEQ(usart_buffer),
					 PACKET_SRC(usart_buffer),
					 PACKET_DST(usart_buffer),
					 PACKET_PLD(usart_buffer),
					 PACKET_CRC(usart_buffer));
			DEBUG1(("CRC: %02X %02X\r\n",
				*(PACKET_CRC(usart_buffer)), *(PACKET_CRC(usart_buffer) + 1)));
			break;
		case LOCATION_TYPE:
			DEBUG1(("location info\r\n"));
			// TIM2
			switch (SUBTYPE(usart_buffer)) {
			case LOC_ON:
				DEBUG1(("Open\r\n"));
				TIM_Cmd(TIM2, ENABLE);
				break;
			case LOC_OFF:
				DEBUG1(("Close\r\n"));
				TIM_Cmd(TIM2, DISABLE);
				break;
			case LOC_CALIB:
				DEBUG1(("Calibration\r\n"));
				calibration(CALI_POS_X, CALI_POS_Y, CALI_POS_Z);
				break;
			}
			break;
		case CMD_TYPE:
			DEBUG1(("command type\r\n"));
			switch (SUBTYPE(usart_buffer)) {
			case CMD_REBOOT:
				DEBUG1(("Reboot cmd\r\n"));
				break;
			case CMD_WR:
				DEBUG1(("write reg cmd\r\n"));
				break;
			case CMD_RR:
				DEBUG1(("read reg cmd\r\n"));
				break;
			case CMD_LOGLV:
				debug_lvl = (int) CMD_LOGLV_PARAM(usart_buffer);
				DEBUG1(("set log level to %d\r\n", debug_lvl));
				break;
			case CMD_UPLOADRANGE:
				upload_range = bytes_to_u32(CMD_UPLOADRANGE_PARAM(usart_buffer));
				DEBUG1(("set upload_range to %d\r\n", upload_range));
				break;
			}
			break;
		default:
			DEBUG1(("Unknown type"));
			DEBUG1(("Total Len: %d", usart_index));
			for (i = 0; i < usart_index; i += 1) {
				DEBUG1(("%02X\r\n", usart_buffer[i]));
			}
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
	DEBUG1(("Loc: %.2lf %.2lf %.2lf\r\n", location.x, location.y, location.z));
	DEBUG1(("Loc: %d %d %d\r\n",
		(int)(location.x * 100),
		(int)(location.y * 100),
		(int)(location.z * 100)));
	DEBUG1(("\r\n"));
	printf(("L"));
	printf(("3"));
	printf("%d ", (int)(location.x * 100));
	printf("%d ", (int)(location.y * 100));
	printf("%d ", (int)(location.z * 100));
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

void message_to_host(u8 * src, u8 * dst, u8 * payload, u8 len) {
	static int count;
	int i;

	char send_buff[128];

	DEBUG1(("Count: %d\r\n", count));
	count += 1;

	DEBUG1(("PAYLOAD: %02X %02X %02X %02X\r\n",
		payload[0], payload[1], payload[2], payload[3]));
	sprintf(send_buff, "M%u", len);

	for (i = 0; i < 8; i++) {
		sprintf(send_buff + 3 + i, "%02X", src[i]);
	}
	for (i = 0; i < 8; i++) {
		sprintf(send_buff + 11 + i, "%02X", dst[i]);
	}
	for (i = 0; i < len; i++) {
		sprintf(send_buff + 19 + i, "%02X", payload[i]);
	}
	send_buff[19 + len] = 0;

	printf("%s", send_buff);
}

void message_request_to_host(u8 * src) {
	int i;
	char send_buff[128];

	send_buff[0] = CMD_TYPE;
	send_buff[1] = CMD_SMSG;
	memcpy(&(send_buff[2]), src, 8);
	// 2 3 4 5 6 7 8 9

	for (i = 0; i < 10; i++) {
		USART_SendData(USART1, (unsigned char) send_buff[i]);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
	}
}
// payload without 'M'
void transfer_message_to_host(u8 * src, u8 * dst, u8 * payload) {
	static int count;
	int i;

	char send_buff[128];
	memset(send_buff, 0, 128);

	DEBUG1(("Count: %d\r\n", count));
	count += 1;

	DEBUG1(("PAYLOAD: %02X %02X %02X %02X\r\n",
		payload[0], payload[1], payload[2], payload[3]));
	send_buff[0] = MESSAGE_TYPE;
	// LEN 1
	memcpy(PACKET_LEN(send_buff), &(payload[1]), 1);
	// SEQ 2 - 5
	memcpy(PACKET_SEQ(send_buff), &(payload[2]), 4);
	// SRC ADDR 6 - 13
	memcpy(PACKET_SRC(send_buff), src, 8);
	// DST ADDR 14 - 21
	memcpy(PACKET_DST(send_buff), dst, 8);
	// Payload 22 - 85
	memcpy(PACKET_PLD(send_buff), &(payload[6]), 64);
	// CRC 86 87
	memcpy(PACKET_CRC(send_buff), &(payload[70]), 2);

	send_buff[88] = 0;

	for (i = 0; i < 88; i++) {
		USART_SendData(USART1, (unsigned char) send_buff[i]);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
	}
}
