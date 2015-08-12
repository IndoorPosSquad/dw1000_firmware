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

extern u8 usart_buffer[64];
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
#define TYPE(buf) (((buf[0]) >> 6) & 0x03)
#define CMD(buf)  (((buf[0]) >> 4) & 0x03)
#define FRAG()
#define PACKET_LENGTH(buf) (buf[1])

// get the first byte of payload
#define DATA1(buf) (buf[2])

/*
 USART1鍒濆����鍖�,娉㈢壒鐜�115200锛屽崟娆�8姣旂壒锛屾棤濂囧伓鏍￠獙锛�1鍋滄����浣�
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

	DEBUG2(("USART鍒濆����鍖朶t\t瀹屾垚, with debug_lvl: %d\r\n", debug_lvl));
}
/*
fputc閲嶅畾鍚�
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
	if(usart_status == 2) {
		switch(TYPE(usart_buffer)) {
		case 0x00: //0x00
			DEBUG1(("reserved type\n"));
			break;
		case 0x01: //0x40
			DEBUG1(("message type\n"));

			break;
		case 0x02: //0x80
			DEBUG1(("location info \n"));
			// TIM2 寮�鍏�
			switch(CMD(usart_buffer)) {
			case 0x00: // 0x80 寮�
				DEBUG1(("Open\n"));
				TIM_Cmd(TIM2, ENABLE);
				break;
			case 0x01: // 0x90 鍏�
				DEBUG1(("Close\n"));
				TIM_Cmd(TIM2, DISABLE);
				break;
			case 0x02: // 0xA0 鏍″噯
				DEBUG1(("Calibration\n"));
				calibration(CALI_POS_X, CALI_POS_Y, CALI_POS_Z);
				break;
			}
			break;
		case 0x03: //0xC0
			DEBUG1(("command type\n"));
			switch(CMD(usart_buffer)) {
			case 0x00: //0xC0
				DEBUG1(("Reboot cmd\n"));
				break;
			case 0x01: //0xD0
				DEBUG1(("write reg cmd\n"));
				break;
			case 0x02: //0xE0
				DEBUG1(("read reg cmd\n"));
				break;
			case 0x03: //0xF0
				debug_lvl = (int) DATA1(usart_buffer);
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
	DEBUG1(("LS_DT: %d %d %d", LS_DATA[0], LS_DATA[1], LS_DATA[2]));
	DEBUG1(("Dist:  %.2lf %.2lf %.2lf\r\n", distance[0], distance[1], distance[2]));
	DEBUG1(("Raw:   %.2lf %.2lf %.2lf\r\n", raw_distance[0], raw_distance[1], raw_distance[2]));
	DEBUG1(("Cali:  %.2lf %.2lf %.2lf\r\n", calib[0], calib[1], calib[2]));
	DEBUG1("Loc: %.2lf %.2lf %.2lf\n", location.x, location.y, location.z);
	DEBUG1(("\r\n"));
	printf("-1");
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
/*
	鍛戒护		鍛戒护瀛�			鍙傛暟1															鍙傛暟2
璁剧疆瀹氫綅鍛ㄦ湡	0x01		瀹氫綅鍛ㄦ湡锛堝崟浣峬s锛涗袱瀛楄妭锛屼綆浣嶅湪鍓嶏紝榛樿����3000ms锛�
璁剧疆鑷����姩閲嶅彂	0x02	鑷����姩閲嶅彂绛夊緟鏃堕棿锛堝崟浣島s锛涗袱瀛楄妭锛屼綆浣嶅湪鍓嶏紱榛樿����1000us)				閲嶅彂娆℃暟锛堥粯璁�1锛�
璁剧疆鏃堕棿鍋忕Щ    0x03    鏃堕棿宸����殑鍋忕Щ锛堜粠纭����欢鑾峰緱鏁版嵁涓����噺鍘伙紱4瀛楄妭锛屼綆浣嶅湪鍓嶏紱榛樿����0锛�
璁剧疆鍏夐�熷亸绉�    0x04    鍏夐�熺殑鐧惧垎姣斿亸绉伙紙鍗曚綅锛�1%锛涗粠鐪熺┖鍏夐�熶腑浠ョ櫨鍒嗘瘮褰㈠紡鍑忓幓锛涢粯璁�0锛�
璇诲彇DW1000瀵勫瓨鍣� 0x05     鍦板潃锛堜竴瀛楄妭锛� 鍋忕Щ鍦板潃锛堜袱瀛楄妭锛屼綆浣嶅湪鍓嶏級 璇诲彇瀛楄妭闀垮害锛堜袱瀛楄妭锛屼綆浣嶅湪鍓�,鏈�澶�128锛�

*/

/*
tmp=0;
//閰嶇疆瀹氫綅鍛ㄦ湡鍗曚綅锛坢s锛�
if((usart_buffer[0]==0x01)&&(usart_index==3))
{
	tmp=usart_buffer[1]+(usart_buffer[2]<<8);
	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
	TIM_TimeBaseStructure.TIM_Period=2*tmp;
	TIM_TimeBaseStructure.TIM_Prescaler=36000;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	printf("\r\n*瀹氫綅鍛ㄦ湡璁剧疆鎴愬姛*\r\n");
	printf("[瀹氫綅鍛ㄦ湡璁剧疆涓�%ums]\r\n",tmp);
}
else if	(((usart_buffer[0]==0x02)&&(usart_index==4)))
{
	// ars_max=usart_buffer[3];
	printf("\r\n*Disabled*\r\n");
	// printf("[鏈�澶ч噸鍙戞����鏁颁负%u娆����\r\n",ars_max);
}
else if	(((usart_buffer[0]==0x03)&&(usart_index==5)))
{
	time_offset= usart_buffer[1]+(usart_buffer[2]<<8)+(usart_buffer[3]<<16)+(usart_buffer[4]<<24);
	printf("\r\n*鐢电����娉㈤����琛屾椂闂村亸绉昏����缃����垚鍔�*\r\n");
	printf("[鐢电����娉㈤����琛屾椂闂村亸绉讳负0x%x%x%x%x]\r\n",usart_buffer[4],usart_buffer[3],usart_buffer[2],usart_buffer[1]);
}
else if	(((usart_buffer[0]==0x04)&&(usart_index==2)))
{
	speed_offset= usart_buffer[1];
	printf("\r\n*鐢电����娉㈤�熷害鍋忕Щ鐧惧垎姣旇����缃����垚鍔�*\r\n");
	printf("[鐢电����娉㈤�熷害鍋忕Щ鐧惧垎姣斾负%u%%]\r\n",speed_offset);
}
else if	(((usart_buffer[0]==0x05)&&(usart_index==6)))
{
	tmp=(usart_buffer[4]+(usart_buffer[5]<<8));
	Read_DW1000(usart_buffer[1],(usart_buffer[2]+(usart_buffer[3]<<8)),tmpp,tmp);
	printf("*璁块棶鍦板潃 0x%02x:0x%02x 璁块棶闀垮害 %d *\r\n[杩斿洖鏁版嵁 0x",usart_buffer[1],(usart_buffer[2]+(usart_buffer[3]<<8)),tmp);
	for(i=0;i<tmp;i++)
	{
		printf("%02x",*(tmpp+tmp-i-1));
	}
	printf("]\r\n");
}
else
{
	printf("Your Input: ");
	for (i = 0; i < usart_index; i++) {
		printf("%02x", usart_buffer[i]);
	}
}
*/
