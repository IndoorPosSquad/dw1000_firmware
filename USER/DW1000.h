//TODO : disable PAN
#define _PAN_ID 0x1074		//个人局域网id
#define _TX_sADDR 0x2014	//发送方短地址
#define _RX_sADDR 0x2015	//接收方短地址
#define _POLLING_FLAG 0x38  //定位申请认证字节
#define _WAVE_SPEED 299792458 //电磁波传播速度

#ifdef TX
void Location_polling(void);
void distance_measurement(void);
void quality_measurement(void);
#endif

#ifdef RX
void data_response(void);
#endif
void DW1000_init(void);
void RX_mode_enable(void);
void to_IDLE(void);
void ACK_send(void);
