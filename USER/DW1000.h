#define IDLE 0

#define SENT_LS_ACK 1
#define CONFIRM_SENT_LS_ACK 2
#define SENT_LS_DATA 3

#define SENT_LS_REQ 1
#define CONFIRM_SENT_LS_REQ 2
#define GOT_LS_ACK 3
#define GOT_LS_DATA 4
#define _POLLING_FLAG 0x38  //定位申请认证字节
#define _WAVE_SPEED 299792458 //电磁波传播速度

#ifdef TX
void Location_polling(void);
void distance_measurement(void);
void quality_measurement(void);
#endif

void DW1000_init(void);
void RX_mode_enable(void);
void to_IDLE(void);
void ACK_send(void);
void set_MAC(u8* mac);
void raw_write(u8* tx_buff, u16* size);
void raw_read(u8* rx_buff, u16* size);
void load_LDE(void);
void send_LS_ACK(u8 *src, u8 *dst);
void parse_rx(u8 *rx_buff, u16 size, u8 *src, u8 *dst, u8 *payload, u16 *pl_size);
void data_response(u8 *src, u8 *dst);
void read_status(u32 *status);
void sent_and_wait(void);
