#ifndef __DW1000_H
#define __DW1000_H

#define IDLE 0
#define SENT_LS_ACK 1
#define CONFIRM_SENT_LS_ACK 2
#define SENT_LS_DATA 3

#define SENT_LS_REQ 1
#define CONFIRM_SENT_LS_REQ 2
#define GOT_LS_ACK 3
#define GOT_LS_DATA 4

#define _WAVE_SPEED 299792458

#define PC13_UP GPIO_ResetBits(GPIOC, GPIO_Pin_13)
#define PC13_DOWN GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define PC0_UP GPIO_ResetBits(GPIOC, GPIO_Pin_0)
#define PC0_DOWN GPIO_SetBits(GPIOC, GPIO_Pin_0)

#ifdef TX
#ifdef LOCATION
void Location_polling(void);
#endif
#ifdef ETC
void ETC_polling(void);
void send_discover_msg(u8 seq);
void handle_reply_discover_msg(u8 * src, u8 * dst, u8 * payload);
#endif
#endif

#ifdef RX
#ifdef ETC
void reply_discover_msg(u8 * dst);
#endif // ifdef ETC
#endif // ifdef RX

void transfer_message(u8 * len,
		      u8 * seq,
		      u8 * src,
		      u8 * dst,
		      u8 * msg_payload, // message payload
		      u8 * crc);
void handle_transfer_message(u8 * src,
			     u8 * dst,
			     u8 * payload); // message payload

void distance_measurement(int n);

void status_forward(void);
void handle_distance_forward(u8 * payload);

void send_package_request(u8 seq);
void send_package_message(u8 * dst);
void handle_package_message(u8 * src, u8 * dst, u8 * payload, u8 len);

void DW1000_init(u8 dip_config);

void DW1000_trigger_reset(void);
void RX_mode_enable(void);
void to_IDLE(void);
void set_MAC(u8* mac);
void raw_write(u8* tx_buff, u16* size);
void raw_read(u8* rx_buff, u16* size);
void load_LDE(void);
void parse_rx(u8 *rx_buff, u16 size, u8 **src, u8 **dst, u8 **payload, u16 *pl_size);
void send_LS_ACK(u8 *src, u8 *dst);
void send_LS_DATA(u8 *src, u8 *dst);
void read_status(u32 *status);
void sent_and_wait(void);
void quality_measurement(void);

void handle_event(void);

u8 Read_DIP_Configuration(void);
void Read_VotTmp(u8 * vot, u8 * tmp);
void Init_VotTmp(u8 * voltage, u8 * temperature);
void Read_Tmp(u8 * temperature);
int get_antenna_delay(u8 n);

#endif
