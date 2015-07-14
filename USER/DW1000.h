#define IDLE 0
#define SENT_LS_ACK 1
#define CONFIRM_SENT_LS_ACK 2
#define SENT_LS_DATA 3

#define SENT_LS_REQ 1
#define CONFIRM_SENT_LS_REQ 2
#define GOT_LS_ACK 3
#define GOT_LS_DATA 4

#define u8 unsigned char
#define u16 unsigned short
#define u32 unsigned int

#define _WAVE_SPEED 299792458

#define DEBUG_LVL 2
#define	DEBUG0(msg) \
        do { if (DEBUG_LVL >= 0) { printf("%c", 0x02); printf msg; } } while (0)
#define	DEBUG1(msg) \
        do { if (DEBUG_LVL >= 1) { printf("%c", 0x02); printf msg; } } while (0)
#define	DEBUG2(msg) \
        do { if (DEBUG_LVL >= 2) { printf("%c", 0x02); printf msg; } } while (0)
#define	DEBUG3(msg) \
        do { if (DEBUG_LVL >= 3) { printf("%c", 0x02); printf msg; } } while (0)

void Location_polling(void);
void distance_measurement(int n);
void distance_forward(void);
void location_info_upload(void);
void handle_distance_forward(u8 * payload);
void quality_measurement(void);

void DW1000_init(void);
void RX_mode_enable(void);
void to_IDLE(void);
void set_MAC(u8* mac);
void raw_write(u8* tx_buff, u16* size);
void raw_read(u8* rx_buff, u16* size);
void load_LDE(void);
void send_LS_ACK(u8 *src, u8 *dst);
void parse_rx(u8 *rx_buff, u16 size, u8 **src, u8 **dst, u8 **payload, u16 *pl_size);
void data_response(u8 *src, u8 *dst);
void read_status(u32 *status);
void sent_and_wait(void);
void Fifoput(u8* data, int len);
void Push(u8* data);
void Pop(u8* data);

void Read_VotTmp(u8 * vot, u8 * tmp);
void Init_VotTmp(u8 * voltage, u8 * temperature);
void Read_Tmp(u8 * temperature);
