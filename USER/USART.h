#ifndef __USART_H__
#define __USART_H__

#include <stdio.h>
#include "CONFIG.h"
void USART1_init(u8 dip_config);
int fputc(int ch, FILE *f);
void usart_handle(void);
void upload_location_info(void);

#ifdef RX
void USART1_send(u8 *data, u16 length);
#endif

#define USART_BUFFER_LEN 64

#define	DEBUG0(msg) \
	do { if (debug_lvl >= 0) { printf("%c", 0x02); printf msg; } } while (0)
#define	DEBUG1(msg) \
	do { if (debug_lvl >= 1) { printf("%c", 0x02); printf msg; } } while (0)
#define	DEBUG2(msg) \
	do { if (debug_lvl >= 2) { printf("%c", 0x02); printf msg; } } while (0)
#define	DEBUG3(msg) \
	do { if (debug_lvl >= 3) { printf("%c", 0x02); printf msg; } } while (0)

#define TYPE(buf)          (u8)(buf[0])
#define RESERVED_TYPE      (u8)'R'
#define MESSAGE_TYPE       (u8)'M'

#define LOCATION_TYPE      'L'

#define PACKET_LENGTH(buf) (buf[1])
#define PACKET_SEQ(buf)    (((buf[2]) << 8) + (buf[3]))
#define PAYLOAD(buf)       (&(buf[4]))

#define CMD(buf)           (buf[1])

//// CMDs
#define L_CMD_ON           'O'
#define L_CMD_OFF          'F'
#define L_CMD_CALIB        'C'

#define CMD_TYPE           'C'

#endif
