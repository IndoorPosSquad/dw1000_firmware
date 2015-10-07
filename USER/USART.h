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

#define USART_BUFFER_LEN 278

#define	DEBUG0(msg) \
	do { if (debug_lvl >= 0) { printf("%c", 0x02); printf msg; } } while (0)
#define	DEBUG1(msg) \
	do { if (debug_lvl >= 1) { printf("%c", 0x02); printf msg; } } while (0)
#define	DEBUG2(msg) \
	do { if (debug_lvl >= 2) { printf("%c", 0x02); printf msg; } } while (0)
#define	DEBUG3(msg) \
	do { if (debug_lvl >= 3) { printf("%c", 0x02); printf msg; } } while (0)

#define TYPE(buf)          (buf[0])
#define SUBTYPE(buf)       (buf[1])
//#define RESERVED_TYPE    (u8)'R'
/// TYPE 1 - MESSAGE
#define MESSAGE_TYPE       (u8)'M'
#define PACKET_LENGTH(buf) (buf[1])
#define PACKET_SEQ(buf)    (((buf[2]) << 8) + (buf[3]))
#define PAYLOAD(buf)       (&(buf[4]))

// TYPE 2 - LOCATION
#define LOCATION_TYPE      (u8)'L'
#define LOC_ON             (u8)'O'
#define LOC_OFF            (u8)'F'
#define LOC_CALIB          (u8)'C'
#define LOC_CALIB_PARAM(buf)   (u16 *)(&buf[2])

// TYPE 3 - COMMANDS
#define CMD_TYPE           (u8)'C'
#define CMD_REBOOT         (u8)'B'
#define CMD_WR             (u8)'W' // write register
#define CMD_RR             (u8)'R' // read register
#define CMD_LOGLV          (u8)'L' // set log level
#define CMD_LOGLV_PARAM(buf)    (buf[2])

#endif
