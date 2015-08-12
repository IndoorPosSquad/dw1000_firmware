#ifndef __USART_H__
#define __USART_H__

#include <stdio.h>
#include "CONFIG.h"
void USART1_init(void);
int fputc(int ch, FILE *f);
void usart_handle(void);
void upload_location_info(void);

#ifdef RX
void USART1_send(u8 *data, u16 length);
#endif

#define	DEBUG0(msg) \
	do { if (debug_lvl >= 0) { printf("%c", 0x02); printf msg; } } while (0)
#define	DEBUG1(msg) \
	do { if (debug_lvl >= 1) { printf("%c", 0x02); printf msg; } } while (0)
#define	DEBUG2(msg) \
	do { if (debug_lvl >= 2) { printf("%c", 0x02); printf msg; } } while (0)
#define	DEBUG3(msg) \
	do { if (debug_lvl >= 3) { printf("%c", 0x02); printf msg; } } while (0)

#endif
