#include <stdio.h>
void USART1_init(void);
int fputc(int ch, FILE *f);
void usart_handle(void);
#ifdef RX
void USART1_send(u8 *data,u16 length);
#endif
