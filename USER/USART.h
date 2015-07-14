#include <stdio.h>
void USART1_init(void);
int fputc(int ch, FILE *f);
void usart_handle(void);

#ifdef TX
void upload_location_info(void);
#endif

#ifdef RX
void USART1_send(u8 *data,u16 length);
#endif
