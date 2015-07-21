#ifndef __SPI_H
#define __SPI_H

void SPI1_init(void);
void Write_DW1000(u8 addr, u16 offset_index, u8 *data, u16 length);
void Read_DW1000(u8 addr, u16 offset_index, u8 *data, u16 length);

#endif
