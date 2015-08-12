#include "utils.h"
#include "usb_istr.h"
#include "usb_lib.h"
#include "usb_pwr.h"
#include "string.h"

// FIFO state variables
#define fifolen 256
// When I started writing those codes, only God and I know what are them supposed to do.
// But now, only God knows.
// USB
uint8_t int_Send_Buffer[2];

u8 Queue[fifolen * 64];
u8 buf[64];
volatile int Length = 0;
volatile int Head = 0, Tail = 0;
// end of FIFO state variables

// FIFO functions
void Push(u8* data) {
	if(Length == fifolen)
		return;
	memcpy(Queue + Head * 64, data, 64);
	Head++;
	Length++;
	if(Head == fifolen)
		Head = 0;
}

void Pop(u8* data) {
	if(Length == 0) {
		memset(data, 0, 64);
		return;
	}
	memcpy(data, Queue + Head * 64, 64);
	Tail++;
	Length--;
	if(Tail == fifolen)
		Tail = 0;
}

void Fifoput(u8* data, int len) {
	if(len < 63) {
		buf[0] = 0x00;
		buf[1] = (u8)(len);
		memcpy(buf + 2, data, len);
		Push(buf);
	} else if(len < 125) {
		// frame 1
		buf[0] = 0x01;
		buf[1] = (u8)(len);
		memcpy(buf + 2, data, 62);
		Push(buf);
		// frame 2 */
		memset(buf, 0, 64);
		buf[0] = 0x02;
		buf[1] = (u8)(len);
		memcpy(buf + 2, data + 62, len - 62);
		Push(buf);
	} else {
		// frame 1
		buf[0] = 0x01;
		buf[1] = (u8)(len);
		memcpy(buf + 2, data, 62);
		Push(buf);
		// frame 2
		memset(buf, 0, 64);
		buf[0] = 0x02;
		buf[1] = (u8)(len);
		memcpy(buf + 2, data + 62, 62);
		Push(buf);
		// frame 3
		memset(buf, 0, 64);
		buf[0] = 0x03;
		buf[1] = (u8)(len);
		memcpy(buf + 2, data + 124, len - 124);
		Push(buf);
	}
	int_Send_Buffer[0] = 0xF0;
	int_Send_Buffer[1] = 0xF0;
	// /\* Copy mouse position info in ENDP1 Tx Packet Memory Area*\/
	// USB_SIL_Write(EP2_IN, int_Send_Buffer, 2);
	// /\* Enable endpoint for transmission *\/
	// SetEPTxStatus(ENDP2, EP_TX_VALID);
	UserToPMABufferCopy(int_Send_Buffer, GetEPTxAddr(ENDP2), 2);
	SetEPTxCount(ENDP2, 2);
	SetEPTxValid(ENDP2);
}
// end of FIFO functions


void float_to_bytes(uint8_t * bytes, float flt) {
	float_bytes fb;

	fb.f = flt;
	bytes[0] = fb.s[0];
	bytes[1] = fb.s[1];
	bytes[2] = fb.s[2];
	bytes[3] = fb.s[3];
}

float bytes_to_float(uint8_t * bytes) {
	float_bytes fb;

	fb.s[0] = bytes[0];
	fb.s[1] = bytes[1];
	fb.s[2] = bytes[2];
	fb.s[3] = bytes[3];

	return fb.f;
}

void u32_to_bytes(uint8_t * bytes, u32 uint) {
	uint_bytes uib;

	uib.ui = uint;
	bytes[0] = uib.s[0];
	bytes[1] = uib.s[1];
	bytes[2] = uib.s[2];
	bytes[3] = uib.s[3];
}

u32 bytes_to_u32(uint8_t * bytes) {
	uint_bytes uib;

	uib.s[0] = bytes[0];
	uib.s[1] = bytes[1];
	uib.s[2] = bytes[2];
	uib.s[3] = bytes[3];

	return uib.ui;
}
