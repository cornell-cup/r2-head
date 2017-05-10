

#ifndef _USB_MAIN
#define _USB_MAIN

#include <stdint.h>
#include "R2Protocol.h"

#define BUFFER_LENGTH   255

int ProcessIO(struct R2ProtocolPacket *packet);

uint16_t readIndex, writeIndex;
uint8_t readBuffer[BUFFER_LENGTH];

enum usb_result{
    REG_DATA = 0,
    WHOAMI_PING,
    WRONG_FORMAT,
    WRONG_DEST
};

#define WHOAREYOU_REQ "WHOAREYOU"

#endif