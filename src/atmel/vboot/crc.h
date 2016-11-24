#ifndef crcH
#define crcH

#ifndef _WIN32
#include <avr/common.h>
#else
#include "fakeio.h"
#endif

uint16_t crc16(const unsigned char* data_p, unsigned char length);

#endif
