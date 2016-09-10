#include "crc.h"

uint16_t crc16(const unsigned char* data_p, unsigned char length)
{
    unsigned char x;
    uint16_t  crc = 0xFFFF;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((uint16_t )(x << 12)) ^ ((uint16_t )(x <<5)) ^ ((uint16_t )x);
    }
    return crc;
}
