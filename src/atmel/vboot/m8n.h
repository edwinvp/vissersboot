#ifndef M8NH
#define M8NH

#include "settings.h"
#include "TinyGPS.h"

#define M8N_W 0x84
#define M8N_R 0x85

void m8n_set_reg_addr(unsigned char r_addr);
int multi_read_m8n(TinyGPS & gps);
unsigned char read_m8n();

#endif