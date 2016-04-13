#ifndef HMC5843_H
#define HMC5843_H

#include "settings.h"
#include <avr/common.h>

#define HMC5843_W	0x3C
#define HMC5843_R	0x3D

void init_hmc5843(void);
int16_t read_hmc5843(char reg_adr);

#endif
