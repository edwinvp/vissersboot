#ifndef HMC5843_H
#define HMC5843_H

#include "settings.h"
#ifndef WIN32
#include <avr/common.h>
#endif

#define HMC5843_W	0x3C
#define HMC5843_R	0x3D

bool init_hmc5843(void);
int16_t read_hmc5843(char reg_adr);

#endif
