#ifndef IST8310H
#define IST8310H

#include "settings.h"
#include "mag_base.h"
#ifndef WIN32
#include <avr/common.h>
#endif

#define IST8310_ADDR 0xE

enum EIST8310_REGISTER {
	EI_WAI = 0x0,
	EI_STAT1 = 0x2,
	EI_DATAXL = 0x3,
	EI_DATAXH = 0x4,
	EI_DATAYL = 0x5,
	EI_DATAYH = 0x6,
	EI_DATAZL = 0x7,
	EI_DATAZH = 0x8,
	EI_STAT2 = 0x9,
	EI_CNTL1 = 0xa,
	EI_CNTL2 = 0xb,
	EI_STR = 0xc,
	EI_TEMPL = 0x1c,
	EI_TEMPH = 0x1d,
	EI_AVGCNTL = 0x41,
	EI_PDCNTL = 0x42	
};

class CIST8310 : public CBaseMag
{
public:	
	virtual bool detect();
	virtual bool init();
	virtual void sample();
};


#endif
