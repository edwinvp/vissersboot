#ifndef HMC5843_H
#define HMC5843_H

#include "settings.h"
#include "compass_rawvalues.h"
#include "mag_base.h"
#ifndef WIN32
#include <avr/common.h>
#endif

#define HMC5843     0x1e

class CHMC5843 : public CBaseMag
{
protected:
	TCompassRawValue read_hmc5843(char reg_adr);

public:
	
	virtual bool detect();
	virtual bool init();
	virtual void sample();
};

#endif
