#ifndef MAG_BASE_H
#define MAG_BASE_H

#ifndef WIN32
#include <avr/common.h>
#endif

#include "settings.h"
#include "compass_rawvalues.h"
#ifndef WIN32
#include "i2c.h"
#endif

class CResult
{
public:
	bool okay;
	int data;
	CResult() : okay(false),data(0) {};
};

class CBaseMag
{
protected:
	void abort_i2c_transfer();
	bool write_i2c_reg(unsigned char addr7, unsigned char regno, unsigned char data);
	CResult read_i2c_reg8(unsigned char addr7, char reg_adr);
	TCompassRawValue read_i2c_reg16_le(unsigned char addr7, char reg_adr);
	TCompassRawValue read_i2c_reg16_be(unsigned char addr7, char reg_adr);
	bool setup_read(unsigned char addr7, char reg_adr);
	void post_sample_check();
	
public:
	TCompassTriple compass_raw;

	virtual bool detect() { return false; };
	virtual bool init() { return false; };
	virtual void sample() {};	
};

#endif
