#ifndef MAG_BASE_H
#define MAG_BASE_H

#ifndef WIN32
#include <avr/common.h>
#endif

#include "settings.h"
#include "compass_rawvalues.h"
#include "i2c.h"

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
	TCompassRawValue m_x;
	TCompassRawValue m_y;
	TCompassRawValue m_z;
	
	bool write_i2c_reg(unsigned char addr7, unsigned char regno, unsigned char data);
	CResult read_i2c_reg8(unsigned char addr7, char reg_adr);
	CResult read_i2c_reg16_le(unsigned char addr7, char reg_adr);
	
public:
	virtual bool init() { return false; };
	virtual void sample() {};	
	
	TCompassRawValue GetX();
	TCompassRawValue GetY();
	TCompassRawValue GetZ();		
};

#endif
