#include <stdio.h>
#include <avr/pgmspace.h>
#include "ist8310.h"
#include "i2c.h"
#include "settings.h"


void CIST8310::test()
{
	// Start single measurement
	write_i2c_reg(IST8310_ADDR,EI_CNTL1,0x01);

	CResult r;
	r = read_i2c_reg8(IST8310_ADDR,EI_WAI);
	b_printf(PSTR("\r\nwai=%d \r\n"), r.data);

	r = read_i2c_reg8(IST8310_ADDR,EI_STAT1);
	b_printf(PSTR("stat1=%d \r\n"), r.data);

	r = read_i2c_reg8(IST8310_ADDR,EI_STAT2);
	b_printf(PSTR("stat2=%d \r\n"), r.data);

	TCompassRawValue rv;
	rv = read_i2c_reg16_le(IST8310_ADDR,EI_TEMPL);
	b_printf(PSTR("temp=%d \r\n"), rv.value);

	_delay_ms(5);
	
	rv = read_i2c_reg16_le(IST8310_ADDR,EI_DATAXL);
	b_printf(PSTR("X=%d \r\n"), rv.value);

	rv = read_i2c_reg16_le(IST8310_ADDR,EI_DATAYL);
	b_printf(PSTR("Y=%d \r\n"), rv.value);

	rv = read_i2c_reg16_le(IST8310_ADDR,EI_DATAZL);
	b_printf(PSTR("Z=%d \r\n"), rv.value);
}

bool CIST8310::detect()
{
	// Read who am I register. expected to be 0x10
	CResult r = read_i2c_reg8(IST8310_ADDR,EI_WAI);
	return r.data == 0x10;
}

bool CIST8310::init()
{
	return true;	
}

void CIST8310::sample()
{
	// Start single measurement
	write_i2c_reg(IST8310_ADDR,EI_CNTL1,0x01);

	_delay_ms(5);

	compass_raw.x = read_i2c_reg16_le(IST8310_ADDR,EI_DATAXL);
	compass_raw.y = read_i2c_reg16_le(IST8310_ADDR,EI_DATAYL);
	compass_raw.z = read_i2c_reg16_le(IST8310_ADDR,EI_DATAZL);
}
