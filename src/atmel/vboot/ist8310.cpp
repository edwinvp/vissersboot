#include <stdio.h>
#include <avr/pgmspace.h>
#include "ist8310.h"
#include "i2c.h"

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

	post_sample_check();
}
