#include <stdio.h>
#include <avr/pgmspace.h>
#include "hmc5843.h"
#include "i2c.h"

#ifndef _WIN32

bool CHMC5843::detect(void)
{
	// Read "identification register A". Expected to read ASCII 'H'
	CResult r = read_i2c_reg8(HMC5843,10);
	return r.data == 'H';
}

//Setup HMC5843 for constant measurement mode
bool CHMC5843::init(void)
{
	//Write to Mode register (2),
	//Clear bit 1, the MD1 bit (by writing 0 to register)
	write_i2c_reg(HMC5843,0x02,0x00);
	
	CResult r = read_i2c_reg8(HMC5843,0x02);
	
	int d = r.data;
	b_printf(PSTR("Reg 2 read back: %d\r\n"),d);
	
	if (!r.okay || (r.data != 0))
		b_printf(PSTR("Initialization of mag failed\r\n"));

	
	return true;
}

TCompassRawValue CHMC5843::read_hmc5843(char reg_adr)
{
	return read_i2c_reg16_be(HMC5843,reg_adr);
}

void CHMC5843::sample()
{
	compass_raw.x.valid=false;
	compass_raw.y.valid=false;
	compass_raw.z.valid=false;

	CResult r = read_i2c_reg8(HMC5843,0x02);
	if (r.okay && ( r.data != 0))  {
		b_printf(PSTR("Huh? Somebody changed the mag mode register!\r\n"));
		write_i2c_reg(HMC5843,0x02,0x00);
	}
	
	compass_raw.x = read_hmc5843(0x03);
	if (compass_raw.x.valid)
		compass_raw.y = read_hmc5843(0x05);
	if (compass_raw.y.valid)
		compass_raw.z = read_hmc5843(0x07);	
}

#else
void init_hmc5843(void)
{
}

int16_t read_hmc5843(char reg_adr)
{
	return 0;
}

#endif
