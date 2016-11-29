#include "hmc5843.h"
#include "i2c.h"

#ifndef _WIN32
//Setup HMC5843 for constant measurement mode
bool init_hmc5843(void)
{
	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(HMC5843_W); //write to the HMC5843
	i2cWaitForComplete();

	i2cSendByte(0x02); //Write to Mode register
	i2cWaitForComplete();

	i2cSendByte(0x00); //Clear bit 1, the MD1 bit
	i2cWaitForComplete();

	i2cSendStop();
	
	return true;
}

int16_t read_hmc5843(char reg_adr)
{
	char lsb(0), msb(0);

	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(HMC5843_W);	// write to this I2C address, R/*W cleared
	i2cWaitForComplete();

	i2cSendByte(reg_adr);	//Read from a given address
	i2cWaitForComplete();

//	i2cSendStop();
//	i2cWaitForComplete();

	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(HMC5843_R); // read from this I2C address, R/*W Set
	i2cWaitForComplete();

	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	msb = i2cGetReceivedByte(); //Read the LSB data
	i2cWaitForComplete();

	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	lsb = i2cGetReceivedByte(); //Read the MSB data
	i2cWaitForComplete();

	i2cSendStop();

	return( (msb<<8) | lsb);
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
