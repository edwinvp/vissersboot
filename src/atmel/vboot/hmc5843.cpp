#include <stdio.h>
#include <avr/pgmspace.h>
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

TCompassRawValue read_hmc5843(char reg_adr)
{
	TCompassRawValue rawvalue;

	i2cSendStart();
	bool a = i2cWaitForComplete();

	i2cSendByte(HMC5843_W);	// write to this I2C address, R/*W cleared
	bool b = i2cWaitForComplete();

	i2cSendByte(reg_adr);	//Read from a given address
	bool c = i2cWaitForComplete();

	i2cSendStart();
	bool d = i2cWaitForComplete();
	
	i2cSendByte(HMC5843_R); // read from this I2C address, R/*W Set
	bool e = i2cWaitForComplete();

	bool ae = a && b && c && d && e;
	
	if (!ae) {
		// We coulnd't start the transaction or something else went wrong.
		// Terminate I2C transaction and reset I2C peripheral.
		i2cSendStop();
		i2cWaitForComplete();
		i2creset();
		rawvalue.valid=false;
		return rawvalue;		
	}

	// Read compass registers
	i2cReceiveByte(TRUE);
	bool f = i2cWaitForComplete();
	char msb = i2cGetReceivedByte(); //Read the LSB data
	bool g = i2cWaitForComplete();

	i2cReceiveByte(FALSE);
	bool h = i2cWaitForComplete();
	char lsb = i2cGetReceivedByte(); //Read the MSB data
	bool i = i2cWaitForComplete();

	bool fi = f && g && h && i;

	// Terminate I2C transaction
	i2cSendStop();
	i2cWaitForComplete();

	if (!fi) {
		// Something went wrong when receiving the values from the I2C-slave.
		rawvalue.valid=false;
		return rawvalue;
	}

	// We read the bytes okay. Make integer from those bytes
	// and set status valid.
	rawvalue.value = (msb<<8) | lsb;
	rawvalue.valid = true;
	
	return rawvalue;
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
