#include "mag_base.h"

bool CBaseMag::write_i2c_reg(unsigned char addr7, unsigned char regno, unsigned char data)
{
	// Init
	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(addr7<<1); //write to the MAG3310
	i2cWaitForComplete();

	i2cSendByte(regno);
	i2cWaitForComplete();

	i2cSendByte(data);
	i2cWaitForComplete();

	i2cSendStop();
	
	return true;
}

CResult CBaseMag::read_i2c_reg16_le(unsigned char addr7, char reg_adr)
{
	CResult r;
	
	unsigned char addr8 = addr7<<1;

	i2cSendStart();
	bool a = i2cWaitForComplete();

	i2cSendByte(addr8);	// write to this I2C address, R/*W cleared
	bool b = i2cWaitForComplete();

	i2cSendByte(reg_adr);	//Read from a given address
	bool c = i2cWaitForComplete();

	i2cSendStart();
	bool d = i2cWaitForComplete();
	
	i2cSendByte(addr8|1); // read from this I2C address, R/*W Set
	bool e = i2cWaitForComplete();

	bool ae = a && b && c && d && e;
	
	if (!ae) {
		// We coulnd't start the transaction or something else went wrong.
		// Terminate I2C transaction and reset I2C peripheral.
		i2cSendStop();
		i2cWaitForComplete();
		i2creset();
		r.okay=false;
		return r;
	}

	// Read compass registers
	i2cReceiveByte(TRUE);
	bool f = i2cWaitForComplete();
	char lsb = i2cGetReceivedByte(); //Read the LSB data
	bool g = i2cWaitForComplete();

	i2cReceiveByte(FALSE);
	bool h = i2cWaitForComplete();
	char msb = i2cGetReceivedByte(); //Read the MSB data
	bool i = i2cWaitForComplete();

	bool fi = f && g && h && i;

	// Terminate I2C transaction
	i2cSendStop();
	i2cWaitForComplete();

	if (!fi) {
		// Something went wrong when receiving the values from the I2C-slave.
		r.okay=false;
		return r;
	}

	// We read the bytes okay. Make integer from those bytes
	// and set status valid.
	r.data = (msb<<8) | lsb;
	r.okay = true;
	
	return r;
}

CResult CBaseMag::read_i2c_reg8(unsigned char addr7, char reg_adr)
{
	CResult r;
	
	unsigned char addr8  = addr7<<1;
	
	i2cSendStart();
	bool a = i2cWaitForComplete();	

	i2cSendByte(addr8);	// write to this I2C address, R/*W cleared
	bool b = i2cWaitForComplete();

	i2cSendByte(reg_adr);	//Read from a given address
	bool c = i2cWaitForComplete();

	i2cSendStart();
	bool d = i2cWaitForComplete();
	
	i2cSendByte(addr8|1); // read from this I2C address, R/*W Set
	bool e = i2cWaitForComplete();

	bool ae = a && b && c && d && e;
	
	if (!ae) {
		// We coulnd't start the transaction or something else went wrong.
		// Terminate I2C transaction and reset I2C peripheral.
		i2cSendStop();
		i2cWaitForComplete();
		i2creset();
		return r;
	}

	// Read compass registers
	i2cReceiveByte(TRUE);
	bool f = i2cWaitForComplete();
	char msb = i2cGetReceivedByte(); //Read the LSB data
	bool g = i2cWaitForComplete();

	bool fi = f && g;

	// Terminate I2C transaction
	i2cSendStop();
	i2cWaitForComplete();

	if (!fi) {
		// Something went wrong when receiving the values from the I2C-slave.
		return r;
	}

	r.data = msb;
	r.okay = true;
	return r;
}
