#include <stdio.h>
#include <avr/pgmspace.h>
#include "mag_base.h"
#include "settings.h"

bool CBaseMag::write_i2c_reg(unsigned char addr7, unsigned char regno, unsigned char data)
{
	// Init
	i2cSendStart();
	i2cWaitForComplete();
	
	unsigned char addr8 = addr7<<1;

	i2cSendByte(addr8); //write to the MAG3310
	i2cWaitForComplete();

	i2cSendByte(regno);
	i2cWaitForComplete();

	i2cSendByte(data);
	i2cWaitForComplete();

	i2cSendStop();
	i2cWaitForComplete();
	
	return true;
}

void CBaseMag::abort_i2c_transfer()
{
	i2cSendStop();
	i2cWaitForComplete();
	i2creset();	
}

#define TWIMSK_START 0x08
#define TWIMSK_RESTART 0x10
#define TWIMSK_SLA_W_ACK 0x18

bool CBaseMag::setup_read(unsigned char addr7, char reg_adr)
{
	unsigned char addr8 = addr7<<1;

	i2cSendStart();
	bool a = i2cWaitForComplete();

	if ((i2cGetStatus() & 0xf8) != TWIMSK_START) {
		b_printf(PSTR("{NO START}"));
		abort_i2c_transfer();
		return false;
	}
	
	if (!a) {			
		abort_i2c_transfer();
		return false;
	}

	i2cSendByte(addr8);	// write to this I2C address, R/*W cleared
	bool b = i2cWaitForComplete();

	if ((i2cGetStatus() & 0xf8) != TWIMSK_SLA_W_ACK) {
		b_printf(PSTR("{NO SLA W ACK}"));
		abort_i2c_transfer();
		return false;
	}
	

	i2cSendByte(reg_adr);	//Read from a given address
	bool c = i2cWaitForComplete();

	i2cSendStart();
	bool d = i2cWaitForComplete();

	if ((i2cGetStatus() & 0xf8) != TWIMSK_RESTART) {
		b_printf(PSTR("{NO RESTART}"));
		abort_i2c_transfer();
		return false;
	}	
	
	if (!d) {
		abort_i2c_transfer();
		return false;
	}

	
	i2cSendByte(addr8|1); // read from this I2C address, R/*W Set
	bool e = i2cWaitForComplete();

	bool ae = a && b && c && d && e;
	
	if (!ae) {
		// We coulnd't start the transaction or something else went wrong.
		// Terminate I2C transaction and reset I2C peripheral.
		abort_i2c_transfer();
		return false;
	}	
	
	return true;
}

// used by IST8310
TCompassRawValue CBaseMag::read_i2c_reg16_le(unsigned char addr7, char reg_adr)
{
	TCompassRawValue r;
	
	if (!setup_read(addr7,reg_adr)) {
		return r;
	}	

	// Read compass registers
	i2cReceiveByte(TRUE);
	bool f = i2cWaitForComplete();
	unsigned char lsb = i2cGetReceivedByte(); //Read the LSB data
	bool g = i2cWaitForComplete();

	i2cReceiveByte(FALSE);
	bool h = i2cWaitForComplete();
	unsigned char msb = i2cGetReceivedByte(); //Read the MSB data
	bool i = i2cWaitForComplete();

	bool fi = f && g && h && i;

	// Terminate I2C transaction
	i2cSendStop();
	i2cWaitForComplete();

	if (!fi) {
		// Something went wrong when receiving the values from the I2C-slave.
		r.valid=false;
		return r;
	}

	// We read the bytes okay. Make integer from those bytes
	// and set status valid.
	r.value = (msb<<8) | lsb;
	r.valid = true;
	
	return r;
}

// used by HMC5843
TCompassRawValue CBaseMag::read_i2c_reg16_be(unsigned char addr7, char reg_adr)
{
	TCompassRawValue r;
	
	if (!setup_read(addr7,reg_adr)) {
		return r;
	}

	// Read compass registers
	i2cReceiveByte(TRUE);
	bool f = i2cWaitForComplete();
	unsigned char msb = i2cGetReceivedByte(); //Read the LSB data
	bool g = i2cWaitForComplete();

	i2cReceiveByte(FALSE);
	bool h = i2cWaitForComplete();
	unsigned char lsb = i2cGetReceivedByte(); //Read the MSB data
	bool i = i2cWaitForComplete();

	bool fi = f && g && h && i;

	// Terminate I2C transaction
	i2cSendStop();
	i2cWaitForComplete();

	if (!fi) {	
		// Something went wrong when receiving the values from the I2C-slave.
		r.valid=false;
		return r;
	}

	// We read the bytes okay. Make integer from those bytes
	// and set status valid.
	r.value = (msb<<8) | lsb;
	r.valid = true;

	return r;
}

CResult CBaseMag::read_i2c_reg8(unsigned char addr7, char reg_adr)
{
	CResult r;
	
	if (!setup_read(addr7,reg_adr)) {
		return r;
	}

	// Read compass register
	i2cReceiveByte(FALSE);
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

void CBaseMag::post_sample_check()
{
	if (!(compass_raw.x.valid && compass_raw.y.valid && compass_raw.z.valid)) {
		b_printf(PSTR("Not all axes read!\r\n"));
		return;
	}
}
