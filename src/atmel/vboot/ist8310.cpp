#include <stdio.h>
#include <avr/pgmspace.h>
#include "ist8310.h"
#include "i2c.h"
#include "settings.h"

void ist8310_wr_reg(unsigned int reg, unsigned char data)
{
	// Init
	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(IST8310_W); //write to the MAG3310
	i2cWaitForComplete();

	i2cSendByte(reg);
	i2cWaitForComplete();

	i2cSendByte(data);
	i2cWaitForComplete();

	i2cSendStop();
}

class CResult
{
public:
	bool okay;
	int data;
	CResult() : okay(false),data(0) {};
};


CResult ist8310_rd_reg16(char reg_adr)
{
	CResult r;

	i2cSendStart();
	bool a = i2cWaitForComplete();

	i2cSendByte(IST8310_W);	// write to this I2C address, R/*W cleared
	bool b = i2cWaitForComplete();

	i2cSendByte(reg_adr);	//Read from a given address
	bool c = i2cWaitForComplete();

	i2cSendStart();
	bool d = i2cWaitForComplete();
	
	i2cSendByte(IST8310_R); // read from this I2C address, R/*W Set
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

CResult ist8310_rd_reg8(unsigned int reg_adr)
{
	CResult r;
	
	i2cSendStart();
	bool a = i2cWaitForComplete();

	i2cSendByte(IST8310_W);	// write to this I2C address, R/*W cleared
	bool b = i2cWaitForComplete();

	i2cSendByte(reg_adr);	//Read from a given address
	bool c = i2cWaitForComplete();

	i2cSendStart();
	bool d = i2cWaitForComplete();
	
	i2cSendByte(IST8310_R); // read from this I2C address, R/*W Set
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

void ist8310_test()
{
	// Start single measurement
	ist8310_wr_reg(EI_CNTL1,0x01);

	CResult r;
	r = ist8310_rd_reg8(EI_WAI);
	b_printf(PSTR("\r\nwai=%d \r\n"), r.data);

	r = ist8310_rd_reg8(EI_STAT1);
	b_printf(PSTR("stat1=%d \r\n"), r.data);

	r = ist8310_rd_reg8(EI_STAT2);
	b_printf(PSTR("stat2=%d \r\n"), r.data);

	r = ist8310_rd_reg8(EI_TEMPH);
	b_printf(PSTR("tempH=%d \r\n"), r.data);

	r = ist8310_rd_reg8(EI_TEMPL);
	b_printf(PSTR("tempL=%d \r\n"), r.data);
	
	r = ist8310_rd_reg16(EI_DATAXL);
	b_printf(PSTR("X=%d \r\n"), r.data);

	r = ist8310_rd_reg16(EI_DATAYL);
	b_printf(PSTR("X=%d \r\n"), r.data);

	r = ist8310_rd_reg16(EI_DATAZL);
	b_printf(PSTR("X=%d \r\n"), r.data);
	

/*
	for (int reg=0;reg<100; reg++) {
		r = ist8310_rd_reg8(reg);
		if (r.okay)
			b_printf(PSTR("r[%d]=%d \r\n"), reg,r.data);
		else
			b_printf(PSTR("r[%d]=(failed) \r\n"), reg);
	}
*/

}