#include "m8n.h"
#include "i2c.h"

void m8n_set_reg_addr(unsigned char r_addr)
{
		i2cSendStart();
		i2cWaitForComplete();
		
		i2cSendByte(M8N_W);
		i2cWaitForComplete();
		
		i2cSendByte(r_addr);
		i2cWaitForComplete();

		i2cSendStop();
		i2cWaitForComplete();		
}

int multi_read_m8n(TinyGPS & gps)
{
	int c(0);
	
	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(M8N_R);
	i2cWaitForComplete();
	
	unsigned char d(0x0);
	bool valid(false);
	do {
		_delay_us(15);

		//i2cReceiveByte(FALSE);
		i2cReceiveByte(TRUE);
		
		i2cWaitForComplete();
		d = i2cGetReceivedByte();
		
		valid = d != 0 && d != 0xff;
		
		if (valid && c < 100) {
/*
			if (d>=31)
				printf(PSTR("%c"), d);
			else
				printf(PSTR("."));
*/				
			
			gps.encode(d);
		}
		
		c++;
	} while (c < 10 || (c < 500 && valid));

		i2cReceiveByte(FALSE);
		i2cWaitForComplete();
		d = i2cGetReceivedByte();

	
	i2cSendStop();
	i2cWaitForComplete();
	
	return c;
	
}

unsigned char read_m8n()
{
	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(M8N_R);
	i2cWaitForComplete();
				
	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	unsigned char d = i2cGetReceivedByte();
	
	i2cSendStop();
	i2cWaitForComplete();
	
	return d;
}