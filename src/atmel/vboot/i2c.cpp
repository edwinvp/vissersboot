#include "i2c.h"

/*********************
 ****I2C Functions****
 *********************/

void i2cInit(void)
{
	// set i2c bit rate
	i2cSetBitrate(50);
	// enable TWI (two-wire interface)
	sbi(TWCR, TWEN);	// Enable TWI
}

void i2cSetBitrate(unsigned char bitrate_div)
{
	// set i2c bitrate
	// SCL freq = F_CPU/(16+2*(bitrate_div))
	cbi(TWSR, TWPS0);
	cbi(TWSR, TWPS1);
	
	//calculate bitrate division		
	outb(TWBR, bitrate_div);
}

void i2cSendStart(void)
{
	WRITE_sda();
	// send start condition
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
}

void i2cSendStop(void)
{
	// transmit stop condition
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

void i2cWaitForComplete(void)
{
	int i = 0;		//time out variable
	
	// wait for i2c interface to complete operation
    while ((!(TWCR & (1<<TWINT))) && (i < 10000))
		i++;
//	if (i>=20000)
//		printf(PSTR("i2c complete timed out\r\n"));
}

void i2cSendByte(unsigned char data)
{
	_delay_us(10);
	//printf(PSTR("sending 0x%x\n"), data);
	WRITE_sda();
	// save data to the TWDR
	TWDR = data;
	// begin send
	TWCR = (1<<TWINT)|(1<<TWEN);
}

void i2cReceiveByte(unsigned char ackFlag)
{
	// begin receive over i2c
	if( ackFlag )
	{
		// ackFlag = TRUE: ACK the recevied data
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWEA));
	}
	else
	{
		// ackFlag = FALSE: NACK the recevied data
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT));
	}
}

unsigned char i2cGetReceivedByte(void)
{
	// retrieve received data byte from i2c TWDR
	return( inb(TWDR) );
}

unsigned char i2cGetStatus(void)
{
	// retrieve current i2c status from i2c TWSR
	return( inb(TWSR) );
}

