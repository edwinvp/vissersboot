//---------------------------------------------------------------------------
#include <stdarg.h>
#include <stdio.h>
//---------------------------------------------------------------------------
#include "fakeio.h"
//---------------------------------------------------------------------------

#pragma hdrstop

//---------------------------------------------------------------------------
#pragma package(smart_init)
//---------------------------------------------------------------------------
void sei() {};
void cli() {};
//---------------------------------------------------------------------------
void eeprom_busy_wait() {};
uint16_t eeprom_read_word(uint16_t* addr) { return 0; };
void eeprom_write_word(uint16_t* addr, uint16_t data) {};
//---------------------------------------------------------------------------
uint16_t OCR1A;
uint16_t OCR1B;
uint8_t PORTB;

uint8_t DDRC;
uint8_t PORTC;
uint8_t PORTD;
uint8_t PINC;
//---------------------------------------------------------------------------
std::queue<char> gps_buffer;
//---------------------------------------------------------------------------
int16_t ext_compass_x(0);
int16_t ext_compass_y(0);
int16_t ext_compass_z(0);
//---------------------------------------------------------------------------
AnsiString prog_op;
//---------------------------------------------------------------------------
int b_printf(const char * format, ... )
{
	int c;

	char buffer[512];
	va_list args;
	va_start (args, format);
	c=vsnprintf (buffer, 511, format, args);

	//do something with the error
    prog_op += buffer;

	va_end (args);
	return c;
}
//---------------------------------------------------------------------------
TCompassRawValue read_hmc5843(char reg_adr)
{
	TCompassRawValue v;

	switch (reg_adr) {
	case 0x03:
		v.value=ext_compass_x;
	case 0x05:
		v.value=ext_compass_y;
	case 0x07:
		v.value=ext_compass_z;
	};

	v.valid=true;

	return v;
}
//---------------------------------------------------------------------------
void m8n_set_reg_addr(int d)
{
	//
}
//---------------------------------------------------------------------------
void multi_read_m8n(TinyGPS & gps)
{
	char c(0xff);

	for (int i(0); i<100; i++) {
		if (!gps_buffer.empty()) {
			c = gps_buffer.front();
			gps_buffer.pop();
			gps.encode(c);
		}
	}
}
//---------------------------------------------------------------------------

