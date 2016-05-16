//---------------------------------------------------------------------------
#include <stdarg.h>
#include <stdio.h>
//---------------------------------------------------------------------------

#pragma hdrstop

#include "fakeio.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
//---------------------------------------------------------------------------
void sei() {};
//---------------------------------------------------------------------------
uint16_t OCR1A;
uint16_t OCR1B;
uint8_t PORTB;

uint8_t DDRC;
uint8_t PORTC;
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
int16_t read_hmc5843(char reg_adr)
{
	switch (reg_adr) {
	case 0x03:
		return ext_compass_x;
	case 0x05:
		return ext_compass_y;
	case 0x07:
		return ext_compass_z;
	};

	return 0;
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

