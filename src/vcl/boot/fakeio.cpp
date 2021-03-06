//---------------------------------------------------------------------------
#include <stdarg.h>
#include <stdio.h>
//---------------------------------------------------------------------------
#include "fakeio.h"
//---------------------------------------------------------------------------

#pragma hdrstop

#define EEPROM_WORDS 512
uint16_t eeprom_data[EEPROM_WORDS];

//---------------------------------------------------------------------------
#pragma package(smart_init)
//---------------------------------------------------------------------------
void sei() {};
void cli() {};
//---------------------------------------------------------------------------
void eeprom_busy_wait() {};
//---------------------------------------------------------------------------
uint16_t eeprom_read_word(uint16_t* addr)
{
	unsigned int dwOffs = reinterpret_cast<unsigned int>(addr);

	if (dwOffs >= 0 && dwOffs < EEPROM_WORDS)
		return eeprom_data[dwOffs];
	return 0;
};
//---------------------------------------------------------------------------
void eeprom_write_word(uint16_t* addr, uint16_t data)
{
	unsigned int dwOffs = reinterpret_cast<unsigned int>(addr);

	if (dwOffs >= 0 && dwOffs < EEPROM_WORDS)
		eeprom_data[dwOffs] = data;
};

//---------------------------------------------------------------------------
uint16_t OCR3A;
uint16_t OCR4D;
uint8_t PORTB;

uint8_t DDRC;
uint8_t DDRD;
uint8_t PORTC;
uint8_t PORTD;
uint8_t PINC;
uint8_t PINF;
uint8_t TC4H;
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
void m8n_set_reg_addr(int d)
{
	//
}
//---------------------------------------------------------------------------
void process_gps_bytes(TinyGPS & gps)
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
extern "C" {
	void USART_SetBaud(int baud)
	{
	}

	void redirect_std_out()
	{
    }
}
//---------------------------------------------------------------------------


