//---------------------------------------------------------------------------
#ifndef fakeioH
#define fakeioH
//---------------------------------------------------------------------------
#include <System.hpp>
#include "TinyGPS.h"
//---------------------------------------------------------------------------
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
//---------------------------------------------------------------------------
struct FILE
{
};
//---------------------------------------------------------------------------
void sei();
//---------------------------------------------------------------------------
#define PORTB5 5
//---------------------------------------------------------------------------
#define _BV(x) (1 << x)
//---------------------------------------------------------------------------
// motor left + motor right PWM
extern uint16_t OCR1A; // motor right (out)
extern uint16_t OCR1B; // motor left (out)
//---------------------------------------------------------------------------
extern volatile unsigned int pd6_pulse_duration; // left motor (in)
extern volatile unsigned int pd5_pulse_duration; // right motor (in)
extern volatile unsigned int pd3_pulse_duration; // Pos (RX channel 3)
extern volatile unsigned int pb3_pulse_duration; // Man/auto (RX channel 4)

//---------------------------------------------------------------------------

extern uint8_t PORTB;
extern AnsiString prog_op;
//---------------------------------------------------------------------------
int printf(const char * fmt, ... );
//---------------------------------------------------------------------------
int main_init (void);
void process (TinyGPS & gps);
extern volatile unsigned long prog_ms;
//---------------------------------------------------------------------------

#endif
