//---------------------------------------------------------------------------
#ifndef fakeioH
#define fakeioH
//---------------------------------------------------------------------------
#include <System.hpp>
#include "TinyGPS.h"
#include "lat_lon.h"
#include "settings.h"
//---------------------------------------------------------------------------
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
//---------------------------------------------------------------------------
// Dummy definitions for simulator...
// ...so there will be no UART compile error
struct FILE
{
};
// ...and no problems for this missing AVR function as well
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
extern uint8_t DDRC;
extern uint8_t PORTC;
//---------------------------------------------------------------------------
int printf(const char * fmt, ... );
//---------------------------------------------------------------------------
int main_init (void);
void process();
void Fake_UART_ISR(unsigned UDR0);
extern volatile unsigned long global_ms_timer;
//---------------------------------------------------------------------------
extern TMainState main_state; // (main) sequencer state
extern unsigned long state_time; // time elapsed in this state machine step [ms]
extern int joy_pulses; // # times the goto/store joystick has been pushed up/down
//---------------------------------------------------------------------------
extern CLatLon gp_mem_1; // memorized GPS position 1 (usually 'home')
extern CLatLon gp_mem_2; // memorized GPS position 2
extern CLatLon gp_mem_3; // memorized GPS position 3
//---------------------------------------------------------------------------
extern CLatLon gp_current; // current GPS position (may be stale or invalid!)
extern CLatLon gp_start; // GPS position when auto steering was switched on
extern CLatLon gp_finish; // auto steering target GPS position
//---------------------------------------------------------------------------
extern float bearing_sp;
extern float distance_m; // distance to finish from current gps pos
extern bool arrived; // TRUE when arriving at the waypoint
//---------------------------------------------------------------------------
// PID registers
extern float p_add;
extern float i_add;
extern float d_add;
//---------------------------------------------------------------------------

#endif

