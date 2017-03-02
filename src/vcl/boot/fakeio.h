//---------------------------------------------------------------------------
#ifndef fakeioH
#define fakeioH
//---------------------------------------------------------------------------
#include <System.hpp>
#include <queue>
#include "TinyGPS.h"
#include "lat_lon.h"
#include "settings.h"
#include "state_machine.h"
#include "compass_calibrate.h"
#include "waypoints.h"
#include "steering.h"
//---------------------------------------------------------------------------
#include "faketypes.h"
//---------------------------------------------------------------------------
#define PORTB5 5
#define PORTD2 2
#define PINC0 0
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
extern volatile unsigned int pb4_pulse_duration; // Man/auto (RX channel 4)
//---------------------------------------------------------------------------
extern uint8_t PORTB;
extern AnsiString prog_op;
extern uint8_t DDRC;
extern uint8_t PORTC;
extern uint8_t PORTD;
extern uint8_t PINC;
//---------------------------------------------------------------------------
extern int16_t ext_compass_x;
extern int16_t ext_compass_y;
extern int16_t ext_compass_z;
//---------------------------------------------------------------------------
int b_printf(const char * fmt, ... );
//---------------------------------------------------------------------------
TCompassRawValue read_hmc5843(char reg_adr);
void m8n_set_reg_addr(int d);
//---------------------------------------------------------------------------
int main_init (void);
void process();
void Fake_UART_ISR(unsigned UDR0);
void multi_read_m8n(TinyGPS & gps);

extern volatile unsigned long global_ms_timer;
//---------------------------------------------------------------------------
extern std::queue<char> gps_buffer;
//---------------------------------------------------------------------------
extern CCompassCalibration cc; // compass calibration vars
//---------------------------------------------------------------------------
extern CStateMachine stm; // (main) sequencer state
//---------------------------------------------------------------------------
extern CWayPoints waypoints;
extern CSteering steering;
//---------------------------------------------------------------------------
extern float distance_m; // distance to finish from current gps pos
//---------------------------------------------------------------------------
// ...and no problems for this missing AVR function as well
void sei();
void cli();
void eeprom_busy_wait();
uint16_t eeprom_read_word(uint16_t* addr);
void eeprom_write_word(uint16_t* addr, uint16_t data);
//---------------------------------------------------------------------------

#endif

