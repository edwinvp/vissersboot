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
#include "mag_base.h"
//---------------------------------------------------------------------------
#include "faketypes.h"
//---------------------------------------------------------------------------
#define PORTB5 5
#define PORTD2 2
#define PORTD6 6
#define PORTC7 7
#define PINC0 0
#define PINF7 7
#define DDC7 7
#define DDD6 6
//---------------------------------------------------------------------------
#define _BV(x) (1 << x)
//---------------------------------------------------------------------------
// motor left + motor right PWM
extern uint16_t OCR3A; // right motor
extern uint16_t OCR4D; // left motor (1/2)
extern uint8_t TC4H; // left motor (2/2)
//---------------------------------------------------------------------------
extern volatile unsigned int k1_pulse_duration; // left motor (in)
extern volatile unsigned int k2_pulse_duration; // right motor (in)
extern volatile unsigned int k3_pulse_duration; // Pos (RX channel 3)
extern volatile unsigned int k4_pulse_duration; // Man/auto (RX channel 4)
extern volatile unsigned char k1_alive;
extern volatile unsigned char k2_alive;
extern volatile unsigned char k3_alive;
extern volatile unsigned char k4_alive;
//---------------------------------------------------------------------------
extern uint8_t PORTB;
extern AnsiString prog_op;
extern uint8_t DDRC;
extern uint8_t DDRD;
extern uint8_t PORTC;
extern uint8_t PORTD;
extern uint8_t PORTF;
extern uint8_t PINC;
extern uint8_t PINF;
//---------------------------------------------------------------------------
extern int16_t ext_compass_x;
extern int16_t ext_compass_y;
extern int16_t ext_compass_z;
//---------------------------------------------------------------------------
int b_printf(const char * fmt, ... );
int printf_P(const char * fmt, ... );
//---------------------------------------------------------------------------
void m8n_set_reg_addr(int d);
//---------------------------------------------------------------------------
int main_init (void);
void process();
void Fake_UART_ISR(unsigned UDR0);
void process_gps_bytes(TinyGPS & gps);

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

class CFakeCompass : public CBaseMag
{
public:
	virtual bool detect() { return true; };
	virtual bool init() { return true; };

	virtual void sample() {
		compass_raw.x.value = ext_compass_x; // 3
		compass_raw.y.value = ext_compass_y; // 5
		compass_raw.z.value = ext_compass_z; // 7
		compass_raw.x.valid=true;
		compass_raw.y.valid=true;
		compass_raw.z.valid=true;
	};
};

extern TinyGPS gps;

#endif

