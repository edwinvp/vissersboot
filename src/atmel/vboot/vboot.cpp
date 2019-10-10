#include "settings.h"
#ifndef _WIN32
#include <avr/pgmspace.h>
#endif
#include "compass_calibrate.h"
#include "state_machine.h"
#include "steering.h"
#include "joystick.h"
#include "waypoints.h"
#include "led_control.h"
#include "fifo.h"

#ifdef _WIN32
// Simulator running under Windows
#include "fakeio.h"
#else
// Atmel target
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Target: ATmega328
//#include <avr/iom328.h>
#include <avr/common.h>
#include <stdio.h>

#include "i2c.h"
#include "m8n.h"
#include "hmc5843.h"
#include "ist8310.h"

#endif

#include "TinyGPS.h"
#include "lat_lon.h"
#include "uart.h"
#include "vboot.h"

#define set_bit(REG, BIT) REG |= _BV(BIT)
#define clear_bit(REG, BIT) REG &= ~_BV(BIT)

// ----------------------------------------------------------------------------
// Control (of motors)
// ----------------------------------------------------------------------------

// Vessel thruster CW/CCW:
// joystick forward:
//    CCW >0    CCW >0
// joystick in reverse:
//    CW  <0    CW  <0
// joystick to the left:
//    CW  <0    CCW >0
// joystick to the right:
//    CCW >0    CW  <0

// ----------------------------------------------------------------------------
// RS-232 console command overview
// ----------------------------------------------------------------------------

/*
	(compass calibration)
	t: go to (compass) calibration mode
	r: reset (forget) compass calibration
	n: set current direction vessel is pointing at as (true) 'North'
	(screens)
	c: view compass data screen
	g: view GPS data screen
	s: view steering screen
	e: servo capture screen
	(settings)
	p: (normal PID 'P'-action configuration)
	i: (normal PID 'I'-action configuration)
	u: (aggressive PID 'P'-action configuration)
	y: (aggressive PID 'I'-action configuration)

	(auto steer test screen)
	a: go straight to auto steer mode
	x: don't stop steering (even after arriving)
	d: substitute PV compass heading (enter 0 for no substution)
	o: substitute SP heading (enter 0 for no substitution)

	"0123456789-+" (valid in PID configuration modi)
*/

int tune_ptr(0);
char tune_buf[16];

int CfgMode(0);

// ----------------------------------------------------------------------------
// VARIABLES
// ----------------------------------------------------------------------------

CStateMachine stm;
CSteering steering;
CJoystick joystick;
CWayPoints waypoints;
CLedControl ledctrl;

CIST8310 mag_ist8310;
CHMC5843 mag_hmc5843;
CBaseMag * mag = 0;

// Tests
bool subst_pv = false;
bool subst_sp = false;

// Periodic message type selector
TMessageMode msg_mode;

// GPS input related
unsigned long gps_fix_age = TinyGPS::GPS_INVALID_AGE;
TinyGPS gps;
bool gps_valid = false;
bool gps_valid_prev = false;
bool compass_working = false;
bool compass_working_prev = false;
long gps_lat, gps_lon, gps_course;

bool btn_prev_state(false);
bool btn_state(false);
bool btn_pressed(false);

// Compass input
int16_t good_compass_smp;
int16_t bad_compass_smp;
int16_t compass_smp;

// Compass calibration, storage etc.
CCompassCalibration cc;

// Auto steering related
float distance_m = 0; // distance to finish from current gps pos

// Global [ms] timer
volatile unsigned long global_ms_timer = 0;

// Timing for periodic processes
unsigned long t_100ms_start_ms(0);
unsigned long t_500ms_start_ms(0);

// ----------------------------------------------------------------------------
// PWM/JOYSTICK/MOTOR vars
// ----------------------------------------------------------------------------

// ML - Motor Left (in) (RX channel 1)
volatile unsigned int k1_rising;
volatile unsigned int k1_pulse_duration;
volatile unsigned char k1_alive = 0;
// MR - Motor Right (in) (RX channel 2)
volatile unsigned int k2_rising;
volatile unsigned int k2_pulse_duration;
volatile unsigned char k2_alive = 0;
// Pos (RX channel 3)
volatile unsigned int k3_rising;
volatile unsigned int k3_pulse_duration;
volatile unsigned char k3_alive = 0;
// Man/auto (RX channel 4)
volatile unsigned int k4_rising;
volatile unsigned int k4_pulse_duration;
volatile unsigned char k4_alive = 0;

// Remote control channel live signal detection
bool k1_ok(false), k2_ok(false), k3_ok(false), k4_ok(false);
bool rc_okay(false);
bool rc_okay_prev(false);

// Old PORTB values for PORTD servo signal detection
volatile unsigned char old_pinb = 0;
// ----------------------------------------------------------------------------
// Global millisecond timer value needed for TinyGps
unsigned long millis()
{
	unsigned long tmr;
	cli();
	tmr = global_ms_timer;
	sei();
	
	return tmr;
}
// ----------------------------------------------------------------------------
ISR(TIMER1_OVF_vect)
{
	// Timer 1 overflow
}
// ----------------------------------------------------------------------------
ISR(TIMER3_OVF_vect)
{
	global_ms_timer += 20;	// timer was set to overflow each 20 [ms]

	// Reset timer 4
	TC4H = 0;
	TCNT4 = 0x00;	
	
	// Force waveform generator to '1'
	// Set on compare match
	TCCR4C |= _BV(COM4D1);
	TCCR4C |= _BV(COM4D0);
	TCCR4C |= _BV(FOC4D);

	// Force waveform generator to '1'
	// Clear on compare match
	TCCR4C |= _BV(COM4D1);
	TCCR4C &= ~_BV(COM4D0);

}
// ----------------------------------------------------------------------------
#ifndef _WIN32
ISR(PCINT0_vect)
{
	// Capture timer value
	const unsigned int tmr_reg(TCNT3);
	// Capture state of PORT B pins 
	const unsigned char PBPINS(PINB);
	// Set bits that have changed since last check
	const unsigned char DIFFPINS = PBPINS ^ old_pinb;

	// PB7 servo pulse measurements
	if (DIFFPINS & _BV(PINB7)) {
		if (PBPINS & _BV(PINB7))
			k1_rising = tmr_reg;
		else {
			k1_pulse_duration = tmr_reg - k1_rising;
			if (k1_pulse_duration > 10000)
				k1_pulse_duration -= 25536;
			++k1_alive;
		}
	}

	// PB6 servo pulse measurements
	if (DIFFPINS & _BV(PINB6)) {
		if (PBPINS & _BV(PINB6))
			k2_rising = tmr_reg;
		else {
			k2_pulse_duration = tmr_reg - k2_rising;
			if (k2_pulse_duration > 10000)
				k2_pulse_duration -= 25536;
			++k2_alive;
		}
	}

	// PB5 servo pulse measurements
	if (DIFFPINS & _BV(PINB5)) {
		if (PBPINS & _BV(PINB5))
			k3_rising = tmr_reg;
		else {
			k3_pulse_duration = tmr_reg - k3_rising;
			if (k3_pulse_duration > 10000)
				k3_pulse_duration -= 25536;
			++k3_alive;
		}
	}

	// PB4 servo pulse measurements
	if (DIFFPINS & _BV(PINB4)) {
		if (PBPINS & _BV(PINB4))
			k4_rising = tmr_reg;
		else {
			k4_pulse_duration = tmr_reg - k4_rising;
			if (k4_pulse_duration > 10000)
				k4_pulse_duration -= 25536;
			++k4_alive;
		}
	}

	old_pinb = PBPINS;
}
// ----------------------------------------------------------------------------
ISR(PCINT1_vect)
{

}
// ----------------------------------------------------------------------------
ISR(PCINT2_vect)
{
}
#endif
// ----------------------------------------------------------------------------
volatile unsigned char comms_char;
/* This variable is volatile so both main and RX interrupt can use it.
It could also be a uint8_t type */

/* Interrupt Service Routine for Receive Complete
NOTE: vector name changes with different AVRs see AVRStudio -
Help - AVR-Libc reference - Library Reference - <avr/interrupt.h>: Interrupts
for vector names other than USART_RXC_vect for ATmega32 */

#ifndef _WIN32
ISR(USART1_RX_vect) {
#else
void Fake_UART_ISR(unsigned UDR0) {
#endif

    // Read UART register (the received byte)
	// into `value`
	comms_char = UDR1;

    fifo_write(comms_char);
}
// ----------------------------------------------------------------------------
void setup_capture_inputs()
{
#ifndef _WIN32
    // Configure PB4 as output (head lights / tail lights; L)
    // PORTB |= _BV(PORTB4);

	// Configure PB7 (RC CH1/K1) as input
	DDRB &= ~_BV(DDB7);
	PORTB &= ~_BV(PORTB7);
	// Configure PB6 as input (CH2/K2-MR)
	DDRB &= ~_BV(DDB6);
	PORTB &= ~_BV(PORTB6);
	// Configure PB5 as input (CH3/K3-MOT POS)
	DDRB &= ~_BV(DDB5);
	PORTB &= ~_BV(PORTB5);
	// Configure PB4 as input (CH4/K4 man/erase)
	DDRB &= ~_BV(DDB4);
	PORTB &= ~_BV(PORTB4);

	// Enable on-pin-change for pins
	PCMSK0 |= _BV(PCINT7); // PB7
	PCMSK0 |= _BV(PCINT6); // PB6
	PCMSK0 |= _BV(PCINT5); // PB5
	PCMSK0 |= _BV(PCINT4); // PB4

	// Configure interrupt on logical state state on PB4 (so PCIE0)
	PCICR |= _BV(PCIE0);
#endif
}
// ----------------------------------------------------------------------------
void setup_timer_3()
{
#ifndef _WIN32
	// Configure timer 3 clock source
	// Set to use a clock source of clkio / 8.
	// So 20 [ms] servo period will be 40000 timer ticks.
	TCCR3B &= ~_BV(CS32);
	TCCR3B |= _BV(CS31);
	TCCR3B &= ~_BV(CS30);	
	
	ICR3 = 40000; // gives a 20 [ms] period
#endif
}
// ----------------------------------------------------------------------------
void setup_timer_4()
{
	// Set CK/64 --> f=250 [kHz]
	TCCR4B = 0b00000111;
}
// ----------------------------------------------------------------------------
void setup_pwm()
{
#ifndef _WIN32
	setup_timer_3();
	setup_timer_4();

	// Configure PC6 (MR) & PD7 (ML) as outputs
	DDRC |= _BV(DDC6);
	DDRD |= _BV(DDD7);

	// Setup timer 3 for PWM
	// Set compare output mode to non-inverting (PWM) mode
	TCCR3A |= _BV(COM3A1);
	TCCR3A &= ~_BV(COM3A0);
	TCCR3A |= _BV(COM3B1);
	TCCR3A &= ~_BV(COM3B0);

	// Set mode 14 (Fast PWM, TOP=ICR3)
	// WGM13 WGM12 WGM11 WGM10
	// 1	 1	   1	 0

	TCCR3B |= _BV(WGM33);
	TCCR3B |= _BV(WGM32);
	TCCR3A |= _BV(WGM31);
	TCCR3A &= ~(_BV(WGM30));

	// Setup timer 4 for PWM
	// Clear on compare match
	TCCR4C |= _BV(COM4D1);
	// Toggle on compare match
	//TCCR4C |= _BV(COM4D0);
	
	//TCCR4C |= _BV(PWM4D);

	//OCR3A =  JOY_CENTER; // MR in center
	OCR3A =  JOY_MAX;

	set_bit(TIMSK3,TOIE3); // timer 3 overflow interrupt


	//OCR4D = JOY_CENTER; // ML in center
	
	cli();
	// 1,024 [ms] = 0x100 (-100%)
	// 1.536 [ms] = 0x180 (neutral)
	// 2,048 [ms] = 0x200 (+100%)
	TC4H = 1;
	OCR4D = 0x00;

	TC4H = 1;
	OCR4A = 0x00;
	
	TC4H = 3;
	OCR4C = 0xff;
	sei();
	
	
#endif
}
// ----------------------------------------------------------------------------
void print_steering_msg()
{
	if (gps_fix_age == TinyGPS::GPS_INVALID_AGE)
	b_printf(PSTR("GPS-NO_FIX "));
	else if (gps_fix_age > GPS_STALE_TIME)
	b_printf(PSTR("GPS-STALE "));
	else {
		b_printf(PSTR("GPS-OK "));
	}

	int a1 = joystick.to_perc(OCR1A);
	int b1 = joystick.to_perc(OCR1B);
	unsigned int sp_d = steering.sp_used;
	unsigned int pv_d = steering.pv_used;
	unsigned int err_d = steering.pid_err;

	if (stm.Step()==msAutoModeCourse)
		b_printf(PSTR("[autoC] "));
	else if (stm.Step()==msReverseThrust)
		b_printf(PSTR("[reverse] "));
	else if (stm.Step()==msAutoModeNormal)
		b_printf(PSTR("[autoN] "));
	else
		b_printf(PSTR("[man] "));

	b_printf(PSTR("age=%ld "), gps_fix_age);

	if (stm.Step()==msAutoModeCourse || stm.Step()==msAutoModeNormal) {
		b_printf(PSTR("sp=%d"), sp_d);
		if (steering.SUBST_SP!=0)
			b_printf(PSTR("*"));
	
		b_printf(PSTR(" "));

		b_printf(PSTR("pv=%d"), pv_d);
		if (steering.SUBST_PV!=0)
			b_printf(PSTR("*"));
	} else
		b_printf(PSTR("sp=-- pv=-- "));

	b_printf(PSTR(" err=%d: A=%d B=%d\r\n"), err_d, a1,b1);
}
// ----------------------------------------------------------------------------
void print_compass_msg()
{
	b_printf(PSTR("\x1b[1;1H"));
	
    b_printf(PSTR(" x=%04d  \r\n y=%04d  \r\n z=%04d  \r\n course=%04d   \r\n sp=%04d   \r\n"),
		mag->compass_raw.x.value, mag->compass_raw.y.value, mag->compass_raw.z.value,
		int(steering.compass_course),
		int(steering.bearing_sp));

    b_printf(PSTR("\r\nsmp#(good/bad)=%04d(%04d/%04d)        \r\n"), compass_smp, good_compass_smp, bad_compass_smp);
	
	cc.print_cal();
}
// ----------------------------------------------------------------------------
void print_debug_msg()
{
	print_steering_msg();
	print_compass_msg(); 	
}

// ----------------------------------------------------------------------------
void print_gps_msg()
{
	if (gps_fix_age == TinyGPS::GPS_INVALID_AGE)
		b_printf(PSTR("GPS-NO_FIX\r\n"));
	else if (gps_fix_age > GPS_STALE_TIME)
		b_printf(PSTR("GPS-STALE\r\n"));
	else {
			b_printf(PSTR("GPS-OK age=%ld. lat=%ld lon=%ld course=%ld\r\n"),
			gps_fix_age, gps_lat, gps_lon, gps_course);
	}
}
// ----------------------------------------------------------------------------
void print_stats()
{
	print_gps_msg();
}
// ----------------------------------------------------------------------------
void clear_stats(void)
{
	compass_smp=0;
	bad_compass_smp=-0;
	good_compass_smp=0;
}
// ----------------------------------------------------------------------------
void tune_PrintValue(double dblParam)
{
	long l(0);

	switch (msg_mode) {
	case mmCompass:
	case mmNone:
	case mmGps:
	case mmLast:
	case mmServoCapture:
	case mmSteering:
		break;		
	case mmPActionNorm:
	case mmIActionNorm:
	case mmPActionAggr:
	case mmIActionAggr:
		l = dblParam*1000.0;
		b_printf(PSTR("(set): %ld (x1000)\r\n"),l);
		break;
	case mmPVSubst:
	case mmSPSubst:
		l = dblParam;
		b_printf(PSTR("(set): %ld (x1)\r\n"),l);
		break;
	case mmDebug:
		break;
    case mmButton:
        break;
	}
}
// ----------------------------------------------------------------------------
void tune_Config(double & dblParam, char c)
{
	tune_buf[tune_ptr++] = c;

	if (tune_ptr>15) {
		tune_ptr=0;
		b_printf(PSTR("(err)\r\n"));
		return;
	}

	if (c==13 || c==10) {
		double nv(0);
		long ld(0);
		tune_buf[tune_ptr]=0;
		int fields = sscanf(tune_buf,"%ld",&ld);
		tune_ptr=0;
		if (fields == 1) {
			dblParam = nv;
			b_printf(PSTR("new value accepted) %ld\r\n"),ld);

			switch (msg_mode) {
			case mmNone:
			case mmCompass:
			case mmGps:
			case mmServoCapture:
			case mmSteering:
				break;										
			case mmPActionNorm:
			case mmIActionNorm:
			case mmPActionAggr:
			case mmIActionAggr:
				dblParam = ld / 1000.0;
				break;
			case mmPVSubst:
			case mmSPSubst:
				dblParam = ld;
			case mmDebug:
				break;
			case mmLast:
				break;
            case mmButton:
                break;
			};

			steering.save_calibration();

		} else {
            b_printf(PSTR("(err,bad)\r\n"));
		}
	}

}

// ----------------------------------------------------------------------------
void handle_parameterization(char c)
{
	switch (msg_mode) {
	case mmPActionNorm:
		tune_Config(steering.pid_normal.TUNE_P, c);
		break;
	case mmIActionNorm:
		tune_Config(steering.pid_normal.TUNE_I, c);
		break;
	case mmPActionAggr:
		tune_Config(steering.pid_aggressive.TUNE_P, c);
		break;
	case mmIActionAggr:
		tune_Config(steering.pid_aggressive.TUNE_I, c);
		break;		
	case mmPVSubst:
		tune_Config(steering.SUBST_PV, c);
		break;
	case mmSPSubst:
		tune_Config(steering.SUBST_SP, c);
		break;
	default:
		;		
	}
}
// ----------------------------------------------------------------------------
void toggle_msg_mode(TMessageMode mm)
{
    if (msg_mode == mm)
        msg_mode = mmNone;
    else
        msg_mode = mm;
}


// ----------------------------------------------------------------------------
// Handles GPS input
// ----------------------------------------------------------------------------
void read_uart()
{

	while (fifo_avail()) {
		char c = fifo_read();

		switch (c) {
		case 't':
			cc.toggle_calibration_mode();
			break;
		case 'r':
			cc.reset_compass_calibration();
			b_printf(PSTR("Compass calibration reset\r\n"));
			break;
		case 'n':
			cc.set_true_north();
			break;
		case 'c':
            toggle_msg_mode(mmCompass);
			break;
		case 'g':
			toggle_msg_mode(mmGps);
			break;
		case 'p':
			toggle_msg_mode(mmPActionNorm);
			break;
		case 'i':
			toggle_msg_mode(mmIActionNorm);
			break;
		case 's':
			toggle_msg_mode(mmSteering);
			break;
		case 'u':
			toggle_msg_mode(mmPActionAggr);
			break;
		case 'y':
			toggle_msg_mode(mmIActionAggr);
			break;			
		case 'e':
			toggle_msg_mode(mmServoCapture);
			break;
		case 'a':
			stm.straight_to_auto = true;
			break;
		case 'x':
            steering.toggle_dont_stop();
			break;
		case 'd':
			msg_mode = mmPVSubst;
			break;
		case 'o':
			msg_mode = mmSPSubst;
			break;
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
		case '.':
		case '-':
		case '+':
		case 13:
		case 10:
			handle_parameterization(c);
			break;
		}
	}
}

// ----------------------------------------------------------------------------
// Periodic message
// ----------------------------------------------------------------------------

void print_servo_msg(bool full)
{
	// Tell whether application considers the remote control up or down
	if (rc_okay)
		b_printf(PSTR("RC UP\r\n"));
	else
		b_printf(PSTR("RC DOWN\r\n"));
	
	// Print the values of the incoming and outgoing servo channels
	if (full) {
		int k1_perc, k2_perc, k3_perc, k4_perc, a1, b1;

		// Display incoming servo signals as received (2000 ... 4000, 0 = no signal)
		k1_perc = joystick.to_perc(k1_pulse_duration);
		k2_perc = joystick.to_perc(k2_pulse_duration);
		k3_perc = joystick.to_perc(k3_pulse_duration);
		k4_perc = joystick.to_perc(k4_pulse_duration);
		// Same for outgoing signals (to motors)
		a1 = joystick.to_perc(OCR1A);
		b1 = joystick.to_perc(OCR1B);

		b_printf(PSTR(" k1=%05d k2=%05d k3=%05d k4=%05d A=%05d B=%05d\r\n"),
    		k1_perc, k2_perc,
    		k3_perc, k4_perc,
    		a1, b1);
			
		// Show how many pulses the capture interrupts have seen
		b_printf(PSTR("Capture status: %d %d %d %d\r\n"),
			k1_alive,k2_alive,k3_alive,k4_alive);
	}
}

void periodic_msg()
{
	switch (msg_mode) {
	case mmServoCapture:
        print_servo_msg(true);
		break;

	case mmPActionNorm:
		b_printf(PSTR("(set normal P-action): "));
		tune_PrintValue(steering.pid_normal.TUNE_P);
		break;

	case mmIActionNorm:
		b_printf(PSTR("(set normal I-action): "));
		tune_PrintValue(steering.pid_normal.TUNE_I);
		break;

	case mmPActionAggr:
		b_printf(PSTR("(set aggressive P-action): "));
		tune_PrintValue(steering.pid_aggressive.TUNE_P);
		break;

	case mmIActionAggr:
		b_printf(PSTR("(set aggressive I-action): "));
		tune_PrintValue(steering.pid_aggressive.TUNE_I);
		break;

	case mmPVSubst:
		b_printf(PSTR("(set PV-subst): "));
		tune_PrintValue(steering.SUBST_PV);
		break;

	case mmSPSubst:
		b_printf(PSTR("(set SP-subst): "));
		tune_PrintValue(steering.SUBST_SP);
		break;

	case mmGps:
		print_gps_msg();
		break;

	case mmSteering:
		print_steering_msg();
		break;

	case mmDebug:
		print_debug_msg();
	break;

	case mmCompass:
		print_compass_msg();
		break;

	case mmLast:
		msg_mode = mmSteering;
		break;

    case mmButton:
        b_printf(PSTR("button: "));
        if (PINF & (1<<PINF7))
            b_printf(PSTR("up\r\n"));
        else
            b_printf(PSTR("down\r\n"));
        break;

	case mmNone:
        break;
	}
}

TLedMode Step2LedMode(TMainState step)
{
    TLedMode lm(lmOff);

	switch (step) {
    	/* in manual/auto mode, just show status of GPS receiver */
    	case msManualMode:
			lm = lmGpsStatus;
        	break;
    	case msAutoModeNormal:
		case msAutoModeCourse: // deliberate fall-through
		case msReverseThrust:  // deliberate fall-through
            lm = lmArriveStatus;
    	    break;
    	case msCmdErrorMan:
    	case msCmdErrorAuto: // deliberate fall-through
            lm = lmFastBlink;
			break;
		case msConfirmGotoPosX:
		case msConfirmStorePosX: // deliberate fall-through
			lm = lmSlowBlink;
			break;
		case msClear2:
			lm = lmSolidOn;
			break;
		case msConfirmClear:
			lm = lmSlowBlink;
			break;

    	default:
        	lm = lmOff;
	}
    return lm;
}

void check_rc()
{
	// Check for remote control servo message activity
	
	k1_ok = k1_alive > 5;
	k2_ok = k2_alive > 5;
	k3_ok = k3_alive > 5;
	k4_ok = k4_alive > 5;

	rc_okay = k1_ok && k2_ok && k3_ok && k4_ok;
	
	// Set signals in 0% position (=3000)
	if (!k1_ok)
		k1_pulse_duration = JOY_CENTER;
	if (!k2_ok)
		k2_pulse_duration = JOY_CENTER;
	if (!k3_ok)
		k3_pulse_duration = JOY_CENTER;
	if (!k4_ok)
		k4_pulse_duration = JOY_CENTER;
	
	if (rc_okay != rc_okay_prev) {		
		print_servo_msg(false);
		
		// See description of `rc_ignore_first_command` declaration
		if (rc_okay)
			stm.rc_ignore_first_command=true;
	}	
	
	rc_okay_prev = rc_okay;		
	
	k1_alive=0;
	k2_alive=0;
	k3_alive=0;
	k4_alive=0;	
}

// ----------------------------------------------------------------------------
// 100 [ms] process
// ----------------------------------------------------------------------------
void process_100ms()
{
	// Detect calibration button presses (when PORTF7 is low)
	btn_state = (PINF & (1<<PINF7))==0;
	// Detect rising edge (button pressed)
	if (btn_state && (!btn_prev_state))
		btn_pressed = true;
	btn_prev_state = btn_state;

	// Go to calibration mode if button is pressed
	if (btn_pressed && !btn_state) {
        btn_pressed = false;
        if (!cc.calibration_mode) {
            b_printf(PSTR("calibration button pressed\r\n"));
            cc.reset_compass_calibration();
            cc.toggle_calibration_mode();
        } else {
            b_printf(PSTR("leaving calibration mode\r\n"));
            cc.load_calibration();
        }   
    }

    // Switch head lights / tail lights on or off    
    if (steering.motor_running())
        PORTD |= _BV(PORTD2);
    else
        PORTD &= ~_BV(PORTD2);

	// Run main state machine
    stm.Run();

	// Steering
	TMainState step = stm.Step();
	if (step == msReverseThrust)
		steering.do_reverse_thrust();
	else if (step == msAutoModeCourse || step == msAutoModeNormal)
		steering.auto_steer();
	else {	
		steering.manual_steering(k1_pulse_duration,k2_pulse_duration);
	}

	TLedMode lm = Step2LedMode(stm.Step());

    // If calibrating, override led mode
    if (cc.calibration_mode) {
        switch (cc.get_state()) {
        case csNotCalibrated:
        case csCenterDetect:
            lm = lmCalibrationPhase1;
            break;
        case csTurn1:
            lm = lmCalibrationPhase2;
            break;
        case csTurn2:
            lm = lmCalibrationPhase3;
			break;
        case csFinish:
        case csCalibrated:
			lm = lmCalibrationPhase4;
			break;
        }
    }

    ledctrl.set_mode(lm);

	bool all_input_valid = gps_valid && compass_working && rc_okay;

	ledctrl.update(all_input_valid,steering.arrived);
	
    cc.update100ms();
}
// ----------------------------------------------------------------------------
// 500 [ms] process
// ----------------------------------------------------------------------------
void process_500ms()
{
    ledctrl.toggle_slow_blink();

	// returns +- latitude/longitude in degrees
	gps.get_position(&gps_lat, &gps_lon, &gps_fix_age);
	gps_course = gps.course();
	
	if (gps_fix_age == TinyGPS::GPS_INVALID_AGE) {
		// No gps fix
		gps_valid = false;
	} else if (gps_fix_age > GPS_STALE_TIME) {
		// Stale GPS position
		gps_valid = false;
	} else {
		// GPS-OK
		gps.f_get_position(&waypoints.gp_current.lat,&waypoints.gp_current.lon,0);
		gps_valid= true;

	}

	if (global_ms_timer > MS_BEFORE_OP_ENABLE)
		steering.set_output_enable(true);

	if (!gps_valid_prev && gps_valid)
		b_printf(PSTR("GPS up\r\n"));
	if (gps_valid_prev && !gps_valid)
		b_printf(PSTR("GPS down\r\n"));

	check_rc();

	periodic_msg();

	gps_valid_prev = gps_valid;
}

// ----------------------------------------------------------------------------
void read_compass_and_gps()
{
	mag->compass_raw.x.clear();
	mag->compass_raw.y.clear();
	mag->compass_raw.z.clear();

	bool expect_working_compass(bad_compass_smp < 4);
	bool periodic_attempt((compass_smp%500)==0);

	if (expect_working_compass || periodic_attempt) {
		// Read compass X,Y,Z reading registers.
		mag->sample();
	}
	
	compass_smp++;
	if (compass_smp>9999)
		compass_smp=0;

	bool sample_valid(mag->compass_raw.x.valid && mag->compass_raw.y.valid && mag->compass_raw.z.valid);
	
	// Only process compass values if they were read successfully via I2C
	if (sample_valid) {
		if (cc.calibration_mode)
			cc.calibrate(mag->compass_raw);
		
		steering.compass_course = cc.calc_course(mag->compass_raw);
		
		bad_compass_smp=0;
		good_compass_smp++;
		if (good_compass_smp>9999)
			good_compass_smp=9999;
		
		} else {
			bad_compass_smp++;
			if (!expect_working_compass)
				good_compass_smp=0;
			if (bad_compass_smp>9999)
				bad_compass_smp=9999;
	}

	compass_working = (good_compass_smp > 0);
	
	if (!compass_working_prev && compass_working)
		b_printf(PSTR("COMPASS up\r\n"));
	if (compass_working_prev && !compass_working)
		b_printf(PSTR("COMPASS down\r\n"));
	
	compass_working_prev = compass_working;
	
	// Read GPS-bytes, but only if compass was reachable
	if (compass_working) {
		m8n_set_reg_addr(0xff);
		multi_read_m8n(gps);
	}	
}

// ----------------------------------------------------------------------------
// MAIN PROCESS
// ----------------------------------------------------------------------------
void process()
{
	unsigned long delta(0);

	// Handle UART (GPS) input
	read_uart(); // also happens with this disabled

	// Calculate initial bearing with Haversine function
	steering.bearing_sp = waypoints.gp_current.bearingTo(waypoints.gp_finish);


	distance_m =
		TinyGPS::distance_between (waypoints.gp_finish.lat, waypoints.gp_finish.lon,
			waypoints.gp_current.lat, waypoints.gp_current.lon);
	steering.arrived = distance_m < 3;

	// Read magnetometer (compass) bytes and GPS-bytes
	read_compass_and_gps();

	// Time to run 100 [ms] process?
	delta = global_ms_timer - t_100ms_start_ms;
	if (delta >= 100) {
		t_100ms_start_ms = global_ms_timer;
		process_100ms();
	}

	// Time to run 500 [ms] process?
	delta = global_ms_timer - t_500ms_start_ms;
	if (delta > 500) {
		t_500ms_start_ms = global_ms_timer;
		process_500ms();
	}
}

void delay_ms(uint16_t x)
{
	uint16_t s = millis();

	uint16_t delta = 0;

	do {
		delta = millis() - s;
	} while (delta < x) ;

}

// ----------------------------------------------------------------------------
// Main loop (AVR only, do not use within simulator)
// ----------------------------------------------------------------------------
void main_loop()
{
	TinyGPS gps;

	while(1) {
		process();
	}
}

// ----------------------------------------------------------------------------
// MAIN/STARTUP
// ----------------------------------------------------------------------------
#ifdef _WIN32
int main_init (void)
#else
int main (void)
#endif
{
#ifndef _WIN32
	/* set pin 5 of PORTB for output*/
	DDRB |= _BV(DDB5);

	// Setup serial (UART) input
	USART_Init();  // Initialize USART
#endif
	sei();         // enable all interrupts

	b_printf(PSTR("Boot!\r\n"));

	// test pwm
	setup_pwm();
	setup_capture_inputs();

	while (1) {
		
		print_servo_msg(true);
		
		_delay_ms(500);

	}

#if 0
	cc.reset_compass_calibration();

	cc.load_calibration();
	steering.load_calibration();
    waypoints.load_waypoints();

	// Setup other peripherals
	setup_capture_inputs();
	setup_pwm();

	// Initial periodic message mode
	msg_mode = mmNone;

	clear_stats();

	// Initialize I2C-bus I/O (and button input on RC0)
#ifndef _WIN32
	DDRD = 0b00000011; // SDA/SCL pins as output (see Atmel manual)
	PORTD = 0b00000011; //pull-ups on the I2C bus
	
	DDRF = 0b00000000; // PORTF7 as input
	PORTF = 0b10000000; // pull-up on cal. button (bit 7)

	i2cInit();
	i2cSetBitrate(30);
	
	int retries(250);
	do {
		b_printf(PSTR("Probing for magnetometer...\r\n"));
		delay_ms(100);

		if (mag_ist8310.detect()) {
			b_printf(PSTR("Found IST8310 mag.\r\n"));
			mag = &mag_ist8310;
			break;
		} else if (mag_hmc5843.detect()) {
			b_printf(PSTR("Found HMC5843 mag.\r\n"));
			mag = &mag_hmc5843;
			break;
		}
	} while (--retries > 0);
	
	if (mag == 0) {
		b_printf(PSTR("Fatal error: magnetometer not found.\r\n"));
		while (1) ;
	}

	b_printf(PSTR("init 1\r\n"));
	mag->init();

	delay_ms(25);
	b_printf(PSTR("init 2\r\n"));
	mag->init();
	delay_ms(25);
	b_printf(PSTR("init 3\r\n"));
	mag->init();
	delay_ms(25);
	b_printf(PSTR("mag init done\r\n"));

#endif

    b_printf(PSTR("main_loop\r\n"));

#ifndef _WIN32
	main_loop();
#endif
#endif

	return 0;
}
// ----------------------------------------------------------------------------
// EOF
// ----------------------------------------------------------------------------
