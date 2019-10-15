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
#include "usb.h"
#include "redirector.h"

#ifdef _WIN32
// Simulator running under Windows
#include "fakeio.h"
#else
// Atmel target
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Target: ATmega32u4
#include <avr/common.h>
#include <stdio.h>

#include "i2c.h"
#include "hmc5843.h"
#include "ist8310.h"

#endif

#include "TinyGPS.h"
#include "lat_lon.h"
#include "uart.h"
#include "vboot.h"

static
uint8_t ctrl_write_PM(const void *addr, uint16_t len);

static void handle_outgoing_bytes(void);
static void handle_incoming_bytes(void);
static void handle_CONTROL(void);
void send_reserved_bytes();

#define set_bit(REG, BIT) REG |= _BV(BIT)
#define clear_bit(REG, BIT) REG &= ~_BV(BIT)
#define toggle_bit(REG, BIT) REG ^= _BV(BIT)
#define assign_bit(REG, BIT, VAL) do{if(VAL) set_bit(REG,BIT) else clear_bit(REG,BIT);}while(0)

// Endpoint 0 size
// NOTE: FTDI defines this as 8 bytes instead, but 64 is much easier to program as we don't have
// split up the bigger transfers.
#define EP0_SIZE 64

#define EP_select(N) do{UENUM = (N)&0x07;}while(0)
#define EP_read8() (UEDATX)
#define EP_read16_le() ({uint16_t L, H; L=UEDATX; H=UEDATX; (H<<8)|L;})
#define EP_write8(V) do{UEDATX = (V);}while(0)
#define EP_write16_le(V) do{UEDATX=(V)&0xff;UEDATX=((V)>>8)&0xff;}while(0)

// Outdata pending
static bool usb_out_data_pending = false;

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
// USB register/variable overview
// ----------------------------------------------------------------------------
enum USB_VAR { urInvalid=0,
    urMagic=1,
    urGpsLat=2,
    urGpsLon=3,    
    urGpsAge=4,
    urGpsValid=5,
    urGpsCourse=6,
    urRcK1=10,
    urRcK2,
    urRcK3,
    urRcK4,
    urMainSeqStep = 20,
    urMotorL = 30,
    urMotorR = 31,
};    

//int tune_ptr(0);
//char tune_buf[16];
int cmd_ptr = 0;

#define MAX_CMD_SIZE 15
char cmd_buf[MAX_CMD_SIZE+1];
char cmd_response[MAX_CMD_SIZE+1];

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
void Fake_UART_ISR(unsigned UDR1) {
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
	// Note: high speed timer must be disconnected from USB PLL!
	clear_bit(PLLFRQ,PLLTM1);
	clear_bit(PLLFRQ,PLLTM0);

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

	OCR3A =  2500; // MR in center

	set_bit(TIMSK3,TOIE3); // timer 3 overflow interrupt

	cli();

	// Based on 16 MHz crystal timing:
	// Timer freq: 250.00 [kHz]
	// 1,024 [ms] = 0x100 (-100%)
	// 1.536 [ms] = 0x180 (neutral)
	// 2,048 [ms] = 0x200 (+100%)	
	TC4H = 0x1;
	OCR4D = 0xc0;
	
	TC4H = 3;
	OCR4C = 0xff;
	sei();
	
	
#endif
}
// ----------------------------------------------------------------------------
void print_steering_msg()
{
/*
	unsigned int sp_d = steering.sp_used;
	unsigned int pv_d = steering.pv_used;
	unsigned int err_d = steering.pid_err;

	if (stm.Step()==msAutoModeCourse || stm.Step()==msAutoModeNormal) {
		b_printf(PSTR("sp=%d"), sp_d);
		if (steering.SUBST_SP!=0)
			b_printf(PSTR("*"));
	
		putchar(' ');

		b_printf(PSTR("pv=%d"), pv_d);
		if (steering.SUBST_PV!=0)
			b_printf(PSTR("*"));
	} else
		b_printf(PSTR("sp=-- pv=-- "));

	b_printf(PSTR(" err=%d: \r\n"), err_d);
*/
}
// ----------------------------------------------------------------------------
void print_compass_msg()
{
/*
	b_printf(PSTR("\x1b[1;1H"));
	
    b_printf(PSTR(" x=%04d  \r\n y=%04d  \r\n z=%04d  \r\n course=%04d   \r\n sp=%04d   \r\n"),
		mag->compass_raw.x.value, mag->compass_raw.y.value, mag->compass_raw.z.value,
		int(steering.compass_course),
		int(steering.bearing_sp));

    b_printf(PSTR("\r\nsmp#(good/bad)=%04d(%04d/%04d)        \r\n"), compass_smp, good_compass_smp, bad_compass_smp);
	
	cc.print_cal();
*/
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
/*
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
	*/
}
// ----------------------------------------------------------------------------
void tune_Config(double & dblParam, char c)
{
	/*
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
*/
}

// ----------------------------------------------------------------------------
void handle_parameterization(char c)
{
	/*
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
	*/
}

// ----------------------------------------------------------------------------
// Handles GPS input
// ----------------------------------------------------------------------------
void read_uart()
{
	// If the GPS module has something to say over the serial
	// link then attempt to decode a NMEA message...
	while (fifo_avail()) {
		char c = fifo_read();
		gps.encode(c);
	}
}

void read_user_input()
{
/*
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
*/
}

// ----------------------------------------------------------------------------
// Periodic message
// ----------------------------------------------------------------------------

void print_servo_msg(bool full)
{
/*
	// Tell whether application considers the remote control up or down
	b_printf(rc_okay ? PSTR("RC UP\r\n") : PSTR("RC DOWN\r\n"));
	
	// Print the values of the incoming and outgoing servo channels
	if (full) {
		int a1, b1;

		// Display incoming servo signals as received (2000 ... 4000, 0 = no signal)
		// Same for outgoing signals (to motors)
		a1 = joystick.to_perc(OCR1A);
		b1 = joystick.to_perc(OCR1B);

		b_printf(PSTR(" A=%05d B=%05d\r\n"),
    		a1, b1);
			
		// Show how many pulses the capture interrupts have seen
		b_printf(PSTR("Capture status: %d %d %d %d\r\n"),
			k1_alive,k2_alive,k3_alive,k4_alive);
	}
*/
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
}

// ----------------------------------------------------------------------------
// MAIN PROCESS
// ----------------------------------------------------------------------------

enum ustate{usDisconnected, usDone};
ustate us(usDisconnected);


void process()
{
	unsigned long delta(0);

	// Handle UART (GPS) input
	read_uart(); // also happens with this disabled
	// Handle user input (typing in commands etc.)
	read_user_input();

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

	switch (us) {
	case usDisconnected:
		if ((USBSTA & (1 << VBUS))) {
			printf_P(PSTR("Plugged in!\r\n"));
			// connected
			UDCON &= ~(1 << DETACH);
					
			//end of reset interrupts
			UDIEN |= (1<<EORSTE);//enable the end of reset interrupt
					
			us = usDone;
		}
		break;
				
	case usDone:
		// TODO / BUG: This condition never seems to be met, at least with my Arduino Leonardo board
		if (!((USBSTA & (1 << VBUS)))) {
			// we got disconnected from the pc/laptop
			printf_P(PSTR("Disconnected!\r\n"));
			us = usDisconnected;
		}
		break;
	}

	// Handle USB control messages
	EP_select(0);
	if ((UEINTX & (1 << RXSTPI))) {
		handle_CONTROL();
	}

	// Receive bytes from USB host (laptop/pc)
	handle_incoming_bytes();
			
	// Send bytes to USB host (laptop/pc)
	handle_outgoing_bytes();	
	
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
// USB
// ----------------------------------------------------------------------------

void oops(int a, char * v)
{	
	if (!a) {
		printf_P(PSTR("oops! %s"),v);
		while (1)
			;
	}
}

void put_hex(unsigned int i)
{	
	printf_P(PSTR("%04x"),i);
}


/* The control request currently being processed */

static usb_header head;

/* USB descriptors, stored in flash */
static const usb_std_device_desc PROGMEM devdesc = {
    sizeof(devdesc),
    usb_desc_device,
    0x0110, // 0x0110 for USB v1.1, 0x0200 for USB v2.0
    0x00, /* vendor specific / device class */
    0x00, /* vendor specific / device sub class */
    0x00, /* vendor specific / device protocol */
    EP0_SIZE, /* EP 0 size 64 bytes, real FTID reports 8 */
    0x0403, // Vendor ID (VID): Future Technology Devices International Limited
    0x6001, // Product ID (PID): FT232
    0x0400, // bcdDevice
	1, // iManufacturer
    2, // iProduct
    0, // iSerialNumber (has nothing to do with that alfanumeric FTDI serial number)
    1 // Number of configurations
};

struct tdevconf
{
	usb_std_config_desc conf;
	usb_std_iface_desc iface;
	usb_std_EP_desc ep1;
	usb_std_EP_desc ep2;
};

static const PROGMEM tdevconf devconf {
	
	{
        sizeof(usb_std_config_desc),
        usb_desc_config,
        sizeof(devconf),
        1,
        1,
		0, 
        0x80, // bus powered
        20/2  // 20 mA		
	},	
	{
        sizeof(usb_std_iface_desc),
        usb_desc_iface,
        0,
        0,
        2, // # of endpoints
        0xff, // vender specific
        0xff, // vender specific
        0xff, // vender specific
		0
	},
	// End point 1
	{
		7, // size
		usb_desc_EP,
		0x81, // address
		0x02, // bulk
		0x0040, // max packet size (64 bytes)
		0, // interval		
	},
	// End point 2
	{
		7, // size
		usb_desc_EP,
		0x02, // address
		0x02, // bulk
		0x0040, // max packet size (64 bytes)
		0, // interval
	}
};

static const usb_std_string_desc iLang PROGMEM = {
	4, //sizeof(usb_std_string_desc) + 2,
	usb_desc_string,
    L"\x0409"
};

/*
#define DESCSTR(STR) { \
	usb_desc_string, \
	STR \
}
*/

#define DESCSTR(STR) { \
	2 + sizeof(STR) - 2, \
	usb_desc_string, \
	STR \
}


// USB product name ("friendly name") that shows up when the host is quizzing the device
static const usb_std_string_desc iProd PROGMEM = DESCSTR(L"QuartelRCBB\0\0");

// FTDI style alphanumeric serial number
static const usb_std_string_desc iSerial PROGMEM = DESCSTR(L"FTP1W65N\0\0");

#undef DESCSTR

/* Handle the standard Get Descriptor request.
 * Return 1 on success
 */
static
uint8_t USB_get_desc(void)
{
    const void *addr;
    uint8_t len, idx = head.wValue;
    switch(head.wValue>>8)
    {
    case usb_desc_device:
        if(idx!=0) return 0;
        addr = &devdesc; len = sizeof(devdesc);
        break;
    case usb_desc_config:
        if(idx!=0) return 0;
        addr = &devconf; len = sizeof(devconf);
        break;
    case usb_desc_string:
        switch(idx)
        {
        case 0: addr = &iLang; break;
        case 1: addr = &iProd; break;
        case 2: addr = &iSerial; break;
        default: return 0;
        }
        /* the first byte of any descriptor is its length in bytes */
        len = pgm_read_byte(addr);
        break;
    default:
        return 0;
    }

    if(len>head.wLength)
        len = head.wLength;

    return !ctrl_write_PM(addr, len);
}

static void setupEP0(void);

static uint16_t userval; /* user register */

static inline void setupusb(void)
{
    /* disable USB interrupts and clear any active */
    UDIEN = 0;
    UDINT = 0;

    set_bit(UDCON, DETACH); /* redundant? */

    /* toggle USB reset */
    clear_bit(USBCON, USBE);
    set_bit(USBCON, USBE);

    /* No required.
     * Gives some time to start reprogramming
     * if previous program gets stuck right away
     */
    _delay_ms(1000);
    putchar('.');

    /* Unfreeze */
    clear_bit(USBCON, FRZCLK);

    /* setup PLL for 8 MHz system clock */
    PLLCSR = 0;
    set_bit(PLLCSR, PLLE);
    loop_until_bit_is_set(PLLCSR, PLOCK);
    putchar('.');

    setupEP0(); /* configure control EP */
    putchar('.');

#ifdef HANDLE_SUSPEND
    set_bit(UDIEN, SUSPE);
#endif
    set_bit(UDIEN, EORSTE);

    /* allow host to un-stick us.
     * Warning: Don't use w/ DETACH on CPU start
     *          or a reset loop will result
     */
    //set_bit(UDCON, RSTCPU);
    clear_bit(UDCON, DETACH);
}

/* Setup the control endpoint. (may be called from ISR) */
static void setupEP0(void)
{
    /* EPs assumed to be configured in increasing order */

    EP_select(0);

    /* un-configure EP 0 */
    clear_bit(UECONX, EPEN);
    clear_bit(UECFG1X, ALLOC);

    /* configure EP 0 */
    set_bit(UECONX, EPEN);
    UECFG0X = 0; /* CONTROL */
    UECFG1X = 0b00110010; // EPSIZE=64B, 1 bank, ALLOC 
#if EP0_SIZE!=64
#  error EP0 size mismatch
#endif

    if(bit_is_clear(UESTA0X, CFGOK)) {
        putchar('!');
        while(1) {} /* oops */
    }
}

static void setup_other_ep()
{
	// The FTDI has two endpoints for serial data, they are:
	//
	// Endpoint 1 (IN):
	//   bEndpointAddress:     0x81
	//   Transfer Type:        Bulk
	//   wMaxPacketSize:     0x0040 (64)
	//   bInterval:            0x00
	//
	// Endpoint 2 (OUT):
	//   bEndpointAddress:     0x02
	//   Transfer Type:        Bulk
	//   wMaxPacketSize:     0x0040 (64)
	//   bInterval:            0x00

    EP_select(1);

    // un-configure EP 1
    clear_bit(UECONX, EPEN);
    clear_bit(UECFG1X, ALLOC);

    // configure EP 1
    set_bit(UECONX, EPEN);
    UECFG0X = 0x81; // BULK, IN
    UECFG1X = 0b00110010; // EPSIZE=64B, 1 bank, ALLOC

	if(bit_is_clear(UESTA0X, CFGOK)) {
		putchar('1!');
		while(1) {} /* oops */
	}

    EP_select(2);

    // un-configure EP 2
    clear_bit(UECONX, EPEN);
    clear_bit(UECFG1X, ALLOC);

    // configure EP 2
    set_bit(UECONX, EPEN);
    UECFG0X = 0x80; /* BULK, OUT */
    UECFG1X = 0b00110010; // EPSIZE=64B, 1 bank, ALLOC

    if(bit_is_clear(UESTA0X, CFGOK)) {
	    putchar('1!');
	    while(1) {} /* oops */
    }

    EP_select(0);	
	
}

ISR(USB_GEN_vect, ISR_BLOCK)
{
    uint8_t status = UDINT, ack = 0;
    putchar('I');
#ifdef HANDLE_SUSPEND
    if(bit_is_set(status, SUSPI))
    {
        ack |= _BV(SUSPI);
        /* USB Suspend */

        /* prepare for wakeup */
        clear_bit(UDIEN, SUSPE);
        set_bit(UDIEN, WAKEUPE);

        set_bit(USBCON, FRZCLK); /* freeze */
    }
    if(bit_is_set(status, WAKEUPI))
    {
        ack |= _BV(WAKEUPI);
        /* USB wakeup */
        clear_bit(USBCON, FRZCLK); /* freeze */

        clear_bit(UDIEN, WAKEUPE);
        set_bit(UDIEN, SUSPE);
    }
#endif
    if(bit_is_set(status, EORSTI))
    {
        ack |= _BV(EORSTI);
        /* coming out of USB reset */

#ifdef HANDLE_SUSPEND
        clear_bit(UDIEN, SUSPE);
        set_bit(UDIEN, WAKEUPE);
#endif

        putchar('E');
        setupEP0();
    }
    /* ack. all active interrupts (write 0)
     * (write 1 has no effect)
     */
    UDINT = ~ack;
}

/* function to write bytes of program memory to a bulk endpoint 
BUG/LIMITATION: this function does not allow writing big chunks, because
the bulk endpoints in this program have a max size of 64 bytes */
static
void bulk_write_PM(const void *addr, uint16_t len)
{
    while(len--) {
	    uint8_t val = pgm_read_byte(addr);
		UEDATX = val;
		addr++;
    }	
}


/* write value from flash to EP0 */
static
uint8_t ctrl_write_PM(const void *addr, uint16_t len)
{

    while(len) {
        uint8_t ntx = EP0_SIZE,
                bsize = UEBCLX,
                epintreg = UEINTX;

        oops(ntx>=bsize, "EP"); /* EP0_SIZE is wrong */

        ntx -= bsize;
        if(ntx>len)
            ntx = len;

        if(bit_is_set(epintreg, RXSTPI))
            return 1; /* another SETUP has started, abort this one */
        if(bit_is_set(epintreg, RXOUTI))
            break; /* stop early? (len computed improperly?) */

        /* Retry until can send */
        if(bit_is_clear(epintreg, TXINI))
            continue;
        oops(ntx>0, "Ep"); /* EP0_SIZE is wrong (or logic error?) */

        len -= ntx;

        while(ntx) {
            uint8_t val = pgm_read_byte(addr);
            EP_write8(val);
            addr++;
            ntx--;
        }

        clear_bit(UEINTX, TXINI);
    }
    return 0;
}

/* Handle standard Set Address request */
static
void USB_set_address(void)
{
    UDADDR = head.wValue&0x7f;

    clear_bit(UEINTX, TXINI); /* send 0 length reply */
    loop_until_bit_is_set(UEINTX, TXINI); /* wait until sent */

    UDADDR = _BV(ADDEN) | (head.wValue&0x7f);

    clear_bit(UEINTX, TXINI); /* magic packet? */
}

static
uint8_t USB_config;

static
void USB_set_config(void)
{
	USB_config = head.wValue;
	
	setup_other_ep();
}

// Called when we encounter an 'alien' USB request/message so we can work out what 
// is needed to support it
static void dump_unsupported_request(void)
{
	putchar('?');
	put_hex(head.bmReqType);
	put_hex(head.bReq);
	put_hex(head.wLength>>8);
	put_hex(head.wLength);	
}

// Handles CONTROL reads (Atmel to pc)
static void usb_control_in(void)
{	
	// Flag that indicates whether the request was supported and should be ack'ed.
	// If at the end of the function it is false, then a the endpoint is STALLed
    uint8_t ok = 0; 

    switch(head.bReq)
    {
    case usb_req_set_feature:
    case usb_req_clear_feature:
        /* No features to handle.
         * We ignore Remote wakeup,
         * and EP0 will never be Halted
         */
        ok = 1;
        break;
    case usb_req_get_status:
        switch(head.bmReqType) {
        case USB_REQ_TYPE_IN:
        case USB_REQ_TYPE_IN | USB_REQ_TYPE_INTERFACE:
        case USB_REQ_TYPE_IN | USB_REQ_TYPE_ENDPOINT:
            // always status 0
            loop_until_bit_is_set(UEINTX, TXINI);
            EP_write16_le(0);
            clear_bit(UEINTX, TXINI);
            ok = 1;
        }
        break;
    case usb_req_set_address:
		// This is an 'out' command, so should be handled by 'usb_control_out' instead
        break;
    case usb_req_get_desc:
        if(head.bmReqType==0x80) {
            ok = USB_get_desc();
        }
        break;
    case usb_req_set_config:
        if(head.bmReqType==0) {
            USB_config = head.wValue;
            ok = 1;
        }
        break;
    case usb_req_get_config:
        if(head.bmReqType==USB_REQ_TYPE_IN) {
            loop_until_bit_is_set(UEINTX, TXINI);
            EP_write8(USB_config);
            clear_bit(UEINTX, TXINI);
            ok = 1;
        }
        break;
    case usb_req_set_iface:
		break;
    case usb_req_get_iface:
		break;
    case usb_req_set_desc:
		break;
    case usb_req_synch_frame:
        break;

    default:
		if ((head.bmReqType & USB_REQ_TYPE_VENDOR)==0) {
			dump_unsupported_request();
		}
    }

	// Vendor specific	
	if (head.bmReqType == (USB_REQ_TYPE_IN|USB_REQ_TYPE_VENDOR)) {
		switch (head.bReq) {
		case FTDI_SIO_READ_EEPROM:
			loop_until_bit_is_set(UEINTX, TXINI);
			EP_write8(0xff);
			EP_write8(0xff);
			clear_bit(UEINTX, TXINI);
			ok=1;
		
			break;

		case FTDI_SIO_GET_LATENCY_TIMER:
			// 16 ms is the default value
			loop_until_bit_is_set(UEINTX, TXINI);
			EP_write8(0x10); // 16 [ms] is the default value
			clear_bit(UEINTX, TXINI);
			ok=1;
			break;
		case FTDI_SIO_GET_MODEM_STATUS:
			// 16 ms is the default value
			loop_until_bit_is_set(UEINTX, TXINI);
			EP_write8(0x00); // 16 [ms] is the default value
			clear_bit(UEINTX, TXINI);
			ok=1;
			break;
		default:
			dump_unsupported_request();			
		};	
		
	}
	
    if(ok) {
        if(head.bmReqType&ReqType_DirD2H) {
            /* Control read.
             * Wait for, and complete, status
             */
            uint8_t sts;
            while(!((sts=UEINTX)&(_BV(RXSTPI)|_BV(RXOUTI)))) {}
            //loop_until_bit_is_set(UEINTX, RXOUTI);
            ok = (sts & _BV(RXOUTI));
            if(!ok) {
                set_bit(UECONX, STALLRQ);
                putchar('S');
            } else {
                clear_bit(UEINTX, RXOUTI);
                clear_bit(UEINTX, TXINI);
            }
        } else {
            /* Control write.
             * indicate completion
             */
            clear_bit(UEINTX, TXINI);
        }
        putchar('C');

    } else {
        /* fail un-handled SETUP */
        set_bit(UECONX, STALLRQ);
        putchar('F');
    }
}

// Handles CONTROL writes (pc to Atmel)
static void usb_control_out(void)
{
	uint8_t ok = 0;

    switch(head.bReq)
    {
    case usb_req_set_feature:
    case usb_req_clear_feature:
        /* No features to handle.
         * We ignore Remote wakeup,
         * and EP0 will never be Halted
         */
        ok = 1;
        break;
    case usb_req_get_status:
		// This is an 'in' command, so should be handled by `usb_control_in` instead
        break;
    case usb_req_set_address:
        if(head.bmReqType==USB_REQ_TYPE_OUT) {
			// Host sets USB address
            putchar('A');
            USB_set_address();
			
            return;
        }
        break;
    case usb_req_get_desc:
		// This is an 'in' command, so should be handled by 'usb_control_in' instead
        break;
    case usb_req_set_config:
        if(head.bmReqType==0) {
			putchar('S');
			USB_set_config();
			putchar('s');			
            ok = 1;					
        }
        break;
    case usb_req_get_config:
        break;
    case usb_req_set_iface:
		break;
    case usb_req_get_iface:
		break;
    case usb_req_set_desc:
		break;
    case usb_req_synch_frame:
        break;

    default:
		if ((head.bmReqType & USB_REQ_TYPE_VENDOR)==0) {
			dump_unsupported_request();
		}
    }
	
	// Vendor specific
	if (head.bmReqType == (USB_REQ_TYPE_OUT|USB_REQ_TYPE_VENDOR)) {
		switch (head.bReq) {
		case FTDI_SIO_RESET:
			ok=1;
			break;			
		case FTDI_SIO_MODEM_CTRL:
		case FTDI_SIO_SET_BAUD_RATE:
		case FTDI_SIO_SET_DATA:
		case FTDI_SIO_SET_FLOW_CTRL:
		case FTDI_SIO_SET_LATENCY_TIMER:
			ok=1;
			break;
		default:
			dump_unsupported_request();
		};		
	}

    if(ok) {
        if(head.bmReqType&ReqType_DirD2H) {
            /* Control read.
             * Wait for, and complete, status
             */
            uint8_t sts;
            while(!((sts=UEINTX)&(_BV(RXSTPI)|_BV(RXOUTI)))) {}
            //loop_until_bit_is_set(UEINTX, RXOUTI);
            ok = (sts & _BV(RXOUTI));
            if(!ok) {
                set_bit(UECONX, STALLRQ);
                putchar('S');
            } else {
                clear_bit(UEINTX, RXOUTI);
                clear_bit(UEINTX, TXINI);
            }
        } else {
            /* Control write.
             * indicate completion
             */
            clear_bit(UEINTX, TXINI);
        }
        putchar('C');

    } else {
        /* fail un-handled SETUP */
        set_bit(UECONX, STALLRQ);
        putchar('F');
    }
}

// Whether we need to echo a character to the pc
static bool do_send_char = false;
// Last usb "serial" character received as sent by pc/laptop
static char usb_char = '\0';

// Every FTDI serial read starts with two reserved bytes
void send_reserved_bytes()
{
	// The original device reserves the first two bytes for the modem and line status
	UEDATX = 0x80; // Modem status.
	UEDATX = 0x00; // Line status.	
}

// Possibly send bytes to the pc/laptop
void handle_outgoing_bytes(void)
{
	// Turn attention to the bulk IN endpoint, because that's were bytes
	// destined for the pc/laptop should go to first
	EP_select(1);
	
	if (do_send_char) {
		do_send_char=false;

		if (bit_is_set(UEINTX,TXINI)) {
			clear_bit(UEINTX,TXINI);			
			send_reserved_bytes();
			// Send one char
			UEDATX = usb_char;			
			clear_bit(UEINTX,FIFOCON);		
		}
	}
	
	if (usb_out_data_pending) {
		clear_bit(UEINTX,FIFOCON);		
		usb_out_data_pending=false;
	}
	
}

static unsigned long testreg = 0;

/*
Variables
RO 0001 '42'
*/

unsigned long read_var(int reg)
{
    unsigned long data(0);
    
    switch (reg) {
    case urMagic:
        data=42;
        break;
    case urGpsLat:
        data = *reinterpret_cast<long*>(&waypoints.gp_current.lat);
        break;
    case urGpsLon: 
        data = *reinterpret_cast<long*>(&waypoints.gp_current.lon);
        break;
    case urGpsAge:
        data = gps_fix_age;
        break;
    case urGpsValid:
        data = gps_valid ? 1 : 0;
        break;
    case urGpsCourse:
        data = gps_course;
        break;
    case urRcK1:
        data=k1_pulse_duration;
        break;
    case urRcK2:
        data=k2_pulse_duration;
        break;
    case urRcK3:
        data=k3_pulse_duration;
        break;
    case urRcK4:
        data=k4_pulse_duration;
        break;        
    case urMainSeqStep:
        data=stm.Step();
        break;
    case urMotorL:
        cli();
        data = OCR4D;
        data |= ((unsigned int)TC4H) << 8;
        data &= 0x03ff;
        sei();
        break;       
    case urMotorR:
        data = OCR3A;
        break;

    default:
        data=0;
    }

    return data;
}

void write_var(int reg, unsigned long d)
{
    switch (reg) {
    case 3:
        testreg = d;
        break;
    }
}

void handle_command(void)
{
    int fields=0;
    unsigned int addr = 0;
    unsigned long data = 0;

	cmd_response[0]=0;
	
	char cc = cmd_buf[0];
	switch (cc) {
    case 'R':        
        putchar('R');
        fields = sscanf(&cmd_buf[1],"%04x",&addr);
        if (fields == 1) {
            data = read_var(addr);
		    sprintf(cmd_response,"OK %08lx\r\n",data);
        }
        else
		    sprintf(cmd_response,"ERR R\r\n");
        break;
    case 'W':
        putchar('W');
        fields = sscanf(&cmd_buf[1],"%04x,%08lx",&addr,&data);
        if (fields==2) {
            write_var(addr,data);
		    sprintf(cmd_response,"OK\r\n");
        } else
		    sprintf(cmd_response,"ERR W\r\n");
        break;
	}
	
	if (cmd_response[0]) {
        putchar('R');

		EP_select(1);

		if (!usb_out_data_pending)
			send_reserved_bytes();
				
        char * p = cmd_response;	
		while (*p) {
		    UEDATX = *p;
            ++p;
		}
		
		EP_select(0);
        cmd_response[0]=0;
		usb_out_data_pending=true;
	}
}

// Possibly receive bytes from the pc/laptop
void handle_incoming_bytes(void)
{
	// Turn attention to the bulk OUT endpoint, because that's were bytes
	// sent from the pc/laptop end up in.
	EP_select(2);
	
	if (bit_is_set(UEINTX, RXOUTI)) {
		// Acknowledge receive int
		clear_bit(UEINTX, RXOUTI);
			
		// See how much bytes we got
		unsigned int N = ((unsigned int)UEBCHX << 8) | UEBCLX;
		
		if (N>0) {
			// Read chars sent by the pc/laptop
			for (int i(0);i<N;i++) {
				usb_char = UEDATX;

				if (cmd_ptr > MAX_CMD_SIZE) {
					cmd_ptr = 0;						
				}

				cmd_buf[cmd_ptr++] = usb_char;				
				
				if (usb_char == '\n' || usb_char == '\r') {
					handle_command();
                	EP_select(2);

					cmd_ptr=0;
					cmd_buf[0]=0;
				}
				
				
			}
		}
		
		clear_bit(UEINTX,FIFOCON);
	}	
}

// Called when the pc/laptop is quizzing/configuring the Atmel
static
void handle_CONTROL(void)
{
    uint8_t ok = 0;
    /* SETUP message */
    head.bmReqType = EP_read8();
    head.bReq = EP_read8();
    head.wValue = EP_read16_le();
    head.wIndex = EP_read16_le();
    head.wLength = EP_read16_le();

    /* ack. first stage of CONTROL.
     * Clears buffer for IN/OUT data
     */
    clear_bit(UEINTX, RXSTPI);

    /* despite what the figure in
     * 21.12.2 (Control Read) would suggest,
     * SW should not clear TXINI here
     * as doing so will send a zero length
     * response.
     */
	
	if (head.bmReqType & USB_REQ_TYPE_IN)
		usb_control_in();
	else
		usb_control_out();	
}

ISR(USB_COM_vect)
{
	// This USB interrupt isn't used.
}

// Performs initial USB and PLL configuration
void setup_usb()
{
	// Start with interrupts disabled
	cli();

	// disable USB general interrupts
	USBCON &= 0b11111110;
	// disable all USB device interrupts
	UDIEN &= 0b10000010;
	// disable USB endpoint interrupts
	UEIENX &= 0b00100000;

	// Re-enable interrupts
	sei();

	// Enable USB pad regulator
	UHWCON |= (1<<UVREGE);

	// Configure PLL (setup 48 MHz USB clock)
	PLLCSR = 0;
	// Set PINDIV because we are using 16 MHz crystal
	PLLCSR |= (1<<PINDIV);
	// Configure 96MHz PLL output (is then divided by 2 to get 48 MHz USB clock)
	PLLFRQ = (1<<PDIV3) | (1<<PDIV1) | (1<<PLLUSB) | (1 << PLLTM0);
	// Enable PLL
	PLLCSR |= (1<<PLLE);
	
	// Wait for PLL lock
	while (!(PLLCSR & (1<<PLOCK)))
		;
			
	// Enable USB
	USBCON |= (1<<USBE)|(1<<OTGPADE);
	// Clear freeze clock bit
	USBCON &= ~(1<<FRZCLK);
	
	// configure USB interface (speed, endpoints, etc.)
	UDCON &= ~(1 << LSM);     // full speed 12 Mbit/s
	
	// disable rest of endpoints
	for (uint8_t i = 1; i <= 6; i++) {
		UENUM = (UENUM & 0xF8) | i;   // select endpoint i
		UECONX &= ~(1 << EPEN);
	}	
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
	// Enable interrupts
	USBCON=0;

	/* set pin 5 of PORTB for output*/
	DDRB |= _BV(DDB5);

	// Setup serial (UART) input
	USART_Init();  // Initialize USART
#endif
	sei();         // enable all interrupts

	redirect_std_out();

	b_printf(PSTR("Boot!\r\n"));
	
	setup_usb();

	cc.reset_compass_calibration();

	cc.load_calibration();
	steering.load_calibration();
    waypoints.load_waypoints();

	// Setup other peripherals
	setup_capture_inputs();
	setup_pwm();

	clear_stats();

	// Initialize I2C-bus I/O (and button input on RC0)
#ifndef _WIN32
	DDRD |= 0b00000011; // SDA/SCL pins as output (see Atmel manual)
	PORTD |= 0b00000011; //pull-ups on the I2C bus
	
	DDRF = 0b00000000; // PORTF7 as input
	PORTF = 0b10000000; // pull-up on cal. button (bit 7)

	i2cInit();
	i2cSetBitrate(30);
	
	// Search for magnetometer
	int retries(250);
	const char * msgMagStatus = PSTR("(not found)\r\n");

	do {
		b_printf(PSTR("Probing for magnetometer..."));
		delay_ms(100);
	
		if (mag_ist8310.detect()) {
			msgMagStatus = PSTR("Found IST8310.\r\n");
			mag = &mag_ist8310;
			break;
		} else if (mag_hmc5843.detect()) {
			msgMagStatus = PSTR("Found HMC5843.\r\n");
			mag = &mag_hmc5843;
			break;
		}
	} while (--retries > 0);	
	b_printf(msgMagStatus);

	
	if (mag == 0) {
		b_printf(PSTR("Halting.\r\n"));
		while (1) ;
	}

	for (int i(0); i<3; i++) {
		delay_ms(25);
		mag->init();
	}
#endif

    b_printf(PSTR("main_loop\r\n"));

#ifndef _WIN32
	main_loop();
#endif

	return 0;
}
// ----------------------------------------------------------------------------
// EOF
// ----------------------------------------------------------------------------
