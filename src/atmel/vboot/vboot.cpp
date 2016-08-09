#include "settings.h"

#ifdef _WIN32
// Simulator running under Windows
#include "fakeio.h"

#else

#define b_printf printf

// Atmel target
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <avr/eeprom.h>

// Target: ATmega328
//#include <avr/iom328.h>
#include <avr/common.h>
#include <stdio.h>

#include "i2c.h"
#include "m8n.h"
#include "hmc5843.h"

#endif

#include "TinyGPS.h"
#include "lat_lon.h"
#include "uart.h"
#include "vboot.h"

// ----------------------------------------------------------------------------
// Besturing (motoren)
// ----------------------------------------------------------------------------

// Boot van achteren gezien:
// joystick in vooruit
//    CCW >0    CCW >0
// joystick in achteruit
//    CW  <0    CW  <0
// joystick links:
//    CW  <0    CCW >0
// joystick rechts:
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
	p: (PID 'P'-action configuration)
	i: (PID 'I'-action configuration)
	(auto steer test screen)
	a: go straight to auto steer mode
	x: don't stop steering (even after arriving)
	d: substitute PV compass heading (enter 0 for no substution)
	o: substitute SP heading (enter 0 for no substitution)

	"0123456789.,-+" (valid in PID configuration modi)
*/

// ----------------------------------------------------------------------------
// Auto steer PID-tune parameters
// ----------------------------------------------------------------------------
double TUNE_P(0.01); // P-action
double TUNE_I(0.0000001); // I-action
double TUNE_D(0.0);  // D-action

double SUBST_SP(0.0);
double SUBST_PV(0.0);

float pv_used(0),sp_used(0);

int tune_ptr(0);
char tune_buf[16];

int CfgMode(0);

// ----------------------------------------------------------------------------
// VARIABLES
// ----------------------------------------------------------------------------

// State machine current step, next step
TMainState main_state,next_state;
// Time elapsed in current state machine step [ms]
unsigned long state_time = 0;

// Tests
bool subst_pv = false;
bool subst_sp = false;
bool dont_stop_steering = false;
bool straight_to_auto = false;

int joy_pulses = 0; // # times the goto/store joystick has been pushed up/down

TMessageMode msg_mode;

// Memorized GPS positions
CLatLon gp_mem_1; // memorized GPS position 1 (usually 'home')
CLatLon gp_mem_2; // memorized GPS position 2
CLatLon gp_mem_3; // memorized GPS position 3
// Current / destination GPS positions
CLatLon gp_current; // current GPS position (may be stale or invalid!)
CLatLon gp_start; // GPS position when auto steering was switched on
CLatLon gp_finish; // auto steering target GPS position

// GPS input related
unsigned long gps_fix_age = TinyGPS::GPS_INVALID_AGE;
TinyGPS gps;
bool gps_valid = false;
bool gps_valid_prev = false;
float gps_cmg = 0; // gps course made good
long gps_lat, gps_lon, gps_course;
int restrict_dir = 0;

// Compass input
TCompassTriple compass_raw;
int16_t compass_smp;

float compass_course_no_offset;
float compass_north_offset;
float compass_course;

bool calibration_mode;

comp_extreme compass_min_x;
comp_extreme compass_max_x;
comp_extreme compass_min_y;
comp_extreme compass_max_y;
comp_extreme compass_min_z;
comp_extreme compass_max_z;


// Auto steering related
float bearing_sp = 0; // calculated initial bearing (from Haversine formulas)
float distance_m = 0; // distance to finish from current gps pos
bool arrived(false); // TRUE when arriving at the waypoint

// LED control related
int blink_times(0);
bool slow_blink(false);
bool slow_blink_prev(false);
bool fast_blink(false);
bool led_signal(false);

// PID controller vars
float p_add(0);
float i_add(0);
float d_add(0);
float pid_err(0);


// Global [ms] timer
volatile unsigned long global_ms_timer = 0;

// Timing for periodic processes
unsigned long t_100ms_start_ms(0);
unsigned long t_500ms_start_ms(0);

bool shown_stats(false);

// ----------------------------------------------------------------------------
// PWM/JOYSTICK/MOTOR vars
// ----------------------------------------------------------------------------
// ML - Motor Left (in)
volatile unsigned int pd6_rising;
volatile unsigned int pd6_pulse_duration;
// MR - Motor Right (in)
volatile unsigned int pd5_rising;
volatile unsigned int pd5_pulse_duration;
// Pos (RX channel 3)
volatile unsigned int pd3_rising;
volatile unsigned int pd3_pulse_duration;
// Man/auto (RX channel 4)
volatile unsigned int pb3_rising;
volatile unsigned int pb3_pulse_duration;
// Old PORTD values for PORTD servo signal detection
volatile unsigned char old_pind = 0;
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
void copy_gps_pin(void)
{
#ifndef _WIN32
	if (PINB & _BV(PINB0))
	{
		PORTD |= _BV(PORTD7);
	} else
	{
		PORTD &= ~_BV(PORTD7);
	}
#endif
}
// ----------------------------------------------------------------------------
ISR(TIMER1_OVF_vect)
{
	// Timer 1 overflow
	global_ms_timer += 20;	// timer was set to overflow each 20 [ms]
}
// ----------------------------------------------------------------------------
#ifndef _WIN32
ISR(PCINT0_vect)
{
	// For PD0 (UART RX) equal to current value of soft GPS pin (PB0)
	copy_gps_pin();


	unsigned int tmr_reg;
	tmr_reg = TCNT1;

	// PB3 servo pulse measurements
	if ((PINB & _BV(PINB3)) ^ (old_pinb & _BV(PINB3))) {
		if (PINB & _BV(PINB3))
			pb3_rising = tmr_reg;
		else {
			pb3_pulse_duration = tmr_reg - pb3_rising;
			if (pb3_pulse_duration > 10000)
				pb3_pulse_duration -= 25536;
		}
	}

	old_pinb = PINB;
}
// ----------------------------------------------------------------------------
ISR(PCINT1_vect)
{

}
// ----------------------------------------------------------------------------
ISR(PCINT2_vect)
{
	unsigned int tmr_reg;
	tmr_reg = TCNT1;

	// PD6 servo pulse measurements
	if ((PIND & _BV(PIND6)) ^ (old_pind & _BV(PIND6))) {
		if (PIND & _BV(PIND6))
			pd6_rising = tmr_reg;
		else {
			pd6_pulse_duration = tmr_reg - pd6_rising;
			if (pd6_pulse_duration > 10000)
				pd6_pulse_duration -= 25536;
		}
	}

	// PD5 servo pulse measurements
	if ((PIND & _BV(PIND5)) ^ (old_pind & _BV(PIND5))) {
		if (PIND & _BV(PIND5))
			pd5_rising = tmr_reg;
		else {
			pd5_pulse_duration = tmr_reg - pd5_rising;
			if (pd5_pulse_duration > 10000)
				pd5_pulse_duration -= 25536;
		}
	}

	// PD3 servo pulse measurements
	if ((PIND & _BV(PIND3)) ^ (old_pind & _BV(PIND3))) {
		if (PIND & _BV(PIND3))
			pd3_rising = tmr_reg;
		else {
			pd3_pulse_duration = tmr_reg - pd3_rising;
			if (pd3_pulse_duration > 10000)
				pd3_pulse_duration -= 25536;
		}
	}

	old_pind = PIND;
}
#endif
// ----------------------------------------------------------------------------
volatile unsigned char value;
/* This variable is volatile so both main and RX interrupt can use it.
It could also be a uint8_t type */

/* Interrupt Service Routine for Receive Complete
NOTE: vector name changes with different AVRs see AVRStudio -
Help - AVR-Libc reference - Library Reference - <avr/interrupt.h>: Interrupts
for vector names other than USART_RXC_vect for ATmega32 */

volatile unsigned char head = 0,tail = 0;
#define FIFO_SIZE 32
#define FIFO_MASK (FIFO_SIZE-1)
volatile char uart_fifo[FIFO_SIZE];
// ----------------------------------------------------------------------------
bool rb_avail()
{
	return (head != tail);
}
// ----------------------------------------------------------------------------
char rb_read()
{
	char data(0);
	if (head != tail) {
		data = uart_fifo[tail];
		unsigned char new_tail(tail);
		new_tail++;
		new_tail &= FIFO_MASK;
		tail = new_tail;
	}
	return data;
}
// ----------------------------------------------------------------------------

#ifndef _WIN32
ISR(USART_RX_vect) {
#else
void Fake_UART_ISR(unsigned UDR0) {
#endif

	// Read UART register (the received byte)
	// into `value`
	value = UDR0;

// // also happens with this disabled
#if 1
	// Store byte in FIFO (if FIFO isn't full)
	unsigned char new_head;
	new_head = head + 1;
	new_head &= FIFO_MASK;

	if (new_head != tail) {
		uart_fifo[head] = value;
		head = new_head;
	}
#endif
}
// ----------------------------------------------------------------------------
/*
void setup_gps_input()
{
#ifndef _WIN32
	// Configure PB0 as input
	DDRB &= ~_BV(DDB0);
	PORTB &= ~_BV(PORTB0);

	// Setup PD7 as output
	DDRD |= _BV(DDD7);
	PORTD  &= ~_BV(PORTD7); // make zero

	// Enable on-pin-change for pin
	PCMSK0 |= _BV(PCINT0);

	// Configure interrupt on logical state state on PB0 (so PCIE0)
	PCICR |= _BV(PCIE0);
#endif
}
*/
// ----------------------------------------------------------------------------
void setup_capture_inputs()
{
#ifndef _WIN32
	// Configure PD6 as input
	DDRD &= ~_BV(DDD6);
	PORTD &= ~_BV(PORTD6);
	// Configure PD5 as input
	DDRD &= ~_BV(DDD5);
	PORTD &= ~_BV(PORTD5);
	// Configure PD3 as input
	DDRD &= ~_BV(DDD3);
	PORTD &= ~_BV(PORTD3);
	// Configure PB3 as input
	DDRB &= ~_BV(DDB3);
	PORTB &= ~_BV(PORTB3);

	// Enable on-pin-change for pins
	PCMSK2 |= _BV(PCINT22); // PD6
	PCMSK2 |= _BV(PCINT21); // PD5
	PCMSK2 |= _BV(PCINT19); // PD3
	PCMSK0 |= _BV(PCINT3); // PB3

	// Configure interrupt on logical state state on PB3 (so PCIE0)
	PCICR |= _BV(PCIE0);

	// Configure interrupt on logical state state on PD3/PD5/PD6 (so PCIE2)
	PCICR |= _BV(PCIE2);

	// Configure timer 1 clock source
	// Set to use a clock source of clkio / 8.
	// So 20 [ms] servo period will be 40000 timer ticks.
	TCCR1B &= ~_BV(CS12);
	TCCR1B |= _BV(CS11);
	TCCR1B &= ~_BV(CS10);

	// Enable timer 1 overflow interrupt
	TIMSK1 |= _BV(TOIE1);
#endif
}
// ----------------------------------------------------------------------------
void setup_pwm()
{
#ifndef _WIN32
	// Configure PB1 & PB2 as outputs
	DDRB |= _BV(DDB1);
	DDRB |= _BV(DDB2);

	// Set compare output mode to non-inverting (PWM) mode
	TCCR1A |= _BV(COM1A1);
	TCCR1A &= ~_BV(COM1A0);
	TCCR1A |= _BV(COM1B1);
	TCCR1A &= ~_BV(COM1B0);

	// Set mode 14 (Fast PWM, TOP=ICR1)
	// WGM13 WGM12 WGM11 WGM10
	// 1	 1	   1	 0

	TCCR1B |= _BV(WGM13);
	TCCR1B |= _BV(WGM12);
	TCCR1A |= _BV(WGM11);
	TCCR1A &= ~(_BV(WGM10));

	OCR1A = JOY_CENTER;
	OCR1B = JOY_CENTER;

	ICR1 = 40000; // gives a 20 [ms] period
#endif
}
// ----------------------------------------------------------------------------
int joy_to_perc(unsigned int raw)
{
	// 2000 (-100%) ... 3000 (0%) .... 4000 (100%)

	int perc(raw);
	perc = (perc - 3000)/10;

	return perc;
}
// ----------------------------------------------------------------------------
void print_steering_msg()
{
	if (gps_fix_age == TinyGPS::GPS_INVALID_AGE)
	b_printf("GPS-NO_FIX ");
	else if (gps_fix_age > GPS_STALE_TIME)
	b_printf("GPS-STALE ");
	else {
		b_printf("GPS-OK ");
	}

	int a1 = joy_to_perc(OCR1A);
	int b1 = joy_to_perc(OCR1B);
	unsigned int sp_d = sp_used;
	unsigned int pv_d = pv_used;
	unsigned int err_d = pid_err;

	if (main_state==msAutoModeCourse)
		b_printf("[autoC] ");
	else if (main_state==msAutoModeNormal)
		b_printf("[autoN] ");
	else
		b_printf("[man] ");

	b_printf("age=%ld ", gps_fix_age);

	if (main_state==msAutoModeCourse || main_state==msAutoModeNormal) {
		b_printf("sp=%d", sp_d);
		if (SUBST_SP!=0)
			b_printf("*");
	
		b_printf(" ");
	
		b_printf("pv=%d", pv_d);
		if (SUBST_PV!=0)
			b_printf("*");
	} else
		b_printf("sp=-- pv=-- ");

	b_printf(" err=%d: A=%d B=%d\r\n", err_d, a1,b1);
}
// ----------------------------------------------------------------------------
void print_compass_msg()
{
		b_printf("x=%04d, y=%04d, z=%04d smp=%04d course=%04d sp=%04d\r\n",
		compass_raw.x, compass_raw.y, compass_raw.z, compass_smp,
		int(compass_course),
		int(bearing_sp));
		//b_printf(" xr=%04d ... %04d\r\n", compass_min_x.fin, compass_max_x.fin);
		//b_printf(" zr=%04d ... %04d\r\n", compass_min_z.fin, compass_max_z.fin);
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
		b_printf("GPS-NO_FIX\r\n");
	else if (gps_fix_age > GPS_STALE_TIME)
		b_printf("GPS-STALE\r\n");
	else {
			b_printf("GPS-OK age=%ld. lat=%ld lon=%ld course=%ld\r\n",
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
	//
}
// ----------------------------------------------------------------------------
float clip_motor(float mtr)
{
	if (mtr>1.0)
		return 1.0;
	else if (mtr<-1.0)
		return -1.0;
	else
		return mtr;
}
// ----------------------------------------------------------------------------
float simple_pid(float pv, float sp,
	bool enable_p, float Kp,
	bool enable_i, float Ki,
	bool enable_d, float Kd)
{
	float cv(0.0);

	bool bBigDiff = fabs(sp-pv) > 180;

	if (sp > 180 && bBigDiff)
		sp -= 360;
	if (pv > 180 && bBigDiff)
		pv -= 360;

	pid_err = sp - pv;

	p_add = Kp * pid_err;
	i_add += Ki * pid_err;
	d_add = 0.0;

	cv = 0;
	if (enable_p)
		cv += p_add;
	if (enable_i)
		cv += i_add;
	if (enable_d)
		cv += d_add;
		
	if (!enable_i)
		i_add=0;

	return cv;
}

// ----------------------------------------------------------------------------
//!\brief Calculate motor set points as a factor (-1.0 ... +1.0)
void calc_motor_setpoints(float & motor_l, float & motor_r, float max_speed, float cv_clipped)
{
	if (main_state == msAutoModeNormal) {
		motor_l = max_speed;
		motor_r = max_speed;
		} else {
		motor_l = 0;
		motor_r = 0;
	}

	motor_l -= cv_clipped;
	motor_r += cv_clipped;

	// Make sure motor set points stay within -1.0 ... 1.0
	motor_l = clip_motor(motor_l);
	motor_r = clip_motor(motor_r);

	if (!dont_stop_steering && arrived) {
		motor_l = 0;
		motor_r = 0;
	}	
}
// ----------------------------------------------------------------------------
void do_restrict_dir(float & pid_cv)
{
	// It doesn't matter to turn CW or CCW if the error is that big,
	// insist on maintaining either CW or CCW
	if (restrict_dir==0) {
		if (pid_err > 135.0) {
			b_printf("Restrict dir -1\r\n");
			restrict_dir = -1;
		} else if (pid_err < -135) {
			b_printf("Restrict dir +1\r\n");
			restrict_dir = 1;
		}
	} else {
		if (restrict_dir==1)
			pid_cv = fabs(pid_cv);
		else
			pid_cv = -fabs(pid_cv);
		
		if ( fabs(pid_err) < 90) {
			b_printf("Cancel dir restrict\r\n");
			restrict_dir=0;
		}
	}
}

// ----------------------------------------------------------------------------
void auto_steer()
{
	float motor_l(0), motor_r(0);

	// Max speed in straight line
	float max_speed(1.0f); // This equals 100 [%] speed
	// Max steering action
	// ... in normal auto mode (sailing with small PID-error)
	float max_correct_normal(0.3*max_speed);
	// ... in "course" auto mode (while initially manoeuvering the vessel)
	float max_correct_course(0.9*max_speed);

	float rel_pid_err = fabs(pid_err) / 180.0f;

	float max_correct(0.0);
	if (main_state == msAutoModeCourse)
		max_correct = max_correct_course*rel_pid_err;
	else		
		max_correct = max_correct_normal;		

	// Use different, manually entered course (a test mode)
	if (SUBST_SP != 0.0)
		sp_used = SUBST_SP;
	else
		sp_used = bearing_sp;

	// Use simulated, manually entered compass (a test mode)
	if (SUBST_PV != 0.0)
		pv_used = SUBST_PV;
	else
		pv_used = compass_course;

	// Only enable I-action when in normal auto mode (not course mode)
	bool enable_i = (main_state == msAutoModeNormal);

	float pid_cv = simple_pid(
		pv_used, // process-value (GPS course)
		sp_used, // set point (bearing from Haversine)
		true, TUNE_P, // P-action
		enable_i, TUNE_I, // I-action
		false, TUNE_D  // D-action
		);

	do_restrict_dir(pid_cv);

	// Clip the PID control variable (CV)
	float cv_clipped(0.0);
	if (pid_cv > max_correct)
		cv_clipped = max_correct;
	else if (pid_cv < -max_correct)
		cv_clipped = -max_correct;
	else
		cv_clipped = pid_cv;

	// Take a moment to see where the boat is pointing at before doing anything
	if (state_time < COURSE_DET_TIME) {
		// Do this by keeping the PID-variables in reset condition
		p_add=0;
		i_add=0;
		d_add=0;
		cv_clipped=0;
	}

	// Calculate L+R motor speed factors (-1.0 ... +1.0)
	calc_motor_setpoints(motor_l,motor_r,max_speed,cv_clipped);

	// Convert to values that the servo hardware understands (2000...4000)
	OCR1A = (float)JOY_CENTER + (motor_l * 1000.0);
	OCR1B = (float)JOY_CENTER + (motor_r * 1000.0);
}
// ----------------------------------------------------------------------------
// Joystick center detect
// ----------------------------------------------------------------------------
bool joy_in_center(unsigned int j)
{
	return (j > (JOY_CENTER - JOY_BAND/2)) &&
		(j < (JOY_CENTER + JOY_BAND/2));
}
// ----------------------------------------------------------------------------
// Joystick down/right detect
// ----------------------------------------------------------------------------
bool joy_in_max(unsigned int j)
{
	return j > (JOY_MAX - JOY_BAND);
}
// ----------------------------------------------------------------------------
// Joystick up/left detect
// ----------------------------------------------------------------------------
bool joy_in_min(unsigned int j)
{
	return j < (JOY_MIN + JOY_BAND);
}
// ----------------------------------------------------------------------------
bool joy_in_goto()
{
	return joy_in_max(pd3_pulse_duration);
}
// ----------------------------------------------------------------------------
bool joy_in_store()
{
	return joy_in_min(pd3_pulse_duration);
}
// ----------------------------------------------------------------------------
bool joy_in_goto_store_center()
{
	return joy_in_center(pd3_pulse_duration);
}
// ----------------------------------------------------------------------------
bool joy_in_manual()
{
	return joy_in_min(pb3_pulse_duration);
}
// ----------------------------------------------------------------------------
bool joy_in_clear()
{
	return joy_in_max(pb3_pulse_duration);
}
// ----------------------------------------------------------------------------
bool set_finish(int memory_no)
{
	gp_finish.clear();

	if (memory_no >= 1 && memory_no <= 3) {
		switch (memory_no) {
		case 1: gp_finish = gp_mem_1; break;
		case 2: gp_finish = gp_mem_2; break;
		case 3: gp_finish = gp_mem_3; break;
		}
	}

	return !gp_finish.empty();
}
// ----------------------------------------------------------------------------
void store_waypoint(int memory_no)
{
	if (memory_no >= 1 && memory_no <= 3) {
		switch (memory_no) {
		case 1: gp_mem_1 = gp_current; break;
		case 2: gp_mem_2 = gp_current; break;
		case 3: gp_mem_3 = gp_current; break;
		}
	}
}

// ----------------------------------------------------------------------------
// State machine steps
// ----------------------------------------------------------------------------
void step_manual_mode()
{
	p_add=0;
	i_add=0;
	d_add=0;

	// In manual mode
	if (joy_in_goto()) {
		joy_pulses = 0;

		if (gps_valid)
			next_state = msCountJoyGoto;
		else
			next_state = msCmdErrorMan;

	} else if (joy_in_store()) {
		joy_pulses = 0;
		next_state = msCountJoyStore;
	} else if (joy_in_clear()) {
		shown_stats = false;
		next_state = msClear1;
	} else if (straight_to_auto) {
		straight_to_auto = false;
		next_state = msAutoModeCourse;
	}
}
// ----------------------------------------------------------------------------
void check_arrived()
{
	if (!dont_stop_steering && arrived) {
		b_printf("Arrived!\r\n");
		next_state = msManualMode;
	}
}
// ----------------------------------------------------------------------------
void abort_auto_if()
{
	// Blink LED fast when 'special' goto/store/clear command is given.
	// We are now in auto mode and that is allowed only in manual mode.
	if (!joy_in_goto_store_center() ||
		joy_in_clear()) {
		next_state = msCmdErrorAuto;
	}

	// Go to manual mode if GPS signal is absent for too long,
	// or joystick is put in manual control mode.
	if (joy_in_manual() || !gps_valid) {
		next_state = msManualMode;
	}
}
// ----------------------------------------------------------------------------
void step_auto_mode_course()
{
	if (state_time > 2000) {
		if (fabs(pid_err) < 16)
			next_state = msAutoModeNormal;
	}


	abort_auto_if();
	check_arrived();
}
// ----------------------------------------------------------------------------
void step_auto_mode_normal()
{
	if (fabs(pid_err) > 32)
		next_state = msAutoModeCourse;
	
	abort_auto_if();
	check_arrived();
}
// ----------------------------------------------------------------------------
void step_count_goto()
{
	if (joy_in_store())
		next_state = msCmdErrorMan; // attempt to run 'opposite' command
	else if (joy_in_goto_store_center()) {
		if (state_time > MIN_JOY_PULSE_DURATION) {
			// pulse valid, count
			joy_pulses++;
			next_state = msCountJoyGotoRetn;
		} else
			next_state = msCmdErrorMan; // pulse wasn't long enough
	} else if (state_time > MAX_JOY_PULSE_DURATION)
		next_state = msCmdErrorMan; // joystick takes too long to go back center
}
// ----------------------------------------------------------------------------
void step_count_goto_retn()
{
	if (joy_in_goto())
		next_state = msCountJoyGoto;
	else if (joy_in_store())
		next_state = msCmdErrorMan;
	else if (state_time > JOY_CMD_ACCEPT_TIME && !slow_blink) {
		blink_times = joy_pulses;
		next_state = msConfirmGotoPosX;
	}
}
// ----------------------------------------------------------------------------
void step_count_store()
{
	if (joy_in_goto())
		next_state = msCmdErrorMan;
	else if (joy_in_goto_store_center()) {
		if (state_time > MIN_JOY_PULSE_DURATION) {
			// pulse valid, count
			joy_pulses++;
			next_state = msCountJoyStoreRetn;
		} else
			next_state = msCmdErrorMan; // pulse wasn't long enough
	} else if (state_time > MAX_JOY_PULSE_DURATION)
		next_state = msCmdErrorMan; // joystick takes too long to go back center
}
// ----------------------------------------------------------------------------
void step_count_store_retn()
{
	if (joy_in_store())
		next_state = msCountJoyStore;
	else if (joy_in_store())
		next_state = msCmdErrorMan;
	else if (state_time > JOY_CMD_ACCEPT_TIME && !slow_blink) {
		blink_times = joy_pulses;
		next_state = msConfirmStorePosX;
	}
}
// ----------------------------------------------------------------------------
void step_clear1()
{
	if (!joy_in_clear())
		next_state = msCmdErrorMan;
	else if (!shown_stats) {
		print_stats();
		shown_stats=true;
	}
	else if (state_time > 1000) {
		next_state = msClear2;
	}
}
// ----------------------------------------------------------------------------
void step_clear2()
{
	if (!joy_in_clear()) {
		gp_mem_1.clear();
		gp_mem_2.clear();
		gp_mem_3.clear();
		next_state = msManualMode;
	}

	if (state_time > 5000)
		next_state = msCmdErrorMan;
}
// ----------------------------------------------------------------------------
void step_cmd_error_man()
{
	if (state_time > 2000) {
		if (joy_in_goto_store_center() && !joy_in_clear()) {
			next_state = msManualMode;
		}
	}
}
// ----------------------------------------------------------------------------
void step_cmd_error_auto()
{
	bool bLetGoOfJoyStick = joy_in_goto_store_center() && !joy_in_clear();

	if (state_time > 1000 || bLetGoOfJoyStick) {
		next_state = msAutoModeCourse;
	}
}
// ----------------------------------------------------------------------------
void step_confirm_goto_pos_x()
{
	if (blink_times > 3)
		next_state = msCmdErrorMan;
	else if (blink_times == 0) {

		if (set_finish(joy_pulses)) {
			b_printf("Set finish to # %d\r\n", joy_pulses);
			next_state = msAutoModeCourse;
		} else
			next_state = msCmdErrorMan;
	}
}
// ----------------------------------------------------------------------------
void step_confirm_store_pos_x()
{
	if (blink_times > 3)
		next_state = msCmdErrorMan;
	else if (blink_times == 0) {
		b_printf("Store waypoint # %d\r\n", joy_pulses);
		store_waypoint(joy_pulses);
		next_state = msManualMode;
	}
}
// ----------------------------------------------------------------------------
void print_step_name(TMainState st)
{
	switch (st) {
	case msAutoModeCourse: b_printf("msAutoModeCourse"); break;
	case msAutoModeNormal: b_printf("msAutoModeNormal"); break;
	case msManualMode: b_printf("msManualMode"); break;
	case msCountJoyGoto: b_printf("msCountJoyGoto"); break;
	case msCountJoyGotoRetn: b_printf("msCountJoyGotoRetn"); break;
	case msConfirmGotoPosX: b_printf("msConfirmGotoPosX"); break;
	case msCountJoyStore: b_printf("msCountJoyStore"); break;
	case msCountJoyStoreRetn: b_printf("msCountJoyStoreRetn"); break;
	case msConfirmStorePosX: b_printf("msConfirmStorePosX"); break;
	case msClear1: b_printf("msClear1"); break;
	case msClear2: b_printf("msClear2"); break;
	case msCmdErrorMan: b_printf("msCmdErrorMan"); break;
	case msCmdErrorAuto: b_printf("msCmdErrorAuto"); break;	
	}
}

// ----------------------------------------------------------------------------
// Main state machine
// ----------------------------------------------------------------------------
void state_mach()
{
	// next state defaults to current state (unchanged)
	next_state = main_state;

	switch (main_state) {
	case msManualMode: // manual control mode
		step_manual_mode();
		break;
	case msAutoModeCourse: // automatic waypoint mode (course)
		step_auto_mode_course();
		break;
	case msAutoModeNormal: // automatic waypoint mode (normal)
		step_auto_mode_normal();
		break;
	case msCountJoyGoto: // count joystick 'up' (goto pos.) command
		step_count_goto();
		break;
	case msCountJoyGotoRetn:
		step_count_goto_retn();
		break;
	case msConfirmGotoPosX:
		step_confirm_goto_pos_x();
		break;

	case msCountJoyStore: // count joystick 'down' (store pos.) command
		step_count_store();
		break;
	case msCountJoyStoreRetn:
		step_count_store_retn();
		break;
	case msConfirmStorePosX:
		step_confirm_store_pos_x();
		break;

	case msClear1:
		step_clear1();
		break;

	case msClear2:
		step_clear2();
		break;

	case msCmdErrorMan:
		step_cmd_error_man();
		break;

	case msCmdErrorAuto:
		step_cmd_error_auto();
		break;

	default:
		// should never get here
		next_state = msManualMode;
	}

	if (main_state != next_state) {
		// Transitioning, reset step time and enter new step
		b_printf("change step");

		print_step_name(next_state);
		b_printf("\r\n");

		state_time = 0;
		main_state = next_state;
	} else {
		// we're in the 100 [ms] process,
		// so we can increase step time like this...
		state_time += 100;
	}
}

void c_init_min(comp_extreme & x)
{
	x.fin = 32767;
	for (int i(0);i<4;i++)
	x.avg[i]=0;
	x.cnt = 0;
};

void c_init_max(comp_extreme & x)
{
	x.fin = -32768;
	for (int i(0);i<4;i++)
	x.avg[i]=0;
	x.cnt = 0;
};

// ----------------------------------------------------------------------------
void reset_compass_calibration()
{
	calibration_mode=false;
	c_init_min(compass_min_x);
	c_init_max(compass_max_x);
	c_init_min(compass_min_y);
	c_init_max(compass_max_y);
	c_init_min(compass_min_z);
	c_init_max(compass_max_z);
	compass_course_no_offset = 0.0f;
	compass_course = 0.0f;
}


uint16_t crc16(const unsigned char* data_p, unsigned char length)
{
	unsigned char x;
	uint16_t  crc = 0xFFFF;

	while (length--){
		x = crc >> 8 ^ *data_p++;
		x ^= x>>4;
		crc = (crc << 8) ^ ((uint16_t )(x << 12)) ^ ((uint16_t )(x <<5)) ^ ((uint16_t )x);
	}
	return crc;
}

void c_store_calibration()
{
	b_printf("Storing calibration settings...");
	
	uint16_t rec[8];

	rec[0]=(uint16_t)compass_north_offset;
	rec[1]=compass_min_x.fin;
	rec[2]=compass_max_x.fin;
	rec[3]=compass_min_y.fin;
	rec[4]=compass_max_y.fin;
	rec[5]=compass_min_z.fin;
	rec[6]=compass_max_z.fin;
	rec[7]=crc16((unsigned char*)rec,7);		
	
	unsigned int addr=0;
	for (int i(0); i<8; i++) {
		eeprom_busy_wait();
		eeprom_write_word((uint16_t*)addr,rec[i]);
		addr+=2;	
	}	
}

void c_load_calibration()
{
	b_printf("Loading calibration settings...");
	
	unsigned int addr=0;
	uint16_t rec[8];
	
	for (int i(0); i<8; i++) {
		eeprom_busy_wait();
		rec[i]=eeprom_read_word((uint16_t*)addr);
		addr+=2;
		
		int v=rec[i];
		b_printf("(Read: %04x)",v);
	}

	uint16_t chk = crc16((unsigned char*)rec,7);
	
	if (chk == rec[7]) {		
		reset_compass_calibration();
		
		compass_north_offset=rec[0];
		compass_min_x.fin=rec[1];
		compass_max_x.fin=rec[2];
		compass_min_y.fin=rec[3];
		compass_max_y.fin=rec[4];
		compass_min_z.fin=rec[5];
		compass_max_z.fin=rec[6];
				
		b_printf("OK\r\n");
		
	} else {
		b_printf("FAILED (checksum)\r\n");
	}
	
}


void set_true_north()
{
	b_printf("Setting true north.\r\n");
	compass_north_offset = 0 - compass_course_no_offset;
	
	c_store_calibration();
}

void toggle_calibration_mode()
{
	if (calibration_mode)
		calibration_mode=false;
	else
		calibration_mode=true;

	b_printf("Calibration mode: ");
	if (calibration_mode)
		b_printf("ON\r\n");
	else
		b_printf("OFF\r\n");
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
	case mmPAction:
	case mmIAction:
		l = dblParam*1000.0;
		b_printf("(set): %ld (x1000)\r\n",l);
		break;
	case mmPVSubst:
	case mmSPSubst:
		l = dblParam;
		b_printf("(set): %ld (x1)\r\n",l);
		break;
	case mmDebug:
		break;
	}
}
// ----------------------------------------------------------------------------
void tune_Config(double & dblParam, char c)
{
	tune_buf[tune_ptr++] = c;

	if (tune_ptr>15) {
		tune_ptr=0;
		b_printf("(err)\r\n");
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
			b_printf("new value accepted) %ld\r\n",ld);

			switch (msg_mode) {
			case mmNone:
			case mmCompass:
			case mmGps:
			case mmServoCapture:
			case mmSteering:
				break;										
			case mmPAction:
			case mmIAction:
				dblParam = ld / 1000.0;
				break;
			case mmPVSubst:
			case mmSPSubst:
				dblParam = ld;
			case mmDebug:
				break;
			case mmLast:
				break;				
			};

		} else {
			b_printf("(err,bad)\r\n");
		}
	}

}

// ----------------------------------------------------------------------------
void handle_parameterization(char c)
{
	switch (msg_mode) {
	case mmPAction:
		tune_Config(TUNE_P, c);
		break;
	case mmIAction:
		tune_Config(TUNE_I, c);
		break;
	case mmPVSubst:
		tune_Config(SUBST_PV, c);
		break;
	case mmSPSubst:
		tune_Config(SUBST_SP, c);
		break;
	default:
		;		
	}
}

// ----------------------------------------------------------------------------
// Handles GPS input
// ----------------------------------------------------------------------------
void read_uart()
{

	while (rb_avail()) {
		char c = rb_read();

		switch (c) {
		case 't':
			toggle_calibration_mode();
			break;
		case 'r':
			reset_compass_calibration();
			b_printf("Compass calibration reset\r\n");
			break;
		case 'n':
			set_true_north();
			break;
		case 'c':
			msg_mode = mmCompass;
			break;
		case 'g':
			msg_mode = mmGps;
			break;
		case 'p':
			msg_mode = mmPAction;
			break;
		case 'i':
			msg_mode = mmIAction;
			break;
		case 's':
			msg_mode = mmSteering;
			break;
		case 'e':
			msg_mode = mmServoCapture;
			break;
		case 'a':
			straight_to_auto = true;
			break;
		case 'x':
			if (dont_stop_steering)
				dont_stop_steering=false;
			else
				dont_stop_steering=true;
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
// Manual mode (joystick 'pass through' steering)
// ----------------------------------------------------------------------------
void manual_steering()
{
	// Pass through motor left and right setpoints to PWM module
	OCR1A = pd5_pulse_duration;
	OCR1B = pd6_pulse_duration;
}

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Periodic message
// ----------------------------------------------------------------------------
void periodic_msg()
{
	int pd6_perc, pd5_perc, pd3_perc, pb3_perc, a1, b1;

	switch (msg_mode) {
	case mmServoCapture:
		// Display current servo signals as received (2000 ... 4000, 0 = no signal)
		pd6_perc = joy_to_perc(pd6_pulse_duration);
		pd5_perc = joy_to_perc(pd5_pulse_duration);
		pd3_perc = joy_to_perc(pd3_pulse_duration);
		pb3_perc = joy_to_perc(pb3_pulse_duration);
		a1 = joy_to_perc(OCR1A);
		b1 = joy_to_perc(OCR1B);

		b_printf(" pd6=%05d pd5=%05d pd3=%05d pb3=%05d A=%05d B=%05d\r\n",
			pd6_perc, pd5_perc,
			pd3_perc, pb3_perc,
			a1, b1);
		break;

	case mmPAction:
		b_printf("(set P-action): ");
		tune_PrintValue(TUNE_P);
		break;

	case mmIAction:
		b_printf("(set I-action): ");
		tune_PrintValue(TUNE_I);
		break;

	case mmPVSubst:
		b_printf("(set PV-subst): ");
		tune_PrintValue(SUBST_PV);
		break;

	case mmSPSubst:
		b_printf("(set SP-subst): ");
		tune_PrintValue(SUBST_SP);
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
	case mmNone:
		msg_mode = mmSteering;
		break;
	}
}
// ----------------------------------------------------------------------------
// Update LED
// ----------------------------------------------------------------------------
void update_led()
{
	fast_blink = !fast_blink;

	switch (main_state) {
	/* in manual/auto mode, just show status of GPS receiver */
	case msManualMode:
		if (gps_valid) {
			// Steady LED on GPS signal okay
			led_signal = true;
		} else {
			// Slowly blink LED when there is no GPS reception
			led_signal = slow_blink;
		}
		break;

	case msAutoModeNormal:
	case msAutoModeCourse:
		led_signal = arrived;
		break;

	case msCmdErrorMan:
	case msCmdErrorAuto: // deliberate fall-through
		// Blink LED fast when an invalid command is given,
		led_signal = fast_blink;
		break;

	case msConfirmGotoPosX:
	case msConfirmStorePosX: // deliberate fall-through

		if (slow_blink_prev && !slow_blink) {
			// We just blinked once
			if (blink_times > 0)
				blink_times--;
		}

		if (blink_times > 0) {
			led_signal = slow_blink;
		} else
			led_signal = false;

		slow_blink_prev = slow_blink;
		break;

	default:
		led_signal = false;
	}

	// Update LED output (PORTB pin 5)
	if (led_signal) {
		// turn led on
		PORTB |= _BV(PORTB5);
	} else {
		// turn led off
		PORTB &= ~_BV(PORTB5);
	}
}

// ----------------------------------------------------------------------------
// 100 [ms] process
// ----------------------------------------------------------------------------
void process_100ms()
{
	// Run main state machine
	state_mach();

	// Steering
	if (main_state == msAutoModeCourse || main_state == msAutoModeNormal)
		auto_steer();
	else
		manual_steering();

	update_led();
}
// ----------------------------------------------------------------------------
// 500 [ms] process
// ----------------------------------------------------------------------------
void process_500ms()
{
	slow_blink = !slow_blink;

	// returns +- latitude/longitude in degrees
	gps.get_position(&gps_lat, &gps_lon, &gps_fix_age);
	gps_course = gps.course();
	gps_cmg = gps_course / 100.0f;

	if (gps_fix_age == TinyGPS::GPS_INVALID_AGE) {
		// No gps fix
		gps_valid = false;
	} else if (gps_fix_age > GPS_STALE_TIME) {
		// Stale GPS position
		gps_valid = false;
	} else {
		// GPS-OK
		gps.f_get_position(&gp_current.lat,&gp_current.lon,0);
		gps_valid= true;

	}

	if (!gps_valid_prev && gps_valid)
		b_printf("GPS up\r\n");
	if (gps_valid_prev && !gps_valid)
		b_printf("GPS down\r\n");

	periodic_msg();

	gps_valid_prev = gps_valid;
}



int16_t c_avg(comp_extreme & x)
{
	long int sum = 0;
	for (int i(0);i<4;i++)
		sum += x.avg[i];

	int16_t avg = sum >> 2;
	return avg;
}

void c_update_min(comp_extreme & x, int16_t newval)
{
	if (newval == 0x7fff || newval == 0x8000 || newval == 0)
		return;

	if (x.cnt > 3) {
			int16_t avg = c_avg(x);
			if (avg < x.fin)
				x.fin = avg;
			x.cnt=0;
	}
	x.avg[x.cnt++] = newval;
}

void c_update_max(comp_extreme & x, int16_t newval)
{
	if (newval == 0x7fff || newval == 0x8000 || newval == 0)
		return;
if (x.cnt > 3) {
			int16_t avg = c_avg(x);
			if (avg > x.fin)
				x.fin = avg;
			x.cnt=0;
	}
	x.avg[x.cnt++] = newval;
}

void c_calibrate()
{
	c_update_min(compass_min_x,compass_raw.x);
	c_update_max(compass_max_x,compass_raw.x);
	c_update_min(compass_min_y,compass_raw.y);
	c_update_max(compass_max_y,compass_raw.y);
	c_update_min(compass_min_z,compass_raw.z);
	c_update_max(compass_max_z,compass_raw.z);
}

float c_clip_degrees(float d)
{
	d = fmod(d,360.0f);

	if (d < 0)
		d += 360.0;

	return d;
}

float c_coords_to_angle(float ix, float iz)
{
	float d(0.0f);
	if (ix>0)
		d = atan(iz/ix);
	else if (ix<0 && iz >=0)
			d = atan(iz/ix) + PI;
		else if (ix<0 && iz < 0)
			d = atan(iz/ix) - PI;
		else if (ix==0 && iz > 0)
			d = PI / 2.0;
		else if (ix==0 && iz < 0)
			d = -PI / 2.0;
		else if (ix==0 && iz == 0)
			d = 0;


	d = 90.0 + d / TWO_PI * 360.0;
	d += 180.0;
	d = c_clip_degrees(d);
	return d;
}

float c_calc_course()
{
	float d(0.0f);

	TCompassTriple centered;
	centered.x=0;
	centered.y=0;
	centered.z=0;

	float w = compass_max_x.fin - compass_min_x.fin;
	float h = compass_max_z.fin - compass_min_z.fin;

	if (w>=0 && h>=0) {
		centered.x = (compass_raw.x - compass_min_x.fin - (w/2.0));
		centered.z = -(compass_raw.z - compass_min_z.fin - (h/2.0));
	}

	if (w>0 && h>0) {
		float ix = (float)centered.x / (w/2.0);
		float iz = (float)centered.z / (h/2.0);

		d = c_coords_to_angle(ix,iz);
	}

	return d;
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
	bearing_sp = gp_current.bearingTo(gp_finish);


	distance_m =
		TinyGPS::distance_between (gp_finish.lat, gp_finish.lon,
			gp_current.lat, gp_current.lon);
	arrived = distance_m < 10;


	compass_raw.x = 0;
	compass_raw.y = 0;
	compass_raw.z = 0;

	compass_raw.x = read_hmc5843(0x03);
	compass_raw.y = read_hmc5843(0x05);
	compass_raw.z = read_hmc5843(0x07);

	if (calibration_mode)
		c_calibrate();

	compass_course_no_offset = c_calc_course();
	compass_course = c_clip_degrees(compass_course_no_offset + compass_north_offset);

	compass_smp++;

	m8n_set_reg_addr(0xff);
	multi_read_m8n(gps);

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

	b_printf("Boot!\r\n");

	c_load_calibration();

	// Setup other peripherals
	setup_capture_inputs();
	setup_pwm();
	//setup_gps_input();

	// Initial state machine / modes
	//main_state = msManualMode;
	//msg_mode = mmServoCapture;
	//msg_mode = mmGps;
	//msg_mode = mmSteering;
	msg_mode = mmDebug;
	//msg_mode = mmNone;

	reset_compass_calibration();

#if 1
	compass_min_x.fin = -307;
	compass_max_x.fin = 364;
	compass_min_z.fin = -532;
	compass_max_z.fin = 76;
#endif

	clear_stats();

	// Initialize I2C-bus I/O
#ifndef _WIN32
	DDRC = 0b00110000;
	PORTC = 0b00110000; //pullups on the I2C bus

	i2cInit();
	i2cSetBitrate(15);

	delay_ms(100);

	init_hmc5843();

	delay_ms(25);
	init_hmc5843();
	delay_ms(25);

#endif


#ifndef _WIN32
	main_loop();
#endif

	return 0;
}
// ----------------------------------------------------------------------------
// EOF
// ----------------------------------------------------------------------------
