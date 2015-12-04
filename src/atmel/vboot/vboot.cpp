#include "settings.h"

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

#endif

#include "TinyGPS.h"
#include "lat_lon.h"
#include "uart.h"

// ----------------------------------------------------------------------------
// VARIABLES
// ----------------------------------------------------------------------------

TMainState main_state,next_state;

unsigned long state_time = 0; // time elapsed in this state machine step [ms]

// Number of times the goto/store joystick axis has been pushed up or down
int joy_goto_cnt = 0; // # of goto commands
int joy_store_cnt = 0;  // # of store commands

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
float gps_cmg = 0; // gps course made good
long gps_lat, gps_lon, gps_course;

// Auto steering related
float bearing_sp = 0; // calculated initial bearing (from Haversine formulas)

// LED control related
int blink_times(0);
bool slow_blink(false);
bool slow_blink_prev(false);
bool fast_blink(false);
bool led_signal(false);

// Global [ms] timer
volatile unsigned long global_ms_timer = 0;

// Timing for periodic processes
unsigned long t_100ms_start_ms(0);
unsigned long t_500ms_start_ms(0);

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
unsigned int millis()
{
	return global_ms_timer;
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

	value = UDR0;             //read UART register into value

	unsigned char new_head;
	new_head = head + 1;
	new_head &= FIFO_MASK;

	if (new_head != tail) {
		uart_fifo[head] = value;
		head = new_head;
	}
}
// ----------------------------------------------------------------------------
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

	OCR1A = JOY_MID;
	OCR1B = JOY_MID;

	ICR1 = 40000; // gives a 20 [ms] period
#endif
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
void auto_steer()
{
	float motor_l(0), motor_r(0);

	float max_speed(0.6f);

	float bearing_error = bearing_sp - gps_cmg;

	float max_correct(0.9*max_speed);

	motor_l = max_speed;
	motor_r = max_speed;

	float adjust = bearing_error;

	if (adjust > max_correct)
		adjust = max_correct;
	else if (adjust < -max_correct)
		adjust = -max_correct;

	motor_l += adjust;
	motor_r -= adjust;

	motor_l = clip_motor(motor_l);
	motor_r = clip_motor(motor_r);

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
// State machine steps
// ----------------------------------------------------------------------------
void step_manual_mode()
{
	// In manual mode
	if (joy_in_goto()) {
		joy_goto_cnt = 0;
		joy_store_cnt = 0;

		if (gps_valid)
			next_state = msCountJoyGoto;
		else
			next_state = msCmdError;

	} else if (joy_in_store()) {
		joy_goto_cnt = 0;
		joy_store_cnt = 0;
		next_state = msCountJoyStore;
	} else if (joy_in_clear()) {
		next_state = msClear1;
	}
}
// ----------------------------------------------------------------------------
void step_auto_mode()
{
	if (joy_in_manual() || !gps_valid) {
		next_state = msManualMode;
	}

}
// ----------------------------------------------------------------------------
void step_count_goto()
{
	if (joy_in_store())
		next_state = msCmdError;
	else if (joy_in_goto_store_center()) {
		if (state_time > MIN_GOTO_STORE_MIN_DURATION) {
			joy_goto_cnt++;
			next_state = msCountJoyGotoRetn;
		} else
			next_state = msCmdError;
	} else if (state_time > MIN_GOTO_STORE_ACCEPT_TIME)
		next_state = msCmdError;
}
// ----------------------------------------------------------------------------
void step_count_goto_retn()
{
	if (joy_in_goto())
		next_state = msCountJoyGoto;
	else if (joy_in_store())
		next_state = msCmdError;
	else if (state_time > MIN_GOTO_STORE_ACCEPT_TIME && !slow_blink) {
		blink_times = joy_goto_cnt;
		next_state = msConfirmGotoPosX;
	}
}
// ----------------------------------------------------------------------------
void step_count_store()
{
	if (joy_in_goto())
		next_state = msCmdError;
}
// ----------------------------------------------------------------------------
void step_count_store_retn()
{
	if (joy_in_goto())
		next_state = msCmdError;
}
// ----------------------------------------------------------------------------
void step_clear1()
{
	if (!joy_in_clear())
		next_state = msCmdError;
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
		next_state = msCmdError;
}
// ----------------------------------------------------------------------------
void step_cmd_error()
{
	if (state_time > 2000) {
		if (joy_in_goto_store_center() && !joy_in_clear()) {
			next_state = msManualMode;
		}
	}
}
// ----------------------------------------------------------------------------

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
	case msAutoMode: // automatic waypoint mode
		step_auto_mode();
		break;

	case msCountJoyGoto: // count joystick 'up' (goto pos.) command
		step_count_goto();
		break;
	case msCountJoyGotoRetn:
		step_count_goto_retn();
		break;
	case msConfirmGotoPosX:

		if (blink_times > 3)
			next_state = msCmdError;
		else if (blink_times == 0)
			next_state = msAutoMode;

		break;

	case msCountJoyStore: // count joystick 'down' (store pos.) command
		step_count_store();
		break;
	case msCountJoyStoreRetn:
		step_count_store_retn();
		break;
	case msConfirmStorePosX:
		if (blink_times == 0)
			next_state = msManualMode;

		break;

	case msClear1:
		step_clear1();
		break;

	case msClear2:
		step_clear2();
		break;

	case msCmdError:
		step_cmd_error();
		break;
	default:
		// should never get here
		next_state = msManualMode;
	}

	if (main_state != next_state) {
		state_time = 0;
		main_state = next_state;
	} else {
		state_time += 100;
	}
}
// ----------------------------------------------------------------------------
// Handles GPS input
// ----------------------------------------------------------------------------
void run_gps_input()
{
	// Soft pass through of gps pin due to the fact
	// we don't have a second UART/USART and the GPS module
	// is connected to an I/O-pin with limited (input capture)
	// features.
	copy_gps_pin();

	// get bytes received by uart,
	// and decode GPS serial stream
	while (rb_avail()) {
		gps.encode(rb_read());
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
// Periodic message
// ----------------------------------------------------------------------------
void periodic_msg()
{
	switch (msg_mode) {
	case mmServoCapture:
		// Display current servo signals as received (2000 ... 4000, 0 = no signal)
		printf(" pd6=%05d pd5=%05d pd3=%05d pb3=%05d \r\n",
			pd6_pulse_duration, pd5_pulse_duration,
			pd3_pulse_duration, pb3_pulse_duration);
		break;

	case mmGps:
		if (gps_fix_age == TinyGPS::GPS_INVALID_AGE)
			printf("GPS-NO_FIX\r\n");
		else if (gps_fix_age > 150000)
			printf("GPS-STALE\r\n");
		else {
			printf("GPS-OK age=%ld. lat=%ld lon=%ld course=%ld\r\n",
				gps_fix_age, gps_lat, gps_lon, gps_course);
		}
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
	case msAutoMode: // deliberate fall-through
		if (gps_valid) {
			// Steady LED on GPS signal okay
			led_signal = true;
		} else {
			// Slowly blink LED when there is no GPS reception
			led_signal = slow_blink;
		}
		break;

	case msCmdError:
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
	if (main_state == msAutoMode)
		auto_steer();
	else
		manual_steering();

	update_led();

	periodic_msg();
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
}

// ----------------------------------------------------------------------------
// MAIN PROCESS
// ----------------------------------------------------------------------------
void process()
{
	unsigned long delta(0);

	// Handle UART (GPS) input
	run_gps_input();

	// Calculate initial bearing with Haversine function
	bearing_sp = gp_current.bearingTo(gp_finish);


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

	// Setup other peripherals
	setup_capture_inputs();
	setup_pwm();
	setup_gps_input();

	// Initial state machine / modes
	main_state = msManualMode;
	msg_mode = mmNone;

	clear_stats();

#ifndef _WIN32
	main_loop();
#endif

	return 0;
}
// ----------------------------------------------------------------------------
// EOF
// ----------------------------------------------------------------------------

