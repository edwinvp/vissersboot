#include "settings.h"
#include "compass_calibrate.h"
#include "state_machine.h"
#include "steering.h"
#include "joystick.h"
#include "waypoints.h"

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

#endif

#include "TinyGPS.h"
#include "lat_lon.h"
#include "uart.h"
#include "vboot.h"

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
	p: (PID 'P'-action configuration)
	i: (PID 'I'-action configuration)
	(auto steer test screen)
	a: go straight to auto steer mode
	x: don't stop steering (even after arriving)
	d: substitute PV compass heading (enter 0 for no substution)
	o: substitute SP heading (enter 0 for no substitution)

	"0123456789.,-+" (valid in PID configuration modi)
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

// Tests
bool subst_pv = false;
bool subst_sp = false;

TMessageMode msg_mode;

// GPS input related
unsigned long gps_fix_age = TinyGPS::GPS_INVALID_AGE;
TinyGPS gps;
bool gps_valid = false;
bool gps_valid_prev = false;
long gps_lat, gps_lon, gps_course;

// Compass input
TCompassTriple compass_raw;
int16_t compass_smp;

CCompassCalibration cc;

// Auto steering related
float distance_m = 0; // distance to finish from current gps pos

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
volatile unsigned char comms_char;
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
	comms_char = UDR0;

// // also happens with this disabled
#if 1
	// Store byte in FIFO (if FIFO isn't full)
	unsigned char new_head;
	new_head = head + 1;
	new_head &= FIFO_MASK;

	if (new_head != tail) {
		uart_fifo[head] = comms_char;
		head = new_head;
	}
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

	OCR1A = JOY_CENTER;
	OCR1B = JOY_CENTER;

	ICR1 = 40000; // gives a 20 [ms] period
#endif
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

	int a1 = joystick.to_perc(OCR1A);
	int b1 = joystick.to_perc(OCR1B);
	unsigned int sp_d = steering.sp_used;
	unsigned int pv_d = steering.pv_used;
	unsigned int err_d = steering.pid_err;

	if (stm.Step()==msAutoModeCourse)
		b_printf("[autoC] ");
	else if (stm.Step()==msAutoModeNormal)
		b_printf("[autoN] ");
	else
		b_printf("[man] ");

	b_printf("age=%ld ", gps_fix_age);

	if (stm.Step()==msAutoModeCourse || stm.Step()==msAutoModeNormal) {
		b_printf("sp=%d", sp_d);
		if (steering.SUBST_SP!=0)
			b_printf("*");
	
		b_printf(" ");
	
		b_printf("pv=%d", pv_d);
		if (steering.SUBST_PV!=0)
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
	int(steering.compass_course),
	int(steering.bearing_sp));
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
		tune_Config(steering.TUNE_P, c);
		break;
	case mmIAction:
		tune_Config(steering.TUNE_I, c);
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
// Handles GPS input
// ----------------------------------------------------------------------------
void read_uart()
{

	while (rb_avail()) {
		char c = rb_read();

		switch (c) {
		case 't':
			cc.toggle_calibration_mode();
			break;
		case 'r':
			cc.reset_compass_calibration();
			b_printf("Compass calibration reset\r\n");
			break;
		case 'n':
			cc.set_true_north();
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
		pd6_perc = joystick.to_perc(pd6_pulse_duration);
		pd5_perc = joystick.to_perc(pd5_pulse_duration);
		pd3_perc = joystick.to_perc(pd3_pulse_duration);
		pb3_perc = joystick.to_perc(pb3_pulse_duration);
		a1 = joystick.to_perc(OCR1A);
		b1 = joystick.to_perc(OCR1B);

		b_printf(" pd6=%05d pd5=%05d pd3=%05d pb3=%05d A=%05d B=%05d\r\n",
			pd6_perc, pd5_perc,
			pd3_perc, pb3_perc,
			a1, b1);
		break;

	case mmPAction:
		b_printf("(set P-action): ");
		tune_PrintValue(steering.TUNE_P);
		break;

	case mmIAction:
		b_printf("(set I-action): ");
		tune_PrintValue(steering.TUNE_I);
		break;

	case mmPVSubst:
		b_printf("(set PV-subst): ");
		tune_PrintValue(steering.SUBST_PV);
		break;

	case mmSPSubst:
		b_printf("(set SP-subst): ");
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

	switch (stm.Step()) {
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
		led_signal = steering.arrived;
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
    stm.Run();

	// Steering
	if (stm.Step() == msAutoModeCourse || stm.Step() == msAutoModeNormal)
		steering.auto_steer();
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

	if (!gps_valid_prev && gps_valid)
		b_printf("GPS up\r\n");
	if (gps_valid_prev && !gps_valid)
		b_printf("GPS down\r\n");

	periodic_msg();

	gps_valid_prev = gps_valid;
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
	steering.arrived = distance_m < 10;


	compass_raw.x = 0;
	compass_raw.y = 0;
	compass_raw.z = 0;

	compass_raw.x = read_hmc5843(0x03);
	compass_raw.y = read_hmc5843(0x05);
	compass_raw.z = read_hmc5843(0x07);

	if (cc.calibration_mode)
		cc.calibrate(compass_raw);
	
	steering.compass_course = cc.calc_course(compass_raw);

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

	cc.load_calibration();
    waypoints.load_waypoints();

	// Setup other peripherals
	setup_capture_inputs();
	setup_pwm();

	// Initial state machine / modes
	//stm.Step() = msManualMode;
	//msg_mode = mmServoCapture;
	//msg_mode = mmGps;
	//msg_mode = mmSteering;
	msg_mode = mmDebug;
	//msg_mode = mmNone;

	cc.reset_compass_calibration();

#if 1
	cc.compass_min_x.fin = -307;
	cc.compass_max_x.fin = 364;
	cc.compass_min_z.fin = -532;
	cc.compass_max_z.fin = 76;
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
