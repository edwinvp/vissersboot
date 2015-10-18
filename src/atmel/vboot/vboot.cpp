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

#ifdef _WIN32
TinyGPS gps;

CLatLon gp_current;
CLatLon gp_start;
CLatLon gp_finish;
float bearing_sp = 0;

#endif

enum {
	BLINK_DELAY_MS = 25,
};

#include "uart.h"

volatile unsigned long prog_ms = 0;

unsigned long start1_ms(0);
unsigned long start2_ms(0);

bool blink(false);

unsigned int millis()
{
	return prog_ms;
}


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

ISR(TIMER1_OVF_vect)
{
	// Timer 1 overflow
	prog_ms += 20;	// timer was set to overflow each 20 [ms]
}

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

ISR(PCINT1_vect)
{

}

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

bool rb_avail()
{
	return (head != tail);
}

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

	OCR1A = 3000;
	OCR1B = 2000;

	ICR1 = 40000; // gives a 20 [ms] period
#endif
}

void clear_stats(void)
{
		//
}

void process()
{
	// Pass through motor left and right setpoints to PWM module
	OCR1A = pd5_pulse_duration;
	OCR1B = pd6_pulse_duration;

#if 0
	// Display current servo signals as received (2000 ... 4000, 0 = no signal)
	unsigned long delta1 = prog_ms - start1_ms;
	if (delta1 >= 100) {
		start1_ms = prog_ms;
		printf(" pd6=%05d pd5=%05d pd3=%05d pb3=%05d \r\n",
			pd6_pulse_duration, pd5_pulse_duration,
			pd3_pulse_duration, pb3_pulse_duration);
		}
#endif

	//bearing_sp = gp_start.bearingTo(gp_finish);
	bearing_sp = gp_current.bearingTo(gp_finish);


	copy_gps_pin();

	// decode GPS serial stream
	while (rb_avail()) {
		gps.encode(rb_read());
	}

	millis();

	unsigned long delta2 = prog_ms - start2_ms;
	if (delta2 > 500) {
		start2_ms = prog_ms;

		blink = !blink;

		if (blink) {
			/* set pin 5 high to turn led on */
			PORTB |= _BV(PORTB5);
		} else {
			/* set pin 5 low to turn led off */
			PORTB &= ~_BV(PORTB5);
		}

		long lat, lon, course;
		unsigned long fix_age; // returns +- latitude/longitude in degrees
		gps.get_position(&lat, &lon, &fix_age);
		course = gps.course();
		if (fix_age == TinyGPS::GPS_INVALID_AGE)
			printf("GPS-NO_FIX\r\n");
		else if (fix_age > 150000)
			printf("GPS-STALE\r\n");
		else {
			printf("GPS-OK age=%ld. lat=%ld lon=%ld course=%ld\r\n",fix_age,lat,lon,
				course);

			gps.f_get_position(&gp_current.lat,&gp_current.lon,0);
		}
	}
}

#ifdef _WIN32
int main_init (void)
#else
int main (void)
#endif
{
#ifndef _WIN32
	/* set pin 5 of PORTB for output*/
	DDRB |= _BV(DDB5);

	USART_Init();  // Initialize USART
#endif
	sei();         // enable all interrupts

	setup_capture_inputs();
	setup_pwm();
	setup_gps_input();


	clear_stats();

#ifndef _WIN32
	TinyGPS gps;

	while(1) {
		process();
	}
#endif

	return 0;
}
