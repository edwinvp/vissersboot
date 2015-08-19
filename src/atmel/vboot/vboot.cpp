#include "settings.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Target: ATmega328
//#include <avr/iom328.h>
#include <avr/common.h>
#include <stdio.h>
#include "TinyGPS.h"

enum {
	BLINK_DELAY_MS = 25,
};

#include "uart.h"

volatile unsigned int icnt;
volatile unsigned long prog_ms = 0;

unsigned int millis()
{
	return prog_ms;
}


void copy_gps_pin(void)
{
	if (PINB & _BV(PINB0))
	{
		PORTD |= _BV(PORTD7);
	} else
	{
		PORTD &= ~_BV(PORTD7);
	}

}

ISR(PCINT0_vect)
{
	// For PD0 (UART RX) equal to current value of soft GPS pin
	copy_gps_pin();	
}

ISR(PCINT1_vect)
{

}

volatile unsigned int pd6_rising;
volatile unsigned int pd6_pulse_duration;

volatile unsigned int pd5_rising;
volatile unsigned int pd5_pulse_duration;


volatile unsigned char old_pind = 0;

ISR(TIMER1_OVF_vect)
{
	// Timer 1 overflow	
	prog_ms += 20;	// timer was set to overflow each 20 [ms]	 	
}


ISR(PCINT2_vect)
{
	unsigned int tmr_reg;
	
	++icnt;
		
	tmr_reg = TCNT1;

	
	if ((PIND & _BV(PIND6)) ^ (old_pind & _BV(PIND6))) {				
		if (PIND & _BV(PIND6)) 
			pd6_rising = tmr_reg;
		else {
			pd6_pulse_duration = tmr_reg - pd6_rising;	
			if (pd6_pulse_duration > 10000)
				pd6_pulse_duration -= 25536;
		}			
	}
	
	if ((PIND & _BV(PIND5)) ^ (old_pind & _BV(PIND5))) {
		if (PIND & _BV(PIND5))
			pd5_rising = tmr_reg;
		else {
			pd5_pulse_duration = tmr_reg - pd5_rising;
			if (pd5_pulse_duration > 10000)
				pd5_pulse_duration -= 25536;			
		}
	}
	
	
	old_pind = PIND;
}

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


ISR(USART_RX_vect) {
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
}

void setup_capture_inputs()
{
	// Configure PD6 as input
	DDRD &= ~_BV(DDD6);	
	PORTD &= ~_BV(PORTD6);	
	// Configure PD5 as input
	DDRD &= ~_BV(DDD5);
	PORTD &= ~_BV(PORTD5);
	
	// Enable on-pin-change for pins
// Disabled temporary	
//	PCMSK2 |= _BV(PCINT22); // PD6
//	PCMSK2 |= _BV(PCINT21); // PD5
	
	// Configure interrupt on logical state state on PD6 (so PCIE2)
	PCICR |= _BV(PCIE2);
	
	// Configure timer 1 clock source
	// Set to use a clock source of clkio / 8.
	// So 20 [ms] servo period will be 40000 timer ticks.	
	TCCR1B &= ~_BV(CS12);
	TCCR1B |= _BV(CS11);
	TCCR1B &= ~_BV(CS10);
	
	// Enable timer 1 overflow interrupt
	TIMSK1 |= _BV(TOIE1);
}

void setup_pwm()
{
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
}

void clear_stats(void) 
{
		//	
}

int main (void)
{
	bool blink(false);
	
	/* set pin 5 of PORTB for output*/
	DDRB |= _BV(DDB5);
	
	USART_Init();  // Initialize USART
	sei();         // enable all interrupts	
	
	setup_capture_inputs();
	setup_pwm();	
	setup_gps_input();

	
	clear_stats();


	TinyGPS gps;
	
	unsigned long start_ms(0);
	
	while(1) {

		// Pass through setpoints
		OCR1A = pd5_pulse_duration;
		OCR1B = pd6_pulse_duration;

#if 0
		printf(" icnt=%05d pd6=%05d pd5=%05d \r\n", 
			icnt, pd6_pulse_duration, pd5_pulse_duration );
		
		printf(" us_str1=%s\r\n", us_str1);
		printf(" us_str2=%s\r\n", us_str2);
		printf(" us_str3=%s\r\n", us_str3);
		
		printf(" sns=%d sf=%d stf1=%d stf2=%d sl=%d \r\n",
			stats_no_stop_bit, stats_frames, stats_too_fast1, stats_too_fast2, stats_lines );
#endif			
			
		
		copy_gps_pin();

		// decode GPS serial stream	
		while (rb_avail()) {
			gps.encode(rb_read());
		}
	
		millis();

	
		unsigned long delta = prog_ms - start_ms;				
		if (delta > 500) {
			start_ms = prog_ms;				

			blink = !blink;

			if (blink) {
				/* set pin 5 high to turn led on */
				PORTB |= _BV(PORTB5);
			} else {
				/* set pin 5 low to turn led off */
				PORTB &= ~_BV(PORTB5);
			}			
			
			long lat, lon;
			unsigned long fix_age; // returns +- latitude/longitude in degrees
			gps.get_position(&lat, &lon, &fix_age);
			if (fix_age == TinyGPS::GPS_INVALID_AGE)
				printf("GPS-NO_FIX\r\n");
			else if (fix_age > 150000)
				printf("GPS-STALE\r\n");
			else {
				printf("GPS-OK age=%ld. lat=%ld lon=%ld \r\n",fix_age,lat,lon);			
			}
		}

		
	}
	
	return 0;
}
