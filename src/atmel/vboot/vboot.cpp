#include "settings.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Target: ATmega328
//#include <avr/iom328.h>
#include <avr/common.h>
#include <stdio.h>

enum {
	BLINK_DELAY_MS = 125,
};

#include "uart.h"

volatile unsigned char value;
/* This variable is volatile so both main and RX interrupt can use it.
It could also be a uint8_t type */

/* Interrupt Service Routine for Receive Complete
NOTE: vector name changes with different AVRs see AVRStudio -
Help - AVR-Libc reference - Library Reference - <avr/interrupt.h>: Interrupts
for vector names other than USART_RXC_vect for ATmega32 */

ISR(USART_RX_vect){
	
	value = UDR0;             //read UART register into value
//	PORTB = ~value;          // output inverted value on LEDs (0=on)
}


ISR(PCINT0_vect)
{

}

ISR(PCINT1_vect)
{

}

volatile unsigned int ml_duty_cycle = 2000;
volatile unsigned int mr_duty_cycle = 4000;

void set_match_mode_a(bool set_signal)
{
	TCCR1A &= ~_BV(COM1A1);
	TCCR1A |= _BV(COM1A0);		
}

void set_match_mode_b(bool set_signal)
{
#if 0
	// Set/clear on match (Timer 1 B) depending on 'set_signal'
	TCCR1A |= _BV(COM1B1);
	
	if (set_signal)
		TCCR1A |= _BV(COM1B0);
	else
		TCCR1A &= ~_BV(COM1B0);		
#else
	TCCR1A &= ~_BV(COM1B1);
	TCCR1A |= _BV(COM1B0);
#endif
		
		
}


volatile unsigned int prev_tmr_a;

ISR(TIMER1_COMPA_vect)
{
	unsigned int adj;
	
	unsigned int old_tmr;
	
	old_tmr = OCR1A;

	//if (TCCR1A & _BV(COM1A0)) {
	//if (PORTB & _BV(PINB1)) {
	if ( (old_tmr - prev_tmr_a)  > 20000 )  {
		// Now we need to tell when to make the signal low again
		// the duration is the duty cycle of the pulse
		//adj = ml_duty_cycle;		
		//set_match_mode_a(false);
		adj = 3000;
	} else {
		// Now we need to tell when to make the signal high again
		// the duration is 40000 ticks (=20 [ms]) - duty cycle
		//adj = 40000 - ml_duty_cycle;
		//set_match_mode_a(true);		
		adj = 37000;
	}
	
	adj += old_tmr;

	OCR1A = adj;
}

ISR(TIMER1_COMPB_vect)
{
/*
	unsigned int adj;

	if (TCCR1A & _BV(COM1B0)) {
		// Now we need to tell when to make the signal low again
		// the duration is the duty cycle of the pulse
		adj = mr_duty_cycle;
		set_match_mode_b(false);
	} else {
		// Now we need to tell when to make the signal high again
		// the duration is 40000 ticks (=20 [ms]) - duty cycle
		adj = 40000 - mr_duty_cycle;
		set_match_mode_b(true);
	}
	
	OCR1B = OCR1B + adj;
*/	
}

volatile unsigned int pd6_rising;
volatile unsigned int pd6_pulse_duration;

volatile unsigned int pd5_rising;
volatile unsigned int pd5_pulse_duration;

volatile unsigned int icnt;

volatile unsigned char old_pind = 0;


ISR(PCINT2_vect)
{
	unsigned int tmr_reg;

	++icnt;
	
	tmr_reg = TCNT1;
	
	if ((PIND & _BV(PIND6)) ^ (old_pind & _BV(PIND6))) {				
		if (PIND & _BV(PIND6)) 
			pd6_rising = tmr_reg;
		else
			pd6_pulse_duration = tmr_reg - pd6_rising;
	}
	
	if ((PIND & _BV(PIND5)) ^ (old_pind & _BV(PIND5))) {
		if (PIND & _BV(PIND5))
			pd5_rising = tmr_reg;
		else
			pd5_pulse_duration = tmr_reg - pd5_rising;
	}
	
	
	old_pind = PIND;
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
	PCMSK2 |= _BV(PCINT22); // PD6
	PCMSK2 |= _BV(PCINT21); // PD5
	
	// Configure interrupt on logical state state on PD6 (so PCIE2)
	PCICR |= _BV(PCIE2);
	
	// Configure timer 1 clock source
	// Set to use a clock source of clkio / 8.
	// So 20 [ms] servo period will be 40000 timer ticks.	
	TCCR1B &= ~_BV(CS12);
	TCCR1B |= _BV(CS11);
	TCCR1B &= ~_BV(CS10);
}

void setup_pwm()
{
	// Configure PB1 & PB2 as outputs
	DDRB |= _BV(DDB1);
	DDRB |= _BV(DDB2);
	
	set_match_mode_a(true);
	set_match_mode_b(true);
	

	// Enable interrupts for out compare match A and B
	TIMSK1 |= _BV(OCIE1A);
	TIMSK1 |= _BV(OCIE1B);
	
	// Set normal mode
	TCCR1B &= ~(_BV(WGM13));
	TCCR1B &= ~(_BV(WGM12));
	
	TCCR1A &= ~(_BV(WGM11));
	TCCR1A &= ~(_BV(WGM10));
	
}

int main (void)
{
	/* set pin 5 of PORTB for output*/
	DDRB |= _BV(DDB5);
	
	USART_Init();  // Initialize USART
	sei();         // enable all interrupts	
	
	setup_capture_inputs();
	setup_pwm();
	
	while(1) {

#if 0		
		/* set pin 5 high to turn led on */
		PORTB |= _BV(PORTB5);
		_delay_ms(BLINK_DELAY_MS);

		printf(" icnt=%05d pd6=%05d pd5=%05d \r\n", icnt, pd6_pulse_duration, pd5_pulse_duration);

		USART_SendByte(value);  // send value

		/* set pin 5 low to turn led off */
		PORTB &= ~_BV(PORTB5);
		_delay_ms(BLINK_DELAY_MS);
#endif
		
	}
	
	return 0;
}
