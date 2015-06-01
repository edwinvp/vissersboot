#include "settings.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Target: ATmega328
//#include <avr/iom328.h>
#include <avr/common.h>
#include <stdio.h>

enum {
	BLINK_DELAY_MS = 10,
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


volatile unsigned char old_pinb = 0;
volatile unsigned int pb0_changes = 0;

volatile unsigned int icnt;
volatile unsigned int ocnt;

ISR(PCINT0_vect)
{

	if ((PINB & _BV(PINB0)) ^ (old_pinb & _BV(PINB0))) {
		ocnt=0;

/*
		if (PINB & _BV(PINB0))
			;
		else {
			icnt=0;
		}
*/		
	
		pb0_changes++;	
	}
		
	
	old_pinb = PINB;
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
	++ocnt;
	
	if (ocnt>10) {	
		ocnt=10;
		pb0_changes = 0;
	}
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

void setup_gps_input()
{
	// Configure PB0 as input
	DDRB &= ~_BV(DDB0);
	PORTB &= ~_BV(PORTB0);
	
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

int main (void)
{
	/* set pin 5 of PORTB for output*/
	DDRB |= _BV(DDB5);
	
	USART_Init();  // Initialize USART
	sei();         // enable all interrupts	
	
	setup_capture_inputs();
	setup_pwm();	
	setup_gps_input();
	
	while(1) {

		// Pass through setpoints
		OCR1A = pd5_pulse_duration;
		OCR1B = pd6_pulse_duration;


		/* set pin 5 high to turn led on */
		PORTB |= _BV(PORTB5);
		_delay_ms(BLINK_DELAY_MS);

		printf(" ocnt=%05d icnt=%05d \r\n", ocnt, icnt);
		printf(" pd6=%05d pd5=%05d pb0_changes=%05d \r\n", 
			pd6_pulse_duration, pd5_pulse_duration, pb0_changes);
		
		//USART_SendByte(value);  // send value

		/* set pin 5 low to turn led off */
		PORTB &= ~_BV(PORTB5);
		_delay_ms(BLINK_DELAY_MS);
		
	}
	
	return 0;
}
