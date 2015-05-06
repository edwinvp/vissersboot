
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


volatile unsigned int pd6_rising;
volatile unsigned int pd6_falling;
volatile unsigned int pd6_pulse_duration;
volatile unsigned int icnt;

ISR(PCINT2_vect)
{
	unsigned int tmr_reg;

	++icnt;
	
	tmr_reg = TCNT1;
		
	if (PIND & (1 << PIND6)) {
		pd6_rising = tmr_reg;
	} else {
		pd6_falling = tmr_reg;		
	
		pd6_pulse_duration = pd6_falling - pd6_rising;
	}
	
}

void setup_capture_inputs()
{
	// Configure PD6 as input
	DDRD &= ~(1 << DDB6);	
	PORTD &= ~(1 << PORTD6);	
	
	PCMSK2 |= (1 << PCINT22);
	
	// Configure interrupt on logical state state on PD6 (so PCIE2)
	PCICR |= (1 << PCIE2);
	
	// Configure timer 1 clock source
	TCCR1B &= ~(1 << CS12);
	TCCR1B &= ~(1 << CS11);
	TCCR1B |= (1 << CS10);
}


int main (void)
{
	/* set pin 5 of PORTB for output*/
	DDRB |= _BV(DDB5);
	
	USART_Init();  // Initialise USART
	sei();         // enable all interrupts	
	
	setup_capture_inputs();
	
	while(1) {
		
		/* set pin 5 high to turn led on */
		PORTB |= _BV(PORTB5);
		_delay_ms(BLINK_DELAY_MS);

		printf(" icnt=%d pd=%d \r\n", icnt, pd6_pulse_duration);

		USART_SendByte(value);  // send value

		/* set pin 5 low to turn led off */
		PORTB &= ~_BV(PORTB5);
		_delay_ms(BLINK_DELAY_MS);
		
	}
	
	return 0;
}
