
#include "settings.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Target: ATmega328
//#include <avr/iom328.h>
#include <avr/common.h>
#include <stdio.h>

enum {
	BLINK_DELAY_MS = 500,
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


int main (void)
{
	/* set pin 5 of PORTB for output*/
	DDRB |= _BV(DDB5);
	
	USART_Init();  // Initialise USART
	sei();         // enable all interrupts	
	
	
	while(1) {
		/* set pin 5 high to turn led on */
		PORTB |= _BV(PORTB5);
		USART_SendByte('A');  // send value 
		_delay_ms(BLINK_DELAY_MS);
		
		/* set pin 5 low to turn led off */
		PORTB &= ~_BV(PORTB5);
		USART_SendByte('B');  // send value
		_delay_ms(BLINK_DELAY_MS);
		

		USART_SendByte(value);  // send value
		_delay_ms(BLINK_DELAY_MS);
		
		printf("zo kan het ook\r\n");
		
	}
	
	return 0;
}
