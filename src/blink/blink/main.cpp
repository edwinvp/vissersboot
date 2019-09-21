/*
 * blink.cpp
 *
 * Created: 20-9-2019 21:30:13
 * Author : Z
 */ 

#define F_CPU 16000000

#include <avr/io.h>
#include <stdio.h>
#include <avr/delay.h>
#include "uart.h"

int main(void)
{
	DDRC = 0x80;
	
	USART_Init();
	
    /* Replace with your application code */
    while (1) 
    {
			PORTC = 0x80;
			
			_delay_ms(1000);
			
			putchar('t');
			putchar('e');
			putchar('s');
			putchar('t');
			putchar('\r');
			putchar('\n');

			PORTC = 0x00;
			_delay_ms(250);
    }
}

