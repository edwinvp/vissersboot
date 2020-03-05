
#include <avr/io.h>
#include <stdio.h>
#include "uart.h"


void USART_SendByte(uint8_t u8Data){

	// Wait until last byte has been transmitted
	while((UCSR1A &(1<<UDRE1)) == 0);

	// Transmit data
	UDR1 = u8Data;
}


// not being used but here for completeness
// Wait until a byte has been received and return received data
uint8_t USART_ReceiveByte(){
	while((UCSR1A &(1<<RXC1)) == 0);
	return UDR1;
}

void USART_Init(int baud)
{
    USART_SetBaud(baud);
}

void USART_SetBaud(int baud)
{
    unsigned int pres = BAUD_PRESCALE_9600;

    UCSR1B = 0;  

    // Determine prescale value
    switch (baud) {
    case 9600:
        pres = BAUD_PRESCALE_9600;
        break;
    case 19200:
        pres = BAUD_PRESCALE_19200;
        break;
    case 38400:
        pres = BAUD_PRESCALE_38400;
        break;
    }   

    // Set baud rate
	/* Load upper 8-bits into the high byte of the UBRR register
    Default frame format is 8 data bits, no parity, 1 stop bit
	to change use UCSRC, see AVR datasheet*/ 
    UBRR1L = pres;// Load lower 8-bits into the low byte of the UBRR register
    UBRR1H = (pres >> 8);

    // Enable receiver and transmitter and receive complete interrupt 
    UCSR1B = ((1<<TXEN1)|(1<<RXEN1) | (1<<RXCIE1));   
}