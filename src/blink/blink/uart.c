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


int printCHAR(char character, FILE *stream)
{
	USART_SendByte(character);
	return 0;
}

// "FILE" descriptor for use with regular USART
FILE uart_str = FDEV_SETUP_STREAM(printCHAR, NULL, _FDEV_SETUP_RW);

void USART_Init(void){
   // Set baud rate
   UBRR1L = BAUD_PRESCALE;// Load lower 8-bits into the low byte of the UBRR register
   UBRR1H = (BAUD_PRESCALE >> 8); 
	/* Load upper 8-bits into the high byte of the UBRR register
    Default frame format is 8 data bits, no parity, 1 stop bit
	to change use UCSRC, see AVR datasheet*/ 

  // Enable receiver and transmitter and receive complete interrupt 
  UCSR1B = ((1<<TXEN1)|(1<<RXEN1) | (1<<RXCIE1));

  // Use our "FILE" descriptor for stdout, so `printCHAR" will be called whenever
  // putchar or printf have something to say.
  stdout = &uart_str; 
}
