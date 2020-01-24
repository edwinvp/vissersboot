#include <stdio.h>
#include "redirector.h"
#include "uart.h"

int printCHAR(char character, FILE *stream)
{
	USART_SendByte(character);
	return 0;
}

// "FILE" descriptor for use with regular USART
FILE uart_str = FDEV_SETUP_STREAM(printCHAR, NULL, _FDEV_SETUP_RW);

void redirect_std_out()
{
	// Use our "FILE" descriptor for stdout, so `printCHAR" will be called whenever
	// putchar or printf have something to say.
	stdout = &uart_str;
}
