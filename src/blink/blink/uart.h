#ifndef uartH
#define uartH

#include "settings.h"

#ifdef __cplusplus
extern "C" {
#endif

// Define baud rate
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

// Configures regular USART for 9600 baud
void USART_Init(void);

// Send out byte over regular USART
void USART_SendByte(uint8_t u8Data);

// Output function for standard libs
int printCHAR(char character, FILE *stream);

#ifdef __cplusplus
};
#endif

#endif
