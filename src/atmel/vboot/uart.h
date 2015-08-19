#ifndef uartH
#define uartH

#include "settings.h"

#ifdef __cplusplus
extern "C" {
#endif

// Define baud rate
//#define USART_BAUDRATE 38400
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

void USART_Init(void);
void USART_SendByte(uint8_t u8Data);
int printCHAR(char character, FILE *stream);

#ifdef __cplusplus
};
#endif

#endif