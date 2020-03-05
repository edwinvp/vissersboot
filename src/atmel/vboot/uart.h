#ifndef uartH
#define uartH

#include "settings.h"
#ifdef _WIN32
#include "fakeio.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Define baud rate
#define BAUD_PRESCALE_9600 (((F_CPU / (9600UL * 16UL))) - 1)
#define BAUD_PRESCALE_19200 (((F_CPU / (19200UL * 16UL))) - 1)
#define BAUD_PRESCALE_38400 (((F_CPU / (38400UL * 16UL))) - 1)

// Configures regular USART for 9600 baud
void USART_Init(int baud);

// Send out byte over regular USART
void USART_SendByte(uint8_t u8Data);

// Output function for standard libs
int printCHAR(char character, FILE *stream);

void USART_SetBaud(int baud);

#ifdef __cplusplus
};
#endif

#endif