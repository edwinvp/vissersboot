/*
 * blink.cpp
 *
 * Created: 20-9-2019 21:30:13
 * Author : Z
 */ 

#define F_CPU 16000000

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "uart.h"

static unsigned int num_gen_int = 0;
static unsigned int num_com_int = 0;
static unsigned char d = 0;

ISR(WDT_vect)
{
	
}

ISR(USART1_RX_vect)
{
	d=UDR1;
}

ISR(USB_GEN_vect)
{
	num_gen_int++;
	
	if(UDINT&(1<<EORSTI)){//if the end of reset interrupt flag is set
		putchar('$');

		//configure endpoint 0 to recieve the initial setup packets
		UENUM = (UENUM & 0xF8) | 0;   // select endpoint 0
		UECONX |= (1 << EPEN);    // enable endpoint 0
		UECFG0X = 0;   // endpoint 0 is a control endpoint
		UECFG1X = (1 << EPSIZE1) | (1 << EPSIZE0) | (1 << ALLOC);    // endpoint 0: 64 bytes, one bank, allocate mem
		
		UEINTX = 0;
		// check configuration
		if (!(UESTA0X & (1 << CFGOK))) {
			//error
		}
		
		UENUM = (UENUM & 0xF8) | 0;   // select endpoint 0 and you are ready to go
		
		// Clear USB reset int flag
		UDINT &= ~(1<<EORSTI);
	}
}

ISR(USB_COM_vect)
{
	putchar('%');

	num_com_int++;
}

void setup_usb()
{
	// Start with interrupts disabled
	cli();

	// disable USB general interrupts
	USBCON &= 0b11111110;
	// disable all USB device interrupts
	UDIEN &= 0b10000010;
	// disable USB endpoint interrupts
	UEIENX &= 0b00100000;

	// Re-enable interrupts
	sei();

	// Enable USB pad regulator
	UHWCON |= (1<<UVREGE);

	// Configure PLL
	PLLCSR = 0;
	// Set PINDIV because we are using 16 MHz crystal
	PLLCSR |= (1<<PINDIV);
	// Configure 96MHz PLL output (is then divided by 2 to get 48 MHz USB clock)
	PLLFRQ = (1<<PDIV3) | (1<<PDIV1) | (1<<PLLUSB) | (1 << PLLTM0);
	// Enable PLL
	PLLCSR |= (1<<PLLE);
	
	// Wait for PLL lock
	while (!(PLLCSR & (1<<PLOCK)))
		;
			
	// Enable USB
	USBCON |= (1<<USBE)|(1<<OTGPADE);
	// Clear freeze clock bit
	USBCON &= ~(1<<FRZCLK);
	
	// configure USB interface (speed, endpoints, etc.)
	UDCON &= ~(1 << LSM);     // full speed 12 Mbit/s
	
	/*
	// configure endpoint 0
	UENUM = (UENUM & 0xF8) | 0;   // select endpoint 0
	UECONX |= (1 << EPEN);    // enable endpoint 0
	UECFG0X = 0;   // endpoint 0 is a control endpoint
	UECFG1X = (1 << EPSIZE1) | (1 << EPSIZE0) | (1 << ALLOC);    // endpoint 0: 64 bytes, one bank, allocate mem
	UEINTX = 0;
	// check configuration
	if (!(UESTA0X & (1 << CFGOK))) {
		//status = STATUS_INVALID_CONFIGURATION_1;
		putchar('?');
		return;
	}
	// disable rest of endpoints
	for (uint8_t i = 1; i <= 6; i++) {
		UENUM = (UENUM & 0xF8) | i;   // select endpoint i
		UECONX &= ~(1 << EPEN);
	}
	*/
	
	// disable rest of endpoints
	for (uint8_t i = 1; i <= 6; i++) {
		UENUM = (UENUM & 0xF8) | i;   // select endpoint i
		UECONX &= ~(1 << EPEN);
	}


/*	
Device Descriptor:
bcdUSB:             0x0110
bDeviceClass:         0x00
bDeviceSubClass:      0x00
bDeviceProtocol:      0x00
bMaxPacketSize0:      0x08 (8)
idVendor:           0x0403 (Future Technology Devices International Limited)
idProduct:          0x6001
bcdDevice:          0x0400
iManufacturer:        0x01
iProduct:             0x02
iSerialNumber:        0x00
bNumConfigurations:   0x01

ConnectionStatus: DeviceConnected
Current Config Value: 0x01
Device Bus Speed:     Full
Device Address:       0x01
Open Pipes:              2

Endpoint Descriptor:
bEndpointAddress:     0x81
Transfer Type:        Bulk
wMaxPacketSize:     0x0040 (64)
bInterval:            0x00

Endpoint Descriptor:
bEndpointAddress:     0x02
Transfer Type:        Bulk
wMaxPacketSize:     0x0040 (64)
bInterval:            0x00
*/
	
	
}


enum ustate{usDisconnected, usConnected, usWaitSetup, usSetup, usDone};

int main(void)
{
	// Enable interrupts
	USBCON=0;

	sei();
	
	DDRC = 0x80;
	
	USART_Init();

	printf_P(PSTR("reboot!\r\n"));

	setup_usb();
	
	printf_P(PSTR("mloop\r\n"));
	
	
	int i=0;

	ustate us(usDisconnected);
	
    /* Replace with your application code */
    while (1) 
    {
			++i;
			PORTC = 0x80;
			
			_delay_ms(50);
			
			if ((i%10)==0)
				printf_P(PSTR("gi=%04d ci=%04d\r\n"),num_gen_int,num_com_int);
			
			PORTC = 0x00;
			_delay_ms(50);

			switch (us) {
			case usDisconnected:				
				if ((USBSTA & (1 << VBUS))) {
					printf_P(PSTR("Vbus\r\n"));
					// connected
					UDCON &= ~(1 << DETACH);
				
					//end of reset interrupts
					UDIEN |= (1<<EORSTE);//enable the end of reset interrupt
						
					us = usWaitSetup;
				}
				break;
					
			case usWaitSetup:
				// wait setup packet
				UENUM = (UENUM & 0xF8) | 0;   // select endpoint 0

				if ((UEINTX & (1 << RXSTPI))) {

					printf_P(PSTR("setup\r\n"));
					
					// Clear RXSTPI
					UEINTX &= ~(1 << RXSTPI);
					us = usSetup;
				}								
				break;
			
			case usSetup:
			
				if ((UEINTX & (1 << RXOUTI))) {
					printf_P(PSTR("out\r\n"));

					// Clear RXOUTI
					UEINTX &= ~(1 << RXOUTI);
					
					us = usDone;
				}
		
				break;
				
			case usDone:
				break;
				
		}

			
    }
}

