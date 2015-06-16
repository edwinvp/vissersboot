#include "settings.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Target: ATmega328
//#include <avr/iom328.h>
#include <avr/common.h>
#include <stdio.h>

enum {
	BLINK_DELAY_MS = 25,
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


volatile unsigned char old_pinb = 0;
volatile unsigned int icnt;
volatile unsigned int ur_prev_t1;
volatile unsigned char u_level=0, wd_sync=255;
volatile unsigned short wd_sreg=0;

volatile unsigned char us_str[64];
volatile unsigned char us_ptr=0;

void new_frame(unsigned short frame)
{
	char printable = frame;

	if (frame<32)
		printable = '.';

	if (printable == '$')
		us_ptr=0;

	us_str[us_ptr] = printable;
	us_ptr++;
	us_str[us_ptr] = 0;

	if (us_ptr==62)
		us_ptr=0;

	//printf("     (frame=%c (%02x))\n", printable,frame);
}


void new_bit(int state)
{
	unsigned short wd_frame;
	
	wd_sreg >>= 1;
	if (state)
		wd_sreg |= 0x400;

	if (wd_sync == 255) {
		// not synchronized, wait for sync
		// Wait until stop bit and idle to start bit has been seen
		if ( (wd_sreg & 0x403) == 0x401) {
			//printf("(sync)");
			wd_frame = (wd_sreg >> 2);
			new_frame(wd_frame);
			wd_sync=0;
		}

	} else {

		if (wd_sync >= 9) {
			// By this time, we could have received a frame,
			// check for stop bit and start condition
			if ( (wd_sreg & 0x403) == 0x001) {
				// Somehow didn't get a stop bit
				//printf("(error)");
				wd_sync=255;
			} else if ( (wd_sreg & 0x403) == 0x401) {
				// Frame detected
				wd_frame = (wd_sreg >> 2);
				new_frame(wd_frame);
				wd_sync=0;
			}
		}

		if (wd_sync >= 20) {
			//printf("(lost)");
			wd_sync=0;
			state = 255;
		}

		wd_sync++;
	}

}

ISR(PCINT0_vect)
{
	unsigned short delta_t1,t1;
	unsigned char u_new_level;
	unsigned short num_bits=0,i=0;
	
	t1 = TCNT1;
	
	if ((PINB & _BV(PINB0)) ^ (old_pinb & _BV(PINB0))) {
		
		delta_t1 = t1 - ur_prev_t1;

		u_new_level = (PINB & _BV(PINB0))!=0;
	
		// Pulse duration less than half bit? Invalid!
		if (delta_t1 < 104)
			num_bits=0;
		else {
			// calculate number of full bit lengths
			num_bits=0;
			while (delta_t1>208) {
				delta_t1 -= 208;
				num_bits++;
			}	
			//num_bits = delta_t1 / 208;

			// allow last bit too be somewhat shorter than usual
			if (delta_t1 >= 156)
				num_bits++;
		}

		if (num_bits) {

			for (i=0; i<num_bits; i++) {
    			new_bit(u_level);
			}

		} else {
			// line changed too fast, or not at all
			wd_sync=255;
		}

		
		u_level = u_new_level;
		ur_prev_t1 = t1;
	
	}
		
	
	old_pinb = PINB;
}

ISR(PCINT1_vect)
{

}

volatile unsigned int pd6_rising;
volatile unsigned int pd6_pulse_duration;

volatile unsigned int pd5_rising;
volatile unsigned int pd5_pulse_duration;


volatile unsigned char old_pind = 0;

ISR(TIMER1_OVF_vect)
{
		//
}


ISR(PCINT2_vect)
{
	unsigned int tmr_reg;

	++icnt;
		
	tmr_reg = TCNT1;

	
	if ((PIND & _BV(PIND6)) ^ (old_pind & _BV(PIND6))) {				
		if (PIND & _BV(PIND6)) 
			pd6_rising = tmr_reg;
		else {
			pd6_pulse_duration = tmr_reg - pd6_rising;	
			if (pd6_pulse_duration > 10000)
				pd6_pulse_duration -= 25536;
		}			
	}
	
	if ((PIND & _BV(PIND5)) ^ (old_pind & _BV(PIND5))) {
		if (PIND & _BV(PIND5))
			pd5_rising = tmr_reg;
		else {
			pd5_pulse_duration = tmr_reg - pd5_rising;
			if (pd5_pulse_duration > 10000)
				pd5_pulse_duration -= 25536;			
		}
	}
	
	
	old_pind = PIND;
}

void setup_gps_input()
{
	us_str[0]=0;
	
	// Configure PB0 as input
	DDRB &= ~_BV(DDB0);
	PORTB &= ~_BV(PORTB0);
	
	// Enable on-pin-change for pin
	PCMSK0 |= _BV(PCINT0);
	
	// Configure interrupt on logical state state on PB0 (so PCIE0)
	PCICR |= _BV(PCIE0);	
}

void setup_capture_inputs()
{
	// Configure PD6 as input
	DDRD &= ~_BV(DDD6);	
	PORTD &= ~_BV(PORTD6);	
	// Configure PD5 as input
	DDRD &= ~_BV(DDD5);
	PORTD &= ~_BV(PORTD5);
	
	// Enable on-pin-change for pins
	PCMSK2 |= _BV(PCINT22); // PD6
	PCMSK2 |= _BV(PCINT21); // PD5
	
	// Configure interrupt on logical state state on PD6 (so PCIE2)
	PCICR |= _BV(PCIE2);
	
	// Configure timer 1 clock source
	// Set to use a clock source of clkio / 8.
	// So 20 [ms] servo period will be 40000 timer ticks.	
	TCCR1B &= ~_BV(CS12);
	TCCR1B |= _BV(CS11);
	TCCR1B &= ~_BV(CS10);
	
	// Enable timer 1 overflow interrupt
	TIMSK1 |= _BV(TOIE1);
}

void setup_pwm()
{
	// Configure PB1 & PB2 as outputs
	DDRB |= _BV(DDB1);
	DDRB |= _BV(DDB2);
	

	// Set compare output mode to non-inverting (PWM) mode
	TCCR1A |= _BV(COM1A1);
	TCCR1A &= ~_BV(COM1A0);
	TCCR1A |= _BV(COM1B1);
	TCCR1A &= ~_BV(COM1B0);
	
	// Set mode 14 (Fast PWM, TOP=ICR1)
	// WGM13 WGM12 WGM11 WGM10
	// 1	 1	   1	 0
		
	TCCR1B |= _BV(WGM13);
	TCCR1B |= _BV(WGM12);	
	TCCR1A |= _BV(WGM11);
	TCCR1A &= ~(_BV(WGM10));
	
	OCR1A = 3000;
	OCR1B = 2000;
	
	ICR1 = 40000; // gives a 20 [ms] period
	
}

int main (void)
{
	/* set pin 5 of PORTB for output*/
	DDRB |= _BV(DDB5);
	
	USART_Init();  // Initialize USART
	sei();         // enable all interrupts	
	
	setup_capture_inputs();
	setup_pwm();	
	setup_gps_input();
	
	while(1) {

		// Pass through setpoints
		OCR1A = pd5_pulse_duration;
		OCR1B = pd6_pulse_duration;


		/* set pin 5 high to turn led on */
		PORTB |= _BV(PORTB5);
		_delay_ms(BLINK_DELAY_MS);

		printf(" icnt=%05d pd6=%05d pd5=%05d \r\n", 
			icnt, pd6_pulse_duration, pd5_pulse_duration );
		
		printf(" us_str=%s\r\n", us_str);
		
		
		//USART_SendByte(value);  // send value

		/* set pin 5 low to turn led off */
		PORTB &= ~_BV(PORTB5);
		_delay_ms(BLINK_DELAY_MS);
		
	}
	
	return 0;
}
