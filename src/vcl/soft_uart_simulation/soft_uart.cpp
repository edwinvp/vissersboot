#include <vcl.h>
#include <conio.h>
#include <windows.h>

#pragma hdrstop
#pragma argsused

#include <tchar.h>
#include <stdio.h>

UnicodeString msg =
	L"$GPGGA,170834,4124.8963,N,08151.6838,W,1,05,1.5,280.2,M,-34.0,M,,,*59\r\n";

int sender_str_pos(1);
int sender_bit_pos(0);
int extra_stop_bits(0);

unsigned short t1(0);

unsigned short sender_frame(0);

unsigned char uart_pin(1);

enum suart_state { u_idle, u_frame } us(u_idle);


unsigned short next_frame()
{
	unsigned short frm(0);
	frm = (unsigned short)msg[sender_str_pos];
	frm <<= 1;
	frm |= 0x200; // add stop bit

	sender_str_pos++;

	if (sender_str_pos > msg.Length())
		sender_str_pos=1;

	return frm;
}

unsigned short ur_prev_t1(0);
unsigned short ur_sreg(0);
unsigned short ur_msk(0);
unsigned char u_level(1);

void isr_pin_change()
{
	unsigned short delta_t1 = t1 - ur_prev_t1;

	unsigned char u_new_level = (uart_pin!=0);

	unsigned short num_bits(0);

#define BANDW 52
	if (delta_t1 < 104)
		num_bits=0;
	else if (delta_t1 > (208-BANDW) && delta_t1 < (208+BANDW) )
		num_bits=1;
	else if (delta_t1 > (416-BANDW) && delta_t1 < (416+BANDW) )
		num_bits=2;
	else if (delta_t1 > (624-BANDW) && delta_t1 < (624+BANDW) )
		num_bits=3;
	else if (delta_t1 > (832-BANDW) && delta_t1 < (832+BANDW) )
		num_bits=4;
	else if (delta_t1 > (1040-BANDW) && delta_t1 < (1040+BANDW) )
		num_bits=5;
	else if (delta_t1 > (1248-BANDW) && delta_t1 < (1248+BANDW) )
		num_bits=6;
	else if (delta_t1 > (1456-BANDW) && delta_t1 < (1456+BANDW) )
		num_bits=7;
	else if (delta_t1 > (1664-BANDW) && delta_t1 < (1664+BANDW) )
		num_bits=8;
	else if (delta_t1 > (1872-BANDW) && delta_t1 < (1872+BANDW) )
		num_bits=9;
	else
		num_bits=10;

	if (!num_bits || num_bits == 10) {
		// line changed too fast, or not at all
		us = u_idle;
		return;
	};

	switch (us) {
	case u_idle:
		// wait for start bit
		ur_sreg=0;

		if (num_bits >= 1 && (u_level==0)) {
			us = u_frame;

			printf("(S=%dx0) ",num_bits);

			if (num_bits==1)
				ur_msk=1;
			else
				ur_msk = 1 << (num_bits-1);

		}

		break;

	case u_frame:
		// receive bits

		if (!u_level)
			printf("(%dx0) ",num_bits);
		else
			printf("(%dx1) ",num_bits);

		while (num_bits--) {
			if (u_level!=0)
				ur_sreg |= ur_msk;

			if (ur_msk == 0x100) {

				char printable = '.';

				unsigned char code = ur_sreg & 0xff;

				if (code>=32 && code<128)
					printable=code;

				printf(" %04x %c\r\n",ur_sreg, printable);
				us = u_idle;
				break;
			} else
				ur_msk <<= 1;
		}


		break;
	}

	ur_prev_t1 = t1;

	u_level = u_new_level;
}

void isr_t1_overflow()
{
	//
}

void set_uart_signal_bit(int new_state)
{
	if ( (new_state!=0) != (uart_pin!=0) ) {
		uart_pin = (new_state!=0);
		isr_pin_change();
	}
}

void do_uart_bit()
{
	if (extra_stop_bits>0) {
		printf("e");
		set_uart_signal_bit(1);
		extra_stop_bits--;
	} else {
		if (sender_frame&1) {
			printf("1");
			set_uart_signal_bit(1);
		} else {
			printf("0");
			set_uart_signal_bit(0);
		}
		sender_frame >>= 1;
	}

	if (!sender_frame) {
		printf(" ");
		sender_frame = next_frame();
		extra_stop_bits = rand()%3;
	}

}

int _tmain(int argc, _TCHAR* argv[])
{

/*
	Timing
		There is a Timer 1 overflow each

		Timer 1 prescaler is 2 Mhz (~ 30,5 overflows each second)
		1 timer tick takes 0,5 [us]

		32,768 [ms] (65536 timer ticks)

		Each bit in RS-232 @ 9600 baud takes:
		 0,104 [ms] and that's 208 timer 1 ticks

		So 1 byte takes 1,041666666 [ms] and 2083 timer 1 ticks
*/

	unsigned long long clk(0), stop(2000000);


	unsigned short uart_presc(0);

	unsigned long long t1_overflows(0);
	unsigned long long clk_uart(0);

	sender_frame = next_frame();

	while (clk < stop && clk_uart < 200) {

		// Timer 1 counter and overflows
		t1++;
		if (!t1) {
			isr_t1_overflow();

			t1_overflows++;
		}

		// Simulated UART signal
		uart_presc++;
		if (uart_presc==208) {
			uart_presc=0;
			clk_uart++;
			do_uart_bit();
		}


		clk++;
	};

	printf("\r\n\r\nPress any key to continue...");
	getch();
	return 0;
}
