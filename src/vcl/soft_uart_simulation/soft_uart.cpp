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

// bit detector
unsigned short ur_prev_t1(0);
unsigned char u_level(1);

// word detector/framing
unsigned short wd_sreg(0);
unsigned char wd_sync(255);
unsigned char wd_frame(0);

// string buf
unsigned char us_ptr(0);
char us_str[32];


void new_frame(unsigned short frame)
{
	char printable = frame;

	if (frame <32 )
		printable = '.';

	us_str[us_ptr] = printable;
	us_ptr++;
	us_str[us_ptr] = 0;

	if (us_ptr==30)
		us_ptr=0;

	printf("     (frame=%c (%02x))\n", printable,frame);
}

void new_bit(int state)
{
	wd_sreg >>= 1;
	if (state)
		wd_sreg |= 0x400;


	if (wd_sync == 255) {
		// not synchronized, wait for sync
		// Wait until stop bit and idle to start bit has been seen
		if ( (wd_sreg & 0x403) == 0x401) {
			printf("(sync)");
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
				printf("(error)");
				wd_sync=255;
			} else if ( (wd_sreg & 0x403) == 0x401) {
				// Frame detected
				wd_frame = (wd_sreg >> 2);
				new_frame(wd_frame);
				wd_sync=0;
			}
		}

		if (wd_sync >= 20) {
			printf("(lost)");
			wd_sync=0;
			state = 255;
		}

		wd_sync++;
	}
}

void isr_pin_change()
{
	unsigned short delta_t1 = t1 - ur_prev_t1;
	unsigned char u_new_level = (uart_pin!=0);
	unsigned short num_bits(0);

	// 1 bit takes 208 timer 1 ticks

	// Pulse duration less than half bit? Invalid!
	if (delta_t1 < 104)
		num_bits=0;
	else {
		// calculate number of full bit lengths
		num_bits = delta_t1 / 208;
		// allow last bit too be somewhat shorter than usual
		if ( (delta_t1 % 208) >= 104)
			num_bits++;
	}

	if (!num_bits) {
		// line changed too fast, or not at all
		wd_sync=255;
		return;
	}

	for (int i(0); i<num_bits; i++) {
		new_bit(u_level);
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
		extra_stop_bits = rand()%10;
		//extra_stop_bits=0;
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

    us_str[0]=0;

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
