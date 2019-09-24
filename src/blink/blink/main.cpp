/*
 * blink.cpp
 *
 * Created: 20-9-2019 21:30:13
 * Author : Z
 */ 

#define FTDI_SIO_GET_LATENCY_TIMER	10 
#define  FTDI_SIO_GET_LATENCY_TIMER_REQUEST FTDI_SIO_GET_LATENCY_TIMER

#define F_CPU 16000000

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "usb.h"

static unsigned int num_gen_int = 0;
static unsigned int num_com_int = 0;
static unsigned char d = 0;

static
uint8_t ctrl_write_PM(const void *addr, uint16_t len);

#define set_bit(REG, BIT) REG |= _BV(BIT)
#define clear_bit(REG, BIT) REG &= ~_BV(BIT)
#define toggle_bit(REG, BIT) REG ^= _BV(BIT)
#define assign_bit(REG, BIT, VAL) do{if(VAL) set_bit(REG,BIT) else clear_bit(REG,BIT);}while(0)

#define EP0_SIZE 64

#define EP_select(N) do{UENUM = (N)&0x07;}while(0)
#define EP_read8() (UEDATX)
#define EP_read16_le() ({uint16_t L, H; L=UEDATX; H=UEDATX; (H<<8)|L;})
#define EP_write8(V) do{UEDATX = (V);}while(0)
#define EP_write16_le(V) do{UEDATX=(V)&0xff;UEDATX=((V)>>8)&0xff;}while(0)

	
ISR(WDT_vect)
{
	
}

ISR(USART1_RX_vect)
{
	d=UDR1;
}

void oops(int a, char * v)
{	
	if (!a) {
		printf_P(PSTR("oops! %s"),v);
		while (1)
			;
	}
}

void put_hex(unsigned int i)
{	
	printf_P(PSTR("%04x"),i);
}


/* The control request currently being processed */

static usb_header head;

/* USB descriptors, stored in flash */

static const usb_std_device_desc PROGMEM devdesc = {
    sizeof(devdesc),
    usb_desc_device,
    0x0110, // 0x0110 for USB v1.1, 0x0200 for USB v2.0
    0x00, /* vendor specific */
    0x00, /* vendor specific */
    0x00, /* vendor specific */
    EP0_SIZE, /* EP 0 size 64 bytes */
    0x0403, // Future Technology Devices International Limited
    0x6001, // FT232
    0x0400,
	1, // iManu
    2,
    0,
    1
};

struct tdevconf
{
usb_std_config_desc conf;
usb_std_iface_desc iface;
};

static const PROGMEM tdevconf devconf {
	
	{
        sizeof(usb_std_config_desc),
        usb_desc_config,
        sizeof(devconf),
        1,
        1,
		0, 
        0x80, // bus powered
        20/2  // 20 mA		
	},	
	{
        sizeof(usb_std_iface_desc),
        usb_desc_iface,
        0,
        0,
        0,
        0xff, // vender specific
        0xff, // vender specific
        0xff, // vender specific
		0
	}
};

static const usb_std_string_desc iLang PROGMEM = {
	sizeof(usb_std_string_desc) + 2,
	usb_desc_string,
    L"\x0409"
};

#define DESCSTR(STR) { \
 sizeof(usb_std_string_desc) + sizeof(STR)-2, \
	usb_desc_string, \
	STR \
}

static const usb_std_string_desc iProd PROGMEM = DESCSTR(L"simpleusb");
static const usb_std_string_desc iSerial PROGMEM = DESCSTR(L"42");

#undef DESCSTR

/* Handle the standard Get Descriptor request.
 * Return 1 on success
 */
static
uint8_t USB_get_desc(void)
{
    const void *addr;
    uint8_t len, idx = head.wValue;
    switch(head.wValue>>8)
    {
    case usb_desc_device:
        if(idx!=0) return 0;
        addr = &devdesc; len = sizeof(devdesc);
        break;
    case usb_desc_config:
        if(idx!=0) return 0;
        addr = &devconf; len = sizeof(devconf);
        break;
    case usb_desc_string:
        switch(idx)
        {
        case 0: addr = &iLang; break;
        case 1: addr = &iProd; break;
        case 2: addr = &iSerial; break;
        default: return 0;
        }
        /* the first byte of any descriptor is it's length in bytes */
        len = pgm_read_byte(addr);
        break;
    default:
        return 0;
    }

    if(len>head.wLength)
        len = head.wLength;

    return !ctrl_write_PM(addr, len);
}

static void setupEP0(void);

static uint16_t userval; /* user register */

static inline void setupusb(void)
{
    /* disable USB interrupts and clear any active */
    UDIEN = 0;
    UDINT = 0;

    set_bit(UDCON, DETACH); /* redundant? */

    /* toggle USB reset */
    clear_bit(USBCON, USBE);
    set_bit(USBCON, USBE);

    /* No required.
     * Gives some time to start reprograming
     * if previous program gets stuck right away
     */
    _delay_ms(1000);
    putchar('.');

    /* Unfreeze */
    clear_bit(USBCON, FRZCLK);

    /* setup PLL for 8 MHz system clock */
    PLLCSR = 0;
    set_bit(PLLCSR, PLLE);
    loop_until_bit_is_set(PLLCSR, PLOCK);
    putchar('.');

    setupEP0(); /* configure control EP */
    putchar('.');

#ifdef HANDLE_SUSPEND
    set_bit(UDIEN, SUSPE);
#endif
    set_bit(UDIEN, EORSTE);

    /* allow host to un-stick us.
     * Warning: Don't use w/ DETACH on CPU start
     *          or a reset loop will result
     */
    //set_bit(UDCON, RSTCPU);
    clear_bit(UDCON, DETACH);
}

/* Setup the control endpoint. (may be called from ISR) */
static void setupEP0(void)
{
    /* EPs assumed to be configured in increasing order */

    EP_select(0);

    /* un-configure EP 0 */
    clear_bit(UECONX, EPEN);
    clear_bit(UECFG1X, ALLOC);

    /* configure EP 0 */
    set_bit(UECONX, EPEN);
    UECFG0X = 0; /* CONTROL */
    UECFG1X = 0b00110010; /* EPSIZE=64B, 1 bank, ALLOC */
#if EP0_SIZE!=64
#  error EP0 size mismatch
#endif

    if(bit_is_clear(UESTA0X, CFGOK)) {
        putchar('!');
        while(1) {} /* oops */
    }
}

ISR(USB_GEN_vect, ISR_BLOCK)
{
    uint8_t status = UDINT, ack = 0;
    putchar('I');
#ifdef HANDLE_SUSPEND
    if(bit_is_set(status, SUSPI))
    {
        ack |= _BV(SUSPI);
        /* USB Suspend */

        /* prepare for wakeup */
        clear_bit(UDIEN, SUSPE);
        set_bit(UDIEN, WAKEUPE);

        set_bit(USBCON, FRZCLK); /* freeze */
    }
    if(bit_is_set(status, WAKEUPI))
    {
        ack |= _BV(WAKEUPI);
        /* USB wakeup */
        clear_bit(USBCON, FRZCLK); /* freeze */

        clear_bit(UDIEN, WAKEUPE);
        set_bit(UDIEN, SUSPE);
    }
#endif
    if(bit_is_set(status, EORSTI))
    {
        ack |= _BV(EORSTI);
        /* coming out of USB reset */

#ifdef HANDLE_SUSPEND
        clear_bit(UDIEN, SUSPE);
        set_bit(UDIEN, WAKEUPE);
#endif

        putchar('E');
        setupEP0();
    }
    /* ack. all active interrupts (write 0)
     * (write 1 has no effect)
     */
    UDINT = ~ack;
}

/* write value from flash to EP0 */
static
uint8_t ctrl_write_PM(const void *addr, uint16_t len)
{
    while(len) {
        uint8_t ntx = EP0_SIZE,
                bsize = UEBCLX,
                epintreg = UEINTX;

        oops(ntx>=bsize, "EP"); /* EP0_SIZE is wrong */

        ntx -= bsize;
        if(ntx>len)
            ntx = len;

        if(bit_is_set(epintreg, RXSTPI))
            return 1; /* another SETUP has started, abort this one */
        if(bit_is_set(epintreg, RXOUTI))
            break; /* stop early? (len computed improperly?) */

        /* Retry until can send */
        if(bit_is_clear(epintreg, TXINI))
            continue;
        oops(ntx>0, "Ep"); /* EP0_SIZE is wrong (or logic error?) */

        len -= ntx;

        while(ntx) {
            uint8_t val = pgm_read_byte(addr);
            EP_write8(val);
            addr++;
            ntx--;
        }

        clear_bit(UEINTX, TXINI);
    }
    return 0;
}

/* Handle standard Set Address request */
static
void USB_set_address(void)
{
    UDADDR = head.wValue&0x7f;

    clear_bit(UEINTX, TXINI); /* send 0 length reply */

    loop_until_bit_is_set(UEINTX, TXINI); /* wait until sent */

    UDADDR = _BV(ADDEN) | (head.wValue&0x7f);

    clear_bit(UEINTX, TXINI); /* magic packet? */
}

static
uint8_t USB_config;

static
void handle_CONTROL(void)
{
    uint8_t ok = 0;
    /* SETUP message */
    head.bmReqType = EP_read8();
    head.bReq = EP_read8();
    head.wValue = EP_read16_le();
    head.wIndex = EP_read16_le();
    head.wLength = EP_read16_le();
	
	int irt(head.bmReqType);
	int ir(head.bReq);
	printf_P(PSTR("ctrl: rt=%04x r=%04x\r\n"),irt,ir);

    /* ack. first stage of CONTROL.
     * Clears buffer for IN/OUT data
     */
    clear_bit(UEINTX, RXSTPI);

    /* despite what the figure in
     * 21.12.2 (Control Read) would suggest,
     * SW should not clear TXINI here
     * as doing so will send a zero length
     * response.
     */

    switch(head.bReq)
    {
    case usb_req_set_feature:
    case usb_req_clear_feature:
        /* No features to handle.
         * We ignore Remote wakeup,
         * and EP0 will never be Halted
         */
        ok = 1;
        break;
    case usb_req_get_status:
        switch(head.bmReqType) {
        case 0b10000000:
        case 0b10000001:
        case 0b10000010:
            /* alway status 0 */
            loop_until_bit_is_set(UEINTX, TXINI);
            EP_write16_le(0);
            clear_bit(UEINTX, TXINI);
            ok = 1;
        }
        break;
    case usb_req_set_address:
        if(head.bmReqType==0) {
            USB_set_address();

            putchar('A');
            return;
        }
        break;
    case usb_req_get_desc:
        if(head.bmReqType==0x80) {
            ok = USB_get_desc();
        }
        break;
    case usb_req_set_config:
        if(head.bmReqType==0) {
            USB_config = head.wValue;
            ok = 1;
        }
        break;
    case usb_req_get_config:
        if(head.bmReqType==0x80) {
            loop_until_bit_is_set(UEINTX, TXINI);
            EP_write8(USB_config);
            clear_bit(UEINTX, TXINI);
            ok = 1;
        }
        break;
    case usb_req_set_iface:
		break;
    case usb_req_get_iface: // FTDI_SIO_GET_LATENCY_TIMER_REQUEST
		printf_P(PSTR("getlat\r\n"));

		break;
    case usb_req_set_desc:
		break;
    case usb_req_synch_frame:
        break;


    /* our (vendor specific) operations */
	/*
    case 0x7f:
        if(head.bmReqType==0b01000010 && head.wLength>=2) {
            // Control Write H2D
            loop_until_bit_is_set(UEINTX, RXOUTI);
            userval = EP_read16_le();
            clear_bit(UEINTX, RXOUTI);
            ok = 1;
        } else if(head.bmReqType==0b11000010 && head.wLength>=2) {
            // Control Read D2H
            loop_until_bit_is_set(UEINTX, TXINI);
            EP_write16_le(userval);
            clear_bit(UEINTX, TXINI);
            ok = 1;
        }
        break;
	*/
	
    default:
        putchar('?');
        put_hex(head.bmReqType);
        put_hex(head.bReq);
        put_hex(head.wLength>>8);
        put_hex(head.wLength);
    }

    if(ok) {
        if(head.bmReqType&ReqType_DirD2H) {
            /* Control read.
             * Wait for, and complete, status
             */
            uint8_t sts;
            while(!((sts=UEINTX)&(_BV(RXSTPI)|_BV(RXOUTI)))) {}
            //loop_until_bit_is_set(UEINTX, RXOUTI);
            ok = (sts & _BV(RXOUTI));
            if(!ok) {
                set_bit(UECONX, STALLRQ);
                putchar('S');
            } else {
                clear_bit(UEINTX, RXOUTI);
                clear_bit(UEINTX, TXINI);
            }
        } else {
            /* Control write.
             * indicate completion
             */
            clear_bit(UEINTX, TXINI);
        }
        putchar('C');

    } else {
        /* fail un-handled SETUP */
        set_bit(UECONX, STALLRQ);
        putchar('F');
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


enum ustate{usDisconnected, usDone};

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
						
					us = usDone;
				}
				break;
					
			case usDone:
				break;				
		}

		if ((UEINTX & (1 << RXSTPI)))
			handle_CONTROL();

			
    }
}

