#include "settings.h"
#include "led_control.h"
#ifndef _WIN32
#include <avr/io.h>
#else
#include "fakeio.h"
#endif
#include "state_machine.h"

// LED output pin (currently PORTB pin 5)
#define LED_PIN PORTB5
#define LED_REG PORTB

CLedControl::CLedControl() : slow_blink_prev(false),
    fast_blink(false),
    led_signal(false),
    slow_blink(false),
    blink_times(0),
    dark_cnt(0),
    msk_ctr(0),
    msk_sreg(0),
    mode(lmOff)
{
    // No further initialization needed.
}

void CLedControl::toggle_slow_blink()
{
    slow_blink = !slow_blink;
}

void CLedControl::set_mode(TLedMode m)
{
    mode = m;
}

//!\brief Called at 100 [ms]
void CLedControl::update(bool gps_valid, bool arrived)
{
	fast_blink = !fast_blink;

    if (slow_blink)
        dark_cnt = 0;
    else
        dark_cnt++;

	switch (mode) {
    	/* in manual/auto mode, just show status of GPS receiver */
    	case lmGpsStatus:
    	    if (gps_valid) {
            	// Steady LED on GPS signal okay
            	led_signal = true;
            } else {
            	// Slowly blink LED when there is no GPS reception
        	    led_signal = slow_blink;
        	}
        	break;

    	case lmArriveStatus:
    	    led_signal = arrived;
    	    break;

        case lmFastBlink:
    	    led_signal = fast_blink;
    	    break;

        case lmSlowBlink:
    	    if (slow_blink_prev && !slow_blink) {
            	// We just blinked once
        	    if (blink_times > 0)
            	    blink_times--;
        	}

    	    if (blink_times > 0) {
            	led_signal = slow_blink;
    	    } else
        	    led_signal = false;

        	slow_blink_prev = slow_blink;
        	break;

        case lmOff:
        	led_signal = false;

        case lmCalibrationPhase1:
        case lmCalibrationPhase2:
        case lmCalibrationPhase3:
        case lmCalibrationPhase4:
            led_signal = (msk_sreg & 1)!=0;
            msk_sreg >>= 1;
            break;

    	default:
        	led_signal = false;
	}

	// Update LED output
	if (led_signal) {
    	// turn led on
    	LED_REG |= _BV(LED_PIN);
    	} else {
    	// turn led off
    	LED_REG &= ~_BV(LED_PIN);
	}

    if ((msk_ctr&7) == 0) {
        switch (mode) {
        case lmCalibrationPhase1:
			msk_sreg = 0x03; // 0b00000011;
			break;
		case lmCalibrationPhase2:
			msk_sreg = 0xf0; // 0b11110000;
			break;
		case lmCalibrationPhase3:
            msk_sreg = 0xcc; // 0b11001100;
            break;
        case lmCalibrationPhase4:
            msk_sreg = 0xff; // 0b11111111;
            break;
        }
    }

    msk_ctr++;
}

bool CLedControl::done_blinking()
{
    return blink_times == 0;
}

bool CLedControl::been_dark_for_a_while()
{
    return (dark_cnt > 2);
}

void CLedControl::blink_x_times(int n)
{
    blink_times = n;
}