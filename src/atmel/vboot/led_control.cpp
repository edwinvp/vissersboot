#include "settings.h"
#include "led_control.h"
#include <avr/io.h>
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