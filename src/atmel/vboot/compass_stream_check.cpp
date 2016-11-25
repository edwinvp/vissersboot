#include "compass_stream_check.h"

#include <stdio.h>
#ifndef _WIN32
#include <avr/io.h>
#else
#include "fakeio.h"
#endif
#include "settings.h"

CCompassStreamCheck::CCompassStreamCheck() :
	compass_same_counter(0),
	compass_diff_counter(0),
	compass_sends_values(false)
{
}

bool CCompassStreamCheck::check(const TCompassTriple & compass_raw)
{
	if (compass_raw_old.equals(compass_raw)) {
		if (compass_same_counter < 50)
			compass_same_counter++;
		else {
			if (compass_sends_values) {
				b_printf("Compass DOWN\r\n");
				compass_diff_counter=0;
				compass_sends_values=false;
			}
		}
	} else {
		if (compass_diff_counter < 10)
			compass_diff_counter++;
		else {
			compass_same_counter=0;
			if (!compass_sends_values) {
				b_printf("Compass UP\r\n");
				compass_sends_values=true;
			}
		}
	}

	compass_raw_old = compass_raw;

	return compass_sends_values;
}

