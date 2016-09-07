#include "settings.h"
#include "compass_calibrate.h"
#include "crc.h"
#include <math.h>
#include <stdio.h>
#include <avr/eeprom.h>

#define pi 3.1415926535897932384626433832795
#define two_pi 6.283185307179586476925286766559

void CCompassCalibration::init_min(comp_extreme & x)
{
	x.fin = 32767;
	for (int i(0);i<4;i++)
	x.avg[i]=0;
	x.cnt = 0;
};

void CCompassCalibration::init_max(comp_extreme & x)
{
	x.fin = -32768;
	for (int i(0);i<4;i++)
	x.avg[i]=0;
	x.cnt = 0;
};

void CCompassCalibration::init()
{
	compass_course_no_offset = 0.0f;
	compass_course = 0.0f;
	
	init_min(compass_min_x);
	init_max(compass_max_x);
	init_min(compass_min_y);
	init_max(compass_max_y);
	init_min(compass_min_z);
	init_max(compass_max_z);
}

int16_t CCompassCalibration::c_avg(comp_extreme & x)
{
	long int sum = 0;
	for (int i(0);i<4;i++)
	sum += x.avg[i];

	int16_t avg = sum >> 2;
	return avg;
}

void CCompassCalibration::update_min(comp_extreme & x, int16_t newval)
{
	if (newval == 0x7fff || newval == 0x8000 || newval == 0)
    	return;
	if (x.cnt > 3) {
		int16_t avg = c_avg(x);
		if (avg < x.fin)
		x.fin = avg;
		x.cnt=0;
	}
	x.avg[x.cnt++] = newval;
}

void CCompassCalibration::update_max(comp_extreme & x, int16_t newval)
{
	if (newval == 0x7fff || newval == 0x8000 || newval == 0)
	    return;
	if (x.cnt > 3) {
		int16_t avg = c_avg(x);
		if (avg > x.fin)
		x.fin = avg;
		x.cnt=0;
	}
	x.avg[x.cnt++] = newval;
}

void CCompassCalibration::calibrate(const TCompassTriple & compass_raw)
{
	update_min(compass_min_x,compass_raw.x);
	update_max(compass_max_x,compass_raw.x);
	update_min(compass_min_y,compass_raw.y);
	update_max(compass_max_y,compass_raw.y);
	update_min(compass_min_z,compass_raw.z);
	update_max(compass_max_z,compass_raw.z);
}

float CCompassCalibration::clip_degrees(float d)
{
	d = fmod(d,360.0f);

	if (d < 0)
	d += 360.0;

	return d;
}

float CCompassCalibration::coords_to_angle(float ix, float iz)
{
	float d(0.0f);
	if (ix>0)
		d = atan(iz/ix);
	else if (ix<0 && iz >=0)
		d = atan(iz/ix) + pi;
	else if (ix<0 && iz < 0)
		d = atan(iz/ix) - pi;
	else if (ix==0 && iz > 0)
		d = pi / 2.0;
	else if (ix==0 && iz < 0)
		d = -pi / 2.0;
	else if (ix==0 && iz == 0)
		d = 0;

	d = 90.0 + d / two_pi * 360.0;
	d += 180.0;
	d = clip_degrees(d);
	return d;
}

float CCompassCalibration::calc_course(const TCompassTriple & compass_raw)
{
	compass_course_no_offset = 0.0f;
	
	TCompassTriple centered;
	centered.x=0;
	centered.y=0;
	centered.z=0;

	float w = compass_max_x.fin - compass_min_x.fin;
	float h = compass_max_z.fin - compass_min_z.fin;

	if (w>=0 && h>=0) {
		centered.x = (compass_raw.x - compass_min_x.fin - (w/2.0));
		centered.z = -(compass_raw.z - compass_min_z.fin - (h/2.0));
	}

	if (w>0 && h>0) {
		float ix = (float)centered.x / (w/2.0);
		float iz = (float)centered.z / (h/2.0);

		compass_course_no_offset = coords_to_angle(ix,iz);
	}

	compass_course = clip_degrees(compass_course_no_offset + compass_north_offset);

	return compass_course;
}

void CCompassCalibration::set_north()
{
	compass_north_offset = 0 - compass_course_no_offset;
}

void CCompassCalibration::store_calibration()
{
    b_printf("Storing compass calibration to EEPROM.\r\n");
    
    // Put data in array of `raw` words
    uint16_t rec[8];
    rec[0]=(uint16_t)compass_north_offset;
    rec[1]=compass_min_x.fin;
    rec[2]=compass_max_x.fin;
    rec[3]=compass_min_y.fin;
    rec[4]=compass_max_y.fin;
    rec[5]=compass_min_z.fin;
    rec[6]=compass_max_z.fin;
    // Calculate checksum over those 7 words
    rec[7]=crc16((unsigned char*)rec,14);
    
    // Write data as words to EEPROM
    unsigned int addr=COMPASS_EEPROM_OFFSET;
    for (int i(0); i<8; i++) {
        eeprom_busy_wait();
        eeprom_write_word((uint16_t*)addr,rec[i]);
        addr+=2;
    }
}
// ----------------------------------------------------------------------------
void CCompassCalibration::load_calibration()
{
    b_printf("Loading compass calibration from EEPROM...");

    // Array to receive 'raw' words from EEPROM    
    uint16_t rec[8];

    // Read data as words from EEPROM    
    unsigned int addr=COMPASS_EEPROM_OFFSET;
    for (int i(0); i<8; i++) {
        eeprom_busy_wait();
        rec[i]=eeprom_read_word((uint16_t*)addr);
        addr+=2;       
    }

    // Calculate checksum over what has just been read
    uint16_t chk = crc16((unsigned char*)rec,14);
    
    // Compare checksum with the one that was stored
    if (chk == rec[7]) {
        // They were the same, data is okay.
        // Now apply that calibration stored earlier:
        reset_compass_calibration();
        
        compass_north_offset=rec[0];
        compass_min_x.fin=rec[1];
        compass_max_x.fin=rec[2];
        compass_min_y.fin=rec[3];
        compass_max_y.fin=rec[4];
        compass_min_z.fin=rec[5];
        compass_max_z.fin=rec[6];
        
        b_printf("OK\r\n");
        
    } else {
        // Data is corrupted somehow (or was never stored before).
        b_printf("FAILED (checksum)\r\n");
    }    
}
// ----------------------------------------------------------------------------
void CCompassCalibration::reset_compass_calibration()
{
    compass_course = 0.0f;
    
    calibration_mode=false;
    init();
}
// ----------------------------------------------------------------------------
void CCompassCalibration::set_true_north()
{
    // Set whatever the boat is pointing to now as 'true north'.
    b_printf("Setting true north.\r\n");
    set_north();
    
    // Then store compass calibration.
    store_calibration();
}
// ----------------------------------------------------------------------------
void CCompassCalibration::toggle_calibration_mode()
{
    if (calibration_mode)
        calibration_mode=false;
    else
        calibration_mode=true;

    b_printf("Calibration mode: ");
    if (calibration_mode)
        b_printf("ON\r\n");
    else
        b_printf("OFF\r\n");
}   
// ----------------------------------------------------------------------------
