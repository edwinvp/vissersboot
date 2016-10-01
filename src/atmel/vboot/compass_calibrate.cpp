#include "settings.h"
#include "compass_calibrate.h"
#include "crc.h"
#include <math.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#define pi 3.1415926535897932384626433832795
#define two_pi 6.283185307179586476925286766559

CCompassCalibration::CCompassCalibration()
{
    m_cal_state = csNotCalibrated;
    prev_quadrant=0;
    quadrant_counter=0;
    stable_quadrant=0;
    prev_stable_quadrant=0;
    cw_quadrants=0;
    step_time=0;
}


void CCompassCalibration::init_min_max(comp_extreme & x)
{
	x.fin_min = 32767;
    x.fin_max = -32768;
	for (int i(0);i<4;i++)
	x.avg[i]=0;
	x.cnt = 0;
};

void CCompassCalibration::init()
{
	compass_course_no_offset = 0.0f;
	compass_course = 0.0f;
    cw_quadrants=0;
	
	init_min_max(mm_x);
	init_min_max(mm_y);
	init_min_max(mm_z);	
}

int16_t CCompassCalibration::c_avg(comp_extreme & x)
{
	long int sum = 0;
	for (int i(0);i<4;i++)
	sum += x.avg[i];

	int16_t avg = sum >> 2;
	return avg;
}

void CCompassCalibration::update_min_max(comp_extreme & x, int16_t newval)
{
	if (newval == 0x7fff || newval == 0x8000 || newval == 0)
    	return;
	if (x.cnt > 3) {
		int16_t avg = c_avg(x);
		if (avg < x.fin_min)
    		x.fin_min = avg;
		if (avg > x.fin_max)
		    x.fin_max = avg;
		x.cnt=0;
	}
	x.avg[x.cnt++] = newval;
}

void CCompassCalibration::calibrate(const TCompassTriple & compass_raw)
{
	update_min_max(mm_x,compass_raw.x);
	update_min_max(mm_y,compass_raw.y);
    update_min_max(mm_z,compass_raw.z);	
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

	float w = mm_x.fin_max - mm_x.fin_min;
	float h = mm_z.fin_max - mm_z.fin_min;

	if (w>=0 && h>=0) {
		centered.x = (compass_raw.x - mm_x.fin_min - (w/2.0));
		centered.z = -(compass_raw.z - mm_z.fin_min - (h/2.0));
	}

	if (w>0 && h>0) {
        m_ix = (float)centered.x / (w/2.0);
		m_iz = (float)centered.z / (h/2.0);

		compass_course_no_offset = coords_to_angle(m_ix,m_iz);
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
    b_printf(PSTR("Storing compass calibration to EEPROM.\r\n"));
    
    // Put data in array of `raw` words
    uint16_t rec[8];
    rec[0]=(uint16_t)compass_north_offset;
    rec[1]=mm_x.fin_min;
    rec[2]=mm_x.fin_max;
    rec[3]=mm_y.fin_min;
    rec[4]=mm_y.fin_max;
    rec[5]=mm_z.fin_min;
    rec[6]=mm_z.fin_max;
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
    b_printf(PSTR("Loading compass calibration from EEPROM..."));

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
        mm_x.fin_min=rec[1];
        mm_x.fin_max=rec[2];
        mm_y.fin_min=rec[3];
        mm_y.fin_max=rec[4];
        mm_z.fin_min=rec[5];
        mm_z.fin_max=rec[6];
        
        b_printf(PSTR("OK\r\n"));
        SetCalState(csCalibrated);

    } else {
        // Data is corrupted somehow (or was never stored before).
        b_printf(PSTR("FAILED (checksum)\r\n"));
    }    
}
// ----------------------------------------------------------------------------
void CCompassCalibration::reset_compass_calibration()
{
    compass_course = 0.0f;
    
    calibration_mode=false;
    init();
    SetCalState(csNotCalibrated);
}
// ----------------------------------------------------------------------------
void CCompassCalibration::set_true_north()
{
    // Set whatever the boat is pointing to now as 'true north'.
    b_printf(PSTR("Setting true north.\r\n"));
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

    b_printf(PSTR("Calibration mode: "));
    if (calibration_mode)
        b_printf(PSTR("ON\r\n"));
    else
        b_printf(PSTR("OFF\r\n"));
}   
// ----------------------------------------------------------------------------
void CCompassCalibration::print_cal()
{
    int iNoOffset = compass_course_no_offset;
    int iix = m_ix * 100.0;
    int iiz = m_iz * 100.0;

    b_printf(PSTR(" no offset=%d px=%d%% pz=%d%% "), iNoOffset, iix, iiz);
	b_printf(PSTR(" xr=%04d ... %04d "), mm_x.fin_min, mm_x.fin_max);
	b_printf(PSTR(" zr=%04d ... %04d "), mm_z.fin_min, mm_z.fin_max);
    b_printf(PSTR(" q=%d cs="), get_quadrant());
    PrintCalState();
    b_printf(PSTR("\r\n"));
}
// ----------------------------------------------------------------------------
void CCompassCalibration::PrintCalState()
{
    switch (m_cal_state) {
    case csNotCalibrated:
        b_printf(PSTR("not calibrated"));
        break;
    case csCenterDetect:
        b_printf(PSTR("center detect"));
        break;
    case csTurn1:
        b_printf(PSTR("turn1"));
        break;
    case csTurn2:
        b_printf(PSTR("turn2"));
        break;
    case csFinish:
        b_printf(PSTR("finish"));
    case csCalibrated:
        b_printf(PSTR("calibrated"));
        break;
    default:
        b_printf(PSTR("???"));
    }
}
// ----------------------------------------------------------------------------
void CCompassCalibration::SetCalState(ECalibrationState new_state)
{
    if (new_state != m_cal_state) {
        step_time=0;
        m_cal_state = new_state;
        b_printf(PSTR("ccstate: "));
        PrintCalState();
        b_printf(PSTR("\r\n"));
    }
}
// ----------------------------------------------------------------------------
int CCompassCalibration::get_quadrant()
{
    if (compass_course_no_offset >= 0 && compass_course_no_offset < 90)
        return 0;
    else if (compass_course_no_offset >= 90 && compass_course_no_offset < 180)
        return 1;
    else if (compass_course_no_offset >= 180 && compass_course_no_offset < 270)
        return 2;
    else if (compass_course_no_offset >= 270 && compass_course_no_offset < 360)
        return 3;
    return -1;
}
// ----------------------------------------------------------------------------
void CCompassCalibration::DetectCwTurn()
{
    // Detect CW turn
    if ((prev_stable_quadrant == (stable_quadrant-1)) ||
       (prev_stable_quadrant == 3 && stable_quadrant == 0)) {
            cw_quadrants++;
            b_printf(PSTR("+turned CW to next quadrant: %d\r\n"),stable_quadrant);
        } else {
            b_printf(PSTR("-moved in other direction, restarting\r\n"));
            cw_quadrants =0;
    }

}
// ----------------------------------------------------------------------------
void CCompassCalibration::update100ms()
{
    int q = get_quadrant();
    bool bQuadrantChanged=false;

    if (q == prev_quadrant)
        quadrant_counter++;
    else
        quadrant_counter=0;

    if (quadrant_counter > 5) {
        if (stable_quadrant != q) {
            // Compass now in another quadrant
            prev_stable_quadrant = stable_quadrant;
            stable_quadrant = q;
            bQuadrantChanged=true;
        }
    }        

    switch (m_cal_state) {
    case csNotCalibrated:
        if (calibration_mode)
            SetCalState(csCenterDetect);
        break;
    case csCenterDetect:
        if (!calibration_mode)
            SetCalState(csNotCalibrated);

        if (bQuadrantChanged) {
            DetectCwTurn();

            if (cw_quadrants >= 5) {
                cw_quadrants=0;
                SetCalState(csTurn1);
            }
        }

        break;
    case csTurn1:
        if (!calibration_mode)
            SetCalState(csNotCalibrated);

        if (bQuadrantChanged) {
            DetectCwTurn();

            if (cw_quadrants >= 4) {
                cw_quadrants=0;
                SetCalState(csTurn2);
            }
        }
        break;
    case csTurn2:
        if (!calibration_mode)
            SetCalState(csNotCalibrated);

        if (bQuadrantChanged) {
            DetectCwTurn();

            if (cw_quadrants >= 4)
                SetCalState(csFinish);
        }
        break;

    case csFinish:
        b_printf(PSTR("Compass now calibrated\r\n"));

        if (step_time > 10) {       
            b_printf(PSTR("Saving to EEPROM\r\n"));
            store_calibration();      
            calibration_mode = false;
            SetCalState(csCalibrated);
        };

        break;

    case csCalibrated:
        if (calibration_mode)
            SetCalState(csCenterDetect);

        break;
    }

    step_time++;
    prev_quadrant = q;
}

ECalibrationState CCompassCalibration::get_state()
{
    return m_cal_state;
}