#include <math.h>
#include <stdio.h>
#ifndef _WIN32
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#else
#include "fakeio.h"
#endif
#include "settings.h"
#include "steering.h"
#include "state_machine.h"
#include "joystick.h"
#include "crc.h"

extern CStateMachine stm;

#define NUM_EEPROM_WORDS 15

CSteering::CSteering() :
    motor_l(0),
    motor_r(0),
    restrict_dir(0),
    arrived(false),
    compass_course(0.0f),
    bearing_sp(0.0f),
    pv_used(0.0f),
    sp_used(0.0f),
	p_add(0),
	i_add(0),
	d_add(0),
	pid_err(0),
	global_max_speed(1.0f),
	dont_stop_steering(false),
	m_output_enable(false)
{
	// Auto steer PID-tune parameters

	// PID settings for initial vessel pointing (more agressive)
	pid_aggressive.TUNE_P=5.0; // P-action
	pid_aggressive.TUNE_I=0.75; // I-action
	pid_aggressive.TUNE_D=0.0;  // D-action
	// Max steering action done by controller
	pid_aggressive.max_steering = 0.9*global_max_speed;

	// PID settings for 'normal' sailing (calmer)
	pid_normal.TUNE_P=0.6;
	pid_normal.TUNE_I=0.0000001;
	pid_normal.TUNE_D=0.0;
	// Max steering action done by controller
	pid_normal.max_steering = 0.3*global_max_speed;

}

// ----------------------------------------------------------------------------
float CSteering::simple_pid(float pv, float sp,
bool enable_p, float Kp,
bool enable_i, float Ki,
bool enable_d, float Kd)
{
    float cv(0.0);

    bool bBigDiff = fabs(sp-pv) > 180;

    if (sp > 180 && bBigDiff)
        sp -= 360;
    if (pv > 180 && bBigDiff)
        pv -= 360;

    pid_err = sp - pv;

    p_add = Kp * pid_err;
    i_add += Ki * pid_err;
    d_add = 0.0;

    cv = 0;
    if (enable_p)
        cv += p_add;
    if (enable_i)
        cv += i_add;
    if (enable_d)
        cv += d_add;
    
    if (!enable_i)
	    i_add=0;

    return cv;
}
// ----------------------------------------------------------------------------
void CSteering::do_restrict_dir(float & pid_cv)
{
    // It doesn't matter to turn CW or CCW if the error is that big,
    // insist on maintaining either CW or CCW
    if (restrict_dir==0) {
        if (pid_err > 135.0) {
            b_printf(PSTR("Restrict dir -1\r\n"));
            restrict_dir = -1;
        } else if (pid_err < -135) {
            b_printf(PSTR("Restrict dir +1\r\n"));
            restrict_dir = 1;
        }   
    } else {
            if (restrict_dir==1)
                pid_cv = fabs(pid_cv);
            else
                pid_cv = -fabs(pid_cv);
        
            if ( fabs(pid_err) < 90) {
                b_printf(PSTR("Cancel dir restrict\r\n"));
                restrict_dir=0;
            }
    }
}
// ----------------------------------------------------------------------------
void CSteering::auto_steer()
{
	float max_correct(0.0);

	bool bPointingTheVessel = (stm.Step() == msAutoModeCourse);

	if (bPointingTheVessel)
		max_correct = pid_aggressive.max_steering;
	else
		max_correct = pid_normal.max_steering;

    sp_used = bearing_sp;
    pv_used = compass_course;

    // Only enable I-action when in normal auto mode (not course mode)
	CPidParams * params(0);
	if (bPointingTheVessel)
		params = &pid_aggressive; // use agressive settings when pointing vessel
	else
		params = &pid_normal; // when sailing use less agressive settings

	// Run PI-controller (D-action not implemented yet)
	float pid_cv = simple_pid(
		pv_used, // process-value (GPS course)
		sp_used, // set point (bearing from Haversine)
		params->p_enable, params->TUNE_P, // P-action
		params->i_enable, params->TUNE_I, // I-action
		params->d_enable, params->TUNE_D  // D-action
		);

	do_restrict_dir(pid_cv);

	// Clip the PID control variable (CV)
	cv_clipped=0.0f;
	if (pid_cv > max_correct)
        cv_clipped = max_correct;
    else if (pid_cv < -max_correct)
        cv_clipped = -max_correct;
    else
        cv_clipped = pid_cv;

    // Take a moment to see where the boat is pointing at before doing anything
    if (stm.TimeInStep() < COURSE_DET_TIME) {
        // Do this by keeping the PID-variables in reset condition
        p_add=0;
        i_add=0;
        d_add=0;
        cv_clipped=0;
    }

	// Calculate L+R motor speed factors (-1.0 ... +1.0)
	motor_l=0;
    motor_r=0;
    calc_motor_setpoints(motor_l,motor_r,global_max_speed,cv_clipped);

	// Convert to values that the servo hardware understands (2000...4000)
	SetMotorSpeeds(motor_l,motor_r);
}
// ----------------------------------------------------------------------------
float CSteering::clip_motor(float mtr)
{
	if (mtr>1.0)
		return 1.0;
    else if (mtr<-1.0)
        return -1.0;
    else
        return mtr;
}
// ----------------------------------------------------------------------------
//!\brief Calculate motor set points as a factor (-1.0 ... +1.0)
void CSteering::calc_motor_setpoints(float & motor_l, float & motor_r, float max_speed, float cv_clipped)
{
    if (stm.Step() == msAutoModeNormal) {
        motor_l = max_speed;
        motor_r = max_speed;
    } else {
        motor_l = 0;
        motor_r = 0;
    }

    motor_l += cv_clipped;
    motor_r -= cv_clipped;

    // Make sure motor set points stay within -1.0 ... 1.0
    motor_l = clip_motor(motor_l);
    motor_r = clip_motor(motor_r);

    if (!dont_stop_steering && arrived) {
        motor_l = 0;
        motor_r = 0;
    }
}
// ----------------------------------------------------------------------------
void CSteering::toggle_dont_stop()
{
	if (dont_stop_steering)
		dont_stop_steering=false;
	else
		dont_stop_steering=true;
}
// ----------------------------------------------------------------------------
void CSteering::reset_i_action()
{
	i_add=0;
}

// ----------------------------------------------------------------------------
// Manual mode (joystick 'pass through' steering)
// ----------------------------------------------------------------------------
void CSteering::manual_steering(unsigned int mot_L_dc,unsigned int mot_R_dc)
{
	motor_l = CJoystick::to_perc(mot_L_dc)/100.0f;
	motor_r = CJoystick::to_perc(mot_R_dc)/100.0f;

	// Pass through motor left and right setpoints to PWM module
    SetMotorSpeeds(motor_l,motor_r);
}
// ----------------------------------------------------------------------------
bool CSteering::motor_running()
{
	float thresh(0.05); // 5 [%]
	return (fabs(motor_l) > thresh) || (fabs(motor_r) > thresh);
}
// ----------------------------------------------------------------------------
void CSteering::do_reverse_thrust()
{
	// Go reverse (fill out negative motor setpoint here)
	float reversing_speed(1.0);
	motor_l = -1.0*reversing_speed;
	motor_r = -1.0*reversing_speed;

	// Convert to values that the servo hardware understands (2000...4000)
	SetMotorSpeeds(motor_l,motor_r);
}
// ----------------------------------------------------------------------------
// Motor factors: ml: -1.0 .. +1.0, mr: -1.0 ... +1.0
void CSteering::SetMotorSpeeds(float ml, float mr)
{          
	// Convert to values that the servo hardware understands
	if (!m_output_enable) {
		ml=0;
		mr=0;
	}
    
    SetPwm(ml,mr);
}    
    
void CSteering::SetPwm(float ml, float mr)
{        
    // Left motor, 10-bit PWM.
    // Range 0x100 ... 0x200 (~ -100 % ... +100 %)
    // Based on 16 MHz crystal timing: Timer freq = 250.00 [kHz]
    //   1,024 [ms] = 0x100 (-100%)
    //   1.536 [ms] = 0x180 (neutral)
    //   2,048 [ms] = 0x200 (+100%)
    unsigned int pl = (float)384.0 + (ml * 128.0);
    
    // Right motor, 16-bit PWM 
    // Range 2000 ... 4000 ( -100 % ... +100 %)
    unsigned int pr = (float)JOY_CENTER + (mr * 1000.0);

    cli();
  
    // Left motor (10-bits)
    TC4H = (pl >> 8)&3; // bits 9:8
    OCR4D = (pl & 255); // bits 7:0
    
    // Right motor (16-bit)
    OCR3A = pr;
    
    sei(); 
}
// ----------------------------------------------------------------------------
float CSteering::get_motor_L_perc()
{
	return motor_l;
}
// ----------------------------------------------------------------------------
float CSteering::get_motor_R_perc()
{
	return motor_r;
}
// ----------------------------------------------------------------------------
void CSteering::set_output_enable(bool output_enable)
{
	m_output_enable = output_enable;
}
// ----------------------------------------------------------------------------
void CSteering::load_calibration()
{
	b_printf(PSTR("Loading PID settings from EEPROM..."));

	// Array to receive 'raw' words from EEPROM
	// Layout:
	// WORD   DESC
	// 0      normal pid flags
	// 1      agressive pid flags
	// 2      normal pid P parameter
	// 4      normal pid I parameter
	// 6      normal pid D parameter
	// 8      agressive pid P parameter
	// 10     agressive pid I parameter
	// 12     agressive pid D parameter
	// 14     CRC-16 checksum

	uint16_t rec[NUM_EEPROM_WORDS];

	// Read data as words from EEPROM
	unsigned int addr=PID_EEPROM_OFFSET;
	for (int i(0); i<NUM_EEPROM_WORDS; i++) {
		eeprom_busy_wait();
		rec[i]=eeprom_read_word((uint16_t*)addr);
		addr+=2;
	}

	// Calculate checksum over what has just been read
	uint16_t chk = crc16((unsigned char*)rec,(NUM_EEPROM_WORDS-1)*2);

	// Compare checksum with the one that was stored
	if (chk == rec[NUM_EEPROM_WORDS-1]) {
		// They were the same, data is okay.
		// Now apply that calibration stored earlier:

		float * fp = reinterpret_cast<float*>(&rec[2]);
		pid_normal.TUNE_P = fp[0];
		pid_normal.TUNE_I = fp[1];
		pid_normal.TUNE_D = fp[2];
		pid_aggressive.TUNE_P = fp[3];
		pid_aggressive.TUNE_I = fp[4];
		pid_aggressive.TUNE_D = fp[5];

		b_printf(PSTR("OK\r\n"));

	} else {
		// Data is corrupted somehow (or was never stored before).
		b_printf(PSTR("FAILED (checksum)\r\n"));
	}
}
// ----------------------------------------------------------------------------
void CSteering::save_calibration()
{
	b_printf(PSTR("Storing PID config to EEPROM.\r\n"));

	// Put data in array of `raw` words
	uint16_t rec[NUM_EEPROM_WORDS];
	rec[0]=0;
	rec[1]=0;

	float * fp = reinterpret_cast<float*>(&rec[2]);
	fp[0] = pid_normal.TUNE_P;
	fp[1] = pid_normal.TUNE_I;
	fp[2] = pid_normal.TUNE_D;
	fp[3] = pid_aggressive.TUNE_P;
	fp[4] = pid_aggressive.TUNE_I;
	fp[5] = pid_aggressive.TUNE_D;

	// Calculate checksum over those 13 words
    rec[NUM_EEPROM_WORDS-1]=crc16((unsigned char*)rec,(NUM_EEPROM_WORDS-1)*2);

    // Write data as words to EEPROM
    unsigned int addr=PID_EEPROM_OFFSET;
	for (int i(0); i<NUM_EEPROM_WORDS; i++) {
        eeprom_busy_wait();
        eeprom_write_word((uint16_t*)addr,rec[i]);
        addr+=2;
    }
}
// ----------------------------------------------------------------------------

