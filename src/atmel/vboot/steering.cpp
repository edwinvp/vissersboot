#include <math.h>
#include <stdio.h>
#ifndef _WIN32
#include <avr/io.h>
#include <avr/pgmspace.h>
#else
#include "fakeio.h"
#endif
#include "settings.h"
#include "steering.h"
#include "state_machine.h"

extern CStateMachine stm;

CSteering::CSteering() :
    restrict_dir(0),
    arrived(false),
    compass_course(0.0f),
    bearing_sp(0.0f),
    pv_used(0.0f),
    sp_used(0.0f),
    SUBST_SP(0.0),
    SUBST_PV(0.0),
	p_add(0),
	i_add(0),
	d_add(0),
	pid_err(0),
	dont_stop_steering(false),
	global_max_speed(1.0f)
{
	// Auto steer PID-tune parameters

	// PID settings for initial vessel pointing (more agressive)
	pid_agressive.TUNE_P=1.0; // P-action
	pid_agressive.TUNE_I=0.005; // I-action
	pid_agressive.TUNE_D=0.0;  // D-action
	// Max steering action done by controller
	pid_agressive.max_steering = 0.9*global_max_speed;

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
	float rel_pid_err = fabs(pid_err) / 180.0f;
	float max_correct(0.0);

	bool bPointingTheVessel = (stm.Step() == msAutoModeCourse);

	if (bPointingTheVessel)
		max_correct = pid_agressive.max_steering*rel_pid_err;
	else
		max_correct = pid_normal.max_steering;

    // Use different, manually entered course (a test mode)
    if (SUBST_SP != 0.0)
        sp_used = SUBST_SP;
    else
        sp_used = bearing_sp;

    // Use simulated, manually entered compass (a test mode)
    if (SUBST_PV != 0.0)
        pv_used = SUBST_PV;
    else
        pv_used = compass_course;

    // Only enable I-action when in normal auto mode (not course mode)
	CPidParams * params(0);
	if (bPointingTheVessel)
		params = &pid_agressive; // use agressive settings when pointing vessel
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
	float motor_l(0), motor_r(0);
    calc_motor_setpoints(motor_l,motor_r,global_max_speed,cv_clipped);

    // Convert to values that the servo hardware understands (2000...4000)
    OCR1A = (float)JOY_CENTER + (motor_l * 1000.0);
    OCR1B = (float)JOY_CENTER + (motor_r * 1000.0);
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

    motor_l -= cv_clipped;
    motor_r += cv_clipped;

    // Make sure motor set points stay within -1.0 ... 1.0
    motor_l = clip_motor(motor_l);
    motor_r = clip_motor(motor_r);

    if (!dont_stop_steering && arrived) {
        motor_l = 0;
        motor_r = 0;
    }
}

void CSteering::toggle_dont_stop()
{
	if (dont_stop_steering)
		dont_stop_steering=false;
	else
		dont_stop_steering=true;
}

void CSteering::reset_i_action()
{
	i_add=0;
}
