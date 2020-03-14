#include <math.h>
#include "state_machine.h"
#include "steering.h"
#include "joystick.h"
#include "lat_lon.h"
#include "waypoints.h"
#include "led_control.h"
#include "compass_calibrate.h"
#ifndef _WIN32
#include <avr/pgmspace.h>
#else
#include "fakeio.h"
#endif

extern CSteering steering;
extern CJoystick joystick;
extern CWayPoints waypoints;
extern CLedControl ledctrl;
extern CCompassCalibration cc;

extern bool gps_valid;
bool set_finish(int memory_no);
void store_waypoint(int memory_no);


#ifndef _WIN32
#include <stdio.h>
#endif

CStateMachine::CStateMachine() : 
    main_state(msManualMode),next_state(msManualMode),
    state_time(0),
    shown_stats(false),
    straight_to_auto(false), joy_pulses(0),
	rc_ignore_first_command(false)
{	
}

TMainState CStateMachine::Step()
{
	return main_state;
}

void CStateMachine::SetNextStep(TMainState ns)
{
	if (ns != main_state) {
		main_state = ns;
	}
}

void CStateMachine::ForceStep(TMainState ns)
{
	main_state = ns;
}

void CStateMachine::Run()
{
	// next state defaults to current state (unchanged)
	next_state = Step();

	switch (Step()) {
    case msManualMode: // manual control mode
		step_manual_mode();
	    break;
    case msAutoModeCourse: // automatic waypoint mode (course)
		step_auto_mode_course();
		break;
	case msAutoModeNormal: // automatic waypoint mode (normal)
		step_auto_mode_normal();
		break;
	case msReverseThrust: // brake using reverse setpoint for a small time
		step_reverse_thrust();
		break;
	case msCountJoyGoto: // count joystick 'up' (goto pos.) command
		step_count_goto();
		break;
	case msCountJoyGotoRetn:
		step_count_goto_retn();
		break;
	case msConfirmGotoPosX:
		step_confirm_goto_pos_x();
		break;

	case msCountJoyStore: // count joystick 'down' (store pos.) command
		step_count_store();
		break;
	case msCountJoyStoreRetn:
		step_count_store_retn();
		break;
	case msConfirmStorePosX:
		step_confirm_store_pos_x();
		break;

	case msClear1:
		step_clear1();
		break;

	case msClear2:
		step_clear2();
		break;

	case msConfirmClear:
		step_confirm_clear();
		break;

	case msCmdErrorMan:
		step_cmd_error_man();
		break;

	case msCmdErrorAuto:
		step_cmd_error_auto();
		break;

	default:
		// should never get here
		next_state = msManualMode;
	}

	if (Step() != next_state) {
		// Transitioning, reset step time and enter new step
		b_printf(PSTR("change step:"));

		print_step_name(next_state);
		b_printf(PSTR("\r\n"));

		state_time = 0;
		SetNextStep(next_state);
		} else {
		// we're in the 100 [ms] process,
		// so we can increase step time like this...
		state_time += 100;
	}
}

// ----------------------------------------------------------------------------
void CStateMachine::print_step_name(TMainState st)
{
	const char * pMsg = PSTR("?");
	
	switch (st) {
		case msAutoModeCourse: pMsg=PSTR("msAutoModeCourse"); break;
		case msAutoModeNormal: pMsg=PSTR("msAutoModeNormal"); break;
		case msReverseThrust: pMsg=PSTR("msReverseThrust"); break;
		case msManualMode: pMsg=PSTR("msManualMode"); break;
		case msCountJoyGoto: pMsg=PSTR("msCountJoyGoto"); break;
		case msCountJoyGotoRetn: pMsg=PSTR("msCountJoyGotoRetn"); break;
		case msConfirmGotoPosX: pMsg=PSTR("msConfirmGotoPosX"); break;
		case msCountJoyStore: pMsg=PSTR("msCountJoyStore"); break;
		case msCountJoyStoreRetn: pMsg=PSTR("msCountJoyStoreRetn"); break;
		case msConfirmStorePosX: pMsg=PSTR("msConfirmStorePosX"); break;
		case msClear1: pMsg=PSTR("msClear1"); break;
		case msClear2: pMsg=PSTR("msClear2"); break;
		case msConfirmClear: pMsg=PSTR("msConfirmClear"); break;
		case msCmdErrorMan: pMsg=PSTR("msCmdErrorMan"); break;
		case msCmdErrorAuto: pMsg=PSTR("msCmdErrorAuto"); break;
		case msLast: pMsg=PSTR("msLast"); break;
	}
	
	b_printf(pMsg);
}

// ----------------------------------------------------------------------------
void CStateMachine::check_arrived()
{
    if (!steering.dont_stop_steering && steering.arrived) {
		b_printf(PSTR("Arrived!\r\n"));

        next_state = msReverseThrust;
    }
}
// ----------------------------------------------------------------------------
void CStateMachine::abort_auto_if()
{
	// Blink LED fast when 'special' goto/store/clear command is given.
    // We are now in auto mode and that is allowed only in manual mode.
    if (!joystick.in_goto_store_center() ||
        joystick.in_clear()) {
        next_state = msCmdErrorAuto;
    }

	// Go to manual mode (via reverse burst) if GPS signal is absent for
	// too long, or joystick is put in manual control mode.
    if (joystick.in_manual() || !gps_valid) {
        next_state = msReverseThrust;
    }
}

// ----------------------------------------------------------------------------
// State machine steps
// ----------------------------------------------------------------------------

void CStateMachine::step_manual_mode()
{
	steering.p_add=0;
	steering.i_add=0;
	steering.d_add=0;

	// In manual mode
	if (joystick.in_goto()) {
		joy_pulses = 0;

        if (cc.calibration_mode)
            cc.set_true_north();
		else if (gps_valid)
			next_state = msCountJoyGoto;
		else
			next_state = msCmdErrorMan;

    } else if (joystick.in_store()) {           
        joy_pulses = 0;
        next_state = msCountJoyStore;
	} else if (joystick.in_clear()) {
		shown_stats = false;
		next_state = msClear1;
	} else if (straight_to_auto) {
		straight_to_auto = false;
		next_state = msAutoModeCourse;
	}
}
// ----------------------------------------------------------------------------
void CStateMachine::step_auto_mode_course()
{
    if (state_time > 2000) {
		if (fabs(steering.pid_err) < 20) {
			steering.reset_i_action();
			next_state = msAutoModeNormal;
		}
    }


    abort_auto_if();
    check_arrived();
}
// ----------------------------------------------------------------------------
void CStateMachine::step_auto_mode_normal()
{
    if (fabs(steering.pid_err) > 90)
		next_state = msAutoModeCourse;
    
    abort_auto_if();
    check_arrived();
}
// ----------------------------------------------------------------------------
void CStateMachine::step_reverse_thrust()
{
	if (state_time > 1000) {
		next_state = msManualMode;
	}
}
// ----------------------------------------------------------------------------
void CStateMachine::step_clear1()
{
	if (!joystick.in_clear())
		next_state = msCmdErrorMan;
    else if (!shown_stats) {
        shown_stats=true;
    }
    else if (state_time > 1000) {
        next_state = msClear2;
    }
}
// ----------------------------------------------------------------------------
void CStateMachine::step_clear2()
{
    if (!joystick.in_clear()) {
		waypoints.forget_all();
		waypoints.store_waypoints();
		ledctrl.blink_x_times(2);
        next_state = msConfirmClear;
    }

	if (state_time > 5000)
		next_state = msCmdErrorMan;
}
// ----------------------------------------------------------------------------
void CStateMachine::step_confirm_clear()
{
	if (state_time > 2000)
		next_state = msManualMode;
}
// ----------------------------------------------------------------------------
void CStateMachine::step_cmd_error_man()
{
    if (state_time > 2000) {
        if (joystick.in_goto_store_center() && !joystick.in_clear()) {
            next_state = msManualMode;
        }
    }
}
// ----------------------------------------------------------------------------
void CStateMachine::step_cmd_error_auto()
{
    bool bLetGoOfJoyStick = joystick.in_goto_store_center() && !joystick.in_clear();

    if (state_time > 1000 || bLetGoOfJoyStick) {
        next_state = msAutoModeCourse;
	}
}
// ----------------------------------------------------------------------------
void CStateMachine::step_confirm_goto_pos_x()
{
    if (!gps_valid || joy_pulses > NUM_WAYPOINTS)
        next_state = msCmdErrorMan;
    else if (ledctrl.done_blinking()) {
        if (waypoints.set_finish(joy_pulses)) {
            b_printf(PSTR("Set finish to # %d\r\n"), joy_pulses);
            next_state = msAutoModeCourse;
        } else
            next_state = msCmdErrorMan;
	}
}
// ----------------------------------------------------------------------------
void CStateMachine::step_confirm_store_pos_x()
{
	if (!gps_valid || joy_pulses > NUM_WAYPOINTS)
		next_state = msCmdErrorMan;
	else if (ledctrl.done_blinking()) {
		b_printf(PSTR("Store waypoint # %d\r\n"), joy_pulses);
		// Define GPS coords as a waypoint.
		waypoints.store_waypoint(joy_pulses);
		// Store waypoints defined so far to EEPROM.
		waypoints.store_waypoints();
		next_state = msManualMode;
	}
}
// ----------------------------------------------------------------------------
void CStateMachine::step_count_goto()
{
    if (!gps_valid || joystick.in_store())
        next_state = msCmdErrorMan; // attempt to run 'opposite' command
    else if (joystick.in_goto_store_center()) {
        if (state_time > MIN_JOY_PULSE_DURATION) {
            // pulse valid, count
            joy_pulses++;
			next_state = msCountJoyGotoRetn;
		} else
		next_state = msCmdErrorMan; // pulse wasn't long enough
	} else if (state_time > MAX_JOY_PULSE_DURATION)
	next_state = msCmdErrorMan; // joystick takes too long to go back center
}
// ----------------------------------------------------------------------------
void CStateMachine::step_count_goto_retn()
{
	if (joystick.in_goto())
		next_state = msCountJoyGoto;
	else if (!gps_valid || joystick.in_store())
		next_state = msCmdErrorMan;
	else if (state_time > JOY_CMD_ACCEPT_TIME && ledctrl.been_dark_for_a_while()) {
        ledctrl.blink_x_times(joy_pulses);
        next_state = msConfirmGotoPosX;
    }
}
// ----------------------------------------------------------------------------
void CStateMachine::step_count_store()
{
	if (rc_ignore_first_command) {
		// On some remote controls the method to switch on the radio comms
		// is the same as storing the first waypoint. So ignore the very first command.
		b_printf(PSTR("Ignoring first command given after RC up.\r\n"));
		rc_ignore_first_command = false;
		next_state = msCmdErrorMan;	
	} else if (!gps_valid || joystick.in_goto())
		next_state = msCmdErrorMan;
	else if (joystick.in_goto_store_center()) {
		if (state_time > MIN_JOY_PULSE_DURATION) {
			// pulse valid, count
			joy_pulses++;
			next_state = msCountJoyStoreRetn;
		} else
		next_state = msCmdErrorMan; // pulse wasn't long enough
	} else if (state_time > MAX_JOY_PULSE_DURATION)
	next_state = msCmdErrorMan; // joystick takes too long to go back center
}
// ----------------------------------------------------------------------------
void CStateMachine::step_count_store_retn()
{
	if (!gps_valid || joystick.in_store())
		next_state = msCountJoyStore;
	else if (joystick.in_store())
		next_state = msCmdErrorMan;
	else if (state_time > JOY_CMD_ACCEPT_TIME && ledctrl.been_dark_for_a_while()) {
		ledctrl.blink_x_times(joy_pulses);
		next_state = msConfirmStorePosX;
	}
}
// ----------------------------------------------------------------------------
unsigned long CStateMachine::TimeInStep()
{
	return state_time;
}
// ----------------------------------------------------------------------------

