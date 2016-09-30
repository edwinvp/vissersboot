#include <math.h>
#include "state_machine.h"
#include "steering.h"
#include "joystick.h"
#include "lat_lon.h"
#include "waypoints.h"
#include "led_control.h"
#include <avr/pgmspace.h>

extern CSteering steering;
extern CJoystick joystick;
extern CWayPoints waypoints;
extern CLedControl ledctrl;

extern bool gps_valid;
void print_stats();
bool set_finish(int memory_no);
void store_waypoint(int memory_no);


#ifndef _WIN32
#include <stdio.h>
#endif

CStateMachine::CStateMachine() : 
    main_state(msManualMode),next_state(msManualMode),
    state_time(0),
    shown_stats(false),
    straight_to_auto(false), joy_pulses(0)
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
	switch (st) {
		case msAutoModeCourse: b_printf(PSTR("msAutoModeCourse")); break;
		case msAutoModeNormal: b_printf(PSTR("msAutoModeNormal")); break;
		case msManualMode: b_printf(PSTR("msManualMode")); break;
		case msCountJoyGoto: b_printf(PSTR("msCountJoyGoto")); break;
		case msCountJoyGotoRetn: b_printf(PSTR("msCountJoyGotoRetn")); break;
		case msConfirmGotoPosX: b_printf(PSTR("msConfirmGotoPosX")); break;
		case msCountJoyStore: b_printf(PSTR("msCountJoyStore")); break;
		case msCountJoyStoreRetn: b_printf(PSTR("msCountJoyStoreRetn")); break;
		case msConfirmStorePosX: b_printf(PSTR("msConfirmStorePosX")); break;
		case msClear1: b_printf(PSTR("msClear1")); break;
		case msClear2: b_printf(PSTR("msClear2")); break;
		case msCmdErrorMan: b_printf(PSTR("msCmdErrorMan")); break;
		case msCmdErrorAuto: b_printf(PSTR("msCmdErrorAuto")); break;
	}
}

// ----------------------------------------------------------------------------
void CStateMachine::check_arrived()
{
    if (!steering.dont_stop_steering && steering.arrived) {
        b_printf(PSTR("Arrived!\r\n"));
        next_state = msManualMode;
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

    // Go to manual mode if GPS signal is absent for too long,
    // or joystick is put in manual control mode.
    if (joystick.in_manual() || !gps_valid) {
        next_state = msManualMode;
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

		if (gps_valid)
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
        if (fabs(steering.pid_err) < 10)
        next_state = msAutoModeNormal;
    }


    abort_auto_if();
    check_arrived();
}
// ----------------------------------------------------------------------------
void CStateMachine::step_auto_mode_normal()
{
    if (fabs(steering.pid_err) > 32)
    next_state = msAutoModeCourse;
    
    abort_auto_if();
    check_arrived();
}
// ----------------------------------------------------------------------------
void CStateMachine::step_clear1()
{
    if (!joystick.in_clear())
    next_state = msCmdErrorMan;
    else if (!shown_stats) {
        print_stats();
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
        next_state = msManualMode;
    }

    if (state_time > 5000)
        next_state = msCmdErrorMan;
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
    if (joy_pulses > 3)
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
    if (joy_pulses > 3)
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
    if (joystick.in_store())
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
    else if (joystick.in_store())
        next_state = msCmdErrorMan;
    else if (state_time > JOY_CMD_ACCEPT_TIME && ledctrl.been_dark_for_a_while()) {
        ledctrl.blink_x_times(joy_pulses);
        next_state = msConfirmGotoPosX;
    }
}
// ----------------------------------------------------------------------------
void CStateMachine::step_count_store()
{
    if (joystick.in_goto())
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
    if (joystick.in_store())
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

