#include <math.h>
#include "state_machine.h"
#include "steering.h"
#include "joystick.h"
#include "lat_lon.h"

extern CSteering steering;
extern CJoystick joystick;

extern bool gps_valid;
extern int joy_pulses; // # times the goto/store joystick has been pushed up/down
extern bool shown_stats;
extern bool straight_to_auto;
extern bool arrived; // TRUE when arriving at the waypoint
extern bool dont_stop_steering;
extern CLatLon gp_mem_1; // memorized GPS position 1 (usually 'home')
extern CLatLon gp_mem_2; // memorized GPS position 2
extern CLatLon gp_mem_3; // memorized GPS position 3
extern int blink_times;
extern bool slow_blink;
void print_stats();
bool set_finish(int memory_no);
void store_waypoint(int memory_no);

#ifndef _WIN32
#include <stdio.h>
#endif

CStateMachine::CStateMachine() : main_state(msManualMode),next_state(msManualMode),
	state_time(0)
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
		b_printf("change step");

		print_step_name(next_state);
		b_printf("\r\n");

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
		case msAutoModeCourse: b_printf("msAutoModeCourse"); break;
		case msAutoModeNormal: b_printf("msAutoModeNormal"); break;
		case msManualMode: b_printf("msManualMode"); break;
		case msCountJoyGoto: b_printf("msCountJoyGoto"); break;
		case msCountJoyGotoRetn: b_printf("msCountJoyGotoRetn"); break;
		case msConfirmGotoPosX: b_printf("msConfirmGotoPosX"); break;
		case msCountJoyStore: b_printf("msCountJoyStore"); break;
		case msCountJoyStoreRetn: b_printf("msCountJoyStoreRetn"); break;
		case msConfirmStorePosX: b_printf("msConfirmStorePosX"); break;
		case msClear1: b_printf("msClear1"); break;
		case msClear2: b_printf("msClear2"); break;
		case msCmdErrorMan: b_printf("msCmdErrorMan"); break;
		case msCmdErrorAuto: b_printf("msCmdErrorAuto"); break;
	}
}

// ----------------------------------------------------------------------------
void CStateMachine::check_arrived()
{
    if (!dont_stop_steering && arrived) {
        b_printf("Arrived!\r\n");
        next_state = msManualMode;
    }
}
// ----------------------------------------------------------------------------
void CStateMachine::abort_auto_if()
{
    // Blink LED fast when 'special' goto/store/clear command is given.
    // We are now in auto mode and that is allowed only in manual mode.
    if (!joystick.joy_in_goto_store_center() ||
        joystick.joy_in_clear()) {
        next_state = msCmdErrorAuto;
    }

    // Go to manual mode if GPS signal is absent for too long,
    // or joystick is put in manual control mode.
    if (joystick.joy_in_manual() || !gps_valid) {
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
	if (joystick.joy_in_goto()) {
		joy_pulses = 0;

		if (gps_valid)
			next_state = msCountJoyGoto;
		else
			next_state = msCmdErrorMan;

		} else if (joystick.joy_in_store()) {
			joy_pulses = 0;
			next_state = msCountJoyStore;
		} else if (joystick.joy_in_clear()) {
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
        if (fabs(steering.pid_err) < 16)
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
    if (!joystick.joy_in_clear())
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
    if (!joystick.joy_in_clear()) {
        gp_mem_1.clear();
        gp_mem_2.clear();
        gp_mem_3.clear();
        next_state = msManualMode;
    }

    if (state_time > 5000)
        next_state = msCmdErrorMan;
}
// ----------------------------------------------------------------------------
void CStateMachine::step_cmd_error_man()
{
    if (state_time > 2000) {
        if (joystick.joy_in_goto_store_center() && !joystick.joy_in_clear()) {
            next_state = msManualMode;
        }
    }
}
// ----------------------------------------------------------------------------
void CStateMachine::step_cmd_error_auto()
{
    bool bLetGoOfJoyStick = joystick.joy_in_goto_store_center() && !joystick.joy_in_clear();

    if (state_time > 1000 || bLetGoOfJoyStick) {
        next_state = msAutoModeCourse;
    }
}
// ----------------------------------------------------------------------------
void CStateMachine::step_confirm_goto_pos_x()
{
    if (blink_times > 3)
    next_state = msCmdErrorMan;
    else if (blink_times == 0) {

        if (set_finish(joy_pulses)) {
            b_printf("Set finish to # %d\r\n", joy_pulses);
            next_state = msAutoModeCourse;
        } else
        next_state = msCmdErrorMan;
    }
}
// ----------------------------------------------------------------------------
void CStateMachine::step_confirm_store_pos_x()
{
    if (blink_times > 3)
    next_state = msCmdErrorMan;
    else if (blink_times == 0) {
        b_printf("Store waypoint # %d\r\n", joy_pulses);
        store_waypoint(joy_pulses);
        next_state = msManualMode;
    }
}
// ----------------------------------------------------------------------------
void CStateMachine::step_count_goto()
{
    if (joystick.joy_in_store())
        next_state = msCmdErrorMan; // attempt to run 'opposite' command
    else if (joystick.joy_in_goto_store_center()) {
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
    if (joystick.joy_in_goto())
        next_state = msCountJoyGoto;
    else if (joystick.joy_in_store())
        next_state = msCmdErrorMan;
    else if (state_time > JOY_CMD_ACCEPT_TIME && !slow_blink) {
        blink_times = joy_pulses;
        next_state = msConfirmGotoPosX;
    }
}
// ----------------------------------------------------------------------------
void CStateMachine::step_count_store()
{
    if (joystick.joy_in_goto())
        next_state = msCmdErrorMan;
    else if (joystick.joy_in_goto_store_center()) {
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
    if (joystick.joy_in_store())
        next_state = msCountJoyStore;
    else if (joystick.joy_in_store())
        next_state = msCmdErrorMan;
    else if (state_time > JOY_CMD_ACCEPT_TIME && !slow_blink) {
        blink_times = joy_pulses;
        next_state = msConfirmStorePosX;
    }
}
// ----------------------------------------------------------------------------
unsigned long CStateMachine::TimeInStep()
{
    return state_time;
}
// ----------------------------------------------------------------------------

