#ifndef state_machineH
#define state_machineH

#include "settings.h"

// Main state machine state definitions
enum TMainState {
	msManualMode=0, // manual control mode
	msAutoModeCourse, // 'course' adjustments
	msAutoModeNormal, // automatic waypoint mode
	msCountJoyGoto, // count joystick 'up' command
	msCountJoyGotoRetn,
	msConfirmGotoPosX,
	msCountJoyStore, // count joystick 'down' command
	msCountJoyStoreRetn,
	msConfirmStorePosX,
	msClear1,
	msClear2,
	msCmdErrorMan,
	msCmdErrorAuto,
	msLast
};

class CStateMachine
{
private:
	// State machine current step, next step
	TMainState main_state,next_state;

	// Time elapsed in current state machine step [ms]
	unsigned long state_time;

    // Misc
    void check_arrived();
    void abort_auto_if();

	// Steps / actions
	void step_manual_mode();
    void step_auto_mode_course();
    void step_auto_mode_normal();
    void step_clear1();
    void step_clear2();
    void step_cmd_error_man();
    void step_cmd_error_auto();
    void step_confirm_store_pos_x();
    void step_confirm_goto_pos_x();
    void step_count_goto();
    void step_count_goto_retn();
    void step_count_store();
    void step_count_store_retn();

public:

	CStateMachine();		
	TMainState Step();
	void SetNextStep(TMainState ns);
	void Run();
	static void print_step_name(TMainState st);
	unsigned long TimeInStep();
};


#endif