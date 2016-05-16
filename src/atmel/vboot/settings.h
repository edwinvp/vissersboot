#ifndef SETTINGS_H
#define SETTINGS_H

// ----------------------------------------------------------------------------
// DEFINES
// ----------------------------------------------------------------------------

// CPU frequency [Hz]
#define F_CPU 16000000

// REMOTE CONTROL JOYSTICK defines - in capture/compare (PWM) units (counts)
#define JOY_MIN 2000 /* joystick up or left */
#define JOY_CENTER 3000 /* joystick center value */
#define JOY_MAX 4000 /* joystick down or right */
#define JOY_BAND 500 /* max deviation for centre detection) */

// Minimum/maximum time of goto/store joystick to be out of center,
// in order to be recognized as valid command pulse.
#define MIN_JOY_PULSE_DURATION 100
#define MAX_JOY_PULSE_DURATION 3000
#define JOY_CMD_ACCEPT_TIME 2000

// Time after last GPS valid update until current fix consired stale
#define GPS_STALE_TIME 5000

// Time to find out GPS course - before PID starts steering
#define COURSE_DET_TIME 8000


// Main state machine state definitions
enum TMainState {
	msManualMode=0, // manual control mode
	msAutoMode, // automatic waypoint mode
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

// Periodic message type (which message to log periodically to console)
enum TMessageMode {
	mmNone, // don't log any message
	mmServoCapture, // captured servo signals (remote ctrl. joystick values)
	mmGps, // GPS input debugging
	mmSteering, // Auto steering debugging
	mmCompass, // Compass values

	mmLast
};

#endif

