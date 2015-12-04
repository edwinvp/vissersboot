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

// Minimum time of goto/store joystick to be out of center to be recognized
// as valid command pulse.
#define MIN_GOTO_STORE_MIN_DURATION 100
#define MIN_GOTO_STORE_ACCEPT_TIME 3000

// Time after last GPS valid update until current fix consired stale
#define GPS_STALE_TIME 5000


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
	msCmdError,

	msLast
};

// Periodic message type (which message to log periodically to console)
enum TMessageMode {
	mmNone, // don't log any message
	mmServoCapture, // captured servo signals (remote ctrl. joystick values)
	mmGps, // GPS input debugging

	mmLast
};

#endif

