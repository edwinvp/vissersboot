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
#define GPS_STALE_TIME 10000

// Time to find out GPS course - before PID starts steering
#define COURSE_DET_TIME 4000

// EEPROM offsets
#define COMPASS_EEPROM_OFFSET 0
#define WAYPOINT_EEPROM_OFFSET 64

// Periodic message type (which message to log periodically to console)
enum TMessageMode {
	mmNone, // don't log any message
	mmServoCapture, // captured servo signals (remote ctrl. joystick values)
	mmGps, // GPS input debugging
	mmSteering, // Auto steering debugging
	mmCompass, // Compass values
	mmDebug, // Combination

	mmPAction, // Configure P-action
	mmIAction, // Configure I-action
	mmPVSubst, // Set PV substitution
	mmSPSubst, // Set SP substitution

	mmLast
};

#ifndef _WIN32
#define b_printf printf
#endif

#endif
