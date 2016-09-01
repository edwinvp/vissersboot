#include "settings.h"
#include "joystick.h"

extern volatile unsigned int pd3_pulse_duration;
extern volatile unsigned int pb3_pulse_duration;

CJoystick::CJoystick()
{
    //
}

// ----------------------------------------------------------------------------
// Joystick center detect
// ----------------------------------------------------------------------------
bool CJoystick::joy_in_center(unsigned int j)
{
    return (j > (JOY_CENTER - JOY_BAND/2)) &&
    (j < (JOY_CENTER + JOY_BAND/2));
}
// ----------------------------------------------------------------------------
// Joystick down/right detect
// ----------------------------------------------------------------------------
bool CJoystick::joy_in_max(unsigned int j)
{
    return j > (JOY_MAX - JOY_BAND);
}
// ----------------------------------------------------------------------------
// Joystick up/left detect
// ----------------------------------------------------------------------------
bool CJoystick::joy_in_min(unsigned int j)
{
    return j < (JOY_MIN + JOY_BAND);
}
// ----------------------------------------------------------------------------
bool CJoystick::joy_in_goto()
{
    return joy_in_max(pd3_pulse_duration);
}
// ----------------------------------------------------------------------------
bool CJoystick::joy_in_store()
{
    return joy_in_min(pd3_pulse_duration);
}
// ----------------------------------------------------------------------------
bool CJoystick::joy_in_goto_store_center()
{
    return joy_in_center(pd3_pulse_duration);
}
// ----------------------------------------------------------------------------
bool CJoystick::joy_in_manual()
{
    return joy_in_min(pb3_pulse_duration);
}
// ----------------------------------------------------------------------------
bool CJoystick::joy_in_clear()
{
    return joy_in_max(pb3_pulse_duration);
}
