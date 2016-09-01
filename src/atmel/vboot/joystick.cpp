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
bool CJoystick::in_center(unsigned int j)
{
    return (j > (JOY_CENTER - JOY_BAND/2)) &&
    (j < (JOY_CENTER + JOY_BAND/2));
}
// ----------------------------------------------------------------------------
// Joystick down/right detect
// ----------------------------------------------------------------------------
bool CJoystick::in_max(unsigned int j)
{
    return j > (JOY_MAX - JOY_BAND);
}
// ----------------------------------------------------------------------------
// Joystick up/left detect
// ----------------------------------------------------------------------------
bool CJoystick::in_min(unsigned int j)
{
    return j < (JOY_MIN + JOY_BAND);
}
// ----------------------------------------------------------------------------
bool CJoystick::in_goto()
{
    return in_max(pd3_pulse_duration);
}
// ----------------------------------------------------------------------------
bool CJoystick::in_store()
{
    return in_min(pd3_pulse_duration);
}
// ----------------------------------------------------------------------------
bool CJoystick::in_goto_store_center()
{
    return in_center(pd3_pulse_duration);
}
// ----------------------------------------------------------------------------
bool CJoystick::in_manual()
{
    return in_min(pb3_pulse_duration);
}
// ----------------------------------------------------------------------------
bool CJoystick::in_clear()
{
    return in_max(pb3_pulse_duration);
}
// ----------------------------------------------------------------------------
int CJoystick::to_perc(unsigned int raw)
{
    // 2000 (-100%) ... 3000 (0%) .... 4000 (100%)

    int perc(raw);
    perc = (perc - 3000)/10;

    return perc;
}
