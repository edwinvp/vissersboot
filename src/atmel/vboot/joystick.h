#ifndef joystickH
#define joystickH

class CJoystick
{
private:
	
public:
    CJoystick();

    bool joy_in_center(unsigned int j);
    bool joy_in_max(unsigned int j);
    bool joy_in_min(unsigned int j);
    bool joy_in_goto();
    bool joy_in_store();
    bool joy_in_goto_store_center();
    bool joy_in_manual();
    bool joy_in_clear();
};

#endif
