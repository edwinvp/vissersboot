#ifndef joystickH
#define joystickH

class CJoystick
{
private:
	
public:
    CJoystick();

    bool in_center(unsigned int j);
    bool in_max(unsigned int j);
    bool in_min(unsigned int j);
    bool in_goto();
    bool in_store();
    bool in_goto_store_center();
    bool in_manual();
    bool in_clear();
    int to_perc(unsigned int raw);
};

#endif
