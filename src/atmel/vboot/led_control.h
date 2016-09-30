#ifndef CLedControlH
#define CLedControlH

// Main state machine state definitions
enum TLedMode {
    lmOff = 0,
    lmGpsStatus,
    lmArriveStatus,
    lmSlowBlink,
    lmFastBlink  
};

// LED control related
class CLedControl
{
private:
    bool slow_blink_prev;
    bool fast_blink;
    bool led_signal;
    bool slow_blink;
    unsigned char blink_times;
    unsigned char dark_cnt;

    TLedMode mode;

public:

    CLedControl();
    void set_mode(TLedMode m);
    void update(bool gps_valid, bool arrived);
    void toggle_slow_blink();
    bool done_blinking();
    bool been_dark_for_a_while();
    void blink_x_times(int n);
};

#endif