#ifndef steeringH
#define steeringH

class CSteering
{
private:
    void do_restrict_dir(float & pid_cv);

public:
    // Auto steering related
    int restrict_dir;
    bool arrived; // TRUE when arriving at the waypoint

    float compass_course;
    float bearing_sp; // calculated initial bearing (from Haversine formulas)
    float pv_used;
    float sp_used;

    double SUBST_SP;
    double SUBST_PV;

	// PID controller vars
	float p_add;
	float i_add;
	float d_add;
	float pid_err;
	
    // ----------------------------------------------------------------------------
    // Auto steer PID-tune parameters
    // ----------------------------------------------------------------------------
    double TUNE_P; // P-action
    double TUNE_I; // I-action
    double TUNE_D; // D-action

    // Test (don't stop steering even after arriving at finish)
    bool dont_stop_steering;

	CSteering();

    float simple_pid(float pv, float sp,
        bool enable_p, float Kp,
        bool enable_i, float Ki,
        bool enable_d, float Kd);	

    void auto_steer();

    float clip_motor(float mtr);

    //!\brief Calculate motor set points as a factor (-1.0 ... +1.0)
    void calc_motor_setpoints(float & motor_l, float & motor_r, float max_speed, float cv_clipped);

    void toggle_dont_stop();
};

#endif