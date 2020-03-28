#ifndef steeringH
#define steeringH

class CPidParams
{
public:
	bool p_enable; // Enable P action
	bool i_enable; // Enable I action
	bool d_enable; // Enable D action

	double TUNE_P; // P-action
	double TUNE_I; // I-action
	double TUNE_D; // D-action

	// How much steering to allow under any circumstances (0..1 factor)
	float max_steering;

	CPidParams() : p_enable(true), i_enable(true), d_enable(false),
		TUNE_P(0), TUNE_I(0), TUNE_D(0), max_steering(0.9) {
	}
};

class CSteering
{
private:
	float motor_l;
	float motor_r;

	void CalculatePidError(float pv, float sp);
	void SetMotorSpeeds(float motor_l, float motor_r);

public:
	// Auto steering related
	int choosen_dir;
	bool arrived; // TRUE when arriving at the waypoint

	float compass_course_pv;
    float bearing_sp; // calculated initial bearing (from Haversine formulas)
	float cv_clipped;
    
	// PID controller vars
	float p_add;
	float i_add;
	float d_add;
	float pid_err;

	// Max speed - normally 1.0 --> 100 [%]
	float global_max_speed;
	
    // ----------------------------------------------------------------------------
    // Auto steer PID-tune parameters
	// ----------------------------------------------------------------------------
	// For initially pointing the vessel in the right direction
	CPidParams pid_aggressive;
	// For normal sailing
	CPidParams pid_normal;

    // Test (don't stop steering even after arriving at finish)
	bool dont_stop_steering;

	// When false: inhibit motor setpoints (keep PWM pulse in the middle),
	// (motors will generally not be moving).
	// When true: pass on manual and automatic motors set points.
	bool m_output_enable;

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
	void reset_i_action();
	void manual_steering(unsigned int mot_L_dc,unsigned int mot_R_dc);
	bool motor_running();
	void do_reverse_thrust();
	float get_motor_L_perc();
	float get_motor_R_perc();
	void set_output_enable(bool output_enable);

	void load_calibration();
	void save_calibration();
	void choose_direction();

    static void SetPwm(float ml, float mr);
};

#endif