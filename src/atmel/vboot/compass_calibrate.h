#ifndef compass_calibrateH
#define compass_calibrateH

#ifndef _WIN32
#include <avr/io.h>
#else
#include "faketypes.h"
#endif
#include "compass_rawvalues.h"

//---------------------------------------------------------------------------
enum ECalibrationState {
    csNotCalibrated, csCenterDetect, csTurn1, csTurn2, csFinish, csCalibrated
};
//---------------------------------------------------------------------------
struct comp_extreme {
	int16_t avg[4];
	int16_t fin_min;
    int16_t fin_max;
	int16_t cnt;
};
//---------------------------------------------------------------------------
class CCompassCalibration
{
private:
    ECalibrationState m_cal_state;
    int prev_quadrant;
    int quadrant_counter;
    int stable_quadrant;
    int prev_stable_quadrant;
    int cw_quadrants;
    int step_time;

	void update_min_max(comp_extreme & x, int16_t newval);
	void init_min_max(comp_extreme & x);
	int16_t c_avg(comp_extreme & x);
	float coords_to_angle(float ix, float iz);
	float clip_degrees(float d);
    void SetCalState(ECalibrationState new_state);
    int get_quadrant();
    void PrintCalState();
    void DetectCwTurn();
		
public:
    bool calibration_mode;

	int raw_x;
	int raw_y;
	int raw_z;

    float m_ix;
    float m_iz;

	float compass_north_offset;
	float compass_course_no_offset;
	float compass_course;

	comp_extreme mm_x;
	comp_extreme mm_y;
	comp_extreme mm_z;

    CCompassCalibration();	
	void init();	
	void calibrate(const TCompassTriple & compass_raw);
	float calc_course(const TCompassTriple & compass_raw);
	void set_north();
    void store_calibration();
    void load_calibration();
    void reset_compass_calibration();
    void set_true_north();
    void toggle_calibration_mode();
	void print_cal();
    void update100ms();
    ECalibrationState get_state();
};

//---------------------------------------------------------------------------

#endif
