#ifndef compass_calibrateH
#define compass_calibrateH

#include <avr/io.h>

//---------------------------------------------------------------------------
class TCompassTriple
{
	public:
	int16_t x,y,z;
	TCompassTriple() : x(0), y(0), z(0) {};
};
//---------------------------------------------------------------------------
struct comp_extreme {
	int16_t avg[4];
	int16_t fin;
	int16_t cnt;
};
//---------------------------------------------------------------------------
class CCompassCalibration
{
private:
	void c_update_max(comp_extreme & x, int16_t newval);
	void c_update_min(comp_extreme & x, int16_t newval);
	void c_init_min(comp_extreme & x);
	void c_init_max(comp_extreme & x);
	int16_t c_avg(comp_extreme & x);
	float c_coords_to_angle(float ix, float iz);
	float c_clip_degrees(float d);
		
public:
	float compass_north_offset;
	float compass_course_no_offset;
	float compass_course;

	comp_extreme compass_min_x;
	comp_extreme compass_max_x;
	comp_extreme compass_min_y;
	comp_extreme compass_max_y;
	comp_extreme compass_min_z;
	comp_extreme compass_max_z;
	
	void init();	
	void calibrate(const TCompassTriple & compass_raw);
	float calc_course(const TCompassTriple & compass_raw);
	void set_north();
};

//---------------------------------------------------------------------------

#endif
