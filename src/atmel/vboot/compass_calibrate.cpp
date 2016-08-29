#include "compass_calibrate.h"
#include <math.h>

#define pi 3.1415926535897932384626433832795
#define two_pi 6.283185307179586476925286766559

void CCompassCalibration::c_init_min(comp_extreme & x)
{
	x.fin = 32767;
	for (int i(0);i<4;i++)
	x.avg[i]=0;
	x.cnt = 0;
};

void CCompassCalibration::c_init_max(comp_extreme & x)
{
	x.fin = -32768;
	for (int i(0);i<4;i++)
	x.avg[i]=0;
	x.cnt = 0;
};

void CCompassCalibration::init()
{
	compass_course_no_offset = 0.0f;
	compass_course = 0.0f;
	
	c_init_min(compass_min_x);
	c_init_max(compass_max_x);
	c_init_min(compass_min_y);
	c_init_max(compass_max_y);
	c_init_min(compass_min_z);
	c_init_max(compass_max_z);
}

int16_t CCompassCalibration::c_avg(comp_extreme & x)
{
	long int sum = 0;
	for (int i(0);i<4;i++)
	sum += x.avg[i];

	int16_t avg = sum >> 2;
	return avg;
}

void CCompassCalibration::c_update_min(comp_extreme & x, int16_t newval)
{
	if (newval == 0x7fff || newval == 0x8000 || newval == 0)
	return;

	if (x.cnt > 3) {
		int16_t avg = c_avg(x);
		if (avg < x.fin)
		x.fin = avg;
		x.cnt=0;
	}
	x.avg[x.cnt++] = newval;
}

void CCompassCalibration::c_update_max(comp_extreme & x, int16_t newval)
{
	if (newval == 0x7fff || newval == 0x8000 || newval == 0)
	return;
	if (x.cnt > 3) {
		int16_t avg = c_avg(x);
		if (avg > x.fin)
		x.fin = avg;
		x.cnt=0;
	}
	x.avg[x.cnt++] = newval;
}

void CCompassCalibration::calibrate(const TCompassTriple & compass_raw)
{
	c_update_min(compass_min_x,compass_raw.x);
	c_update_max(compass_max_x,compass_raw.x);
	c_update_min(compass_min_y,compass_raw.y);
	c_update_max(compass_max_y,compass_raw.y);
	c_update_min(compass_min_z,compass_raw.z);
	c_update_max(compass_max_z,compass_raw.z);
}

float CCompassCalibration::c_clip_degrees(float d)
{
	d = fmod(d,360.0f);

	if (d < 0)
	d += 360.0;

	return d;
}

float CCompassCalibration::c_coords_to_angle(float ix, float iz)
{
	float d(0.0f);
	if (ix>0)
		d = atan(iz/ix);
	else if (ix<0 && iz >=0)
		d = atan(iz/ix) + pi;
	else if (ix<0 && iz < 0)
		d = atan(iz/ix) - pi;
	else if (ix==0 && iz > 0)
		d = pi / 2.0;
	else if (ix==0 && iz < 0)
		d = -pi / 2.0;
	else if (ix==0 && iz == 0)
		d = 0;

	d = 90.0 + d / two_pi * 360.0;
	d += 180.0;
	d = c_clip_degrees(d);
	return d;
}

float CCompassCalibration::calc_course(const TCompassTriple & compass_raw)
{
	compass_course_no_offset = 0.0f;
	
	TCompassTriple centered;
	centered.x=0;
	centered.y=0;
	centered.z=0;

	float w = compass_max_x.fin - compass_min_x.fin;
	float h = compass_max_z.fin - compass_min_z.fin;

	if (w>=0 && h>=0) {
		centered.x = (compass_raw.x - compass_min_x.fin - (w/2.0));
		centered.z = -(compass_raw.z - compass_min_z.fin - (h/2.0));
	}

	if (w>0 && h>0) {
		float ix = (float)centered.x / (w/2.0);
		float iz = (float)centered.z / (h/2.0);

		compass_course_no_offset = c_coords_to_angle(ix,iz);
	}

	compass_course = c_clip_degrees(compass_course_no_offset + compass_north_offset);

	return compass_course;
}

void CCompassCalibration::set_north()
{
	compass_north_offset = 0 - compass_course_no_offset;
}