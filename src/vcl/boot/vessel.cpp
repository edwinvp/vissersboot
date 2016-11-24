//---------------------------------------------------------------------------
#include <math.h>

#pragma hdrstop

#include "vessel.h"
//---------------------------------------------------------------------------
#define MIN(a,b) (a < b ? a : b)
//---------------------------------------------------------------------------
#pragma package(smart_init)
//---------------------------------------------------------------------------
#define pi 3.1415926535897932384626433832795f
//---------------------------------------------------------------------------
TVessel::TVessel()
{
	position.lat = 0.0f;
	position.lon = 0.0f;

	// heading: 0=North, 90=East, 180=South, 270=West
	heading = 0.0f;
	bearing_sp = 0.0f;
	compass_course = 0.0f;

	speed = 0.0f;
	motor_left = 0.0f;
	motor_right = 0.0f;
}
//---------------------------------------------------------------------------
void TVessel::Move(float interval)
{
	float s = speed * (interval / 1000.0);
	float h = heading / 360.0 * 2.0 * pi;
	position.lat += cos(h) * s;
	position.lon += sin(h) * s;
}
//---------------------------------------------------------------------------
float TVessel::ClipMotor(float m)
{
	if (m<-1.0)
		return -1.0;
	else if (m>1.0)
		return 1.0;
	else
		return m;
}
//---------------------------------------------------------------------------
void TVessel::CalcSpeedAndHeading()
{
	// Max full power speed is 5 [km/h]

	// Clip motors to a range of -1.0 ... +1.0
	float ml = ClipMotor(motor_left);
	float mr = ClipMotor(motor_right);

	// 1. calculate vector of the effect caused by motor L
	// 2. calculate vector of the effect caused by motor R
	// 3. add vectors togethers
	// 4. apply new heading and speed

	float speed_adj = 0.00001;
	float corner_adj = 0.01;

	// 1. Left motor (on positive throttle) causes a clockwise turn
	float ml_heading = corner_adj * ml * -90.0;
	float ml_speed = speed_adj * ml;

	// 2. Right motor (on positive throttle) causes a counter clockwise turn
	float mr_heading = corner_adj * mr * 90.0;
	float mr_speed = speed_adj * mr;

	// 3+4. Add effects together
	heading = heading + ml_heading + mr_heading;
	speed = ml_speed + mr_speed;
}
//---------------------------------------------------------------------------

