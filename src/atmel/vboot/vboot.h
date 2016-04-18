#ifndef MAIN_H
#define MAIN_H
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

#ifdef WIN32
extern comp_extreme compass_min_x;
extern comp_extreme compass_max_x;
extern comp_extreme compass_min_y;
extern comp_extreme compass_max_y;
extern comp_extreme compass_min_z;
extern comp_extreme compass_max_z;
extern float compass_course;
#endif

//---------------------------------------------------------------------------
#endif

