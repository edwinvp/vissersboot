//---------------------------------------------------------------------------
#ifndef vesselH
#define vesselH
//---------------------------------------------------------------------------
#include "gpsloc.h"
//---------------------------------------------------------------------------
class TVessel
{
public:
	TGpsLoc position;
	float heading;
	float speed;
	float motor_left;
	float motor_right;

	TVessel();
};

//---------------------------------------------------------------------------
#endif
