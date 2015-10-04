//---------------------------------------------------------------------------
#ifndef vesselH
#define vesselH
//---------------------------------------------------------------------------
#include "gpsloc.h"
//---------------------------------------------------------------------------
class TVessel
{
private:
	float ClipMotor(float m);
public:
	TGpsLoc position;
	float heading; // degrees (0=N, 90=E, 180=S, 270=W)
	float speed;
	float motor_left; // -1...1 (-1=full speed reverse, 1=full speed ahead)
	float motor_right; // -1...1 (-1=full speed reverse, 1=full speed ahead)

	void Move(float interval);
	void CalcSpeedAndHeading();

	TVessel();
};

//---------------------------------------------------------------------------
#endif
