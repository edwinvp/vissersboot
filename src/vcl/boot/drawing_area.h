//---------------------------------------------------------------------------

#ifndef drawing_areaH
#define drawing_areaH
//---------------------------------------------------------------------------
#include <Graphics.hpp>
#include "gpsloc.h"
//---------------------------------------------------------------------------
class TScreenPos
{
public:
	int x;
	int y;
	bool visible;

	TScreenPos() : x(0), y(0), visible(true) {};
};
//---------------------------------------------------------------------------
class TSphericalPos
{
public:
	float colatitude;
	float r;
	float longitude;

	bool visible;

	TSphericalPos() : colatitude(0), r(0), longitude(0), visible(true) {};
};
//---------------------------------------------------------------------------
class TDrawingArea
{
private:
	Graphics::TBitmap * bmp;

	TGpsLoc center_loc;
	float zoom_factor;

	TColor gps_point_col;

	void LonRing(float lat);
	void LatRing(float lon);
	TSphericalPos GetSphericalPos(TGpsLoc loc);

public:
	TDrawingArea();
	~TDrawingArea();

	void SetZoomFactor(float new_zoom);
	void SetCenterLoc(TGpsLoc loc);
	void SetScreenDims(int w, int h);
	void RenderTo(TCanvas * canv);
	TScreenPos LatLon2XY(TGpsLoc loc);
	void PlotGpsPoint(TGpsLoc loc);
	void Rings();

};
//---------------------------------------------------------------------------
#endif
