//---------------------------------------------------------------------------

#ifndef drawing_areaH
#define drawing_areaH
//---------------------------------------------------------------------------
#include <Graphics.hpp>
#include <vector>
#include "gpsloc.h"
#include "vessel.h"
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
	double colatitude;
	double r;
	double longitude;

	bool visible;

	TSphericalPos() : colatitude(0), r(0), longitude(0), visible(true) {};
};
//---------------------------------------------------------------------------
class TDrawPoint
{
public:
	TColor clr;
	TGpsLoc loc;

	TDrawPoint() : clr(clBlack) {};
};
//---------------------------------------------------------------------------
class TDrawingArea
{
private:
	Graphics::TBitmap * bmp;

	TGpsLoc center_loc;
	float zoom_factor;
	float radius_of_world;
	float real_earth_radius; // meters
	double focal_length;

	std::vector<TDrawPoint> draw_points;

	TColor gps_point_col;

	void LonRing(double lat);
	void LatRing(double lon);
	TSphericalPos GetSphericalPos(TGpsLoc loc);
	void CalcRadiusOfWorld();
	float DistanceToDegrees(float distance_m);
	double PixelsToDeg(int px);
	void DrawArrow(TGpsLoc base, TGpsLoc arrow_head);
	void DrawShip(TGpsLoc position, float heading);
	void GpsRect(TGpsLoc position, double zz);
	void GpsRound(TGpsLoc position, double rr);
	double Width2Lat();

	TGpsLoc SearchPnt(TScreenPos sp, TGpsLoc g_center, double area, double step);

public:
	TDrawingArea();
	~TDrawingArea();

	void AddDrawPoint(TDrawPoint dp);

	void SetZoomFactor(float new_zoom);
	void SetCenterLoc(TGpsLoc loc);
	void SetScreenDims(int w, int h);
	void RenderTo(TCanvas * canv, TVessel & vessel,
		const std::vector<TGpsLoc> & vessel_path);

	TScreenPos LatLon2XY(TGpsLoc loc);
	TGpsLoc XY2LatLon(TScreenPos sp);

	void PlotGpsPoint(TGpsLoc loc);
	void PlotGpsLine(const TGpsLoc a,const TGpsLoc b);
	void PlotVessel(TVessel & vessel);
	void Rings();

};
//---------------------------------------------------------------------------
#endif
