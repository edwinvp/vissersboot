//---------------------------------------------------------------------------

#pragma hdrstop

#include "drawing_area.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)

//---------------------------------------------------------------------------
#define pi 3.1415926535897932384626433832795f
//---------------------------------------------------------------------------
TDrawingArea::TDrawingArea() : bmp(0)
{
	bmp = new Graphics::TBitmap();

	gps_point_col = clRed;

	center_loc.lat = 0;
	center_loc.lon = 0;

	zoom_factor = 1.0f;
}
//---------------------------------------------------------------------------
TDrawingArea::~TDrawingArea()
{
	if (bmp) {
		delete bmp;
		bmp=0;
	}
}
//---------------------------------------------------------------------------
void TDrawingArea::SetScreenDims(int w,int h)
{
	if (bmp->Width != w || bmp->Height != h) {
		bmp->Width = w;
		bmp->Height = h;
	}
}
//---------------------------------------------------------------------------
void TDrawingArea::RenderTo(TCanvas * canv)
{
	if (!bmp->Width || !bmp->Height)
		return;

	bmp->Canvas->Pen->Color = clBlack;
	bmp->Canvas->Brush->Color = clWhite;
	bmp->Canvas->Rectangle(0,0,bmp->Width,bmp->Height);

	TGpsLoc zero,utrecht,amsterdam,rotterdam;

	// zero loc
	zero.lat = 0;
	zero.lon = 0;
	// Utrecht
	utrecht.lat = 52.0906949f;
	utrecht.lon = 5.1220616f;
	// Amsterdam
	amsterdam.lat = 52.3788516f;
	amsterdam.lon = 4.9004368f;
	// Rotterdam
	rotterdam.lat = 51.9290846f;
	rotterdam.lon = 4.4931016f;

	// Kralingse plas
	TGpsLoc kp1, kp2, kp3,kp4;
	// kp1  kp2
	// kp3  kp4
	kp1.lat = 51.9413472f;
	kp1.lon = 4.513745f;
	kp2.lat = 51.9380235f;
	kp2.lon = 4.5250358f;
	kp3.lat = 51.9340276f;
	kp3.lon = 4.5056552f;
	kp4.lat = 51.9281928f;
	kp4.lon = 4.5183095f;


	TGpsLoc start,finish;
	// midden in de plas
	start.lat = 51.9364818f;
	start.lon = 4.5162849f;
	// wat verder weg (voor de tuin)
	finish.lat = 51.932278f;
	finish.lon = 4.521163f;

	SetZoomFactor(6590);
	SetCenterLoc(kp4);

#if 1
	PlotGpsPoint(zero);
	gps_point_col = clYellow;
	PlotGpsPoint(utrecht);
	gps_point_col = clRed;
	PlotGpsPoint(amsterdam);
	gps_point_col = clGreen;
	PlotGpsPoint(rotterdam);
	gps_point_col = clBlack;
	PlotGpsPoint(zero);

	gps_point_col = clBlue;
	PlotGpsPoint(kp1);
	PlotGpsPoint(kp2);
	PlotGpsPoint(kp3);
	PlotGpsPoint(kp4);

	gps_point_col = clWhite;
	PlotGpsPoint(start);
	PlotGpsPoint(finish);

#endif

	//Rings();
	canv->Draw(0,0,bmp);
}
//---------------------------------------------------------------------------
void TDrawingArea::Rings()
{
	gps_point_col = clRed;

	for (int lat=-180; lat<=180; lat+=10)
		LonRing(lat);

	gps_point_col = clYellow;

	for (int lon=-180; lon<=180; lon+=10)
		LatRing(lon);
}
//---------------------------------------------------------------------------
void TDrawingArea::LonRing(float lat)
{
	for (int lon(-180); lon<=180; lon+=5) {
		TGpsLoc l;
		l.lat = lat;
		l.lon = lon;
		PlotGpsPoint(l);
	}
}
//---------------------------------------------------------------------------
void TDrawingArea::LatRing(float lon)
{
	for (int lat(-180); lat<=180; lat+=5) {
		TGpsLoc l;
		l.lat = lat;
		l.lon = lon;
		PlotGpsPoint(l);
	}
}
//---------------------------------------------------------------------------
TSphericalPos TDrawingArea::GetSphericalPos(TGpsLoc loc)
{
	TGpsLoc scrolled;
	scrolled.lat = loc.lat - center_loc.lat;
	scrolled.lon = loc.lon - center_loc.lon;

	float latitude = scrolled.lat / 180.0 * pi;
	float half_pi = 0.5*pi;

	float radius_of_world(1.0f);
	if (bmp->Width < bmp->Height)
		radius_of_world = 0.3f * bmp->Width;
	else
		radius_of_world = 0.3f * bmp->Height;

	TSphericalPos sp;
	sp.r = radius_of_world;
	sp.longitude = scrolled.lon / 180.0 * pi;
	sp.colatitude = half_pi - latitude;

	sp.visible =
		(scrolled.lat >= -180.0) && (scrolled.lat <= 180.0) &&
		(scrolled.lon >= -180.0) && (scrolled.lon <= 180.0);

	return sp;
}
//---------------------------------------------------------------------------
TScreenPos TDrawingArea::LatLon2XY(TGpsLoc loc)
{
	TScreenPos scr;

	TSphericalPos sp = GetSphericalPos(loc);

	float x_offset = 0.5f * bmp->Width;
	float y_offset = 0.5f * bmp->Height;

	float r = sp.r;
	float x = r * sin(sp.colatitude) * sin(sp.longitude);
	float y = r * cos(sp.colatitude);
	float z = r * sin(sp.colatitude) * cos(sp.longitude);

	// The 3d to 2d projection assumes
	// that the 'camera' is at x=0, y=0, z=0
	// So, we push back our object
	z += sp.r*1.0;

	float focal_length = 600.0;

	scr.x = x_offset + x * focal_length / (focal_length + z) * zoom_factor;
	scr.y = y_offset - y * focal_length / (focal_length + z) * zoom_factor;
	scr.visible = sp.visible;

	return scr;
}
//---------------------------------------------------------------------------
void TDrawingArea::PlotGpsPoint(TGpsLoc loc)
{
	TScreenPos sp = LatLon2XY(loc);

	if (sp.visible) {
		bmp->Canvas->Pen->Color = clBlack;
		bmp->Canvas->Brush->Color = gps_point_col;
		bmp->Canvas->Ellipse(sp.x-3,sp.y-3,sp.x+3,sp.y+3);
	}
}
//---------------------------------------------------------------------------
void TDrawingArea::SetCenterLoc(TGpsLoc loc)
{
	center_loc = loc;
}
//---------------------------------------------------------------------------
void TDrawingArea::SetZoomFactor(float new_zoom)
{
	zoom_factor = new_zoom;
}
//---------------------------------------------------------------------------

