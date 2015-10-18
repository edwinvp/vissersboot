//---------------------------------------------------------------------------

#pragma hdrstop

#include "drawing_area.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)

//---------------------------------------------------------------------------
#define pi 3.1415926535897932384626433832795
//---------------------------------------------------------------------------
TDrawingArea::TDrawingArea() : bmp(0)
{
	bmp = new Graphics::TBitmap();

	gps_point_col = clRed;

	center_loc.lat = 0;
	center_loc.lon = 0;

	focal_length = 500.0;

	zoom_factor = 1.0f;

	radius_of_world = 1.0f;

	real_earth_radius = 6371000.0f;

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
void TDrawingArea::CalcRadiusOfWorld()
{
	radius_of_world = 200.0f;
}
//---------------------------------------------------------------------------
void TDrawingArea::AddDrawPoint(TDrawPoint dp)
{
	draw_points.push_back(dp);
}
//---------------------------------------------------------------------------
void TDrawingArea::RenderTo(TCanvas * canv, TVessel & vessel,
	const std::vector<TGpsLoc> & vessel_path)
{
	if (!bmp->Width || !bmp->Height)
		return;

	bmp->Canvas->Pen->Color = clBlack;
	bmp->Canvas->Brush->Color = clWhite;
	bmp->Canvas->Rectangle(0,0,bmp->Width,bmp->Height);

    CalcRadiusOfWorld();

	for (unsigned int idx(0);idx<draw_points.size();++idx) {
		gps_point_col = draw_points[idx].clr;
		PlotGpsPoint(draw_points[idx].loc);
	}

	bool bFirst(true);
	TGpsLoc a,b;
	for (unsigned int idx(0);idx<vessel_path.size();++idx) {
		b = vessel_path[idx];
		if (bFirst) {
			bFirst=false;
		} else {
			PlotGpsLine(a,b);
		}
		a = b;
	}



#if 1
/*	PlotGpsPoint(zero);
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
	PlotGpsPoint(finish);*/

	PlotVessel(vessel);

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
void TDrawingArea::LonRing(double lat)
{
	for (int lon(-180); lon<=180; lon+=5) {
		TGpsLoc l;
		l.lat = lat;
		l.lon = lon;
		PlotGpsPoint(l);
	}
}
//---------------------------------------------------------------------------
void TDrawingArea::LatRing(double lon)
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

	double latitude = scrolled.latRadians();
	double half_pi = 0.5*pi;

	TSphericalPos sp;
	sp.r = radius_of_world;
	sp.longitude = scrolled.lonRadians();
	sp.colatitude = half_pi - latitude;

	sp.visible =
		(scrolled.lat >= -180.0) && (scrolled.lat <= 180.0) &&
		(scrolled.lon >= -180.0) && (scrolled.lon <= 180.0);

	return sp;
}
//---------------------------------------------------------------------------
TGpsLoc TDrawingArea::SearchPnt(TScreenPos sp, TGpsLoc g_center,
	double area, double step)
{
	TGpsLoc g_closest = g_center;
	bool first(true);

	double dist(0.0);
	double closest_dist(0.0);

	double min_lat = g_center.lat - area;
	double max_lat = g_center.lat + area;
	double min_lon = g_center.lon - area;
	double max_lon = g_center.lon + area;

	for (double lat(min_lat); lat<=(max_lat); lat+=step) {
		for (double lon(min_lon); lon<=(max_lon); lon+=step) {

			TGpsLoc g_cur_loc;
			g_cur_loc.lat = lat;
			g_cur_loc.lon = lon;

			TScreenPos sp_cur_loc = LatLon2XY(g_cur_loc);

			if (sp_cur_loc.visible) {
				double dx = fabs(sp_cur_loc.x - sp.x);
				double dy = fabs(sp_cur_loc.y - sp.y);
				double dist = sqrt(dx*dx+dy*dy);

				if (first || dist < closest_dist) {
					first=false;
					g_closest = g_cur_loc;
					closest_dist = dist;

					if (dist == 0)
						return g_closest;
				}
			}
		}
	}

	return g_closest;
}

//---------------------------------------------------------------------------
TGpsLoc TDrawingArea::XY2LatLon(TScreenPos sp)
{
	TGpsLoc g_closest;
	g_closest = SearchPnt(sp, g_closest, 90.0, 5.0);
	g_closest = SearchPnt(sp, g_closest, 5.0, 0.1);
	g_closest = SearchPnt(sp, g_closest, 0.1, 0.01);
	g_closest = SearchPnt(sp, g_closest, 0.01, 0.001);
	g_closest = SearchPnt(sp, g_closest, 0.001, 0.0001);
	g_closest = SearchPnt(sp, g_closest, 0.0001, 0.00001);
	g_closest = SearchPnt(sp, g_closest, 0.00001, 0.000001);
	g_closest = SearchPnt(sp, g_closest, 0.000001, 0.0000001);

	return g_closest;
}
//---------------------------------------------------------------------------
TScreenPos TDrawingArea::LatLon2XY(TGpsLoc loc)
{
	TScreenPos scr;

	TSphericalPos sp = GetSphericalPos(loc);

	double x_offset = 0.5f * bmp->Width;
	double y_offset = 0.5f * bmp->Height;

	double r = sp.r;
	double x = r * sin(sp.colatitude) * sin(sp.longitude);
	double y = r * cos(sp.colatitude);
	double z = r * sin(sp.colatitude) * cos(sp.longitude);

	// The 3d to 2d projection assumes
	// that the 'camera' is at x=0, y=0, z=0
	// So, we push back our object
	z += radius_of_world*1.5;

	scr.x = x_offset + x * focal_length / (focal_length + z) * zoom_factor;
	scr.y = y_offset - y * focal_length / (focal_length + z) * zoom_factor;
	scr.visible = sp.visible;

	return scr;
}
//---------------------------------------------------------------------------
void TDrawingArea::PlotGpsLine(const TGpsLoc a,const TGpsLoc b)
{
	TScreenPos sp_a = LatLon2XY(a);
	TScreenPos sp_b = LatLon2XY(b);

	if (sp_a.visible && sp_b.visible) {
		bmp->Canvas->Pen->Color = clBlack;
		bmp->Canvas->Brush->Color = gps_point_col;
		bmp->Canvas->MoveTo(sp_a.x,sp_a.y);
		bmp->Canvas->LineTo(sp_b.x,sp_b.y);
	}

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
double TDrawingArea::PixelsToDeg(int px)
{
	return px;
}
//---------------------------------------------------------------------------
float TDrawingArea::DistanceToDegrees(float distance_m)
{
	return distance_m / real_earth_radius * radius_of_world;
}
//---------------------------------------------------------------------------
void TDrawingArea::DrawArrow(TGpsLoc base, TGpsLoc arrow_head)
{
	TScreenPos spA = LatLon2XY(base);
	TScreenPos spB = LatLon2XY(arrow_head);

	TScreenPos ah1;
	TScreenPos ah2;

	float dx = spB.x - spA.x;
	float dy = spB.y - spA.y;

	float r = sqrt(dx*dx + dy*dy);
	float p = atan2(dy,dx);

	float ah_width = 15;
	float r_inner = r - ah_width;
	float q = tan(ah_width / r_inner);

	bmp->Canvas->Brush->Color = clFuchsia;
	ah1.x = spA.x + r_inner*cos(p - q);
	ah1.y = spA.y + r_inner*sin(p - q);

	ah2.x = spA.x + r_inner*cos(p + q);
	ah2.y = spA.y + r_inner*sin(p + q);

	bmp->Canvas->MoveTo(spA.x,spA.y);
	bmp->Canvas->LineTo(spB.x,spB.y);
	bmp->Canvas->MoveTo(ah1.x,ah1.y);
	bmp->Canvas->LineTo(spB.x,spB.y);
	bmp->Canvas->LineTo(ah2.x,ah2.y);

}
//---------------------------------------------------------------------------
void TDrawingArea::GpsRound(TGpsLoc position, double rr)
{
	for (int deg(0); deg<360; deg+=5) {
		TGpsLoc a(position);
		double degRadians = TGpsLoc::toRadians(deg);
		a.lat += rr*cos(degRadians);
		a.lon += rr*sin(degRadians);
		PlotGpsPoint(a);
	}
}
//---------------------------------------------------------------------------
void TDrawingArea::GpsRect(TGpsLoc position, double zz)
{
	TGpsLoc a(position);
	a.lon-=zz;
	a.lat-=zz;

	TGpsLoc b(position);
	b.lon+=zz;
	b.lat-=zz;

	TGpsLoc c(position);
	c.lon+=zz;
	c.lat+=zz;

	TGpsLoc d(position);
	d.lon-=zz;
	d.lat+=zz;

	PlotGpsPoint(a);
	PlotGpsPoint(b);
	PlotGpsPoint(c);
	PlotGpsPoint(d);
}
//---------------------------------------------------------------------------
void TDrawingArea::DrawShip(TGpsLoc position, float heading)
{
	double ship_len = 1.0f; // meter
	double ship_width = 0.2f; // meter

	double r_len = DistanceToDegrees(ship_len);
	double r_width = DistanceToDegrees(ship_width);

	double heading_rad = TGpsLoc::toRadians(heading);
	TGpsLoc g_bow2(position);
	g_bow2.lat += r_len*cos(heading_rad);
	g_bow2.lon += r_len*sin(heading_rad);

	// Shape of the vessel:
	//
	//  sp_aftl                   sp_bow1
	//
	//                                    sp_bow2
	//
	//  sp_aft2                   sp_bow2

	TGpsLoc g_aft1(position);
	g_aft1.lat += r_width*cos(heading_rad - 0.5*pi);
	g_aft1.lon += r_width*sin(heading_rad - 0.5*pi);

	TGpsLoc g_aft2(position);
	g_aft2.lat += r_width*cos(heading_rad + 0.5*pi);
	g_aft2.lon += r_width*sin(heading_rad + 0.5*pi);

	float len_to_bow_start = 0.8*r_len;
	float r_bow2 = sqrt(len_to_bow_start*len_to_bow_start + r_width*r_width);
	float b_rad = heading_rad;
	float b_rad_diff = atan(ship_len / ship_width);
	TGpsLoc g_bow1(position);
	g_bow1.lat += r_bow2*cos(heading_rad - 0.5*pi + b_rad_diff);
	g_bow1.lon += r_bow2*sin(heading_rad - 0.5*pi + b_rad_diff);

	TGpsLoc g_bow3(position);
	g_bow3.lat += r_bow2*cos(heading_rad + 0.5*pi - b_rad_diff);
	g_bow3.lon += r_bow2*sin(heading_rad + 0.5*pi - b_rad_diff);


	TScreenPos sp_aft1 = LatLon2XY(g_aft1);
	TScreenPos sp_aft2 = LatLon2XY(g_aft2);
	TScreenPos sp_bow1 = LatLon2XY(g_bow1);
	TScreenPos sp_bow2 = LatLon2XY(g_bow2);
	TScreenPos sp_bow3 = LatLon2XY(g_bow3);

	bmp->Canvas->Pen->Color = clGray;

	bmp->Canvas->MoveTo(sp_aft1.x,sp_aft1.y);
	bmp->Canvas->LineTo(sp_aft2.x,sp_aft2.y);
	bmp->Canvas->LineTo(sp_bow3.x,sp_bow3.y);
	bmp->Canvas->LineTo(sp_bow2.x,sp_bow2.y);
	bmp->Canvas->LineTo(sp_bow1.x,sp_bow1.y);
	bmp->Canvas->LineTo(sp_aft1.x,sp_aft1.y);
}
//---------------------------------------------------------------------------
double TDrawingArea::Width2Lat()
{
	TScreenPos m;
	m.x = bmp->Width/2;
	m.y = bmp->Width/2;
	TScreenPos n(m);
	n.x += bmp->Width/10;

	TGpsLoc g_m = XY2LatLon(m);
	TGpsLoc g_n = XY2LatLon(n);
	double dlat = fabs(g_m.lat - g_n.lat);
	double dlon = fabs(g_m.lon - g_n.lon);
	double r = sqrt(dlat*dlat+dlon*dlon);

	return r * 10.0;
}
//---------------------------------------------------------------------------
void TDrawingArea::PlotVessel(TVessel & vessel)
{
	gps_point_col = clFuchsia;

	PlotGpsPoint(vessel.position);

	double r = Width2Lat() * 0.2;
	if (r<0.000001)
		r=0.000001;

	DrawShip(vessel.position, vessel.heading);

	// facing to heading (where the boat is pointing to)
	bmp->Canvas->Pen->Color=clSilver;
	float p = TGpsLoc::toRadians(vessel.heading);
	TGpsLoc arrow_head_pos;
	arrow_head_pos = vessel.position;
	arrow_head_pos.lat += r*cos(p);
	arrow_head_pos.lon += r*sin(p);
	DrawArrow(vessel.position,arrow_head_pos);

	// heading set point (where we should go to)
	bmp->Canvas->Pen->Color=clBlue;
	p = TGpsLoc::toRadians(vessel.bearing_sp);
	arrow_head_pos = vessel.position;
	arrow_head_pos.lat += r*cos(p);
	arrow_head_pos.lon += r*sin(p);
	DrawArrow(vessel.position,arrow_head_pos);


}

