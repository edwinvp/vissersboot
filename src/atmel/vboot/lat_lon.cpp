﻿//---------------------------------------------------------------------------
#include <math.h>
#ifdef _WIN32
#include <Vcl.h>
#else
#include <stdio.h>
#include <avr/pgmspace.h>
#include "settings.h"
#endif

#ifdef WIN32
#pragma hdrstop
#endif

#include "lat_lon.h"
//---------------------------------------------------------------------------
#ifdef WIN32
#pragma package(smart_init)
#endif
//---------------------------------------------------------------------------
#define pi 3.1415926535897932384626433832795
//---------------------------------------------------------------------------
double CLatLon::toRadians(float angle) const
{
	return angle / 180.0 * pi;
}
//---------------------------------------------------------------------------
double CLatLon::toDegrees(float angle) const
{
	return angle / pi * 180.0;
}
//---------------------------------------------------------------------------
double CLatLon::latRadians() const
{
	return toRadians(lat);
}
//---------------------------------------------------------------------------
double CLatLon::lonRadians() const
{
	return toRadians(lon);
}
//---------------------------------------------------------------------------
/**
 * Returns the (initial) bearing from 'this' point to destination point.
 *
 * @param   {LatLon} point - Latitude/longitude of destination point.
 * @returns {number} Initial bearing in degrees from north.
 *
 * @example
 *     var p1 = new LatLon(52.205, 0.119), p2 = new LatLon(48.857, 2.351);
 *     var b1 = p1.bearingTo(p2); // b1.toFixed(1): 156.2
 */

float CLatLon::bearingTo(const CLatLon & point) {

	double phi1 = latRadians(), phi2 = point.latRadians();
	double deltaLambda = toRadians(point.lon-lon);

	// see http://mathforum.org/library/drmath/view/55417.html
	double y =     sin(deltaLambda) * cos(phi2);
	double x =     cos(phi1) * sin(phi2) -
				  sin(phi1) * cos(phi2) * cos(deltaLambda);

	double theta(0.0);
	if (x!=0 || y!=0)
		theta = atan2(y, x);

	return fmod(toDegrees(theta)+360,360);
};

#ifdef _WIN32
UnicodeString CLatLon::ToString() const
{
	return L"lat: " + FloatToStr(lat) + L" lon: " + FloatToStr(lon);
}
#endif

void CLatLon::clear()
{
	lat=0;
	lon=0;
}

bool CLatLon::empty()
{
	return (lat==0) && (lon==0);
}

#ifndef _WIN32

void CLatLon::print_wp()
{
	if (empty()) {
		b_printf(PSTR("(empty)"));		
	} else {
		b_printf(PSTR("lat=%f"),lat);
		b_printf(PSTR(" lon=%f"),lon);
	}
	b_printf(PSTR("\r\n"));
}
#endif
// ----------------------------------------------------------------------------
