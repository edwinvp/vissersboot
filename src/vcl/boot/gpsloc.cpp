//---------------------------------------------------------------------------
#include <math.h>

#pragma hdrstop

#include "gpsloc.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
//---------------------------------------------------------------------------
#define pi 3.1415926535897932384626433832795
//---------------------------------------------------------------------------
TGpsLoc::TGpsLoc() : lat(0), lon(0)
{
}

UnicodeString TGpsLoc::MakeDegreeAndMinutes(double angle) const
{
	float absolute = fabs(angle);
	int degrees = (int)absolute;
	float mins = (absolute - degrees) * 60.0f;

	UnicodeString extra;
	if (mins < 10)
		extra += L"0";

	UnicodeString s;
	s.printf(L"%02d%s%.5f",degrees, extra.c_str(), mins);
	return s;
}

UnicodeString TGpsLoc::Lat2NMEA() const
{
	UnicodeString s = MakeDegreeAndMinutes(lat);

	if (lat>=0)
		s+=L",N";
	else
		s+=L",S";

	return s;
}

UnicodeString TGpsLoc::Lon2NMEA() const
{
	UnicodeString s = MakeDegreeAndMinutes(lon);

	if (lon>=0)
		s+=L",E";
	else
		s+=L",W";

	return s;
}

UnicodeString TGpsLoc::CMG2NMEA(float course_made_good) const
{
	UnicodeString s;
	s.printf(L"%0.1f",course_made_good);
	while (s.Length()<5) {
		s=L"0" + s;
	}

	return s;
}

UnicodeString TGpsLoc::GetGPRMC(float course_made_good, bool bValidity) const
{
	UnicodeString s,sFormat;

#if 0
	UnicodeString sPrefix(L"$GPRMC");
#else
	UnicodeString sPrefix(L"$GNRMC");
#endif

	if (bValidity)
		sFormat = sPrefix+L",123519,A,%s,%s,000.0,%s,230394,003.1,W";
	else
		sFormat = sPrefix+L",123519,V,%s,%s,000.0,%s,230394,003.1,W";

	s.printf(sFormat.c_str(),
		Lat2NMEA().c_str(),
		Lon2NMEA().c_str(),
		CMG2NMEA(course_made_good).c_str() );

	int checksum = 0;
	for (int i = 2; i <= s.Length(); i++)
		checksum ^= (unsigned char)s[i];

	UnicodeString sChk;
	sChk.printf(L"*%02X\r\n",checksum);

	s+=sChk;

	return s;
}

double TGpsLoc::toRadians(double deg)
{
	return deg / 180.0 * pi;
}

double TGpsLoc::lonRadians() const
{
	return toRadians(lon);
}

double TGpsLoc::latRadians() const
{
	return toRadians(lat);
}

