//---------------------------------------------------------------------------
#include <math.h>

#pragma hdrstop

#include "gpsloc.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)

TGpsLoc::TGpsLoc() : lat(0), lon(0)
{
}

UnicodeString TGpsLoc::Lat2NMEA() const
{
	float absolute = fabs(lat);
	int degrees = (int)absolute;
	float mins = (absolute - degrees) * 60.0f;

	UnicodeString extra;
	if (mins < 10)
		extra += L"0";

	UnicodeString s;
	s.printf(L"%02d%s%.5f",degrees, extra.c_str(), mins);

	if (lat>=0)
		s+=L",N";
	else
		s+=L",S";

	return s;
}

UnicodeString TGpsLoc::Lon2NMEA() const
{
	float absolute = fabs(lon);
	int degrees = (int)absolute;
	float mins = (absolute - degrees) * 60.0f;

	UnicodeString extra;
	if (mins < 10)
		extra += L"00";
	else if (mins < 100)
		extra += L"0";

	UnicodeString s;
	s.printf(L"%03d%s%.5f",degrees, extra.c_str(), mins);

	if (lon>=0)
		s+=L",E";
	else
		s+=L",W";

	return s;
}

UnicodeString TGpsLoc::GetGPRMC() const
{
	UnicodeString s;
	s.printf(L"$GPRMC,123519,A,%s,%s,000.0,000.0,230394,003.1,W",
		Lat2NMEA().c_str(),
		Lon2NMEA().c_str() );

	int checksum = 0;
	for (int i = 2; i <= s.Length(); i++)
		checksum ^= (unsigned char)s[i];

	UnicodeString sChk;
	sChk.printf(L"*%02X\r\n",checksum);

	s+=sChk;

	return s;
}
