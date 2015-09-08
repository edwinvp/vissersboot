//---------------------------------------------------------------------------

#ifndef gpslocH
#define gpslocH
//---------------------------------------------------------------------------
#include <System.hpp>
//---------------------------------------------------------------------------
class TGpsLoc
{
public:
	float lat; // North/South (degrees, positive = north)
	float lon; // East/West (degrees, positive = east)

	TGpsLoc();

	UnicodeString GetGPRMC() const;
	UnicodeString Lat2NMEA() const;
	UnicodeString Lon2NMEA() const;

};
//---------------------------------------------------------------------------
#endif
