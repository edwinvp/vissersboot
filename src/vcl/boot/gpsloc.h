//---------------------------------------------------------------------------

#ifndef gpslocH
#define gpslocH
//---------------------------------------------------------------------------
#include <System.hpp>
//---------------------------------------------------------------------------
class TGpsLoc
{
public:
	double lat; // North/South (degrees, positive = north)
	double lon; // East/West (degrees, positive = east)

	TGpsLoc();

	UnicodeString GetGPRMC() const;
	UnicodeString Lat2NMEA() const;
	UnicodeString Lon2NMEA() const;

};
//---------------------------------------------------------------------------
#endif
