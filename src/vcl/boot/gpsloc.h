//---------------------------------------------------------------------------

#ifndef gpslocH
#define gpslocH
//---------------------------------------------------------------------------
#include <System.hpp>
//---------------------------------------------------------------------------
class TGpsLoc
{
private:
	UnicodeString MakeDegreeAndMinutes(double angle) const;
public:
	double lat; // North/South (degrees, positive = north)
	double lon; // East/West (degrees, positive = east)

	TGpsLoc();

	static double toRadians(double deg);
	double lonRadians() const;
	double latRadians() const;

	UnicodeString GetGPRMC() const;
	UnicodeString Lat2NMEA() const;
	UnicodeString Lon2NMEA() const;

};
//---------------------------------------------------------------------------
#endif
