//---------------------------------------------------------------------------

#ifndef lat_lonH
#define lat_lonH
//---------------------------------------------------------------------------
class CLatLon
{
public:
	float lat; // Latitude in degrees.
	float lon; // Longitude in degrees.

	CLatLon() : lat(0), lon(0) {};

	double toRadians(float angle) const;
	double toDegrees(float angle) const;

	double latRadians() const;
	double lonRadians() const;

	float bearingTo(const CLatLon & point);

};
//---------------------------------------------------------------------------

#endif
