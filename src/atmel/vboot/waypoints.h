#ifndef waypointsH
#define waypointsH

#include "lat_lon.h"

#define NUM_WAYPOINTS 5

class CWayPoints
{
private:

public:
    // Memorized GPS positions
    CLatLon gp_mem_1; // memorized GPS position 1 (usually 'home')
    CLatLon gp_mem_2; // memorized GPS position 2
    CLatLon gp_mem_3; // memorized GPS position 3
	CLatLon gp_mem_4; // memorized GPS position 4
	CLatLon gp_mem_5; // memorized GPS position 5
    // Current / destination GPS positions
    CLatLon gp_current; // current GPS position (may be stale or invalid!)
    CLatLon gp_start; // GPS position when auto steering was switched on
    CLatLon gp_finish; // auto steering target GPS position

    CWayPoints(); 
    void store_waypoint(int memory_no);
    bool set_finish(int memory_no);
    void forget_all();
    void load_waypoints();
    void store_waypoints();
	void print_waypoints();
   
};

#endif