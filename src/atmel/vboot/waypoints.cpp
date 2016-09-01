#include "waypoints.h"

CWayPoints::CWayPoints()
{

}
// ----------------------------------------------------------------------------
bool CWayPoints::set_finish(int memory_no)
{
    gp_finish.clear();

    if (memory_no >= 1 && memory_no <= 3) {
        switch (memory_no) {
            case 1: gp_finish = gp_mem_1; break;
            case 2: gp_finish = gp_mem_2; break;
            case 3: gp_finish = gp_mem_3; break;
        }
    }

    return !gp_finish.empty();
}
// ----------------------------------------------------------------------------
void CWayPoints::store_waypoint(int memory_no)
{
    if (memory_no >= 1 && memory_no <= 3) {
        switch (memory_no) {
            case 1: gp_mem_1 = gp_current; break;
            case 2: gp_mem_2 = gp_current; break;
            case 3: gp_mem_3 = gp_current; break;
        }
    }
}
// ----------------------------------------------------------------------------
void CWayPoints::forget_all()
{
    gp_mem_1.clear();
    gp_mem_2.clear();
    gp_mem_3.clear();
}
