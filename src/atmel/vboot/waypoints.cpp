#include "settings.h"
#include "waypoints.h"
#include <avr/eeprom.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include "crc.h"

#define AR_SIZE (1 + 3 * 2 * (sizeof(float)/sizeof(uint16_t)))

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
// ----------------------------------------------------------------------------
void CWayPoints::load_waypoints()
{
    b_printf(PSTR("Loading waypoint settings..."));
   
    uint16_t rec[AR_SIZE];
    
    unsigned int addr=WAYPOINT_EEPROM_OFFSET;
    for (unsigned int i(0); i<AR_SIZE; i++) {
        eeprom_busy_wait();
        rec[i]=eeprom_read_word((uint16_t*)addr);
        addr+=2;        
    }

    uint16_t chk = crc16((unsigned char*)rec,6 * sizeof(float));
    
    if (chk == rec[AR_SIZE-1]) {

        float * fp = reinterpret_cast<float*>(&rec[0]);
        gp_mem_1.lat=fp[0];
        gp_mem_1.lon=fp[1];
        gp_mem_2.lat=fp[2];
        gp_mem_2.lon=fp[3];
        gp_mem_3.lat=fp[4];
        gp_mem_3.lon=fp[5];               
        
        b_printf(PSTR("OK\r\n"));
        
    } else {
        b_printf(PSTR("FAILED (checksum)\r\n"));
    }

}
// ----------------------------------------------------------------------------
void CWayPoints::store_waypoints()
{
    b_printf(PSTR("Storing waypoints to EEPROM.\r\n"));
    
    uint16_t rec[AR_SIZE];

    float * fp = reinterpret_cast<float*>(&rec[0]);
    fp[0]=gp_mem_1.lat;
    fp[1]=gp_mem_1.lon;
    fp[2]=gp_mem_2.lat;
    fp[3]=gp_mem_2.lon;
    fp[4]=gp_mem_3.lat;
    fp[5]=gp_mem_3.lon;
    rec[AR_SIZE-1]=crc16(reinterpret_cast<unsigned char*>(rec),6 * sizeof(float));
    
    unsigned int addr=WAYPOINT_EEPROM_OFFSET;
    for (unsigned int i(0); i<AR_SIZE; i++) {
        eeprom_busy_wait();
        eeprom_write_word((uint16_t*)addr,rec[i]);
        addr+=2;
    }

}
// ----------------------------------------------------------------------------
