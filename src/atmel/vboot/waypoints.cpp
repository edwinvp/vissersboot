#include "settings.h"
#include "waypoints.h"
#include <stdio.h>
#ifndef _WIN32
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#else
#include "fakeio.h"
#endif
#include "crc.h"

#define NUM_WP_FLOATS (NUM_WAYPOINTS * 2)
#define AR_SIZE (1 + NUM_WP_FLOATS * (sizeof(float)/sizeof(uint16_t)))

CWayPoints::CWayPoints()
{

}
// ----------------------------------------------------------------------------
bool CWayPoints::set_finish(int memory_no)
{
    gp_finish.clear();

    if (memory_no >= 1 && memory_no <= NUM_WAYPOINTS) {
        switch (memory_no) {
            case 1: gp_finish = gp_mem_1; break;
            case 2: gp_finish = gp_mem_2; break;
            case 3: gp_finish = gp_mem_3; break;
			case 4: gp_finish = gp_mem_4; break;
			case 5: gp_finish = gp_mem_5; break;
        }
    }

    return !gp_finish.empty();
}
// ----------------------------------------------------------------------------
void CWayPoints::store_waypoint(int memory_no)
{
    if (memory_no >= 1 && memory_no <= NUM_WAYPOINTS) {
        switch (memory_no) {
            case 1: gp_mem_1 = gp_current; break;
            case 2: gp_mem_2 = gp_current; break;
            case 3: gp_mem_3 = gp_current; break;
			case 4: gp_mem_4 = gp_current; break;
			case 5: gp_mem_5 = gp_current; break;
        }
    }
}
// ----------------------------------------------------------------------------
void CWayPoints::forget_all()
{
    gp_mem_1.clear();
    gp_mem_2.clear();
    gp_mem_3.clear();
	gp_mem_4.clear();
	gp_mem_5.clear();
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

    uint16_t chk = crc16((unsigned char*)rec,NUM_WP_FLOATS * sizeof(float));
    
    if (chk == rec[AR_SIZE-1]) {

        float * fp = reinterpret_cast<float*>(&rec[0]);
			
        gp_mem_1.lat=fp[0];
        gp_mem_1.lon=fp[1];
        gp_mem_2.lat=fp[2];
        gp_mem_2.lon=fp[3];
        gp_mem_3.lat=fp[4];
        gp_mem_3.lon=fp[5];               
        gp_mem_4.lat=fp[6];
        gp_mem_4.lon=fp[7];
        gp_mem_5.lat=fp[8];
        gp_mem_5.lon=fp[9];
        
        b_printf(PSTR("OK\r\n"));
        
    } else {
        b_printf(PSTR("FAILED (checksum)\r\n"));
    }
	
	print_waypoints();
}
// ----------------------------------------------------------------------------
void CWayPoints::store_waypoints()
{
    b_printf(PSTR("Storing waypoints to EEPROM.\r\n"));
	print_waypoints();
    
    uint16_t rec[AR_SIZE];

    float * fp = reinterpret_cast<float*>(&rec[0]);
    fp[0]=gp_mem_1.lat;
    fp[1]=gp_mem_1.lon;
    fp[2]=gp_mem_2.lat;
    fp[3]=gp_mem_2.lon;
    fp[4]=gp_mem_3.lat;
    fp[5]=gp_mem_3.lon;
    fp[6]=gp_mem_4.lat;
    fp[7]=gp_mem_4.lon;
    fp[8]=gp_mem_5.lat;
    fp[9]=gp_mem_5.lon;

    rec[AR_SIZE-1]=crc16(reinterpret_cast<unsigned char*>(rec),NUM_WP_FLOATS * sizeof(float));
    
    unsigned int addr=WAYPOINT_EEPROM_OFFSET;
    for (unsigned int i(0); i<AR_SIZE; i++) {
        eeprom_busy_wait();
        eeprom_write_word((uint16_t*)addr,rec[i]);
        addr+=2;
    }

}
// ----------------------------------------------------------------------------
void CWayPoints::print_waypoints()
{
	b_printf(PSTR("WP1="));
	gp_mem_1.print_wp();
	b_printf(PSTR("WP2="));
	gp_mem_2.print_wp();
	b_printf(PSTR("WP3="));
	gp_mem_3.print_wp();
	b_printf(PSTR("WP4="));
	gp_mem_4.print_wp();
	b_printf(PSTR("WP5="));
	gp_mem_5.print_wp();	
}
// ----------------------------------------------------------------------------
