#ifndef WProgramH
#define WProgramH

// 2015-08-19 (EPU): Fake WProgram.h to satisfy TinyGPS.cpp 

typedef unsigned char byte;
#define PI 3.1415926535897932384626433832795
#define TWO_PI 6.283185307179586476925286766559
#include <math.h>

#ifdef NO_ARDUINO_LIB
#define sq(a) (a*a)
#define radians(d) ((d) * (PI / 180.0))
#define degrees(r) ((r) / (PI / 180.0))
#endif

// function is expected to return the number of [ms] the program has been running
// (but we don't care about that)
extern unsigned int epu_millis();

#endif