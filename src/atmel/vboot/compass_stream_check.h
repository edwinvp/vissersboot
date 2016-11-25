#ifndef compass_stream_checkH
#define compass_stream_checkH

#include "compass_calibrate.h"

class CCompassStreamCheck
{
private:
	TCompassTriple compass_raw_old;
	int16_t compass_same_counter;
	int16_t compass_diff_counter;
	bool compass_sends_values;

public:
	CCompassStreamCheck();
	bool check(const TCompassTriple & compass_raw);
};

#endif
