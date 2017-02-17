#ifndef compass_rawvaluesH
#define compass_rawvaluesH

//---------------------------------------------------------------------------
class TCompassRawValue
{
	public:
	int16_t value;
	bool valid;
	TCompassRawValue() : value(0), valid(false) {};
	void clear() {
		value=0;
		valid=false;
	};
	bool operator==(const TCompassRawValue & rhs) const {
		return value == rhs.value && valid == rhs.valid;
	};
};

#endif
