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

//---------------------------------------------------------------------------
class TCompassTriple
{
	public:
	TCompassRawValue x,y,z;
	TCompassTriple() {};
	bool equals(const TCompassTriple & rhs) const {
		return (x==rhs.x) && (y==rhs.y) && (z==rhs.z);
	};
};

#endif
