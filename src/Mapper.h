#include <pointmatcher/PointMatcher.h>

typedef float T;
typedef PointMatcher<T> PM;

class Mapper
{
private:
	PM::DataPoints map;

public:
	Mapper();

	void updateMap(const PM::DataPoints& cloud);

	const PM::DataPoints& getMap();
};
