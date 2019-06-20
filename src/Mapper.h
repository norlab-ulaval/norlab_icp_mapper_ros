#include <pointmatcher/PointMatcher.h>

typedef float T;
typedef PointMatcher<T> PM;

class Mapper
{
private:
	PM::DataPoints map;
	PM::ICP icp;
	PM::TransformationParameters odom;
	std::shared_ptr<PM::Transformation> transformation;

public:
	Mapper();
	
	void updateMap(const PM::DataPoints& cloud);
	
	const PM::DataPoints& getMap();
	
	const PM::TransformationParameters& getOdom();
};
