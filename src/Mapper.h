#include <pointmatcher/PointMatcher.h>

typedef float T;
typedef PointMatcher<T> PM;

class Mapper
{
private:
	PM::ICP icp;
	std::shared_ptr<PM::Transformation> transformation;
	PM::DataPoints map;
	PM::TransformationParameters sensorPose;

public:
	Mapper(bool is3D);
	
	void updateMap(const PM::DataPoints& cloudInSensorFrame, const PM::TransformationParameters& estimatedSensorPose);
	
	const PM::DataPoints& getMap();
	
	const PM::TransformationParameters& getSensorPose();
};
