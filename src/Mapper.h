#include <pointmatcher/PointMatcher.h>

typedef float T;
typedef PointMatcher<T> PM;

class Mapper
{
private:
	PM::DataPointsFilters inputFilters;
	PM::DataPointsFilters mapPostFilters;
	PM::ICP icp;
	PM::TransformationParameters sensorPose;
	std::shared_ptr<PM::Transformation> transformation;
	PM::DataPoints map;

public:
	Mapper(std::string icpConfigFilePath, PM::DataPointsFilters inputFilters, PM::DataPointsFilters mapPostFilters, bool is3D);
	
	void updateMap(PM::DataPoints& cloudInSensorFrame, PM::TransformationParameters& estimatedSensorPose);
	
	const PM::DataPoints& getMap();
	
	const PM::TransformationParameters& getSensorPose();
};
