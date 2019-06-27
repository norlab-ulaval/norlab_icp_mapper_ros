#include <pointmatcher/PointMatcher.h>
#include <future>

typedef float T;
typedef PointMatcher<T> PM;

class Mapper
{
private:
	PM::DataPointsFilters inputFilters;
	PM::DataPointsFilters mapPostFilters;
	PM::ICP icp;
	PM::DataPoints map;
	PM::TransformationParameters sensorPose;
	std::shared_ptr<PM::Transformation> transformation;
	bool isOnline;
	bool newMapAvailable;
	std::mutex mapLock;
	std::future<void> mapBuilderFuture;
	
	void buildMap(PM::DataPoints currentCloud, PM::TransformationParameters currentSensorPose);

public:
	Mapper(std::string icpConfigFilePath, PM::DataPointsFilters inputFilters, PM::DataPointsFilters mapPostFilters, bool is3D, bool isOnline);
	
	void updateMap(PM::DataPoints& cloudInSensorFrame, PM::TransformationParameters& estimatedSensorPose);
	
	PM::DataPoints getMap();
	
	bool getNewMap(PM::DataPoints& mapOut);
	
	const PM::TransformationParameters& getSensorPose();
};
