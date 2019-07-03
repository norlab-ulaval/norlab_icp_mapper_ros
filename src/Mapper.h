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
	std::time_t lastTimeMapWasUpdated;
	PM::TransformationParameters lastSensorPoseWhereMapWasUpdated;
	double minDistNewPoint;
	std::string mapUpdateCondition;
	double mapUpdateOverlap;
	double mapUpdateDelay;
	double mapUpdateDistance;
	bool is3D;
	bool isOnline;
	bool newMapAvailable;
	std::mutex mapLock;
	std::future<void> mapBuilderFuture;
	
	bool shouldMapBeUpdated(const std::time_t& currentTime, const PM::TransformationParameters& currentSensorPose, const double& currentOverlap);
	
	void updateMap(const PM::DataPoints& currentCloud, const std::time_t& timeStamp);
	
	void buildMap(PM::DataPoints currentCloud, PM::DataPoints currentMap, PM::TransformationParameters currentSensorPose);
	
	PM::DataPoints retrievePointsFurtherThanMinDistNewPoint(const PM::DataPoints& currentCloud, const PM::DataPoints& currentMap);

public:
	Mapper(std::string icpConfigFilePath, PM::DataPointsFilters inputFilters, PM::DataPointsFilters mapPostFilters, double minDistNewPoint,
		   std::string mapUpdateCondition, double mapUpdateOverlap, double mapUpdateDelay, double mapUpdateDistance, bool is3D, bool isOnline);
	
	void processCloud(PM::DataPoints& cloudInSensorFrame, PM::TransformationParameters& estimatedSensorPose, std::time_t timeStamp);
	
	PM::DataPoints getMap();
	
	bool getNewMap(PM::DataPoints& mapOut);
	
	const PM::TransformationParameters& getSensorPose();
};
