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
	std::string mapUpdateCondition;
	double minOverlap;
	double maxTime;
	double maxDistance;
	bool is3D;
	bool isOnline;
	bool newMapAvailable;
	std::mutex mapLock;
	std::future<void> mapBuilderFuture;
	
	bool shouldMapBeUpdated(std::time_t currentTime, PM::TransformationParameters currentSensorPose, double currentOverlap);
	
	void updateMap(PM::DataPoints currentCloud, std::time_t timeStamp);
	
	void buildMap(PM::DataPoints currentCloud, PM::DataPoints currentMap, PM::TransformationParameters currentSensorPose);

public:
	Mapper(std::string icpConfigFilePath, PM::DataPointsFilters inputFilters, PM::DataPointsFilters mapPostFilters, std::string mapUpdateCondition,
		   double minOverlap, double maxTime, double maxDistance, bool is3D, bool isOnline);
	
	void processCloud(PM::DataPoints& cloudInSensorFrame, PM::TransformationParameters& estimatedSensorPose, std::time_t timeStamp);
	
	PM::DataPoints getMap();
	
	bool getNewMap(PM::DataPoints& mapOut);
	
	const PM::TransformationParameters& getSensorPose();
};
