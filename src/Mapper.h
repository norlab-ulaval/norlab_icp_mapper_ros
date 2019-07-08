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
	std::shared_ptr<PM::DataPointsFilter> radiusFilter;
	std::time_t lastTimeMapWasUpdated;
	PM::TransformationParameters lastSensorPoseWhereMapWasUpdated;
	std::string mapUpdateCondition;
	double mapUpdateOverlap;
	double mapUpdateDelay;
	double mapUpdateDistance;
	double minDistNewPoint;
	double sensorMaxRange;
	double priorDynamic;
	double thresholdDynamic;
	double beamHalfAngle;
	double epsilonA;
	double epsilonD;
	double alpha;
	double beta;
	bool is3D;
	bool isOnline;
	bool computeProbDynamic;
	bool newMapAvailable;
	std::mutex mapLock;
	std::future<void> mapBuilderFuture;
	
	bool shouldUpdateMap(const std::time_t& currentTime, const PM::TransformationParameters& currentSensorPose, const double& currentOverlap);
	
	void updateMap(const PM::DataPoints& currentCloud, const std::time_t& timeStamp);
	
	void buildMap(PM::DataPoints currentCloud, PM::DataPoints currentMap, PM::TransformationParameters currentSensorPose);
	
	PM::DataPoints retrievePointsFurtherThanMinDistNewPoint(const PM::DataPoints& currentCloud, const PM::DataPoints& currentMap);
	
	void computeProbabilityOfPointsBeingDynamic(const PM::DataPoints& currentCloud, PM::DataPoints& currentMap,
												const PM::TransformationParameters& currentSensorPose);
	
	void convertToSphericalCoordinates(const PM::DataPoints& points, PM::Matrix& radii, PM::Matrix& angles);

public:
	Mapper(std::string icpConfigFilePath, PM::DataPointsFilters inputFilters, PM::DataPointsFilters mapPostFilters, std::string mapUpdateCondition,
		   double mapUpdateOverlap, double mapUpdateDelay, double mapUpdateDistance, double minDistNewPoint, double sensorMaxRange,
		   double priorDynamic, double thresholdDynamic, double beamHalfAngle, double epsilonA, double epsilonD, double alpha, double beta,
		   bool is3D, bool isOnline, bool computeProbDynamic);
	
	void processCloud(PM::DataPoints& cloudInSensorFrame, PM::TransformationParameters& estimatedSensorPose, std::time_t timeStamp);
	
	PM::DataPoints getMap();
	
	bool getNewMap(PM::DataPoints& mapOut);
	
	const PM::TransformationParameters& getSensorPose();
};
