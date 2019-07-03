#include "Mapper.h"
#include <nabo/nabo.h>
#include <fstream>
#include <chrono>

Mapper::Mapper(std::string icpConfigFilePath, PM::DataPointsFilters inputFilters, PM::DataPointsFilters mapPostFilters, double minDistNewPoint,
			   std::string mapUpdateCondition, double mapUpdateOverlap, double mapUpdateDelay, double mapUpdateDistance, bool is3D, bool isOnline):
		inputFilters(inputFilters),
		mapPostFilters(mapPostFilters),
		transformation(PM::get().TransformationRegistrar.create("RigidTransformation")),
		minDistNewPoint(minDistNewPoint),
		mapUpdateCondition(mapUpdateCondition),
		mapUpdateOverlap(mapUpdateOverlap),
		mapUpdateDelay(mapUpdateDelay),
		mapUpdateDistance(mapUpdateDistance),
		is3D(is3D),
		isOnline(isOnline),
		newMapAvailable(false)
{
	if(icpConfigFilePath != "")
	{
		std::ifstream ifs(icpConfigFilePath.c_str());
		if(ifs.good())
		{
			icp.loadFromYaml(ifs);
		}
		else
		{
			icp.setDefault();
		}
	}
	else
	{
		icp.setDefault();
	}
	
	int nbRows = is3D ? 4 : 3;
	sensorPose = PM::Matrix::Identity(nbRows, nbRows);
}

void Mapper::processCloud(PM::DataPoints& cloudInSensorFrame, PM::TransformationParameters& estimatedSensorPose, std::time_t timeStamp)
{
	inputFilters.apply(cloudInSensorFrame);
	
	PM::DataPoints cloudInMapFrameBeforeCorrection = transformation->compute(cloudInSensorFrame, estimatedSensorPose);
	
	mapLock.lock();
	PM::DataPoints currentMap = map;
	mapLock.unlock();
	
	if(currentMap.getNbPoints() == 0)
	{
		sensorPose = estimatedSensorPose;
		
		updateMap(cloudInMapFrameBeforeCorrection, timeStamp);
	}
	else
	{
		PM::TransformationParameters correction = icp(cloudInMapFrameBeforeCorrection, currentMap);
		sensorPose = correction * estimatedSensorPose;
		
		if(shouldMapBeUpdated(timeStamp, sensorPose, icp.errorMinimizer->getOverlap()))
		{
			updateMap(transformation->compute(cloudInMapFrameBeforeCorrection, correction), timeStamp);
		}
	}
}

bool Mapper::shouldMapBeUpdated(const std::time_t& currentTime, const PM::TransformationParameters& currentSensorPose, const double& currentOverlap)
{
	if(isOnline)
	{
		// if previous map is not done building
		if(mapBuilderFuture.valid() && mapBuilderFuture.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
		{
			return false;
		}
	}
	
	if(mapUpdateCondition == "overlap")
	{
		return currentOverlap < mapUpdateOverlap;
	}
	else if(mapUpdateCondition == "delay")
	{
		return std::difftime(currentTime, lastTimeMapWasUpdated) > mapUpdateDelay;
	}
	else if(mapUpdateCondition == "distance")
	{
		int nbRows = is3D ? 3 : 2;
		PM::Vector lastSensorLocation = lastSensorPoseWhereMapWasUpdated.topRightCorner(nbRows, 1);
		PM::Vector currentSensorLocation = currentSensorPose.topRightCorner(nbRows, 1);
		return std::abs((currentSensorLocation - lastSensorLocation).norm()) > mapUpdateDistance;
	}
	else
	{
		throw std::runtime_error("Invalid map update condition: " + mapUpdateCondition);
	}
}

void Mapper::updateMap(const PM::DataPoints& currentCloud, const std::time_t& timeStamp)
{
	mapLock.lock();
	PM::DataPoints currentMap = map;
	mapLock.unlock();
	
	lastTimeMapWasUpdated = timeStamp;
	lastSensorPoseWhereMapWasUpdated = sensorPose;
	
	if(isOnline && currentMap.getNbPoints() != 0)
	{
		mapBuilderFuture = std::async(&Mapper::buildMap, this, currentCloud, currentMap, sensorPose);
	}
	else
	{
		buildMap(currentCloud, currentMap, sensorPose);
	}
}

void Mapper::buildMap(PM::DataPoints currentCloud, PM::DataPoints currentMap, PM::TransformationParameters currentSensorPose)
{
	if(currentMap.getNbPoints() == 0)
	{
		currentMap = currentCloud;
	}
	else
	{
		PM::DataPoints cloudPointsToKeep = retrievePointsFurtherThanMinDistNewPoint(currentCloud, currentMap);
		currentMap.concatenate(cloudPointsToKeep);
	}
	
	PM::DataPoints mapInSensorFrame = transformation->compute(currentMap, currentSensorPose.inverse());
	mapPostFilters.apply(mapInSensorFrame);                                              // TODO: find efficient way to compute this...
	currentMap = transformation->compute(mapInSensorFrame, currentSensorPose);
	
	mapLock.lock();
	map = currentMap;
	newMapAvailable = true;
	mapLock.unlock();
}

PM::DataPoints Mapper::retrievePointsFurtherThanMinDistNewPoint(const PM::DataPoints& currentCloud, const PM::DataPoints& currentMap)
{
	typedef Nabo::NearestNeighbourSearch<T> NNS;
	
	PM::Matches matches(PM::Matches::Dists(1, currentCloud.getNbPoints()), PM::Matches::Ids(1, currentCloud.getNbPoints()));
	std::shared_ptr<NNS> nns = std::shared_ptr<NNS>(NNS::create(currentMap.features, currentMap.features.rows() - 1,
																NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
	
	nns->knn(currentCloud.features, matches.ids, matches.dists, 1, 0);
	
	int goodPointCount = 0;
	PM::DataPoints goodPoints(currentCloud.createSimilarEmpty());
	for(int i = 0; i < currentCloud.getNbPoints(); ++i)
	{
		if(matches.dists(i) >= std::pow(minDistNewPoint, 2))
		{
			goodPoints.setColFrom(goodPointCount, currentCloud, i);
			goodPointCount++;
		}
	}
	goodPoints.conservativeResize(goodPointCount);
	
	return goodPoints;
}

PM::DataPoints Mapper::getMap()
{
	mapLock.lock();
	PM::DataPoints currentMap = map;
	mapLock.unlock();
	
	return currentMap;
}

bool Mapper::getNewMap(PM::DataPoints& mapOut)
{
	bool mapReturned = false;
	
	mapLock.lock();
	if(newMapAvailable)
	{
		mapOut = map;
		newMapAvailable = false;
		mapReturned = true;
	}
	mapLock.unlock();
	
	return mapReturned;
}

const PM::TransformationParameters& Mapper::getSensorPose()
{
	return sensorPose;
}
