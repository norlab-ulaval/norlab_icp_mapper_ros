#include "Mapper.h"
#include <fstream>
#include <chrono>

Mapper::Mapper(std::string icpConfigFilePath, PM::DataPointsFilters inputFilters, PM::DataPointsFilters mapPostFilters, std::string mapUpdateCondition,
			   double minOverlap, double maxTime, double maxDistance, bool is3D, bool isOnline):
		inputFilters(inputFilters),
		mapPostFilters(mapPostFilters),
		transformation(PM::get().TransformationRegistrar.create("RigidTransformation")),
		mapUpdateCondition(mapUpdateCondition),
		minOverlap(minOverlap),
		maxTime(maxTime),
		maxDistance(maxDistance),
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
	
	if(is3D)
	{
		sensorPose = PM::Matrix::Identity(4, 4);
	}
	else
	{
		sensorPose = PM::Matrix::Identity(3, 3);
	}
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

bool Mapper::shouldMapBeUpdated(std::time_t currentTime, PM::TransformationParameters currentSensorPose, double currentOverlap)
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
		return currentOverlap < minOverlap;
	}
	else if(mapUpdateCondition == "time")
	{
		return std::difftime(currentTime, lastTimeMapWasUpdated) > maxTime;
	}
	else if(mapUpdateCondition == "distance")
	{
		int nbRows = is3D ? 3 : 2;
		PM::Vector lastSensorLocation = lastSensorPoseWhereMapWasUpdated.topRightCorner(nbRows, 1);
		PM::Vector currentSensorLocation = currentSensorPose.topRightCorner(nbRows, 1);
		return std::abs((currentSensorLocation - lastSensorLocation).norm()) > maxDistance;
	}
	else
	{
		throw std::runtime_error("Invalid map update condition: " + mapUpdateCondition);
	}
}

void Mapper::updateMap(PM::DataPoints currentCloud, std::time_t timeStamp)
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
		currentMap.concatenate(currentCloud);
	}
	
	PM::DataPoints mapInSensorFrame = transformation->compute(currentMap, currentSensorPose.inverse());
	mapPostFilters.apply(mapInSensorFrame);                                              // TODO: find efficient way to compute this...
	currentMap = transformation->compute(mapInSensorFrame, currentSensorPose);
	
	mapLock.lock();
	map = currentMap;
	newMapAvailable = true;
	mapLock.unlock();
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
