#include "Mapper.h"
#include <fstream>
#include <chrono>

Mapper::Mapper(std::string icpConfigFilePath, PM::DataPointsFilters inputFilters, PM::DataPointsFilters mapPostFilters, bool is3D, bool isOnline):
		inputFilters(inputFilters),
		mapPostFilters(mapPostFilters),
		transformation(PM::get().TransformationRegistrar.create("RigidTransformation")),
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

void Mapper::buildMap(PM::DataPoints currentCloud, PM::TransformationParameters currentSensorPose)
{
	mapLock.lock();
	PM::DataPoints currentMap = map;
	mapLock.unlock();
	
	currentMap.concatenate(currentCloud);
	PM::DataPoints mapInSensorFrame = transformation->compute(currentMap, currentSensorPose.inverse());
	mapPostFilters.apply(mapInSensorFrame);                                              // TODO: find efficient way to compute this...
	currentMap = transformation->compute(mapInSensorFrame, currentSensorPose);
	
	mapLock.lock();
	map = currentMap;
	newMapAvailable = true;
	mapLock.unlock();
}

void Mapper::updateMap(PM::DataPoints& cloudInSensorFrame, PM::TransformationParameters& estimatedSensorPose)
{
	inputFilters.apply(cloudInSensorFrame);
	
	PM::DataPoints cloudInMapFrameBeforeCorrection = transformation->compute(cloudInSensorFrame, estimatedSensorPose);
	
	mapLock.lock();
	PM::DataPoints currentMap = map;
	mapLock.unlock();
	
	if(currentMap.getNbPoints() == 0)
	{
		sensorPose = estimatedSensorPose;
		
		PM::DataPoints mapInSensorFrame = transformation->compute(cloudInMapFrameBeforeCorrection, sensorPose.inverse());
		mapPostFilters.apply(mapInSensorFrame);
		currentMap = transformation->compute(mapInSensorFrame, sensorPose);
		
		mapLock.lock();
		map = currentMap;
		newMapAvailable = true;
		mapLock.unlock();
	}
	else
	{
		PM::TransformationParameters correction = icp(cloudInMapFrameBeforeCorrection, currentMap);
		PM::DataPoints cloudInMapFrameAfterCorrection = transformation->compute(cloudInMapFrameBeforeCorrection, correction);
		sensorPose = correction * estimatedSensorPose;
		
		if(isOnline)
		{
			if(!mapBuilderFuture.valid() || mapBuilderFuture.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
			{
				mapBuilderFuture = std::async(&Mapper::buildMap, this, cloudInMapFrameAfterCorrection, sensorPose);
			}
		}
		else
		{
			buildMap(cloudInMapFrameBeforeCorrection, sensorPose);
		}
	}
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
