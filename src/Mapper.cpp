#include "Mapper.h"
#include <fstream>

Mapper::Mapper(std::string icpConfigFilePath, PM::DataPointsFilters inputFilters, PM::DataPointsFilters mapPostFilters, bool is3D):
		inputFilters(inputFilters),
		mapPostFilters(mapPostFilters),
		transformation(PM::get().TransformationRegistrar.create("RigidTransformation"))
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

void Mapper::updateMap(PM::DataPoints& cloudInSensorFrame, PM::TransformationParameters& estimatedSensorPose)
{
	inputFilters.apply(cloudInSensorFrame);
	
	PM::DataPoints mapInSensorFrame = transformation->compute(map, estimatedSensorPose.inverse());
	
	if(mapInSensorFrame.getNbPoints() == 0)
	{
		mapInSensorFrame = cloudInSensorFrame;
		sensorPose = estimatedSensorPose;
		map = transformation->compute(mapInSensorFrame, sensorPose);
		
	}
	else
	{
		PM::TransformationParameters correction = icp(cloudInSensorFrame, mapInSensorFrame);
		mapInSensorFrame.concatenate(transformation->compute(cloudInSensorFrame, correction));
		
		mapPostFilters.apply(mapInSensorFrame);
		map = transformation->compute(mapInSensorFrame, estimatedSensorPose);
		sensorPose =  estimatedSensorPose;
		
	}
	
}

const PM::DataPoints& Mapper::getMap()
{
	return map;
}

const PM::TransformationParameters& Mapper::getSensorPose()
{
	return sensorPose;
}
