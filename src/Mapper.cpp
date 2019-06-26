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
	
	PM::DataPoints cloudInMapFrame = transformation->compute(cloudInSensorFrame, estimatedSensorPose);
	
	if(map.getNbPoints() == 0)
	{
		map = cloudInMapFrame;
		sensorPose = estimatedSensorPose;
	}
	else
	{
		PM::TransformationParameters correction = icp(cloudInMapFrame, map);
		map.concatenate(transformation->compute(cloudInMapFrame, correction));
		sensorPose = correction * estimatedSensorPose;
	}
	
	mapPostFilters.apply(map);
}

const PM::DataPoints& Mapper::getMap()
{
	return map;
}

const PM::TransformationParameters& Mapper::getSensorPose()
{
	return sensorPose;
}
