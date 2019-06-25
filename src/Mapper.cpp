#include "Mapper.h"

Mapper::Mapper(bool is3D):
		transformation(PM::get().TransformationRegistrar.create("RigidTransformation"))
{
	icp.setDefault();
	
	if(is3D)
	{
		sensorPose = PM::Matrix::Identity(4, 4);
	}
	else
	{
		sensorPose = PM::Matrix::Identity(3, 3);
	}
}

void Mapper::updateMap(const PM::DataPoints& cloudInSensorFrame, const PM::TransformationParameters& estimatedSensorPose)
{
	PointMatcherSupport::Parametrizable::Parameters params;
	params["xMin"] = "-100";
	params["xMax"] = "100";
	params["yMin"] = "-100";
	params["yMax"] = "100";
	params["zMin"] = "-3";
	params["zMax"] = "-0.3";
	params["removeInside"] = "1";
	std::shared_ptr<PM::DataPointsFilter> floorFilter = PM::get().DataPointsFilterRegistrar.create("BoundingBoxDataPointsFilter", params);
	PM::DataPoints filteredCloud = floorFilter->filter(cloudInSensorFrame);
	params.clear();
	params["xMin"] = "-100";
	params["xMax"] = "100";
	params["yMin"] = "-100";
	params["yMax"] = "100";
	params["zMin"] = "1";
	params["zMax"] = "5";
	params["removeInside"] = "1";
	std::shared_ptr<PM::DataPointsFilter> ceilingFilter = PM::get().DataPointsFilterRegistrar.create("BoundingBoxDataPointsFilter", params);
	filteredCloud = ceilingFilter->filter(filteredCloud);
	params.clear();
	params["xMin"] = "-0.3";
	params["xMax"] = "0.3";
	params["yMin"] = "-5";
	params["yMax"] = "0";
	params["zMin"] = "-2";
	params["zMax"] = "2";
	params["removeInside"] = "1";
	std::shared_ptr<PM::DataPointsFilter> driverFilter = PM::get().DataPointsFilterRegistrar.create("BoundingBoxDataPointsFilter", params);
	filteredCloud = driverFilter->filter(filteredCloud);
	
	PM::DataPoints cloudInMapFrame = transformation->compute(filteredCloud, estimatedSensorPose);
	
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
}

const PM::DataPoints& Mapper::getMap()
{
	return map;
}

const PM::TransformationParameters& Mapper::getSensorPose()
{
	return sensorPose;
}
