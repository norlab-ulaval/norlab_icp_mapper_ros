#include "Mapper.h"

Mapper::Mapper():
	transformation(PM::get().REG(Transformation).create("RigidTransformation")),
	odom(PM::Matrix::Identity(4, 4))
{
	icp.setDefault();
}

void Mapper::updateMap(const PM::DataPoints& cloud)
{
	std::shared_ptr<PM::DataPointsFilter> distanceFilter = PM::get().DataPointsFilterRegistrar.create(
			"DistanceLimitDataPointsFilter",
			{{
					 "dist", PointMatcherSupport::toParam(3)
			 }});
	const PM::DataPoints& filteredCloud = distanceFilter->filter(cloud);

	if(map.getNbPoints() == 0)
	{
		map = filteredCloud;
	}
	else
	{

		odom = icp(filteredCloud, map, odom);
		map.concatenate(transformation->compute(filteredCloud, odom));
	}
}

const PM::DataPoints& Mapper::getMap()
{
	return map;
}

const PM::TransformationParameters& Mapper::getOdom()
{
	return odom;
}
