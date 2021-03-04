#include "Trajectory.h"
#include <pointmatcher/PointMatcher.h>

Trajectory::Trajectory(int dimension):
		dimension(dimension)
{
}

void Trajectory::addPoint(Vector point, std::int64_t timeStamp)
{
	times.conservativeResize(1, times.cols() + 1);
	times(0, times.cols() - 1) = timeStamp;

	points.conservativeResize(dimension, points.cols() + 1);
	points.col(points.cols() - 1) = point;
}

void Trajectory::save(std::string filename)
{
	Matrix t(1, times.cols());
	for(int i = 0; i < times.cols(); i++)
	{
		t(0, i) = (times(0, i) - times(0, 0)) / 1e9;
	}

	PointMatcher<float>::DataPoints trajectory;
	trajectory.addDescriptor("t", t);
	trajectory.addFeature("x", points.row(0));
	trajectory.addFeature("y", points.row(1));
	if(dimension == 3)
	{
		trajectory.addFeature("z", points.row(2));
	}
	trajectory.save(filename);
}
