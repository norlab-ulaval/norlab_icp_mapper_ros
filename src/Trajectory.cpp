#include "Trajectory.h"
#include <pointmatcher/PointMatcher.h>

Trajectory::Trajectory(int dimension):
		dimension(dimension)
{
}

void Trajectory::addPoint(Vector point)
{
	points.conservativeResize(dimension, points.cols() + 1);
	points.col(points.cols() - 1) = point;
}

void Trajectory::save(std::string filename)
{
	PointMatcher<float>::DataPoints trajectory;
	trajectory.addFeature("x", points.row(0));
	trajectory.addFeature("y", points.row(1));
	if(dimension == 3)
	{
		trajectory.addFeature("z", points.row(2));
	}
	trajectory.save(filename);
}
