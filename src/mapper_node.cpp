#include <ros/ros.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <pointmatcher/PointMatcher.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mapper_node");

	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	// Do interesting stuff here

	return 0;
}
