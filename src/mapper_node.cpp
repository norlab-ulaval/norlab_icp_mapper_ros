#include <ros/ros.h>
#include "Mapper.h"
#include <pointmatcher_ros/PointMatcher_ROS.h>

ros::Publisher mapPublisher;
Mapper mapper;

void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	PM::DataPoints cloud = PointMatcher_ROS::rosMsgToPointMatcherCloud<T>(cloudMsgIn);

	mapper.updateMap(cloud);
	const PM::DataPoints& map = mapper.getMap();

	sensor_msgs::PointCloud2 mapMsgOut = PointMatcher_ROS::pointMatcherCloudToRosMsg<T>(map, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp);
	mapPublisher.publish(mapMsgOut);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mapper_node");

	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	ros::Subscriber sub = n.subscribe("cloud_in", 0, gotCloud);
	mapPublisher = n.advertise<sensor_msgs::PointCloud2>("cloud_out", 0);

	ros::spin();

	return 0;
}
