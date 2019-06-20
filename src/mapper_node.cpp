#include "Mapper.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <memory>
#include <mutex>
#include <thread>

Mapper mapper;
std::string odomFrame;
std::string mapFrame;
std::string sensorFrame;
std::string robotFrame;
std::shared_ptr<PM::Transformation> transformation;
bool is3D;
PM::TransformationParameters mapToOdom;
ros::Subscriber sub;
ros::Publisher mapPublisher;
ros::Publisher odomPublisher;
std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
tf2_ros::Buffer tfBuffer;
std::mutex mapTfLock;

void mapTfPublisherLoop()
{
	ros::Rate publishRate(10);
	
	while(ros::ok())
	{
		mapTfLock.lock();
		geometry_msgs::TransformStamped mapToOdomTf = PointMatcher_ROS::pointMatcherTransformationToRosTf<T>(mapToOdom);
		mapToOdomTf.header.frame_id = mapFrame;
		mapToOdomTf.child_frame_id = odomFrame;
		mapToOdomTf.header.stamp = ros::Time::now();
		tfBroadcaster->sendTransform(mapToOdomTf);
		mapTfLock.unlock();
		
		publishRate.sleep();
	}
}

void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	ros::Time timeStamp = cloudMsgIn.header.stamp;
	PM::DataPoints cloud = PointMatcher_ROS::rosMsgToPointMatcherCloud<T>(cloudMsgIn);
	
	geometry_msgs::TransformStamped sensorToOdomTf = tfBuffer.lookupTransform(odomFrame, sensorFrame, ros::Time(0), ros::Duration(3));
	PM::TransformationParameters sensorToOdom = PointMatcher_ROS::rosTfToPointMatcherTransformation<T>(sensorToOdomTf);
	PM::TransformationParameters sensorToMap = mapToOdom.inverse() * sensorToOdom;
	PM::DataPoints transformedCloud = transformation->compute(cloud, sensorToMap);
	
	mapper.updateMap(transformedCloud);
	const PM::DataPoints& map = mapper.getMap();
	
	sensor_msgs::PointCloud2 mapMsgOut = PointMatcher_ROS::pointMatcherCloudToRosMsg<T>(map, mapFrame, timeStamp);
	mapPublisher.publish(mapMsgOut);
	
	geometry_msgs::TransformStamped robotToSensorTf = tfBuffer.lookupTransform(sensorFrame, robotFrame, ros::Time(0), ros::Duration(3));
	PM::TransformationParameters robotToSensor = PointMatcher_ROS::rosTfToPointMatcherTransformation<T>(robotToSensorTf);
	PM::TransformationParameters robotToMap = sensorToMap * robotToSensor;
	nav_msgs::Odometry odomMsgOut = PointMatcher_ROS::pointMatcherTransformationToOdomMsg<T>(robotToMap, mapFrame, timeStamp);
	odomPublisher.publish(odomMsgOut);
}

void gotScan(const sensor_msgs::LaserScan& scanMsgIn)
{
	// Handle 2D scans
}

void retrieveParameters(const ros::NodeHandle& pn)
{
	pn.param<std::string>("odom_frame", odomFrame, "odom");
	pn.param<std::string>("map_frame", mapFrame, "map");
	pn.param<std::string>("sensor_frame", sensorFrame, "velodyne");
	pn.param<std::string>("robot_frame", robotFrame, "base_link");
	pn.param<bool>("is_3D", is3D, true);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mapper_node");
	
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	retrieveParameters(pn);
	
	transformation = PM::get().REG(Transformation).create("RigidTransformation");
	
	if(is3D)
	{
		sub = n.subscribe("points_in", 1, gotCloud);
		mapToOdom = PM::Matrix::Identity(4, 4);
	}
	else
	{
		sub = n.subscribe("points_in", 1, gotScan);
		mapToOdom = PM::Matrix::Identity(3, 3);
	}
	mapPublisher = n.advertise<sensor_msgs::PointCloud2>("map", 1);
	odomPublisher = n.advertise<nav_msgs::Odometry>("odom_out", 1);
	
	tfBroadcaster = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster);
	tf2_ros::TransformListener tfListener(tfBuffer);
	
	std::thread mapTfPublisherThread = std::thread(mapTfPublisherLoop);
	
	ros::spin();
	
	return 0;
}
