#include "Mapper.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <memory>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <fstream>

// ========================================================= Node Parameters =========================================================
std::string odomFrame;               // Name of the frame used for odometry.
std::string sensorFrame;             // Name of the frame in which the points are published.
std::string robotFrame;              // Name of the frame centered on the robot.
std::string mapFileName;             // Name of the file in which the final map is saved when is_online is false.
std::string icpConfig;               // Name of the file containing the libpointmatcher icp config.
std::string inputFiltersConfig;      // Name of the file containing the filters applied to the sensor points.
std::string mapPostFiltersConfig;    // Name of the file containing the filters applied to the map after the update.
std::string mapUpdateCondition;      // Condition for map update. It can either be 'overlap', 'time' or 'distance'.
double mapPublishRate;               // Rate at which the map is published (in Hertz). It can be slower depending on the map update rate.
double mapTfPublishRate;             // Rate at which the map tf is published (in Hertz).
double maxIdleTime;                  // Delay to wait being idle before shutting down ROS when is_online is false (in seconds).
double minOverlap;                   // Overlap between sensor and map points under which the map is updated.
double maxTime;                      // Time since last map update over which the map is updated (in seconds).
double maxDistance;                  // Euclidean distance from last map update over which the map is updated (in meters).
bool is3D;                           // true when a 3D sensor is used, false when a 2D sensor is used.
bool isOnline;                       // true when real-time mapping is wanted, false otherwise.
// ===================================================================================================================================

PM::DataPointsFilters inputFilters;
PM::DataPointsFilters mapPostFilters;
std::shared_ptr<PM::Transformation> transformation;
std::unique_ptr<Mapper> mapper;
PM::TransformationParameters odomToMap;
ros::Subscriber sub;
ros::Publisher mapPublisher;
ros::Publisher odomPublisher;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
std::mutex mapTfLock;
std::time_t lastTimePointsWereProcessed;
std::mutex idleTimeLock;

void retrieveParameters(const ros::NodeHandle& pn)
{
	pn.param<std::string>("odom_frame", odomFrame, "odom");
	pn.param<std::string>("sensor_frame", sensorFrame, "velodyne");
	pn.param<std::string>("robot_frame", robotFrame, "base_link");
	pn.param<std::string>("map_file_name", mapFileName, "map.vtk");
	pn.param<std::string>("icp_config", icpConfig, "");
	pn.param<std::string>("input_filters_config", inputFiltersConfig, "");
	pn.param<std::string>("map_post_filters_config", mapPostFiltersConfig, "");
	pn.param<std::string>("map_update_condition", mapUpdateCondition, "overlap");
	pn.param<double>("map_publish_rate", mapPublishRate, 10);
	pn.param<double>("map_tf_publish_rate", mapTfPublishRate, 10);
	pn.param<double>("max_idle_time", maxIdleTime, 10);
	pn.param<double>("min_overlap", minOverlap, 0.9);
	pn.param<double>("max_time", maxTime, 2);
	pn.param<double>("max_distance", maxDistance, 1);
	pn.param<bool>("is_3D", is3D, true);
	pn.param<bool>("is_online", isOnline, true);
}

void loadExternalParameters()
{
	if(inputFiltersConfig != "")
	{
		std::ifstream ifs(inputFiltersConfig.c_str());
		if(ifs.good())
		{
			inputFilters = PM::DataPointsFilters(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load input filters config from YAML file " << inputFiltersConfig);
		}
	}
	
	if(mapPostFiltersConfig != "")
	{
		std::ifstream ifs(mapPostFiltersConfig.c_str());
		if(ifs.good())
		{
			mapPostFilters = PM::DataPointsFilters(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load map post-filters config from YAML file " << mapPostFiltersConfig);
		}
	}
}

void mapperShutdownLoop()
{
	double idleTime = 0;
	
	while(ros::ok())
	{
		idleTimeLock.lock();
		if(lastTimePointsWereProcessed)
		{
			idleTime = std::difftime(std::time(nullptr), lastTimePointsWereProcessed);
		}
		idleTimeLock.unlock();
		
		if(idleTime > maxIdleTime)
		{
			ROS_INFO("Saving map to %s", mapFileName.c_str());
			mapper->getMap().save(mapFileName);
			ROS_INFO("Shutting down ROS");
			ros::shutdown();
		}
		
		sleep(1);
	}
}

void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	ros::Time timeStamp = cloudMsgIn.header.stamp;
	PM::DataPoints cloud = PointMatcher_ROS::rosMsgToPointMatcherCloud<T>(cloudMsgIn);
	
	PM::TransformationParameters sensorToOdom;
	try
	{
		geometry_msgs::TransformStamped sensorToOdomTf = tfBuffer->lookupTransform(odomFrame, sensorFrame, timeStamp, ros::Duration(0.1));
		sensorToOdom = PointMatcher_ROS::rosTfToPointMatcherTransformation<T>(sensorToOdomTf);
	}
	catch(tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
		return;
	}
	
	PM::TransformationParameters sensorToMapBeforeUpdate = odomToMap * sensorToOdom;
	mapper->updateMap(cloud, sensorToMapBeforeUpdate, timeStamp.toSec());
	const PM::TransformationParameters& sensorToMapAfterUpdate = mapper->getSensorPose();
	
	mapTfLock.lock();
	odomToMap = transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());
	mapTfLock.unlock();
	
	PM::TransformationParameters robotToSensor;
	try
	{
		geometry_msgs::TransformStamped robotToSensorTf = tfBuffer->lookupTransform(sensorFrame, robotFrame, timeStamp, ros::Duration(0.1));
		robotToSensor = PointMatcher_ROS::rosTfToPointMatcherTransformation<T>(robotToSensorTf);
	}
	catch(tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
		return;
	}
	
	PM::TransformationParameters robotToMap = sensorToMapAfterUpdate * robotToSensor;
	nav_msgs::Odometry odomMsgOut = PointMatcher_ROS::pointMatcherTransformationToOdomMsg<T>(robotToMap, "map", timeStamp);
	odomPublisher.publish(odomMsgOut);
	
	idleTimeLock.lock();
	lastTimePointsWereProcessed = std::time(nullptr);
	idleTimeLock.unlock();
}

void gotScan(const sensor_msgs::LaserScan& scanMsgIn)
{
	// Handle 2D scans
}

void mapPublisherLoop()
{
	ros::Rate publishRate(mapPublishRate);
	
	PM::DataPoints newMap;
	while(ros::ok())
	{
		if(mapper->getNewMap(newMap))
		{
			sensor_msgs::PointCloud2 mapMsgOut = PointMatcher_ROS::pointMatcherCloudToRosMsg<T>(newMap, "map", ros::Time::now());
			mapPublisher.publish(mapMsgOut);
		}
		
		publishRate.sleep();
	}
}

void mapTfPublisherLoop()
{
	ros::Rate publishRate(mapTfPublishRate);
	
	while(ros::ok())
	{
		mapTfLock.lock();
		PM::TransformationParameters currentOdomToMap = odomToMap;
		mapTfLock.unlock();
		
		geometry_msgs::TransformStamped currentOdomToMapTf = PointMatcher_ROS::pointMatcherTransformationToRosTf<T>(currentOdomToMap, "map", odomFrame,
																													ros::Time::now());
		tfBroadcaster->sendTransform(currentOdomToMapTf);
		
		publishRate.sleep();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mapper_node");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	retrieveParameters(pn);
	loadExternalParameters();
	
	transformation = PM::get().TransformationRegistrar.create("RigidTransformation");
	
	mapper = std::unique_ptr<Mapper>(new Mapper(icpConfig, inputFilters, mapPostFilters, mapUpdateCondition, minOverlap, maxTime, maxDistance, is3D, isOnline));
	
	std::thread mapperShutdownThread;
	int messageQueueSize;
	if(isOnline)
	{
		tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
		messageQueueSize = 1;
	}
	else
	{
		mapperShutdownThread = std::thread(mapperShutdownLoop);
		tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(ros::Duration(ros::DURATION_MAX)));
		messageQueueSize = 0;
	}
	tf2_ros::TransformListener tfListener(*tfBuffer);
	tfBroadcaster = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster);
	
	if(is3D)
	{
		sub = n.subscribe("points_in", messageQueueSize, gotCloud);
		odomToMap = PM::Matrix::Identity(4, 4);
	}
	else
	{
		sub = n.subscribe("points_in", messageQueueSize, gotScan);
		odomToMap = PM::Matrix::Identity(3, 3);
	}
	mapPublisher = n.advertise<sensor_msgs::PointCloud2>("map", 1);
	odomPublisher = n.advertise<nav_msgs::Odometry>("icp_odom", 1);
	
	std::thread mapPublisherThread = std::thread(mapPublisherLoop);
	std::thread mapTfPublisherThread = std::thread(mapTfPublisherLoop);
	
	ros::spin();
	
	return 0;
}
