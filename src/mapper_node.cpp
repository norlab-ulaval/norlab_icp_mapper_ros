#include "NodeParameters.h"
#include "Mapper.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <map_msgs/SaveMap.h>
#include <memory>
#include <mutex>
#include <thread>

std::unique_ptr<NodeParameters> params;
std::shared_ptr<PM::Transformation> transformation;
std::unique_ptr<Mapper> mapper;
PM::TransformationParameters odomToMap;
ros::Subscriber sub;
ros::Publisher mapPublisher;
ros::Publisher odomPublisher;
ros::ServiceServer saveMapService;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
std::mutex mapTfLock;
std::chrono::time_point<std::chrono::steady_clock> lastTimePointsWereProcessed;
std::mutex idleTimeLock;

void loadInitialMap()
{
	if(!params->initialMapFileName.empty())
	{
		PM::DataPoints initialMap = PM::DataPoints::load(params->initialMapFileName);
		initialMap = transformation->compute(initialMap, params->initialMapPose);
		mapper->setMap(initialMap);
	}
}

void saveMap(std::string mapFileName)
{
	ROS_INFO("Saving map to %s", mapFileName.c_str());
	mapper->getMap().save(mapFileName);
}

void mapperShutdownLoop()
{
	std::chrono::duration<float> idleTime = std::chrono::duration<float>::zero();
	
	while(ros::ok())
	{
		idleTimeLock.lock();
		if(lastTimePointsWereProcessed.time_since_epoch().count())
		{
			idleTime = std::chrono::steady_clock::now() - lastTimePointsWereProcessed;
		}
		idleTimeLock.unlock();
		
		if(idleTime > std::chrono::duration<float>(params->maxIdleTime))
		{
			saveMap(params->finalMapFileName);
			ROS_INFO("Shutting down ROS");
			ros::shutdown();
		}
		
		std::this_thread::sleep_for(std::chrono::duration<float>(0.1));
	}
}

void gotPointMatcherCloud(PM::DataPoints cloud, ros::Time timeStamp)
{
	try
	{
		int nbRows = params->is3D ? 4 : 3;
		
		geometry_msgs::TransformStamped sensorToOdomTf = tfBuffer->lookupTransform(params->odomFrame, params->sensorFrame, timeStamp, ros::Duration(0.1));
		PM::TransformationParameters sensorToOdom = PointMatcher_ROS::rosTfToPointMatcherTransformation<T>(sensorToOdomTf, nbRows);
		
		PM::TransformationParameters sensorToMapBeforeUpdate = odomToMap * sensorToOdom;
		mapper->processCloud(cloud, sensorToMapBeforeUpdate, std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.toNSec())));
		const PM::TransformationParameters& sensorToMapAfterUpdate = mapper->getSensorPose();
		
		mapTfLock.lock();
		odomToMap = transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());
		mapTfLock.unlock();
		
		geometry_msgs::TransformStamped robotToSensorTf = tfBuffer->lookupTransform(params->sensorFrame, params->robotFrame, timeStamp, ros::Duration(0.1));
		PM::TransformationParameters robotToSensor = PointMatcher_ROS::rosTfToPointMatcherTransformation<T>(robotToSensorTf, nbRows);
		
		PM::TransformationParameters robotToMap = sensorToMapAfterUpdate * robotToSensor;
		nav_msgs::Odometry odomMsgOut = PointMatcher_ROS::pointMatcherTransformationToOdomMsg<T>(robotToMap, "map", timeStamp);
		odomPublisher.publish(odomMsgOut);
		
		idleTimeLock.lock();
		lastTimePointsWereProcessed = std::chrono::steady_clock::now();
		idleTimeLock.unlock();
	}
	catch(tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
		return;
	}
}

void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	gotPointMatcherCloud(PointMatcher_ROS::rosMsgToPointMatcherCloud<T>(cloudMsgIn), cloudMsgIn.header.stamp);
}

void gotScan(const sensor_msgs::LaserScan& scanMsgIn)
{
	gotPointMatcherCloud(PointMatcher_ROS::rosMsgToPointMatcherCloud<T>(scanMsgIn), scanMsgIn.header.stamp);
}

bool saveMapCallback(map_msgs::SaveMap::Request& req, map_msgs::SaveMap::Response& res)
{
	try
	{
		saveMap(req.filename.data);
		return true;
	}
	catch(const std::runtime_error& e)
	{
		ROS_ERROR_STREAM("Unable to save: " << e.what());
		return false;
	}
}

void mapPublisherLoop()
{
	ros::Rate publishRate(params->mapPublishRate);
	
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
	ros::Rate publishRate(params->mapTfPublishRate);
	
	while(ros::ok())
	{
		mapTfLock.lock();
		PM::TransformationParameters currentOdomToMap = odomToMap;
		mapTfLock.unlock();
		
		geometry_msgs::TransformStamped currentOdomToMapTf = PointMatcher_ROS::pointMatcherTransformationToRosTf<T>(currentOdomToMap, "map", params->odomFrame,
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
	
	params = std::unique_ptr<NodeParameters>(new NodeParameters(pn));
	
	transformation = PM::get().TransformationRegistrar.create("RigidTransformation");
	
	mapper = std::unique_ptr<Mapper>(new Mapper(params->icpConfig, params->inputFiltersConfig, params->mapPostFiltersConfig, params->mapUpdateCondition,
												params->mapUpdateOverlap, params->mapUpdateDelay, params->mapUpdateDistance, params->minDistNewPoint,
												params->sensorMaxRange, params->priorDynamic, params->thresholdDynamic, params->beamHalfAngle, params->epsilonA,
												params->epsilonD, params->alpha, params->beta, params->is3D, params->isOnline, params->computeProbDynamic,
												params->isMapping));
	
	loadInitialMap();
	
	std::thread mapperShutdownThread;
	int messageQueueSize;
	if(params->isOnline)
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
	
	if(params->is3D)
	{
		sub = n.subscribe("points_in", messageQueueSize, gotCloud);
		odomToMap = PM::Matrix::Identity(4, 4);
	}
	else
	{
		sub = n.subscribe("points_in", messageQueueSize, gotScan);
		odomToMap = PM::Matrix::Identity(3, 3);
	}
	mapPublisher = n.advertise<sensor_msgs::PointCloud2>("map", 2, true);
	odomPublisher = n.advertise<nav_msgs::Odometry>("icp_odom", 50, true);
	
	saveMapService = n.advertiseService("save_map", saveMapCallback);
	
	std::thread mapPublisherThread = std::thread(mapPublisherLoop);
	std::thread mapTfPublisherThread = std::thread(mapTfPublisherLoop);
	
	ros::spin();
	
	mapPublisherThread.join();
	mapTfPublisherThread.join();
	if(!params->isOnline)
	{
		mapperShutdownThread.join();
	}
	
	return 0;
}
