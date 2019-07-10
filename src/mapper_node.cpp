#include "Mapper.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <memory>
#include <mutex>
#include <thread>
#include <fstream>

std::string odomFrame;
std::string sensorFrame;
std::string robotFrame;
std::string initialMapFileName;
std::string initialMapPose;
std::string finalMapFileName;
std::string icpConfig;
std::string inputFiltersConfig;
std::string mapPostFiltersConfig;
std::string mapUpdateCondition;
float mapUpdateOverlap;
float mapUpdateDelay;
float mapUpdateDistance;
float mapPublishRate;
float mapTfPublishRate;
float maxIdleTime;
float minDistNewPoint;
float sensorMaxRange;
float priorDynamic;
float thresholdDynamic;
float beamHalfAngle;
float epsilonA;
float epsilonD;
float alpha;
float beta;
bool is3D;
bool isOnline;
bool computeProbDynamic;
bool isMapping;

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
std::chrono::time_point<std::chrono::steady_clock> lastTimePointsWereProcessed;
std::mutex idleTimeLock;

// ========================================================= Node Parameters =========================================================
// odom_frame: Name of the frame used for odometry.
// sensor_frame: Name of the frame in which the points are published.
// robot_frame: Name of the frame centered on the robot.
// initial_map_file_name: Name of the file from which the initial map is loaded.
// initial_map_pose: Transformation matrix in homogeneous coordinates describing the pose of the initial map in the current map frame.
// final_map_file_name: Name of the file in which the final map is saved when is_online is false.
// icp_config: Name of the file containing the libpointmatcher icp config.
// input_filters_config: Name of the file containing the filters applied to the sensor points.
// map_post_filters_config: Name of the file containing the filters applied to the map after the update.
// map_update_condition: Condition for map update. It can either be 'overlap', 'delay' or 'distance'.
// map_update_overlap: Overlap between sensor and map points under which the map is updated.
// map_update_delay: Delay since last map update over which the map is updated (in seconds).
// map_update_distance: Euclidean distance from last map update over which the map is updated (in meters).
// map_publish_rate: Rate at which the map is published (in Hertz). It can be slower depending on the map update rate.
// map_tf_publish_rate: Rate at which the map tf is published (in Hertz).
// max_idle_time: Delay to wait being idle before shutting down ROS when is_online is false (in seconds). The specified delay is respected up to a precision of 1/10th of a second.
// min_dist_new_point: Distance from current map points under which a new point is not added to the map (in meters).
// sensor_max_range: Maximum reading distance of the laser. Used to cut the global map before matching (in meters).
// prior_dynamic: A priori probability of points being dynamic.
// threshold_dynamic: Probability at which a point is considered permanently dynamic.
// beam_half_angle: Half angle of the cones formed by the sensor laser beams (in rad).
// epsilon_a: Error proportional to the sensor distance.
// epsilon_d: Fix error on the sensor distance (in meters).
// alpha: Probability of staying static given that the point was static.
// beta: Probability of staying dynamic given that the point was dynamic.
// is_3D: true when a 3D sensor is used, false when a 2D sensor is used.
// is_online: true when online mapping is wanted, false otherwise.
// compute_prob_dynamic: true when computation of probability of points being dynamic is wanted, false otherwise.
// is_mapping: true when map updates are wanted, false when only localization is wanted.
// ===================================================================================================================================

void retrieveParameters(const ros::NodeHandle& pn)
{
	pn.param<std::string>("odom_frame", odomFrame, "odom");
	pn.param<std::string>("sensor_frame", sensorFrame, "velodyne");
	pn.param<std::string>("robot_frame", robotFrame, "base_link");
	pn.param<std::string>("initial_map_file_name", initialMapFileName, "");
	pn.param<std::string>("initial_map_pose", initialMapPose, "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]");
	pn.param<std::string>("final_map_file_name", finalMapFileName, "map.vtk");
	pn.param<std::string>("icp_config", icpConfig, "");
	pn.param<std::string>("input_filters_config", inputFiltersConfig, "");
	pn.param<std::string>("map_post_filters_config", mapPostFiltersConfig, "");
	pn.param<std::string>("map_update_condition", mapUpdateCondition, "overlap");
	pn.param<float>("map_update_overlap", mapUpdateOverlap, 0.9);
	pn.param<float>("map_update_delay", mapUpdateDelay, 1);
	pn.param<float>("map_update_distance", mapUpdateDistance, 0.5);
	pn.param<float>("map_publish_rate", mapPublishRate, 10);
	pn.param<float>("map_tf_publish_rate", mapTfPublishRate, 10);
	pn.param<float>("max_idle_time", maxIdleTime, 10);
	pn.param<float>("min_dist_new_point", minDistNewPoint, 0.03);
	pn.param<float>("sensor_max_range", sensorMaxRange, 80);
	pn.param<float>("prior_dynamic", priorDynamic, 0.6);
	pn.param<float>("threshold_dynamic", thresholdDynamic, 0.9);
	pn.param<float>("beam_half_angle", beamHalfAngle, 0.01);
	pn.param<float>("epsilon_a", epsilonA, 0.01);
	pn.param<float>("epsilon_d", epsilonD, 0.01);
	pn.param<float>("alpha", alpha, 0.8);
	pn.param<float>("beta", beta, 0.99);
	pn.param<bool>("is_3D", is3D, true);
	pn.param<bool>("is_online", isOnline, true);
	pn.param<bool>("compute_prob_dynamic", computeProbDynamic, false);
	pn.param<bool>("is_mapping", isMapping, true);
}

void loadExternalParameters()
{
	if(!inputFiltersConfig.empty())
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
	
	if(!mapPostFiltersConfig.empty())
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

PM::TransformationParameters parseInitialMapPose()
{
	int nbRows = is3D ? 4 : 3;
	
	PM::TransformationParameters parsedPose = PM::TransformationParameters::Identity(nbRows, nbRows);
	
	initialMapPose.erase(std::remove(initialMapPose.begin(), initialMapPose.end(), '['), initialMapPose.end());
	initialMapPose.erase(std::remove(initialMapPose.begin(), initialMapPose.end(), ']'), initialMapPose.end());
	std::replace(initialMapPose.begin(), initialMapPose.end(), ',', ' ');
	std::replace(initialMapPose.begin(), initialMapPose.end(), ';', ' ');
	
	float poseMatrix[nbRows * nbRows];
	std::stringstream poseStringStream(initialMapPose);
	for(int i = 0; i < nbRows * nbRows; i++)
	{
		if(!(poseStringStream >> poseMatrix[i]))
		{
			ROS_ERROR_STREAM("An error occurred while trying to parse the initial map pose. No transformation will be applied to the initial map.");
			return parsedPose;
		}
	}
	
	float extraOutput = 0;
	if((poseStringStream >> extraOutput))
	{
		ROS_ERROR_STREAM("Wrong initial pose matrix size. No transformation will be applied to the initial map.");
		return parsedPose;
	}
	
	for(int i = 0; i < nbRows * nbRows; i++)
	{
		parsedPose(i / nbRows, i % nbRows) = poseMatrix[i];
	}
	
	return parsedPose;
}

void loadInitialMap()
{
	if(!initialMapFileName.empty())
	{
		PM::DataPoints initialMap = PM::DataPoints::load(initialMapFileName);
		PM::TransformationParameters initialMapTransformation = parseInitialMapPose();
		initialMap = transformation->compute(initialMap, initialMapTransformation);
		mapper->setMap(initialMap);
	}
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
		
		if(idleTime > std::chrono::duration<float>(maxIdleTime))
		{
			ROS_INFO("Saving map to %s", finalMapFileName.c_str());
			mapper->getMap().save(finalMapFileName);
			ROS_INFO("Shutting down ROS");
			ros::shutdown();
		}
		
		std::this_thread::sleep_for(std::chrono::duration<float>(0.1));
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
	mapper->processCloud(cloud, sensorToMapBeforeUpdate, std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.toNSec())));
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
	lastTimePointsWereProcessed = std::chrono::steady_clock::now();
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
	
	mapper = std::unique_ptr<Mapper>(new Mapper(icpConfig, inputFilters, mapPostFilters, mapUpdateCondition, mapUpdateOverlap, mapUpdateDelay,
												mapUpdateDistance, minDistNewPoint, sensorMaxRange, priorDynamic, thresholdDynamic, beamHalfAngle, epsilonA,
												epsilonD, alpha, beta, is3D, isOnline, computeProbDynamic, isMapping));
	
	loadInitialMap();
	
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
	mapPublisher = n.advertise<sensor_msgs::PointCloud2>("map", 2, true);
	odomPublisher = n.advertise<nav_msgs::Odometry>("icp_odom", 50, true);
	
	std::thread mapPublisherThread = std::thread(mapPublisherLoop);
	std::thread mapTfPublisherThread = std::thread(mapTfPublisherLoop);
	
	ros::spin();
	
	mapPublisherThread.join();
	mapTfPublisherThread.join();
	if(!isOnline)
	{
		mapperShutdownThread.join();
	}
	
	return 0;
}
