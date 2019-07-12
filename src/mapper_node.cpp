#include "Mapper.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <map_msgs/SaveMap.h>
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

void validateParameters()
{
	if(!initialMapFileName.empty())
	{
		std::ifstream ifs(initialMapFileName.c_str());
		if(!ifs.good())
		{
			throw std::runtime_error("Invalid initial map file: " + initialMapFileName);
		}
		ifs.close();
	}
	
	if(!isOnline)
	{
		std::ofstream ofs(finalMapFileName.c_str(), std::ios_base::app);
		if(!ofs.good())
		{
			throw std::runtime_error("Invalid final map file: " + finalMapFileName);
		}
		ofs.close();
	}
	
	if(!icpConfig.empty())
	{
		std::ifstream ifs(icpConfig.c_str());
		if(!ifs.good())
		{
			throw std::runtime_error("Invalid icp config file: " + icpConfig);
		}
		ifs.close();
	}
	
	if(!inputFiltersConfig.empty())
	{
		std::ifstream ifs(inputFiltersConfig.c_str());
		if(!ifs.good())
		{
			throw std::runtime_error("Invalid input filters config file: " + inputFiltersConfig);
		}
		ifs.close();
	}
	
	if(!mapPostFiltersConfig.empty())
	{
		std::ifstream ifs(mapPostFiltersConfig.c_str());
		if(!ifs.good())
		{
			throw std::runtime_error("Invalid map post filters config file: " + mapPostFiltersConfig);
		}
		ifs.close();
	}
	
	if(mapUpdateCondition != "overlap" && mapUpdateCondition != "delay" && mapUpdateCondition != "distance")
	{
		throw std::runtime_error("Invalid map update condition: " + mapUpdateCondition);
	}
	
	if(mapUpdateOverlap < 0 || mapUpdateOverlap > 1)
	{
		throw std::runtime_error("Invalid map update overlap: " + std::to_string(mapUpdateOverlap));
	}
	
	if(mapUpdateDelay < 0)
	{
		throw std::runtime_error("Invalid map update delay: " + std::to_string(mapUpdateDelay));
	}
	
	if(mapUpdateDistance < 0)
	{
		throw std::runtime_error("Invalid map update distance: " + std::to_string(mapUpdateDistance));
	}
	
	if(mapPublishRate <= 0)
	{
		throw std::runtime_error("Invalid map publish rate: " + std::to_string(mapPublishRate));
	}
	
	if(mapTfPublishRate <= 0)
	{
		throw std::runtime_error("Invalid map tf publish rate: " + std::to_string(mapTfPublishRate));
	}
	
	if(!isOnline)
	{
		if(maxIdleTime < 0)
		{
			throw std::runtime_error("Invalid max idle time: " + std::to_string(maxIdleTime));
		}
	}
	
	if(minDistNewPoint < 0)
	{
		throw std::runtime_error("Invalid minimum distance of new point: " + std::to_string(minDistNewPoint));
	}
	
	if(sensorMaxRange < 0)
	{
		throw std::runtime_error("Invalid sensor max range: " + std::to_string(sensorMaxRange));
	}
	
	if(priorDynamic < 0 || priorDynamic > 1)
	{
		throw std::runtime_error("Invalid prior dynamic: " + std::to_string(priorDynamic));
	}
	
	if(thresholdDynamic < 0 || thresholdDynamic > 1)
	{
		throw std::runtime_error("Invalid threshold dynamic: " + std::to_string(thresholdDynamic));
	}
	
	if(beamHalfAngle < 0 || beamHalfAngle > M_PI_2)
	{
		throw std::runtime_error("Invalid beam half angle: " + std::to_string(beamHalfAngle));
	}
	
	if(epsilonA < 0)
	{
		throw std::runtime_error("Invalid epsilon a: " + std::to_string(epsilonA));
	}
	
	if(epsilonD < 0)
	{
		throw std::runtime_error("Invalid epsilon d: " + std::to_string(epsilonD));
	}
	
	if(alpha < 0 || alpha > 1)
	{
		throw std::runtime_error("Invalid alpha: " + std::to_string(alpha));
	}
	
	if(beta < 0 || beta > 1)
	{
		throw std::runtime_error("Invalid beta: " + std::to_string(beta));
	}
	
	if(!isMapping && initialMapFileName.empty())
	{
		throw std::runtime_error("is mapping is set to false, but initial map file name was not specified.");
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
			throw std::runtime_error("An error occurred while trying to parse the initial map pose.");
		}
	}
	
	float extraOutput = 0;
	if((poseStringStream >> extraOutput))
	{
		throw std::runtime_error("Wrong initial pose matrix size.");
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
		
		if(idleTime > std::chrono::duration<float>(maxIdleTime))
		{
			saveMap(finalMapFileName);
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
		int nbRows = is3D ? 4 : 3;
		
		geometry_msgs::TransformStamped sensorToOdomTf = tfBuffer->lookupTransform(odomFrame, sensorFrame, timeStamp, ros::Duration(0.1));
		PM::TransformationParameters sensorToOdom = PointMatcher_ROS::rosTfToPointMatcherTransformation<T>(sensorToOdomTf, nbRows);
		
		PM::TransformationParameters sensorToMapBeforeUpdate = odomToMap * sensorToOdom;
		mapper->processCloud(cloud, sensorToMapBeforeUpdate, std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.toNSec())));
		const PM::TransformationParameters& sensorToMapAfterUpdate = mapper->getSensorPose();
		
		mapTfLock.lock();
		odomToMap = transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());
		mapTfLock.unlock();
		
		geometry_msgs::TransformStamped robotToSensorTf = tfBuffer->lookupTransform(sensorFrame, robotFrame, timeStamp, ros::Duration(0.1));
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
	validateParameters();
	
	transformation = PM::get().TransformationRegistrar.create("RigidTransformation");
	
	mapper = std::unique_ptr<Mapper>(new Mapper(icpConfig, inputFiltersConfig, mapPostFiltersConfig, mapUpdateCondition, mapUpdateOverlap, mapUpdateDelay,
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
	
	saveMapService = n.advertiseService("save_map", saveMapCallback);
	
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
