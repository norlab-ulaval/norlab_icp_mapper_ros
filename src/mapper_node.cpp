#include "NodeParameters.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <std_srvs/Empty.h>
#include "norlab_icp_mapper_ros/SaveMap.h"
#include "norlab_icp_mapper_ros/LoadMap.h"
#include "norlab_icp_mapper_ros/SaveTrajectory.h"
#include <geometry_msgs/Pose.h>
#include <memory>
#include <mutex>
#include <thread>
#include <norlab_icp_mapper/Trajectory.h>

typedef PointMatcher<float> PM;

std::unique_ptr<NodeParameters> params;
std::shared_ptr<PM::Transformation> transformation;
std::unique_ptr<norlab_icp_mapper::Mapper> mapper;
std::unique_ptr<Trajectory> robotTrajectory;
PM::TransformationParameters odomToMap;
ros::Publisher mapPublisher;
ros::Publisher odomPublisher;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
std::mutex mapTfLock;
std::chrono::time_point<std::chrono::steady_clock> lastTimeInputWasProcessed;
std::mutex idleTimeLock;
bool hasToSetRobotPose;
PM::TransformationParameters robotPoseToSet;

void saveMap(const std::string& mapFileName)
{
	ROS_INFO("Saving map to %s", mapFileName.c_str());
	mapper->getMap().save(mapFileName);
}

void loadMap(const std::string& mapFileName)
{
	ROS_INFO("Loading map from %s", mapFileName.c_str());
	PM::DataPoints map = PM::DataPoints::load(mapFileName);
	int euclideanDim = params->is3D ? 3 : 2;
	if(map.getEuclideanDim() != euclideanDim)
	{
		throw std::runtime_error("Invalid map dimension");
	}
	mapper->setMap(map);
}

void setRobotPose(const PM::TransformationParameters& robotPose)
{
	robotPoseToSet = robotPose;
	hasToSetRobotPose = true;
}

void saveTrajectory(const std::string& trajectoryFileName)
{
	ROS_INFO("Saving trajectory to %s", trajectoryFileName.c_str());
	robotTrajectory->save(trajectoryFileName);
}

void mapperShutdownLoop()
{
	std::chrono::duration<float> idleTime = std::chrono::duration<float>::zero();

	while(ros::ok())
	{
		idleTimeLock.lock();
		if(lastTimeInputWasProcessed.time_since_epoch().count())
		{
			idleTime = std::chrono::steady_clock::now() - lastTimeInputWasProcessed;
		}
		idleTimeLock.unlock();

		if(idleTime > std::chrono::duration<float>(params->maxIdleTime))
		{
			saveMap(params->finalMapFileName);
			saveTrajectory(params->finalTrajectoryFileName);
			ROS_INFO("Shutting down ROS");
			ros::shutdown();
		}

		std::this_thread::sleep_for(std::chrono::duration<float>(0.1));
	}
}

PM::TransformationParameters findTransform(const std::string& sourceFrame, const std::string& targetFrame, const ros::Time& time, const int& transformDimension)
{
	geometry_msgs::TransformStamped tf = tfBuffer->lookupTransform(targetFrame, sourceFrame, time, ros::Duration(0.1));
	return PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(tf, transformDimension);
}

void gotInput(const PM::DataPoints& input, const std::string& sensorFrame, const ros::Time& timeStamp)
{
	try
	{
		PM::TransformationParameters sensorToOdom = findTransform(sensorFrame, params->odomFrame, timeStamp, input.getHomogeneousDim());
		PM::TransformationParameters sensorToMapBeforeUpdate = odomToMap * sensorToOdom;

		if(hasToSetRobotPose)
		{
			PM::TransformationParameters sensorToRobot = findTransform(sensorFrame, params->robotFrame, timeStamp, input.getHomogeneousDim());
			sensorToMapBeforeUpdate = robotPoseToSet * sensorToRobot;
			hasToSetRobotPose = false;
		}

		mapper->processInput(input, sensorToMapBeforeUpdate, std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.toNSec())));
		const PM::TransformationParameters& sensorToMapAfterUpdate = mapper->getPose();

		mapTfLock.lock();
		odomToMap = transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());
		mapTfLock.unlock();

		PM::TransformationParameters robotToSensor = findTransform(params->robotFrame, sensorFrame, timeStamp, input.getHomogeneousDim());
		PM::TransformationParameters robotToMap = sensorToMapAfterUpdate * robotToSensor;

		robotTrajectory->addPoint(robotToMap.topRightCorner(input.getEuclideanDim(), 1));
		nav_msgs::Odometry odomMsgOut = PointMatcher_ROS::pointMatcherTransformationToOdomMsg<float>(robotToMap, "map", params->robotFrame, timeStamp);
		odomPublisher.publish(odomMsgOut);

		idleTimeLock.lock();
		lastTimeInputWasProcessed = std::chrono::steady_clock::now();
		idleTimeLock.unlock();
	}
	catch(const tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
	}
}

void pointCloud2Callback(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(cloudMsgIn), cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp);
}

void laserScanCallback(const sensor_msgs::LaserScan& scanMsgIn)
{
	gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(scanMsgIn), scanMsgIn.header.frame_id, scanMsgIn.header.stamp);
}

bool reloadYamlConfigCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	ROS_INFO("Reloading YAML config");
	mapper->loadYamlConfig(params->inputFiltersConfig, params->icpConfig, params->mapPostFiltersConfig);
	return true;
}

bool saveMapCallback(norlab_icp_mapper_ros::SaveMap::Request& req, norlab_icp_mapper_ros::SaveMap::Response& res)
{
	try
	{
		saveMap(req.map_file_name.data);
		return true;
	}
	catch(const std::runtime_error& e)
	{
		ROS_ERROR_STREAM("Unable to save: " << e.what());
		return false;
	}
}

PM::TransformationParameters rosMsgToPointMatcherPose(const geometry_msgs::Pose& pose)
{
	Eigen::Vector3f epsilon(pose.orientation.x, pose.orientation.y, pose.orientation.z);
	float eta = pose.orientation.w;
	PM::Matrix skewSymmetricEpsilon = PM::Matrix::Zero(3, 3);
	skewSymmetricEpsilon << 0, -epsilon[2], epsilon[1],
			epsilon[2], 0, -epsilon[0],
			-epsilon[1], epsilon[0], 0;
	PM::Matrix rotationMatrix = (((eta * eta) - epsilon.dot(epsilon)) * PM::Matrix::Identity(3, 3)) +
								(2 * eta * skewSymmetricEpsilon) + (2 * epsilon * epsilon.transpose());

	Eigen::Vector3f positionVector(pose.position.x, pose.position.y, pose.position.z);

	int euclideanDim = params->is3D ? 3 : 2;
	PM::TransformationParameters transformationParameters = PM::TransformationParameters::Identity(euclideanDim + 1, euclideanDim + 1);
	transformationParameters.topLeftCorner(euclideanDim, euclideanDim) = rotationMatrix.topLeftCorner(euclideanDim, euclideanDim);
	transformationParameters.topRightCorner(euclideanDim, 1) = positionVector.head(euclideanDim);
	return transformationParameters;
}

bool loadMapCallback(norlab_icp_mapper_ros::LoadMap::Request& req, norlab_icp_mapper_ros::LoadMap::Response& res)
{
	try
	{
		loadMap(req.map_file_name.data);
		setRobotPose(rosMsgToPointMatcherPose(req.pose));
		robotTrajectory->clearPoints();
		return true;
	}
	catch(const std::runtime_error& e)
	{
		ROS_ERROR_STREAM("Unable to load: " << e.what());
		return false;
	}
}

bool saveTrajectoryCallback(norlab_icp_mapper_ros::SaveTrajectory::Request& req, norlab_icp_mapper_ros::SaveTrajectory::Response& res)
{
	try
	{
		saveTrajectory(req.trajectory_file_name.data);
		return true;
	}
	catch(const std::runtime_error& e)
	{
		ROS_ERROR_STREAM("Unable to save: " << e.what());
		return false;
	}
}

bool enableMappingCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	ROS_INFO("Enabling mapping");
	mapper->setIsMapping(true);
	return true;
}

bool disableMappingCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	ROS_INFO("Disabling mapping");
	mapper->setIsMapping(false);
	return true;
}

void mapPublisherLoop()
{
	ros::Rate publishRate(params->mapPublishRate);

	PM::DataPoints newMap;
	while(ros::ok())
	{
		if(mapper->getNewLocalMap(newMap))
		{
			sensor_msgs::PointCloud2 mapMsgOut = PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(newMap, "map", ros::Time::now());
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

		geometry_msgs::TransformStamped currentOdomToMapTf = PointMatcher_ROS::pointMatcherTransformationToRosTf<float>(currentOdomToMap, "map",
																														params->odomFrame,
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

	mapper = std::unique_ptr<norlab_icp_mapper::Mapper>(new norlab_icp_mapper::Mapper(params->inputFiltersConfig, params->icpConfig,
																					  params->mapPostFiltersConfig, params->mapUpdateCondition,
																					  params->mapUpdateOverlap, params->mapUpdateDelay,
																					  params->mapUpdateDistance, params->minDistNewPoint,
																					  params->sensorMaxRange, params->priorDynamic, params->thresholdDynamic,
																					  params->beamHalfAngle, params->epsilonA, params->epsilonD, params->alpha,
																					  params->beta, params->is3D, params->isOnline, params->computeProbDynamic,
																					  params->isMapping, params->saveMapCellsOnHardDrive));

	if(!params->initialMapFileName.empty())
	{
		loadMap(params->initialMapFileName);
	}
	setRobotPose(params->initialRobotPose);

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

	ros::Subscriber sub;
	if(params->is3D)
	{
		sub = n.subscribe("points_in", messageQueueSize, pointCloud2Callback);
		robotTrajectory = std::unique_ptr<Trajectory>(new Trajectory(3));
		odomToMap = PM::Matrix::Identity(4, 4);
	}
	else
	{
		sub = n.subscribe("points_in", messageQueueSize, laserScanCallback);
		robotTrajectory = std::unique_ptr<Trajectory>(new Trajectory(2));
		odomToMap = PM::Matrix::Identity(3, 3);
	}

	mapPublisher = n.advertise<sensor_msgs::PointCloud2>("map", 2, true);
	odomPublisher = n.advertise<nav_msgs::Odometry>("icp_odom", 50, true);

	ros::ServiceServer reloadYamlConfigService = n.advertiseService("reload_yaml_config", reloadYamlConfigCallback);
	ros::ServiceServer saveMapService = n.advertiseService("save_map", saveMapCallback);
	ros::ServiceServer loadMapService = n.advertiseService("load_map", loadMapCallback);
	ros::ServiceServer saveTrajectoryService = n.advertiseService("save_trajectory", saveTrajectoryCallback);
	ros::ServiceServer enableMappingService = n.advertiseService("enable_mapping", enableMappingCallback);
	ros::ServiceServer disableMappingService = n.advertiseService("disable_mapping", disableMappingCallback);

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
