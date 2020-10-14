#include "NodeParameters.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <std_srvs/Empty.h>
#include <map_msgs/SaveMap.h>
#include <memory>
#include <mutex>
#include <thread>
#include <fstream>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include "Trajectory.h"

const double GRAVITATIONAL_ACCELERATION = -9.807;

std::unique_ptr<NodeParameters> params;
std::shared_ptr<PM::Transformation> transformation;
std::unique_ptr<norlab_icp_mapper::Mapper> mapper;
std::unique_ptr<Trajectory> trajectory;
PM::TransformationParameters odomToMap;
ros::Subscriber sub;
ros::Publisher mapPublisher;
ros::Publisher odomPublisher;
ros::ServiceServer reloadYamlConfigService;
ros::ServiceServer saveMapService;
ros::ServiceServer saveTrajectoryService;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
std::mutex mapTfLock;
std::chrono::time_point<std::chrono::steady_clock> lastTimeInputWasProcessed;
std::mutex idleTimeLock;
std::ofstream meanResidualFile;
ros::Publisher residualPublisher;
std::mutex imuMeasurementMutex;
double latestLinearAcceleration;
double latestAngularSpeed;
PM::ICP icp;

void loadInitialMap()
{
	if(!params->initialMapFileName.empty())
	{
		PM::DataPoints initialMap = PM::DataPoints::load(params->initialMapFileName);
		
		int euclideanDim = params->is3D ? 3 : 2;
		if(initialMap.getEuclideanDim() != euclideanDim)
		{
			throw std::runtime_error("Invalid initial map dimension.");
		}
		
		initialMap = transformation->compute(initialMap, params->initialMapPose);
		mapper->setMap(initialMap, PM::TransformationParameters::Identity(euclideanDim + 1, euclideanDim + 1));
	}
}

void saveMap(std::string mapFileName)
{
	ROS_INFO("Saving map to %s", mapFileName.c_str());
	mapper->getMap().save(mapFileName);
}

void saveTrajectory(std::string trajectoryFileName)
{
	ROS_INFO("Saving trajectory to %s", trajectoryFileName.c_str());
	trajectory->save(trajectoryFileName);
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

PM::TransformationParameters findTransform(std::string sourceFrame, std::string targetFrame, ros::Time time, int transformDimension)
{
	geometry_msgs::TransformStamped tf = tfBuffer->lookupTransform(targetFrame, sourceFrame, time, ros::Duration(0.1));
	return PointMatcher_ROS::rosTfToPointMatcherTransformation<T>(tf, transformDimension);
}

void gotInput(PM::DataPoints input, ros::Time timeStamp)
{
	try
	{
		double linearAcceleration, angularSpeed;
		imuMeasurementMutex.lock();
		linearAcceleration = latestLinearAcceleration;
		angularSpeed = latestAngularSpeed;
		imuMeasurementMutex.unlock();
		
		PM::TransformationParameters sensorToOdom = findTransform(params->sensorFrame, params->odomFrame, timeStamp, input.getHomogeneousDim());
		PM::TransformationParameters sensorToMapBeforeUpdate = odomToMap * sensorToOdom;
		
		PM::TransformationParameters sensorToMapAfterUpdate;
		if(params->computeResidual)
		{
			PM::DataPoints map = mapper->getMap();
			
			mapper->processInput(input, sensorToMapBeforeUpdate, std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.toNSec())));
			sensorToMapAfterUpdate = mapper->getSensorPose();
		
			PM::DataPoints inputInMapFrame = transformation->compute(input, sensorToMapAfterUpdate);
			
			icp.matcher->init(map);
			PM::Matches matches(icp.matcher->findClosests(inputInMapFrame));
			PM::Parameters outlierParams;
			outlierParams["ratio"] = "1.0";
			std::shared_ptr<PM::OutlierFilter> outlierFilter = PM::get().OutlierFilterRegistrar.create("TrimmedDistOutlierFilter", outlierParams);
			PM::OutlierWeights outlierWeights = outlierFilter->compute(inputInMapFrame, map, matches);
			float meanResidual = icp.errorMinimizer->getResidualError(inputInMapFrame, map, outlierWeights, matches) / inputInMapFrame.getNbPoints();
			
			std_msgs::Float32 residualMsg;
			residualMsg.data = meanResidual;
			residualPublisher.publish(residualMsg);
			
			if(meanResidualFile.is_open())
			{
				meanResidualFile << timeStamp << "," << linearAcceleration << "," << angularSpeed << "," << meanResidual << std::endl;
			}
		}
		else
		{
			mapper->processInput(input, sensorToMapBeforeUpdate, std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.toNSec())));
			sensorToMapAfterUpdate = mapper->getSensorPose();
		}
		
		mapTfLock.lock();
		odomToMap = transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());
		mapTfLock.unlock();
		
		PM::TransformationParameters robotToSensor = findTransform(params->robotFrame, params->sensorFrame, timeStamp, input.getHomogeneousDim());
		PM::TransformationParameters robotToMap = sensorToMapAfterUpdate * robotToSensor;
		
		trajectory->addPoint(robotToMap.topRightCorner(input.getEuclideanDim(), 1));
		nav_msgs::Odometry odomMsgOut = PointMatcher_ROS::pointMatcherTransformationToOdomMsg<T>(robotToMap, "map", timeStamp);
		odomPublisher.publish(odomMsgOut);
		
		idleTimeLock.lock();
		lastTimeInputWasProcessed = std::chrono::steady_clock::now();
		idleTimeLock.unlock();
	}
	catch(tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
		return;
	}
}

void pointCloud2Callback(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<T>(cloudMsgIn), cloudMsgIn.header.stamp);
}

void laserScanCallback(const sensor_msgs::LaserScan& scanMsgIn)
{
	gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<T>(scanMsgIn), scanMsgIn.header.stamp);
}

bool reloadYamlConfigCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	mapper->loadYamlConfig();
	return true;
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

bool saveTrajectoryCallback(map_msgs::SaveMap::Request& req, map_msgs::SaveMap::Response& res)
{
	try
	{
		saveTrajectory(req.filename.data);
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

void imuCallback(const sensor_msgs::Imu& msg)
{
	double linearAcceleration = std::sqrt(std::pow(msg.linear_acceleration.x, 2) +
										  std::pow(msg.linear_acceleration.y, 2) +
										  std::pow(msg.linear_acceleration.z - GRAVITATIONAL_ACCELERATION, 2));
	double angularSpeed = std::sqrt(std::pow(msg.angular_velocity.x, 2) +
									std::pow(msg.angular_velocity.y, 2) +
									std::pow(msg.angular_velocity.z, 2));
	
	imuMeasurementMutex.lock();
	latestLinearAcceleration = linearAcceleration;
	latestAngularSpeed = angularSpeed;
	imuMeasurementMutex.unlock();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mapper_node");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	params = std::unique_ptr<NodeParameters>(new NodeParameters(pn));
	
	transformation = PM::get().TransformationRegistrar.create("RigidTransformation");
	
	mapper = std::unique_ptr<norlab_icp_mapper::Mapper>(
			new norlab_icp_mapper::Mapper(params->icpConfig, params->inputFiltersConfig, params->mapPostFiltersConfig, params->mapUpdateCondition,
										  params->mapUpdateOverlap, params->mapUpdateDelay, params->mapUpdateDistance, params->minDistNewPoint,
										  params->sensorMaxRange, params->priorDynamic, params->thresholdDynamic, params->beamHalfAngle, params->epsilonA,
										  params->epsilonD, params->alpha, params->beta, params->is3D, params->isOnline, params->computeProbDynamic,
										  params->isMapping));
	
	loadInitialMap();
	
	std::ifstream ifs(params->icpConfig.c_str());
	icp.loadFromYaml(ifs);
	ifs.close();
	
	if(!params->meanResidualFileName.empty())
	{
		meanResidualFile.open(params->meanResidualFileName);
		meanResidualFile.close();
		meanResidualFile.open(params->meanResidualFileName, std::ios::app);
		meanResidualFile << "stamp,linear_acceleration,angular_speed,residual" << std::endl;
	}
	
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
		sub = n.subscribe("points_in", messageQueueSize, pointCloud2Callback);
		trajectory = std::unique_ptr<Trajectory>(new Trajectory(3));
		odomToMap = PM::Matrix::Identity(4, 4);
	}
	else
	{
		sub = n.subscribe("points_in", messageQueueSize, laserScanCallback);
		trajectory = std::unique_ptr<Trajectory>(new Trajectory(2));
		odomToMap = PM::Matrix::Identity(3, 3);
	}
	
	ros::Subscriber sub2 = n.subscribe("acceleration_topic", messageQueueSize, imuCallback);
	
	residualPublisher = n.advertise<std_msgs::Float32>("residual", 1);
	
	mapPublisher = n.advertise<sensor_msgs::PointCloud2>("map", 2, true);
	odomPublisher = n.advertise<nav_msgs::Odometry>("icp_odom", 50, true);
	
	reloadYamlConfigService = n.advertiseService("reload_yaml_config", reloadYamlConfigCallback);
	saveMapService = n.advertiseService("save_map", saveMapCallback);
	saveTrajectoryService = n.advertiseService("save_trajectory", saveTrajectoryCallback);
	
	std::thread mapPublisherThread = std::thread(mapPublisherLoop);
	std::thread mapTfPublisherThread = std::thread(mapTfPublisherLoop);
	
	ros::spin();
	
	mapPublisherThread.join();
	mapTfPublisherThread.join();
	if(!params->isOnline)
	{
		mapperShutdownThread.join();
	}
	
	meanResidualFile.close();
	
	return 0;
}
