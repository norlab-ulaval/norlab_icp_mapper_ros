#include "NodeParameters.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <std_srvs/Empty.h>
#include <map_msgs/SaveMap.h>
#include <memory>
#include <mutex>
#include <thread>
#include <fstream>
#include <std_msgs/Float32.h>
#include "Trajectory.h"
#include <imu_odom/Inertia.h>

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
std::mutex inertiaMeasurementsMutex;
std::list<imu_odom::Inertia> inertiaMeasurements;
PM::ICP icp;

bool firstIcpOdomSet = false;
PM::TransformationParameters firstIcpOdom;
PM::TransformationParameters lastIcpOdom;

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
		std::vector<imu_odom::Inertia> cloudInertiaMeasurements;
		if(params->isOnline)
		{
			// use inertia measurement at the beginning of the point cloud or latest one if not possible
			inertiaMeasurementsMutex.lock();
			while(inertiaMeasurements.size() >= 2 && (++inertiaMeasurements.begin())->header.stamp <= timeStamp)
			{
				inertiaMeasurements.pop_front();
			}
			if(!inertiaMeasurements.empty())
			{
				cloudInertiaMeasurements.push_back(inertiaMeasurements.front());
			}
			inertiaMeasurementsMutex.unlock();
		}
		else
		{
			// take all inertia measurements throughout the point cloud
			inertiaMeasurementsMutex.lock();
			ros::Time latestInertiaMeasurementTime = inertiaMeasurements.back().header.stamp;
			inertiaMeasurementsMutex.unlock();
			while(latestInertiaMeasurementTime < (timeStamp + ros::Duration(0.1)))
			{
				ros::Duration(0.01).sleep();
				inertiaMeasurementsMutex.lock();
				latestInertiaMeasurementTime = inertiaMeasurements.back().header.stamp;
				inertiaMeasurementsMutex.unlock();
			}
			
			inertiaMeasurementsMutex.lock();
			while(inertiaMeasurements.size() >= 2 && (++inertiaMeasurements.begin())->header.stamp <= timeStamp)
			{
				inertiaMeasurements.pop_front();
			}
			for(auto it = inertiaMeasurements.begin(); it != inertiaMeasurements.end(); it++)
			{
				if(it->header.stamp < (timeStamp + ros::Duration(0.1)))
				{
					cloudInertiaMeasurements.push_back(*it);
				}
			}
			inertiaMeasurementsMutex.unlock();
		}

		std::string linearSpeedNoisesX = "";
		std::string linearSpeedNoisesY = "";
		std::string linearSpeedNoisesZ = "";
		std::string linearAccelerationNoisesX = "";
		std::string linearAccelerationNoisesY = "";
		std::string linearAccelerationNoisesZ = "";
		std::string angularSpeedNoisesX = "";
		std::string angularSpeedNoisesY = "";
		std::string angularSpeedNoisesZ = "";
		std::string angularAccelerationNoisesX = "";
		std::string angularAccelerationNoisesY = "";
		std::string angularAccelerationNoisesZ = "";
		std::string measureTimes = "";
		if(cloudInertiaMeasurements.size() > 0)
		{
			for(int i = 0; i < cloudInertiaMeasurements.size(); i++)
			{
				linearSpeedNoisesX += std::to_string((std::log(std::fabs(cloudInertiaMeasurements[i].linear_velocity.x) + 0.001) / 65.0) + 0.105) +
								  (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
				linearSpeedNoisesY += std::to_string((std::log(std::fabs(cloudInertiaMeasurements[i].linear_velocity.y) + 0.001) / 65.0) + 0.105) +
								  (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
				linearSpeedNoisesZ += std::to_string((std::log(std::fabs(cloudInertiaMeasurements[i].linear_velocity.z) + 0.001) / 65.0) + 0.105) +
								  (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
				linearAccelerationNoisesX += std::to_string(0.0f * std::fabs(cloudInertiaMeasurements[i].linear_acceleration.x)) +
								  (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
				linearAccelerationNoisesY += std::to_string(0.0f * std::fabs(cloudInertiaMeasurements[i].linear_acceleration.y)) +
										 (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
				linearAccelerationNoisesZ += std::to_string(0.0f * std::fabs(cloudInertiaMeasurements[i].linear_acceleration.z)) +
										 (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
				angularSpeedNoisesX += std::to_string(std::pow(std::fabs(cloudInertiaMeasurements[i].angular_velocity.x)/65.0, 2)) +
								  (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
				angularSpeedNoisesY += std::to_string(std::pow(std::fabs(cloudInertiaMeasurements[i].angular_velocity.y)/65.0, 2)) +
								  (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
				angularSpeedNoisesZ += std::to_string(std::pow(std::fabs(cloudInertiaMeasurements[i].angular_velocity.z)/65.0, 2)) +
								  (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
				angularAccelerationNoisesX += std::to_string(0.0f * std::fabs(cloudInertiaMeasurements[i].angular_acceleration.x)) +
										 (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
				angularAccelerationNoisesY += std::to_string(0.0f * std::fabs(cloudInertiaMeasurements[i].angular_acceleration.y)) +
										 (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
				angularAccelerationNoisesZ += std::to_string(0.0f * std::fabs(cloudInertiaMeasurements[i].angular_acceleration.z)) +
										 (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
				measureTimes += std::to_string((cloudInertiaMeasurements[i].header.stamp - timeStamp).toSec()) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
			}
		}
		else
		{
			linearSpeedNoisesX = "0";
			linearSpeedNoisesY = "0";
			linearSpeedNoisesZ = "0";
			linearAccelerationNoisesX = "0";
			linearAccelerationNoisesY = "0";
			linearAccelerationNoisesZ = "0";
			angularSpeedNoisesX = "0";
			angularSpeedNoisesY = "0";
			angularSpeedNoisesZ = "0";
			angularAccelerationNoisesX = "0";
			angularAccelerationNoisesY = "0";
			angularAccelerationNoisesZ = "0";
			measureTimes = "0";
		}
		
		PM::TransformationParameters sensorToOdom = findTransform(params->sensorFrame, params->odomFrame, timeStamp, input.getHomogeneousDim());
		PM::TransformationParameters sensorToMapBeforeUpdate = odomToMap * sensorToOdom;
		
		PM::TransformationParameters sensorToMapAfterUpdate;
		if(params->computeResidual && cloudInertiaMeasurements.size() > 0 && firstIcpOdomSet)
		{
			PM::DataPoints map = mapper->getMap();
			
			mapper->processInput(input, sensorToMapBeforeUpdate,
								 std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.toNSec())),
								 linearSpeedNoisesX, linearSpeedNoisesY, linearSpeedNoisesZ, linearAccelerationNoisesX, linearAccelerationNoisesY,
								 linearAccelerationNoisesZ, angularSpeedNoisesX, angularSpeedNoisesY, angularSpeedNoisesZ, angularAccelerationNoisesX,
								 angularAccelerationNoisesY, angularAccelerationNoisesZ, measureTimes);
			sensorToMapAfterUpdate = mapper->getSensorPose();

			PM::DataPoints inputInMapFrame = transformation->compute(input, sensorToMapAfterUpdate);
			
			icp.matcher->init(map);
			PM::Matches matches(icp.matcher->findClosests(inputInMapFrame));
			PM::Parameters outlierParams;
			outlierParams["ratio"] = "1.0";
			std::shared_ptr<PM::OutlierFilter> outlierFilter = PM::get().OutlierFilterRegistrar.create("TrimmedDistOutlierFilter", outlierParams);
			PM::OutlierWeights outlierWeights = outlierFilter->compute(inputInMapFrame, map, matches);
			typename PM::ErrorMinimizer::ErrorElements errorElements(inputInMapFrame, map, outlierWeights, matches);
			float meanResidual = (errorElements.reading.features - errorElements.reference.features).colwise().norm().mean();
			
			std_msgs::Float32 residualMsg;
			residualMsg.data = meanResidual;
			residualPublisher.publish(residualMsg);
			
			imu_odom::Inertia currentInertia;
			for(const auto& inertiaMeasurement: cloudInertiaMeasurements)
			{
				currentInertia.linear_velocity.x += inertiaMeasurement.linear_velocity.x / cloudInertiaMeasurements.size();
				currentInertia.linear_velocity.y += inertiaMeasurement.linear_velocity.y / cloudInertiaMeasurements.size();
				currentInertia.linear_velocity.z += inertiaMeasurement.linear_velocity.z / cloudInertiaMeasurements.size();
				currentInertia.linear_acceleration.x += inertiaMeasurement.linear_acceleration.x / cloudInertiaMeasurements.size();
				currentInertia.linear_acceleration.y += inertiaMeasurement.linear_acceleration.y / cloudInertiaMeasurements.size();
				currentInertia.linear_acceleration.z += inertiaMeasurement.linear_acceleration.z / cloudInertiaMeasurements.size();
				currentInertia.angular_velocity.x += inertiaMeasurement.angular_velocity.x / cloudInertiaMeasurements.size();
				currentInertia.angular_velocity.y += inertiaMeasurement.angular_velocity.y / cloudInertiaMeasurements.size();
				currentInertia.angular_velocity.z += inertiaMeasurement.angular_velocity.z / cloudInertiaMeasurements.size();
				currentInertia.angular_acceleration.x += inertiaMeasurement.angular_acceleration.x / cloudInertiaMeasurements.size();
				currentInertia.angular_acceleration.y += inertiaMeasurement.angular_acceleration.y / cloudInertiaMeasurements.size();
				currentInertia.angular_acceleration.z += inertiaMeasurement.angular_acceleration.z / cloudInertiaMeasurements.size();
			}
			
			float linearSpeed = std::sqrt(std::pow(currentInertia.linear_velocity.x, 2) + std::pow(currentInertia.linear_velocity.y, 2) + std::pow(currentInertia.linear_velocity.z, 2));
			float linearAcceleration = std::sqrt(std::pow(currentInertia.linear_acceleration.x, 2) + std::pow(currentInertia.linear_acceleration.y, 2) + std::pow(currentInertia.linear_acceleration.z, 2));
			float angularSpeed = std::sqrt(std::pow(currentInertia.angular_velocity.x, 2) + std::pow(currentInertia.angular_velocity.y, 2) + std::pow(currentInertia.angular_velocity.z, 2));
			float angularAcceleration = std::sqrt(std::pow(currentInertia.angular_acceleration.x, 2) + std::pow(currentInertia.angular_acceleration.y, 2) + std::pow(currentInertia.angular_acceleration.z, 2));
			
			meanResidualFile << timeStamp << "," << linearSpeed << "," << linearAcceleration << "," << angularSpeed << "," << angularAcceleration << "," << meanResidual << std::endl;
		}
		else
		{
			mapper->processInput(input, sensorToMapBeforeUpdate,
								 std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.toNSec())),
								 linearSpeedNoisesX, linearSpeedNoisesY, linearSpeedNoisesZ, linearAccelerationNoisesX, linearAccelerationNoisesY,
								 linearAccelerationNoisesZ, angularSpeedNoisesX, angularSpeedNoisesY, angularSpeedNoisesZ, angularAccelerationNoisesX,
								 angularAccelerationNoisesY, angularAccelerationNoisesZ, measureTimes);
			sensorToMapAfterUpdate = mapper->getSensorPose();
		}
		
		mapTfLock.lock();
		odomToMap = transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());
		mapTfLock.unlock();
		
		PM::TransformationParameters robotToSensor = findTransform(params->robotFrame, params->sensorFrame, timeStamp, input.getHomogeneousDim());
		PM::TransformationParameters robotToMap = sensorToMapAfterUpdate * robotToSensor;
		
		if(!firstIcpOdomSet)
		{
			firstIcpOdomSet = true;
			firstIcpOdom = robotToMap;
		}
		lastIcpOdom = robotToMap;
		
		trajectory->addPoint(robotToMap.topRightCorner(input.getEuclideanDim(), 1));
		nav_msgs::Odometry odomMsgOut = PointMatcher_ROS::pointMatcherTransformationToOdomMsg<T>(robotToMap, "map", params->robotFrame, timeStamp);
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

void inertiaCallback(const imu_odom::Inertia& msg)
{
	inertiaMeasurementsMutex.lock();
	inertiaMeasurements.emplace_back(msg);
	inertiaMeasurementsMutex.unlock();
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
										  params->useSkewWeights, params->isMapping, params->skewModel, params->cornerPointWeight, params->weightQuantile,
										  params->rangePrecision));
	
	loadInitialMap();
	
	std::ifstream ifs(params->icpConfig.c_str());
	icp.loadFromYaml(ifs);
	ifs.close();
	
	if(params->computeResidual)
	{
		meanResidualFile.open(params->meanResidualFileName);
		meanResidualFile << "stamp,linear_speed,linear_acceleration,angular_speed,angular_acceleration,residual" << std::endl;
	}
	
	std::thread mapperShutdownThread;
	int messageQueueSize;
	int inertiaQueueSize;
	if(params->isOnline)
	{
		tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
		messageQueueSize = 1;
		inertiaQueueSize = 100;
	}
	else
	{
		mapperShutdownThread = std::thread(mapperShutdownLoop);
		tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(ros::Duration(ros::DURATION_MAX)));
		messageQueueSize = 0;
		inertiaQueueSize = 0;
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
	
	ros::Subscriber sub2 = n.subscribe("inertia_topic", inertiaQueueSize, inertiaCallback);
	
	residualPublisher = n.advertise<std_msgs::Float32>("residual", 1);
	
	mapPublisher = n.advertise<sensor_msgs::PointCloud2>("map", 2, true);
	odomPublisher = n.advertise<nav_msgs::Odometry>("icp_odom", 50, true);
	
	reloadYamlConfigService = n.advertiseService("reload_yaml_config", reloadYamlConfigCallback);
	saveMapService = n.advertiseService("save_map", saveMapCallback);
	saveTrajectoryService = n.advertiseService("save_trajectory", saveTrajectoryCallback);
	
	std::thread mapPublisherThread = std::thread(mapPublisherLoop);
	std::thread mapTfPublisherThread = std::thread(mapTfPublisherLoop);
	
	ros::MultiThreadedSpinner spinner;
	spinner.spin();
	
	mapPublisherThread.join();
	mapTfPublisherThread.join();
	if(!params->isOnline)
	{
		mapperShutdownThread.join();
	}
	
	meanResidualFile.close();
	
	std::ofstream finalTransformationFile;
	finalTransformationFile.open(params->finalTransformationFileName, std::ios::app);
	finalTransformationFile << firstIcpOdom.inverse() * lastIcpOdom << std::endl;
	finalTransformationFile.close();
	
	PM::TransformationParameters finalMapPose = odomToMap.inverse();
	finalMapPose.topRightCorner<3, 1>() = odomToMap.inverse().topLeftCorner<3, 3>() * -lastIcpOdom.topRightCorner<3, 1>();
	std::ofstream finalMapPoseFile;
	finalMapPoseFile.open(params->finalMapPoseFileName);
	finalMapPoseFile << finalMapPose << std::endl;
	finalMapPoseFile.close();
	
	return 0;
}
