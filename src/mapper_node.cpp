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
		bool validInertiaMeasurement = false;
		imu_odom::Inertia currentInertia;
		if(params->isOnline)
		{
			// use inertia measurement at the beginning of the point cloud or latest one if not possible
			inertiaMeasurementsMutex.lock();
			while(inertiaMeasurements.size() >= 2 && inertiaMeasurements.front().header.stamp < timeStamp)
			{
				inertiaMeasurements.pop_front();
			}
			if(!inertiaMeasurements.empty())
			{
				validInertiaMeasurement = true;
				currentInertia = inertiaMeasurements.front();
			}
			inertiaMeasurementsMutex.unlock();
		}
		else
		{
			// compute average inertia measurement throughout the point cloud
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
			while(inertiaMeasurements.front().header.stamp < timeStamp)
			{
				inertiaMeasurements.pop_front();
			}
			int inertiaMeasurementCounter = 0;
			for(auto it = inertiaMeasurements.begin(); it != inertiaMeasurements.end(); it++)
			{
				if(it->header.stamp < (timeStamp + ros::Duration(0.1)))
				{
					validInertiaMeasurement = true;
					inertiaMeasurementCounter ++;
					currentInertia.linear_velocity.x += it->linear_velocity.x;
					currentInertia.linear_velocity.y += it->linear_velocity.y;
					currentInertia.linear_velocity.z += it->linear_velocity.z;
					currentInertia.linear_acceleration.x += it->linear_acceleration.x;
					currentInertia.linear_acceleration.y += it->linear_acceleration.y;
					currentInertia.linear_acceleration.z += it->linear_acceleration.z;
					currentInertia.angular_velocity.x += it->angular_velocity.x;
					currentInertia.angular_velocity.y += it->angular_velocity.y;
					currentInertia.angular_velocity.z += it->angular_velocity.z;
					currentInertia.angular_acceleration.x += it->angular_acceleration.x;
					currentInertia.angular_acceleration.y += it->angular_acceleration.y;
					currentInertia.angular_acceleration.z += it->angular_acceleration.z;
				}
			}
			inertiaMeasurementsMutex.unlock();
			currentInertia.linear_velocity.x /= inertiaMeasurementCounter;
			currentInertia.linear_velocity.y /= inertiaMeasurementCounter;
			currentInertia.linear_velocity.z /= inertiaMeasurementCounter;
			currentInertia.linear_acceleration.x /= inertiaMeasurementCounter;
			currentInertia.linear_acceleration.y /= inertiaMeasurementCounter;
			currentInertia.linear_acceleration.z /= inertiaMeasurementCounter;
			currentInertia.angular_velocity.x /= inertiaMeasurementCounter;
			currentInertia.angular_velocity.y /= inertiaMeasurementCounter;
			currentInertia.angular_velocity.z /= inertiaMeasurementCounter;
			currentInertia.angular_acceleration.x /= inertiaMeasurementCounter;
			currentInertia.angular_acceleration.y /= inertiaMeasurementCounter;
			currentInertia.angular_acceleration.z /= inertiaMeasurementCounter;
		}
		
		float linearSpeedNoiseX = 0.0f * currentInertia.linear_velocity.x;
		float linearSpeedNoiseY = 0.0f * currentInertia.linear_velocity.y;
		float linearSpeedNoiseZ = 0.0f * currentInertia.linear_velocity.z;
		float linearAccelerationNoiseX = 1.4f * std::log(currentInertia.linear_acceleration.x + 0.3) + 1.7;
		float linearAccelerationNoiseY = 1.4f * std::log(currentInertia.linear_acceleration.y + 0.3) + 1.7;
		float linearAccelerationNoiseZ = 1.4f * std::log(currentInertia.linear_acceleration.z + 0.3) + 1.7;
		float angularSpeedNoiseX = 1.0f * std::pow(currentInertia.angular_velocity.x / 3.5, 4);
		float angularSpeedNoiseY = 1.0f * std::pow(currentInertia.angular_velocity.y / 3.5, 4);
		float angularSpeedNoiseZ = 1.0f * std::pow(currentInertia.angular_velocity.z / 3.5, 4);
		float angularAccelerationNoiseX = 0.0f * currentInertia.angular_acceleration.x;
		float angularAccelerationNoiseY = 0.0f * currentInertia.angular_acceleration.y;
		float angularAccelerationNoiseZ = 0.0f * currentInertia.angular_acceleration.z;
		
		PM::TransformationParameters sensorToOdom = findTransform(params->sensorFrame, params->odomFrame, timeStamp, input.getHomogeneousDim());
		PM::TransformationParameters sensorToMapBeforeUpdate = odomToMap * sensorToOdom;
		
		PM::TransformationParameters sensorToMapAfterUpdate;
		if(params->computeResidual && validInertiaMeasurement && firstIcpOdomSet)
		{
			PM::DataPoints map = mapper->getMap();
			
			mapper->processInput(input, sensorToMapBeforeUpdate,
								 std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.toNSec())),
								 linearSpeedNoiseX, linearSpeedNoiseY, linearSpeedNoiseZ, linearAccelerationNoiseX, linearAccelerationNoiseY,
								 linearAccelerationNoiseZ, angularSpeedNoiseX, angularSpeedNoiseY, angularSpeedNoiseZ, angularAccelerationNoiseX,
								 angularAccelerationNoiseY, angularAccelerationNoiseZ);
			sensorToMapAfterUpdate = mapper->getSensorPose();

			PM::DataPoints inputInMapFrame = transformation->compute(input, sensorToMapAfterUpdate);
			
			icp.matcher->init(map);
			PM::Matches matches(icp.matcher->findClosests(inputInMapFrame));
			PM::Parameters outlierParams;
			outlierParams["ratio"] = "1.0";
			std::shared_ptr<PM::OutlierFilter> outlierFilter = PM::get().OutlierFilterRegistrar.create("TrimmedDistOutlierFilter", outlierParams);
			PM::OutlierWeights outlierWeights = outlierFilter->compute(inputInMapFrame, map, matches);
			std::shared_ptr<PM::ErrorMinimizer> errorMinimizer = PM::get().ErrorMinimizerRegistrar.create("PointToPointErrorMinimizer");
			float meanResidual = errorMinimizer->getResidualError(inputInMapFrame, map, outlierWeights, matches) / inputInMapFrame.getNbPoints();
			
			std_msgs::Float32 residualMsg;
			residualMsg.data = meanResidual;
			residualPublisher.publish(residualMsg);
			
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
								 linearSpeedNoiseX, linearSpeedNoiseY, linearSpeedNoiseZ, linearAccelerationNoiseX, linearAccelerationNoiseY,
								 linearAccelerationNoiseZ, angularSpeedNoiseX, angularSpeedNoiseY, angularSpeedNoiseZ, angularAccelerationNoiseX,
								 angularAccelerationNoiseY, angularAccelerationNoiseZ);
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
	std::cout << "Transformation between first and last pose:" << std::endl << firstIcpOdom.inverse() * lastIcpOdom << std::endl;
	
	return 0;
}
