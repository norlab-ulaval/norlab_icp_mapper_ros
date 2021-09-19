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
#include <diagnostic_msgs/DiagnosticArray.h>

typedef PointMatcher<float> PM;

std::unique_ptr<ros::NodeHandle> nodeHandle;
std::unique_ptr<NodeParameters> params;
std::shared_ptr<PM::Transformation> transformation;
std::unique_ptr<norlab_icp_mapper::Mapper> mapper;
std::unique_ptr<Trajectory> robotTrajectory;
PM::TransformationParameters odomToMap;
ros::Publisher mapPublisher;
ros::Publisher odomPublisher;
ros::Publisher diagnosticPublisher;
ros::Publisher lastPointCloudBeforeFailurePublisher;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
std::mutex mapTfLock;
std::chrono::time_point<std::chrono::steady_clock> lastTimeInputWasProcessed;
std::mutex idleTimeLock;
bool hasToSetRobotPose;
PM::TransformationParameters robotPoseToSet;
PM::TransformationParameters previousRobotToMap;
ros::Time previousTimeStamp;
unsigned previousSequenceNumber;
PM::DataPoints lastPointCloudBeforeFailure;
ros::Time lastPointCloudBeforeFailureTimeStamp;
std::atomic_bool mapperEnabled;
int consecutiveConvergenceErrorCount;
PM::DataPoints backupMap;
std::atomic_bool backupMapReceived;

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

void gotInput(const PM::DataPoints& input, const std::string& sensorFrame, const ros::Time& timeStamp, const unsigned& sequenceNumber)
{
	diagnostic_msgs::DiagnosticStatus statusMsg;
	statusMsg.hardware_id = "N/A";
	statusMsg.name = "ICP mapper status";

	diagnostic_msgs::KeyValue currentRosTimeKV;
	currentRosTimeKV.key = "ROS time now (approx.)";
	currentRosTimeKV.value = std::to_string(ros::Time::now().toSec());
	statusMsg.values.push_back(currentRosTimeKV);

	diagnostic_msgs::KeyValue currentCloudStampKV;
	currentCloudStampKV.key = "Msg. stamp";
	currentCloudStampKV.value = std::to_string(timeStamp.toSec());
	statusMsg.values.push_back(currentCloudStampKV);

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

		norlab_icp_mapper::DiagnosticInformation info = mapper->processInput(input, sensorToMapBeforeUpdate,
																			 std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.toNSec())));
		const PM::TransformationParameters& sensorToMapAfterUpdate = mapper->getPose();
		lastPointCloudBeforeFailure = transformation->compute(input, sensorToMapAfterUpdate);
		lastPointCloudBeforeFailureTimeStamp = timeStamp;

		mapTfLock.lock();
		odomToMap = transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());
		mapTfLock.unlock();

		PM::TransformationParameters robotToSensor = findTransform(params->robotFrame, sensorFrame, timeStamp, input.getHomogeneousDim());
		PM::TransformationParameters robotToMap = sensorToMapAfterUpdate * robotToSensor;

		robotTrajectory->addPoint(robotToMap.topRightCorner(input.getEuclideanDim(), 1));
		nav_msgs::Odometry odomMsgOut = PointMatcher_ROS::pointMatcherTransformationToOdomMsg<float>(robotToMap, "map", params->robotFrame, timeStamp);

		if(!previousTimeStamp.isZero())
		{
			Eigen::Vector3f linearDisplacement = robotToMap.topRightCorner(input.getEuclideanDim(), 1) - previousRobotToMap.topRightCorner(input.getEuclideanDim(), 1);
			float deltaTime = (float) (timeStamp - previousTimeStamp).toSec();
			Eigen::Vector3f linearVelocity = linearDisplacement / deltaTime;
			odomMsgOut.twist.twist.linear.x = linearVelocity(0);
			odomMsgOut.twist.twist.linear.y = linearVelocity(1);
			odomMsgOut.twist.twist.linear.z = linearVelocity(2);
		}
		previousTimeStamp = timeStamp;
		previousRobotToMap = robotToMap;

		odomPublisher.publish(odomMsgOut);
		if(params->backupInRAM)
		{
			std::stringstream ss;
			ss << odomToMap;
			nodeHandle->setParam("/backup_localization", ss.str());
		}

		if(!params->publishTfsBetweenRegistrations)
		{
			geometry_msgs::TransformStamped odomToMapTf = PointMatcher_ROS::pointMatcherTransformationToRosTf<float>(odomToMap, "map", params->odomFrame, timeStamp);
			tfBroadcaster->sendTransform(odomToMapTf);
		}

		idleTimeLock.lock();
		lastTimeInputWasProcessed = std::chrono::steady_clock::now();
		idleTimeLock.unlock();

		statusMsg.level = diagnostic_msgs::DiagnosticStatus::OK;
		statusMsg.message = "ICP convergence OK.";

		diagnostic_msgs::KeyValue nbPointsInputBeforeFilteringKV;
		nbPointsInputBeforeFilteringKV.key = "Input points before filtering";
		nbPointsInputBeforeFilteringKV.value = std::to_string(info.nbPointsInputBeforeFiltering);
		statusMsg.values.push_back(nbPointsInputBeforeFilteringKV);

		diagnostic_msgs::KeyValue nbPointsInputAfterFilteringKV;
		nbPointsInputAfterFilteringKV.key = "Input points after filtering";
		nbPointsInputAfterFilteringKV.value = std::to_string(info.nbPointsInputAfterFiltering);
		statusMsg.values.push_back(nbPointsInputAfterFilteringKV);

		diagnostic_msgs::KeyValue inputFilteringTimeKV;
		inputFilteringTimeKV.key = "Input filters duration [s]";
		inputFilteringTimeKV.value = std::to_string(info.inputFilteringTime);
		statusMsg.values.push_back(inputFilteringTimeKV);

		diagnostic_msgs::KeyValue nbPointsReferenceKV;
		nbPointsReferenceKV.key = "Reference points";
		nbPointsReferenceKV.value = std::to_string(info.nbPointsReference);
		statusMsg.values.push_back(nbPointsReferenceKV);

		diagnostic_msgs::KeyValue estimatedOverlapKV;
		estimatedOverlapKV.key = "Estimated overlap";
		estimatedOverlapKV.value = std::to_string(info.estimatedOverlap);
		statusMsg.values.push_back(estimatedOverlapKV);

		diagnostic_msgs::KeyValue processingTimeKV;
		processingTimeKV.key = "Processing time";
		processingTimeKV.value = std::to_string(info.processingTime);
		statusMsg.values.push_back(processingTimeKV);

		if(previousSequenceNumber != 0)
		{
			diagnostic_msgs::KeyValue realTimeCapabilityKV;
			realTimeCapabilityKV.key = "Real-time capability %";
			realTimeCapabilityKV.value = std::to_string(info.processingTimePercentage * (float) (sequenceNumber - previousSequenceNumber));
			statusMsg.values.push_back(realTimeCapabilityKV);
		}
		previousSequenceNumber = sequenceNumber;
		consecutiveConvergenceErrorCount = 0;
	}
	catch(const tf2::ExtrapolationException& ex)
	{
		ROS_WARN("%s", ex.what());

		statusMsg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
		statusMsg.message = "TF Extrapolation Exception: " + std::string(ex.what());
	}
	catch(const tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());

		statusMsg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
		statusMsg.message = "Other TF Exception: " + std::string(ex.what());
	}
	catch(const PM::ConvergenceError& ex)
	{
		ROS_WARN("%s", ex.what());

		if(++consecutiveConvergenceErrorCount >= params->consecutiveConvergenceErrorsBeforeFailure)
		{
			sensor_msgs::PointCloud2 pointCloudMsg = PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(lastPointCloudBeforeFailure, "map", lastPointCloudBeforeFailureTimeStamp);
			lastPointCloudBeforeFailurePublisher.publish(pointCloudMsg);
			mapperEnabled.store(false);
		}

		statusMsg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
		statusMsg.message = "ICP failed to converge " + std::to_string(consecutiveConvergenceErrorCount) + " times in a row: " + std::string(ex.what());
	}
	catch(const std::exception& ex)
	{
		ROS_WARN("%s", ex.what());

		statusMsg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
		statusMsg.message = "Unexpected exception: " + std::string(ex.what());
	}
	catch(...)
	{
		ROS_WARN("%s", "Unexpected exception");

		statusMsg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
		statusMsg.message = "Unexpected exception";
	}

	diagnostic_msgs::DiagnosticArray diagnosticMsg;
	diagnosticMsg.header.stamp = timeStamp;
	diagnosticMsg.status.push_back(statusMsg);
	diagnosticPublisher.publish(diagnosticMsg);
}

void pointCloud2Callback(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	if(mapperEnabled.load())
	{
		gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(cloudMsgIn), cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp, cloudMsgIn.header.seq);
	}
}

void laserScanCallback(const sensor_msgs::LaserScan& scanMsgIn)
{
	if(mapperEnabled.load())
	{
		gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(scanMsgIn), scanMsgIn.header.frame_id, scanMsgIn.header.stamp, scanMsgIn.header.seq);
	}
}

void backupMapCallback(const sensor_msgs::PointCloud2& msg)
{
	backupMap = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(msg);
	backupMapReceived.store(true);
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

PM::TransformationParameters rosMsgToPointMatcherPose(const geometry_msgs::Pose& pose) // TODO: put this in libpointmatcher_ros
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

bool clearMapCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	ROS_INFO("Clearing map...");
	mapper->clearMap();
	return true;
}

bool reviveMapperCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	ROS_INFO("Reviving mapper...");
	mapperEnabled.store(true);
	return true;
}

bool reviveAndClearMapCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	ROS_INFO("Reviving and clearing map...");
	mapper->clearMap();
	mapperEnabled.store(true);
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
	nodeHandle = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
	ros::NodeHandle pn("~");

	params = std::unique_ptr<NodeParameters>(new NodeParameters(pn));

	transformation = PM::get().TransformationRegistrar.create("RigidTransformation");
	mapperEnabled.store(true);
	consecutiveConvergenceErrorCount = 0;
	backupMapReceived.store(false);

	if(params->is3D)
	{
		odomToMap = PM::Matrix::Identity(4, 4);
	}
	else
	{
		odomToMap = PM::Matrix::Identity(3, 3);
	}

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

	int messageQueueSize;
	if(params->isOnline)
	{
		tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
		messageQueueSize = 1;
	}
	else
	{
		tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(ros::Duration(ros::DURATION_MAX)));
		messageQueueSize = 0;
	}
	tf2_ros::TransformListener tfListener(*tfBuffer);
	tfBroadcaster = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster);

	// try to overwrite initial map and pose if backup in RAM is enabled
	if(params->backupInRAM)
	{
		ros::Subscriber backupMapSubscriber = nodeHandle->subscribe("/backup_map", 1, backupMapCallback);
		ros::Time startTime = ros::Time::now();
		while(ros::Time::now() - startTime <= ros::Duration(5) && !backupMapReceived.load())
		{
			ros::spinOnce();
		}
		if(backupMapReceived.load())
		{
			ROS_INFO("Loading backup map from RAM...");
			int euclideanDim = params->is3D ? 3 : 2;
			if(backupMap.getEuclideanDim() != euclideanDim)
			{
				throw std::runtime_error("Invalid map dimension");
			}
			mapper->setMap(backupMap);
		}

		std::string backupLocalization;
		if(nodeHandle->getParam("/backup_localization", backupLocalization))
		{
			ROS_INFO("Loading backup localization from RAM...");
			odomToMap = NodeParameters::parseTransformationParameters(backupLocalization, params->is3D);
			hasToSetRobotPose = false;
		}
	}

	std::thread mapperShutdownThread;
	if(!params->isOnline)
	{
		mapperShutdownThread = std::thread(mapperShutdownLoop);

	}

	mapPublisher = nodeHandle->advertise<sensor_msgs::PointCloud2>("map", 2, true);
	odomPublisher = nodeHandle->advertise<nav_msgs::Odometry>("icp_odom", 50, true);
	diagnosticPublisher = nodeHandle->advertise<diagnostic_msgs::DiagnosticArray>("icp_diagnostics", 2, true);
	lastPointCloudBeforeFailurePublisher = nodeHandle->advertise<sensor_msgs::PointCloud2>("last_point_cloud_before_failure", 2, true);

	ros::Subscriber pointCloudSubscriber;
	if(params->is3D)
	{
		robotTrajectory = std::unique_ptr<Trajectory>(new Trajectory(3));
		pointCloudSubscriber = nodeHandle->subscribe("points_in", messageQueueSize, pointCloud2Callback);
	}
	else
	{
		robotTrajectory = std::unique_ptr<Trajectory>(new Trajectory(2));
		pointCloudSubscriber = nodeHandle->subscribe("points_in", messageQueueSize, laserScanCallback);
	}

	ros::ServiceServer reloadYamlConfigService = nodeHandle->advertiseService("reload_yaml_config", reloadYamlConfigCallback);
	ros::ServiceServer saveMapService = nodeHandle->advertiseService("save_map", saveMapCallback);
	ros::ServiceServer loadMapService = nodeHandle->advertiseService("load_map", loadMapCallback);
	ros::ServiceServer saveTrajectoryService = nodeHandle->advertiseService("save_trajectory", saveTrajectoryCallback);
	ros::ServiceServer enableMappingService = nodeHandle->advertiseService("enable_mapping", enableMappingCallback);
	ros::ServiceServer disableMappingService = nodeHandle->advertiseService("disable_mapping", disableMappingCallback);
	ros::ServiceServer clearMapService = nodeHandle->advertiseService("clear_map", clearMapCallback);
	ros::ServiceServer reviveMapperService = nodeHandle->advertiseService("revive_mapper", reviveMapperCallback);
	ros::ServiceServer reviveAndClearMapService = nodeHandle->advertiseService("revive_and_clear_map", reviveAndClearMapCallback);

	std::thread mapPublisherThread = std::thread(mapPublisherLoop);
	std::thread mapTfPublisherThread;
	if(params->publishTfsBetweenRegistrations)
	{
		mapTfPublisherThread = std::thread(mapTfPublisherLoop);
	}

	ros::spin();

	mapPublisherThread.join();
	if(params->publishTfsBetweenRegistrations)
	{
		mapTfPublisherThread.join();
	}
	if(!params->isOnline)
	{
		mapperShutdownThread.join();
	}

	return 0;
}
