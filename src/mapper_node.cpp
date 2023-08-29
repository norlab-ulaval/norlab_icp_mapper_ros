#include "NodeParameters.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <std_srvs/srv/empty.hpp>
#include <map_msgs/srv/save_map.hpp>
#include <memory>
#include <mutex>
#include <thread>
#include <fstream>
#include <std_msgs/msg/float32.hpp>
#include "Trajectory.h"
#include <imu_odom/msg/inertia.hpp>
#include <set>
#include <visualization_msgs/msg/marker_array.hpp>

class MapperNode : public rclcpp::Node
{
public:
    MapperNode():
            Node("mapper_node")
    {
        params = std::unique_ptr<NodeParameters>(new NodeParameters(*this));

        transformation = PM::get().TransformationRegistrar.create("RigidTransformation");

        mapper = std::unique_ptr<norlab_icp_mapper::Mapper>(
                new norlab_icp_mapper::Mapper(params->icpConfig, params->inputFiltersConfig, params->mapPostFiltersConfig, params->mapUpdateCondition,
                                              params->mapUpdateOverlap, params->mapUpdateDelay, params->mapUpdateDistance, params->minDistNewPoint,
                                              params->sensorMaxRange, params->priorDynamic, params->thresholdDynamic, params->beamHalfAngle, params->epsilonA,
                                              params->epsilonD, params->alpha, params->beta, params->is3D, params->isOnline, params->computeProbDynamic,
                                              params->useCRVModel, params->useICRAModel, params->isMapping, params->skewModel, params->cornerPointUncertainty,
                                              params->uncertaintyThreshold, params->uncertaintyQuantile, params->softUncertaintyThreshold, params->binaryUncertaintyThreshold,
                                              params->afterDeskewing, params->scaleFactor));

        loadInitialMap();

        if(!params->icpConfig.empty())
        {
            std::ifstream ifs(params->icpConfig.c_str());
            icp.loadFromYaml(ifs);
            ifs.close();
        }

        lastMarkersSize = 0;

        if(params->computeResidual)
        {
            meanResidualFile.open(params->meanResidualFileName);
            meanResidualFile << "stamp,x,y,z,linear_speed,linear_acceleration,angular_speed,angular_acceleration,residual" << std::endl;
        }

        if(params->recordInertia)
        {
            inertiaFile.open(params->inertiaFileName);
            inertiaFile
                    << "stamp,linear_velocity_x,linear_velocity_y,linear_velocity_z,linear_acceleration_x,linear_acceleration_y,linear_acceleration_z,angular_velocity_x,angular_velocity_y,angular_velocity_z,angular_acceleration_x,angular_acceleration_y,angular_acceleration_z"
                    << std::endl;
        }

        int messageQueueSize;
        int inertiaQueueSize;
        if(params->isOnline)
        {
            tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(this->get_clock()));
            messageQueueSize = 1;
            inertiaQueueSize = 100;
        }
        else
        {
            mapperShutdownThread = std::thread(&MapperNode::mapperShutdownLoop, this);
            tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(this->get_clock(), std::chrono::seconds(1000000)));
            messageQueueSize = 0;
            inertiaQueueSize = 0;
        }

        tfListener = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(*tfBuffer));
        tfBroadcaster = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster(*this));

        mapPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 2);
        markerPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("cov_markers", 2);
        odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("icp_odom", 50);
        filteredCloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", messageQueueSize);
        residualPublisher = this->create_publisher<std_msgs::msg::Float32>("residual", 1);

        removeNanFilter = PM::get().DataPointsFilterRegistrar.create("RemoveNaNDataPointsFilter");

        pointCloudCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions pointCloudSubscriptionOptions;
        pointCloudSubscriptionOptions.callback_group = pointCloudCallbackGroup;
        if(params->is3D)
        {
            trajectory = std::unique_ptr<Trajectory>(new Trajectory(3));
            odomToMap = PM::Matrix::Identity(4, 4);
            pointCloud2Subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>("points_in", messageQueueSize,
                                                                                               std::bind(&MapperNode::pointCloud2Callback, this, std::placeholders::_1),
                                                                                               pointCloudSubscriptionOptions);
        }
        else
        {
            trajectory = std::unique_ptr<Trajectory>(new Trajectory(2));
            odomToMap = PM::Matrix::Identity(3, 3);
            laserScanSubscription = this->create_subscription<sensor_msgs::msg::LaserScan>("points_in", messageQueueSize,
                                                                                           std::bind(&MapperNode::laserScanCallback, this, std::placeholders::_1),
                                                                                           pointCloudSubscriptionOptions);
        }

        inertiaCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions inertiaSubscriptionOptions;
        inertiaSubscriptionOptions.callback_group = inertiaCallbackGroup;
        inertiaSubscription = this->create_subscription<imu_odom::msg::Inertia>("inertia_topic", inertiaQueueSize,
                                                                                std::bind(&MapperNode::inertiaCallback, this, std::placeholders::_1), inertiaSubscriptionOptions);

        reloadYamlConfigService = this->create_service<std_srvs::srv::Empty>("reload_yaml_config",
                                                                             std::bind(&MapperNode::reloadYamlConfigCallback, this, std::placeholders::_1, std::placeholders::_2));
        saveMapService = this->create_service<map_msgs::srv::SaveMap>("save_map", std::bind(&MapperNode::saveMapCallback, this, std::placeholders::_1, std::placeholders::_2));
        saveTrajectoryService = this->create_service<map_msgs::srv::SaveMap>("save_trajectory",
                                                                             std::bind(&MapperNode::saveTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));

        mapPublisherThread = std::thread(&MapperNode::mapPublisherLoop, this);
        mapTfPublisherThread = std::thread(&MapperNode::mapTfPublisherLoop, this);
    }

    ~MapperNode()
    {
        mapPublisherThread.join();
        mapTfPublisherThread.join();
        if(!params->isOnline)
        {
            mapperShutdownThread.join();
        }

        meanResidualFile.close();
        inertiaFile.close();

        if(!params->finalTransformationFileName.empty())
        {
            std::ofstream finalTransformationFile;
            finalTransformationFile.open(params->finalTransformationFileName, std::ios::app);
            finalTransformationFile << firstIcpOdom.inverse() * lastIcpOdom << std::endl;
            finalTransformationFile.close();
        }

        PM::TransformationParameters finalMapPose = odomToMap.inverse();
        finalMapPose.topRightCorner<3, 1>() = odomToMap.inverse().topLeftCorner<3, 3>() * -lastIcpOdom.topRightCorner<3, 1>();
        if(!params->finalMapPoseFileName.empty())
        {
            std::ofstream finalMapPoseFile;
            finalMapPoseFile.open(params->finalMapPoseFileName);
            finalMapPoseFile << finalMapPose << std::endl;
            finalMapPoseFile.close();
        }

        if(!params->scanDirectory.empty())
        {
            lastScan.save(params->scanDirectory + "/registered_last_scan.vtk");
        }
    }

private:
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

            if(!params->scanDirectory.empty())
            {
                initialMap.save(params->scanDirectory + "/map.vtk");
            }
        }
    }

    void saveMap(std::string mapFileName)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s", mapFileName.c_str());
        mapper->getMap().save(mapFileName);
    }

    void saveTrajectory(std::string trajectoryFileName)
    {
        RCLCPP_INFO(this->get_logger(), "Saving trajectory to %s", trajectoryFileName.c_str());
        trajectory->save(trajectoryFileName);
    }

    void mapperShutdownLoop()
    {
        std::chrono::duration<float> idleTime = std::chrono::duration<float>::zero();

        while(rclcpp::ok())
        {
            idleTimeLock.lock();
            if(lastTimeInputWasProcessed.time_since_epoch().count())
            {
                idleTime = std::chrono::steady_clock::now() - lastTimeInputWasProcessed;
            }
            idleTimeLock.unlock();

            if(idleTime > std::chrono::duration<float>(params->maxIdleTime))
            {
                if(!params->finalMapFileName.empty())
                {
                    saveMap(params->finalMapFileName);
                }
                if(!params->finalTrajectoryFileName.empty())
                {
                    saveTrajectory(params->finalTrajectoryFileName);
                }
                RCLCPP_INFO(this->get_logger(), "Shutting down ROS");
                rclcpp::shutdown();
            }

            std::this_thread::sleep_for(std::chrono::duration<float>(0.1));
        }
    }

    norlab_icp_mapper::PM::TransformationParameters findTransform(std::string sourceFrame, std::string targetFrame, rclcpp::Time time, int transformDimension)
    {
        geometry_msgs::msg::TransformStamped tf = tfBuffer->lookupTransform(targetFrame, sourceFrame, time, std::chrono::milliseconds(100));
        return PointMatcher_ROS::rosTfToPointMatcherTransformation<T>(tf, transformDimension);
    }

    void gotInput(norlab_icp_mapper::PM::DataPoints input, rclcpp::Time timeStamp)
    {
        try
        {
            std::vector<imu_odom::msg::Inertia> cloudInertiaMeasurements;
            if(params->isOnline)
            {
                // use inertia measurement at the beginning of the point cloud or latest one if not possible
                inertiaMeasurementsMutex.lock();
                while(inertiaMeasurements.size() >= 2 && rclcpp::Time((++inertiaMeasurements.begin())->header.stamp) <= timeStamp)
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
                rclcpp::Time latestInertiaMeasurementTime = inertiaMeasurements.back().header.stamp;
                inertiaMeasurementsMutex.unlock();
                while(rclcpp::ok() && latestInertiaMeasurementTime < timeStamp + rclcpp::Duration(std::chrono::milliseconds(100)))
                {
                    this->get_clock()->sleep_for(rclcpp::Duration(std::chrono::milliseconds(10)));
                    inertiaMeasurementsMutex.lock();
                    latestInertiaMeasurementTime = inertiaMeasurements.back().header.stamp;
                    inertiaMeasurementsMutex.unlock();
                }

                inertiaMeasurementsMutex.lock();
                while(inertiaMeasurements.size() >= 2 && rclcpp::Time((++inertiaMeasurements.begin())->header.stamp) <= timeStamp)
                {
                    inertiaMeasurements.pop_front();
                }
                for(auto it = inertiaMeasurements.begin(); it != inertiaMeasurements.end(); it++)
                {
                    if(rclcpp::Time(it->header.stamp) < timeStamp + rclcpp::Duration(std::chrono::milliseconds(100)))
                    {
                        cloudInertiaMeasurements.push_back(*it);
                    }
                }
                inertiaMeasurementsMutex.unlock();
            }

            std::string linearSpeedsX = "";
            std::string linearSpeedsY = "";
            std::string linearSpeedsZ = "";
            std::string linearSpeedVariancesX = "";
            std::string linearSpeedCovariancesXY = "";
            std::string linearSpeedCovariancesXZ = "";
            std::string linearSpeedVariancesY = "";
            std::string linearSpeedCovariancesYZ = "";
            std::string linearSpeedVariancesZ = "";
            std::string linearAccelerationsX = "";
            std::string linearAccelerationsY = "";
            std::string linearAccelerationsZ = "";
            std::string angularSpeedsX = "";
            std::string angularSpeedsY = "";
            std::string angularSpeedsZ = "";
            std::string angularSpeedVariancesX = "";
            std::string angularSpeedCovariancesXY = "";
            std::string angularSpeedCovariancesXZ = "";
            std::string angularSpeedVariancesY = "";
            std::string angularSpeedCovariancesYZ = "";
            std::string angularSpeedVariancesZ = "";
            std::string angularAccelerationsX = "";
            std::string angularAccelerationsY = "";
            std::string angularAccelerationsZ = "";
            std::string measureTimes = "";
            if(cloudInertiaMeasurements.size() > 0)
            {
                for(int i = 0; i < cloudInertiaMeasurements.size(); i++)
                {
                    linearSpeedsX += std::to_string(cloudInertiaMeasurements[i].linear_velocity.x) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    linearSpeedsY += std::to_string(cloudInertiaMeasurements[i].linear_velocity.y) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    linearSpeedsZ += std::to_string(cloudInertiaMeasurements[i].linear_velocity.z) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    linearSpeedVariancesX += std::to_string(cloudInertiaMeasurements[i].linear_velocity_covariance[0]) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    linearSpeedCovariancesXY += std::to_string(cloudInertiaMeasurements[i].linear_velocity_covariance[1]) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    linearSpeedCovariancesXZ += std::to_string(cloudInertiaMeasurements[i].linear_velocity_covariance[2]) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    linearSpeedVariancesY += std::to_string(cloudInertiaMeasurements[i].linear_velocity_covariance[4]) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    linearSpeedCovariancesYZ += std::to_string(cloudInertiaMeasurements[i].linear_velocity_covariance[5]) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    linearSpeedVariancesZ += std::to_string(cloudInertiaMeasurements[i].linear_velocity_covariance[8]) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    linearAccelerationsX += std::to_string(cloudInertiaMeasurements[i].linear_acceleration.x) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    linearAccelerationsY += std::to_string(cloudInertiaMeasurements[i].linear_acceleration.y) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    linearAccelerationsZ += std::to_string(cloudInertiaMeasurements[i].linear_acceleration.z) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    angularSpeedsX += std::to_string(cloudInertiaMeasurements[i].angular_velocity.x) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    angularSpeedsY += std::to_string(cloudInertiaMeasurements[i].angular_velocity.y) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    angularSpeedsZ += std::to_string(cloudInertiaMeasurements[i].angular_velocity.z) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    angularSpeedVariancesX += std::to_string(cloudInertiaMeasurements[i].angular_velocity_covariance[0]) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    angularSpeedCovariancesXY += std::to_string(cloudInertiaMeasurements[i].angular_velocity_covariance[1]) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    angularSpeedCovariancesXZ += std::to_string(cloudInertiaMeasurements[i].angular_velocity_covariance[2]) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    angularSpeedVariancesY += std::to_string(cloudInertiaMeasurements[i].angular_velocity_covariance[4]) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    angularSpeedCovariancesYZ += std::to_string(cloudInertiaMeasurements[i].angular_velocity_covariance[5]) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    angularSpeedVariancesZ += std::to_string(cloudInertiaMeasurements[i].angular_velocity_covariance[8]) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    angularAccelerationsX += std::to_string(cloudInertiaMeasurements[i].angular_acceleration.x) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    angularAccelerationsY += std::to_string(cloudInertiaMeasurements[i].angular_acceleration.y) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    angularAccelerationsZ += std::to_string(cloudInertiaMeasurements[i].angular_acceleration.z) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                    measureTimes += std::to_string((rclcpp::Time(cloudInertiaMeasurements[i].header.stamp) - timeStamp).seconds()) + (i + 1 < cloudInertiaMeasurements.size() ? "," : "");
                }
            }
            else
            {
                linearSpeedsX = "0";
                linearSpeedsY = "0";
                linearSpeedsZ = "0";
                linearSpeedVariancesX = "1";
                linearSpeedCovariancesXY = "0";
                linearSpeedCovariancesXZ = "0";
                linearSpeedVariancesY = "1";
                linearSpeedCovariancesYZ = "0";
                linearSpeedVariancesZ = "1";
                linearAccelerationsX = "0";
                linearAccelerationsY = "0";
                linearAccelerationsZ = "0";
                angularSpeedsX = "0";
                angularSpeedsY = "0";
                angularSpeedsZ = "0";
                angularSpeedVariancesX = "1";
                angularSpeedCovariancesXY = "0";
                angularSpeedCovariancesXZ = "0";
                angularSpeedVariancesY = "1";
                angularSpeedCovariancesYZ = "0";
                angularSpeedVariancesZ = "1";
                angularAccelerationsX = "0";
                angularAccelerationsY = "0";
                angularAccelerationsZ = "0";
                measureTimes = "0";
            }

            PM::TransformationParameters sensorToOdom = findTransform(params->sensorFrame, params->odomFrame, timeStamp, input.getHomogeneousDim());
            PM::TransformationParameters sensorToMapBeforeUpdate = odomToMap * sensorToOdom;

            if(!params->scanDirectory.empty() && nbRegistrations == 0)
            {
                PM::DataPoints priorFirstScan = removeNanFilter->filter(transformation->compute(input, sensorToMapBeforeUpdate));
                priorFirstScan.save(params->scanDirectory + "/prior_first_scan.vtk");
            }

            PM::TransformationParameters sensorToMapAfterUpdate;
            if(params->computeResidual && cloudInertiaMeasurements.size() > 0 && nbRegistrations > 0)
            {
                PM::DataPoints map = mapper->getMap();

                mapper->processInput(input, sensorToMapBeforeUpdate,
                                     std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.nanoseconds())),
                                     linearSpeedsX, linearSpeedsY, linearSpeedsZ, linearSpeedVariancesX, linearSpeedCovariancesXY, linearSpeedCovariancesXZ,
                                     linearSpeedVariancesY, linearSpeedCovariancesYZ, linearSpeedVariancesZ, linearAccelerationsX, linearAccelerationsY, linearAccelerationsZ,
                                     angularSpeedsX, angularSpeedsY, angularSpeedsZ, angularSpeedVariancesX, angularSpeedCovariancesXY, angularSpeedCovariancesXZ,
                                     angularSpeedVariancesY, angularSpeedCovariancesYZ, angularSpeedVariancesZ, angularAccelerationsX, angularAccelerationsY, angularAccelerationsZ,
                                     measureTimes);
                sensorToMapAfterUpdate = mapper->getSensorPose();

                PM::DataPoints inputInMapFrame = transformation->compute(input, sensorToMapAfterUpdate);

                PM::Parameters matcherParams;
                matcherParams["knn"] = "1";
                matcherParams["maxDist"] = "1";
                matcherParams["epsilon"] = "0";
                std::shared_ptr<PM::Matcher> matcher = PM::get().MatcherRegistrar.create("KDTreeMatcher", matcherParams);
                matcher->init(map);
                PM::Matches matches(matcher->findClosests(inputInMapFrame));

                if(params->perpendicularResidual)
                {
                    int i = 0;
                    std::set<int> mapPointIds;
                    for(int j = 0; j < matches.ids.cols(); j++)
                    {
                        int readingId = j;
                        int referenceId = matches.ids(0, j);
                        Eigen::Vector3f normal = map.getDescriptorViewByName("normals").col(referenceId);
                        if(std::acos(std::abs(normal(1) / normal.norm())) <= 0.5)
                        {
                            inputInMapFrame.setColFrom(i, inputInMapFrame, readingId);
                            i++;
                            mapPointIds.insert(referenceId);
                        }
                    }
                    inputInMapFrame.conservativeResize(i);
                    i = 0;
                    for(auto it = mapPointIds.begin(); it != mapPointIds.end(); it++)
                    {
                        map.setColFrom(i, map, *it);
                        i++;
                    }
                    map.conservativeResize(i);
                    matcher->init(map);
                    matches = matcher->findClosests(inputInMapFrame);
                }

                double error = 0;
                for(int i = 0; i < matches.ids.cols(); i++)
                {
                    Eigen::Vector3f inputPoint = inputInMapFrame.features.col(i).head(3);
                    Eigen::Vector3f mapPoint = map.features.col(matches.ids(0, i)).head(3);
                    Eigen::Vector3f normal = map.getDescriptorViewByName("normals").col(matches.ids(0, i));
                    normal /= normal.norm();
                    if(params->pointToPlaneResidual)
                    {
                        error += std::abs((mapPoint - inputPoint).dot(normal));
                    }
                    else
                    {
                        error += (mapPoint - inputPoint).norm();
                    }
                }
                float meanResidual = error / matches.ids.cols();

                std_msgs::msg::Float32 residualMsg;
                residualMsg.data = meanResidual;
                residualPublisher->publish(residualMsg);

                imu_odom::msg::Inertia currentInertia;
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

                float x = sensorToMapAfterUpdate(0, 3);
                float y = sensorToMapAfterUpdate(1, 3);
                float z = sensorToMapAfterUpdate(2, 3);
                float linearSpeed = std::sqrt(
                        std::pow(currentInertia.linear_velocity.x, 2) + std::pow(currentInertia.linear_velocity.y, 2) + std::pow(currentInertia.linear_velocity.z, 2));
                float linearAcceleration = std::sqrt(std::pow(currentInertia.linear_acceleration.x, 2) + std::pow(currentInertia.linear_acceleration.y, 2) +
                                                     std::pow(currentInertia.linear_acceleration.z, 2));
                float angularSpeed = std::sqrt(
                        std::pow(currentInertia.angular_velocity.x, 2) + std::pow(currentInertia.angular_velocity.y, 2) + std::pow(currentInertia.angular_velocity.z, 2));
                float angularAcceleration = std::sqrt(std::pow(currentInertia.angular_acceleration.x, 2) + std::pow(currentInertia.angular_acceleration.y, 2) +
                                                      std::pow(currentInertia.angular_acceleration.z, 2));

                meanResidualFile << std::setprecision(20) << timeStamp.seconds() << "," << x << "," << y << "," << z << "," << linearSpeed << "," << linearAcceleration << ","
                                 << angularSpeed << "," << angularAcceleration << "," << meanResidual << std::endl;
            }
            else
            {
                mapper->processInput(input, sensorToMapBeforeUpdate,
                                     std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.nanoseconds())),
                                     linearSpeedsX, linearSpeedsY, linearSpeedsZ, linearSpeedVariancesX, linearSpeedCovariancesXY, linearSpeedCovariancesXZ,
                                     linearSpeedVariancesY, linearSpeedCovariancesYZ, linearSpeedVariancesZ, linearAccelerationsX, linearAccelerationsY, linearAccelerationsZ,
                                     angularSpeedsX, angularSpeedsY, angularSpeedsZ, angularSpeedVariancesX, angularSpeedCovariancesXY, angularSpeedCovariancesXZ,
                                     angularSpeedVariancesY, angularSpeedCovariancesYZ, angularSpeedVariancesZ, angularAccelerationsX, angularAccelerationsY, angularAccelerationsZ,
                                     measureTimes);
                sensorToMapAfterUpdate = mapper->getSensorPose();
            }

            if(!params->scanDirectory.empty())
            {
                if(nbRegistrations == 0)
                {
                    PM::DataPoints registeredFirstScan = transformation->compute(input, sensorToMapAfterUpdate);
                    registeredFirstScan.save(params->scanDirectory + "/registered_first_scan.vtk");
                }
                lastScan = transformation->compute(input, sensorToMapAfterUpdate);
            }

            sensor_msgs::msg::PointCloud2 filteredCloud = PointMatcher_ROS::pointMatcherCloudToRosMsg<T>(input, params->sensorFrame, timeStamp);
            filteredCloudPublisher->publish(filteredCloud);

            mapTfLock.lock();
            odomToMap = transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());
            mapTfLock.unlock();

            PM::TransformationParameters robotToSensor = findTransform(params->robotFrame, params->sensorFrame, timeStamp, input.getHomogeneousDim());
            PM::TransformationParameters robotToMap = sensorToMapAfterUpdate * robotToSensor;

            if((++nbRegistrations) == 6)
            {
                firstIcpOdom = robotToMap;
            }
            lastIcpOdom = robotToMap;

            trajectory->addPoint(robotToMap.topRightCorner(input.getEuclideanDim(), 1));
            nav_msgs::msg::Odometry odomMsgOut = PointMatcher_ROS::pointMatcherTransformationToOdomMsg<T>(robotToMap, "map", params->robotFrame, timeStamp);
            odomPublisher->publish(odomMsgOut);

            idleTimeLock.lock();
            lastTimeInputWasProcessed = std::chrono::steady_clock::now();
            idleTimeLock.unlock();
        }
        catch(tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }
    }

    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2& cloudMsgIn)
    {
        gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<T>(cloudMsgIn), cloudMsgIn.header.stamp);
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan& scanMsgIn)
    {
        gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<T>(scanMsgIn), scanMsgIn.header.stamp);
    }

    bool reloadYamlConfigCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        mapper->loadYamlConfig();
        return true;
    }

    bool saveMapCallback(const std::shared_ptr<map_msgs::srv::SaveMap::Request> req, std::shared_ptr<map_msgs::srv::SaveMap::Response> res)
    {
        try
        {
            saveMap(req->filename.data);
            return true;
        }
        catch(const std::runtime_error& e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to save: " << e.what());
            return false;
        }
    }

    bool saveTrajectoryCallback(const std::shared_ptr<map_msgs::srv::SaveMap::Request> req, std::shared_ptr<map_msgs::srv::SaveMap::Response> res)
    {
        try
        {
            saveTrajectory(req->filename.data);
            return true;
        }
        catch(const std::runtime_error& e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to save: " << e.what());
            return false;
        }
    }

    void mapPublisherLoop()
    {
        rclcpp::Rate publishRate(params->mapPublishRate);

        PM::DataPoints newMap;
        while(rclcpp::ok())
        {
            if(mapper->getNewMap(newMap))
            {
                sensor_msgs::msg::PointCloud2 mapMsgOut = PointMatcher_ROS::pointMatcherCloudToRosMsg<T>(newMap, "map", this->get_clock()->now());
                mapPublisher->publish(mapMsgOut);

                if(newMap.descriptorExists("covXScale") && newMap.descriptorExists("covYScale") && newMap.descriptorExists("covZScale")
                   && newMap.descriptorExists("covX") && newMap.descriptorExists("covY") && newMap.descriptorExists("covZ"))
                {
                    const auto& covX = newMap.getDescriptorViewByName("covX");
                    const auto& covY = newMap.getDescriptorViewByName("covY");
                    const auto& covZ = newMap.getDescriptorViewByName("covZ");
                    const auto& covXScale = newMap.getDescriptorViewByName("covXScale").unaryExpr([](float element)
                                                                                                  { return element < 1e-6 ? 1e-6 : element; });
                    const auto& covYScale = newMap.getDescriptorViewByName("covYScale").unaryExpr([](float element)
                                                                                                  { return element < 1e-6 ? 1e-6 : element; });
                    const auto& covZScale = newMap.getDescriptorViewByName("covZScale").unaryExpr([](float element)
                                                                                                  { return element < 1e-6 ? 1e-6 : element; });

                    visualization_msgs::msg::MarkerArray markerArray;
                    for(int i = 0; i < newMap.getNbPoints(); ++i)
                    {
                        Eigen::Matrix3f R = Eigen::Matrix3f::Zero();
                        R.col(0) = covX.col(i);
                        R.col(1) = covY.col(i);
                        R.col(2) = covZ.col(i);

                        if(R.determinant() < 0)
                        {
                            R.col(0) *= -1;
                        }

                        Eigen::Quaternionf q(R);
                        q.normalize();

                        visualization_msgs::msg::Marker marker;
                        marker.header.frame_id = "map";
                        marker.header.stamp = this->get_clock()->now();
                        marker.id = i;
                        marker.type = visualization_msgs::msg::Marker::SPHERE;
                        marker.action = visualization_msgs::msg::Marker::ADD;
                        marker.pose.position.x = newMap.features(0, i);
                        marker.pose.position.y = newMap.features(1, i);
                        marker.pose.position.z = newMap.features(2, i);
                        marker.pose.orientation.x = q.x();
                        marker.pose.orientation.y = q.y();
                        marker.pose.orientation.z = q.z();
                        marker.pose.orientation.w = q.w();
                        marker.scale.x = 2 * 2 * covXScale(0, i); // 2 sigmas
                        marker.scale.y = 2 * 2 * covYScale(0, i);
                        marker.scale.z = 2 * 2 * covZScale(0, i);
                        marker.color.r = 1.0f;
                        marker.color.g = 1.0f;
                        marker.color.b = 1.0f;
                        marker.color.a = 0.5f;

                        markerArray.markers.emplace_back(marker);
                    }

                    size_t currentMarkersSize = markerArray.markers.size();
                    for(size_t i = currentMarkersSize; i < lastMarkersSize; ++i)
                    {
                        visualization_msgs::msg::Marker marker;
                        marker.header.frame_id = "map";
                        marker.id = i;
                        marker.action = visualization_msgs::msg::Marker::DELETE;
                        markerArray.markers.emplace_back(marker);
                    }
                    lastMarkersSize = currentMarkersSize;

                    markerPublisher->publish(markerArray);
                }
            }
            publishRate.sleep();
        }
    }

    void mapTfPublisherLoop()
    {
        rclcpp::Rate publishRate(params->mapTfPublishRate);

        while(rclcpp::ok())
        {
            mapTfLock.lock();
            PM::TransformationParameters currentOdomToMap = odomToMap;
            mapTfLock.unlock();

            geometry_msgs::msg::TransformStamped currentOdomToMapTf = PointMatcher_ROS::pointMatcherTransformationToRosTf<T>(currentOdomToMap, "map", params->odomFrame,
                                                                                                                             this->get_clock()->now());
            tfBroadcaster->sendTransform(currentOdomToMapTf);

            publishRate.sleep();
        }
    }

    void inertiaCallback(const imu_odom::msg::Inertia& msg)
    {
        inertiaMeasurementsMutex.lock();
        inertiaMeasurements.emplace_back(msg);
        inertiaMeasurementsMutex.unlock();
        if(params->recordInertia)
        {
            inertiaFile << std::setprecision(20) << rclcpp::Time(msg.header.stamp).seconds() << "," << msg.linear_velocity.x << "," << msg.linear_velocity.y << ","
                        << msg.linear_velocity.z << "," << msg.linear_acceleration.x << "," << msg.linear_acceleration.y << "," << msg.linear_acceleration.z << ","
                        << msg.angular_velocity.x << "," << msg.angular_velocity.y << "," << msg.angular_velocity.z << "," << msg.angular_acceleration.x << ","
                        << msg.angular_acceleration.y << "," << msg.angular_acceleration.z << std::endl;
        }
    }

    typedef norlab_icp_mapper::PM PM;
    typedef norlab_icp_mapper::T T;
    std::unique_ptr<NodeParameters> params;
    std::shared_ptr<PM::Transformation> transformation;
    std::unique_ptr<norlab_icp_mapper::Mapper> mapper;
    std::unique_ptr<Trajectory> trajectory;
    PM::TransformationParameters odomToMap;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filteredCloudPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr residualPublisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud2Subscription;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScanSubscription;
    rclcpp::Subscription<imu_odom::msg::Inertia>::SharedPtr inertiaSubscription;
    rclcpp::CallbackGroup::SharedPtr pointCloudCallbackGroup;
    rclcpp::CallbackGroup::SharedPtr inertiaCallbackGroup;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reloadYamlConfigService;
    rclcpp::Service<map_msgs::srv::SaveMap>::SharedPtr saveMapService;
    rclcpp::Service<map_msgs::srv::SaveMap>::SharedPtr saveTrajectoryService;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    std::mutex mapTfLock;
    std::chrono::time_point<std::chrono::steady_clock> lastTimeInputWasProcessed;
    std::mutex idleTimeLock;
    std::ofstream meanResidualFile;
    std::ofstream inertiaFile;
    std::mutex inertiaMeasurementsMutex;
    std::list<imu_odom::msg::Inertia> inertiaMeasurements;
    PM::ICP icp;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher;
    size_t lastMarkersSize;
    std::shared_ptr<PM::DataPointsFilter> removeNanFilter;
    PM::DataPoints lastScan;

    int nbRegistrations = 0;
    PM::TransformationParameters firstIcpOdom;
    PM::TransformationParameters lastIcpOdom;
    std::thread mapPublisherThread;
    std::thread mapTfPublisherThread;
    std::thread mapperShutdownThread;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<MapperNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
