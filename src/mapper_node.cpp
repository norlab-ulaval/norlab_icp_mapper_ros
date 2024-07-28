#include "NodeParameters.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <norlab_icp_mapper/ImuMeasurement.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <norlab_icp_mapper/Trajectory.h>
#include <norlab_icp_mapper_ros/srv/save_map.hpp>
#include <norlab_icp_mapper_ros/srv/load_map.hpp>
#include <norlab_icp_mapper_ros/srv/save_trajectory.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/empty.hpp>
#include <memory>
#include <mutex>
#include <thread>
#include <fstream>

class MapperNode : public rclcpp::Node
{
public:
    MapperNode():
            Node("mapper_node")
    {
        params = std::unique_ptr<NodeParameters>(new NodeParameters(*this));

        transformation = PM::get().TransformationRegistrar.create("RigidTransformation");

        mapper = std::unique_ptr<norlab_icp_mapper::Mapper>(new norlab_icp_mapper::Mapper(params->inputFiltersConfig, params->icpConfig,
                                                                                          params->mapPostFiltersConfig, params->mapUpdateCondition,
                                                                                          params->mapUpdateOverlap, params->mapUpdateDelay,
                                                                                          params->mapUpdateDistance, params->minDistNewPoint,
                                                                                          params->sensorMaxRange, params->priorDynamic, params->thresholdDynamic,
                                                                                          params->beamHalfAngle, params->epsilonA, params->epsilonD, params->alpha,
                                                                                          params->beta, params->is3D, params->computeProbDynamic,
                                                                                          params->isMapping, params->saveMapCellsOnHardDrive, params->imuToLidar));

        if(!params->initialMapFileName.empty())
        {
            loadMap(params->initialMapFileName);
        }
        if(!params->initialRobotPoseString.empty())
        {
            setRobotPose(params->initialRobotPose);
        }

        mapperShutdownThread = std::thread(&MapperNode::mapperShutdownLoop, this);

        tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(this->get_clock(), std::chrono::seconds(1000000)));
        tfListener = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(*tfBuffer));
        tfBroadcaster = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster(*this));

        mapPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 2);
        odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("icp_odom", 50);

        initialRobotPoseIsSet.store(false);
        sensorVelocity = Eigen::Matrix<float, 3, 1>::Zero();

        imuCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions imuSubscriptionOptions;
        imuSubscriptionOptions.callback_group = imuCallbackGroup;
        imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>("imu_in", 0, std::bind(&MapperNode::imuCallback, this, std::placeholders::_1), imuSubscriptionOptions);

        pointCloudCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions pointCloudSubscriptionOptions;
        pointCloudSubscriptionOptions.callback_group = pointCloudCallbackGroup;
        if(params->is3D)
        {
            robotToMap = PM::Matrix::Identity(4, 4);
            pointCloud2Subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>("points_in", 0,
                                                                                               std::bind(&MapperNode::pointCloud2Callback, this, std::placeholders::_1),
                                                                                               pointCloudSubscriptionOptions);
        }
        else
        {
            robotToMap = PM::Matrix::Identity(3, 3);
            laserScanSubscription = this->create_subscription<sensor_msgs::msg::LaserScan>("points_in", 0,
                                                                                           std::bind(&MapperNode::laserScanCallback, this, std::placeholders::_1),
                                                                                           pointCloudSubscriptionOptions);
        }

        reloadYamlConfigService = this->create_service<std_srvs::srv::Empty>("reload_yaml_config",
                                                                             std::bind(&MapperNode::reloadYamlConfigCallback, this, std::placeholders::_1,
                                                                                       std::placeholders::_2));
        saveMapService = this->create_service<norlab_icp_mapper_ros::srv::SaveMap>("save_map",
                                                                                   std::bind(&MapperNode::saveMapCallback, this, std::placeholders::_1,
                                                                                             std::placeholders::_2));
        loadMapService = this->create_service<norlab_icp_mapper_ros::srv::LoadMap>("load_map",
                                                                                   std::bind(&MapperNode::loadMapCallback, this, std::placeholders::_1,
                                                                                             std::placeholders::_2));
        saveTrajectoryService = this->create_service<norlab_icp_mapper_ros::srv::SaveTrajectory>("save_trajectory",
                                                                                                 std::bind(&MapperNode::saveTrajectoryCallback, this,
                                                                                                           std::placeholders::_1, std::placeholders::_2));
        enableMappingService = this->create_service<std_srvs::srv::Empty>("enable_mapping",
                                                                          std::bind(&MapperNode::enableMappingCallback, this, std::placeholders::_1,
                                                                                    std::placeholders::_2));
        disableMappingService = this->create_service<std_srvs::srv::Empty>("disable_mapping",
                                                                           std::bind(&MapperNode::disableMappingCallback, this, std::placeholders::_1,
                                                                                     std::placeholders::_2));

        mapPublisherThread = std::thread(&MapperNode::mapPublisherLoop, this);
        if(params->publishTfsBetweenRegistrations)
        {
            mapTfPublisherThread = std::thread(&MapperNode::mapTfPublisherLoop, this);
        }
    }

private:
    typedef PointMatcher<float> PM;

    std::unique_ptr<NodeParameters> params;
    std::shared_ptr<PM::Transformation> transformation;
    std::unique_ptr<norlab_icp_mapper::Mapper> mapper;
    PM::TransformationParameters robotPoseToSet;
    bool hasToSetRobotPose;
    std::thread mapperShutdownThread;
    std::mutex idleTimeLock;
    std::chrono::time_point<std::chrono::steady_clock> lastTimeInputWasProcessed;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    std::vector<StampedState> imuTrajectory;
    std::mutex mapTfLock;
    PM::TransformationParameters robotToMap;
    std::atomic_bool initialRobotPoseIsSet;
    Eigen::Matrix<float, 3, 1> sensorVelocity;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    sensor_msgs::msg::PointCloud2 previousPointCloud2;
    sensor_msgs::msg::LaserScan previousLaserScan;
    rclcpp::CallbackGroup::SharedPtr imuCallbackGroup;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;
    rclcpp::CallbackGroup::SharedPtr pointCloudCallbackGroup;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud2Subscription;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScanSubscription;
    std::mutex imuMeasurementsLock;
    std::list<sensor_msgs::msg::Imu> imuMeasurements;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reloadYamlConfigService;
    rclcpp::Service<norlab_icp_mapper_ros::srv::SaveMap>::SharedPtr saveMapService;
    rclcpp::Service<norlab_icp_mapper_ros::srv::LoadMap>::SharedPtr loadMapService;
    rclcpp::Service<norlab_icp_mapper_ros::srv::SaveTrajectory>::SharedPtr saveTrajectoryService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enableMappingService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disableMappingService;
    std::thread mapPublisherThread;
    std::thread mapTfPublisherThread;

    std::string appendToFilePath(const std::string& filePath, const std::string& suffix)
    {
        std::string::size_type const extensionPosition(filePath.find_last_of('.'));
        std::string mapPathWithoutExtension = filePath.substr(0, extensionPosition);
        std::string extension = filePath.substr(extensionPosition, filePath.length() - 1);

        return mapPathWithoutExtension + suffix + extension;
    }

    void saveMap(const std::string& mapFileName)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s", mapFileName.c_str());
        mapper->getMap().save(mapFileName);
    }

    void loadMap(const std::string& mapFileName)
    {
        RCLCPP_INFO(this->get_logger(), "Loading map from %s", mapFileName.c_str());
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

    void appendToImuTrajectory(const std::vector<StampedState>& intraScanLidarTrajectory, const PM::TransformationParameters& imuToLidar,
                               const std::vector<ImuMeasurement>& intraScanImuMeasurements)
    {
        for(unsigned int i = 0; i < intraScanLidarTrajectory.size(); ++i)
        {
            Eigen::Matrix<float, 4, 4> currentLidarPose = intraScanLidarTrajectory[i].pose;
            Eigen::Matrix<float, 3, 1> currentLidarLinearVelocity = intraScanLidarTrajectory[i].velocity;
            Eigen::Matrix<float, 4, 4> currentImuPose = currentLidarPose * imuToLidar;
            ImuMeasurement currentImuMeasurement;
            if(i < intraScanLidarTrajectory.size() - 1)
            {
                currentImuMeasurement = intraScanImuMeasurements[i];
            }
            else
            {
                currentImuMeasurement = intraScanImuMeasurements[intraScanImuMeasurements.size() - 1];
            }
            Eigen::Matrix<float, 3, 1> currentAngularVelocity = currentImuPose.topLeftCorner<3, 3>() * currentImuMeasurement.angularVelocity;
            Eigen::Matrix<float, 3, 1> currentImuLeverArm = currentImuPose.topRightCorner<3, 1>() - currentLidarPose.topRightCorner<3, 1>();
            Eigen::Matrix<float, 3, 1> currentImuLinearVelocity = currentLidarLinearVelocity + currentAngularVelocity.cross(currentImuLeverArm);

            imuTrajectory.push_back({intraScanLidarTrajectory[i].timeStamp, currentImuPose, currentImuLinearVelocity});
        }
    }

    void saveTrajectory(const std::string& trajectoryFileName)
    {
        RCLCPP_INFO(this->get_logger(), "Saving trajectory to %s", trajectoryFileName.c_str());
        std::ofstream trajectoryFile(trajectoryFileName);
        trajectoryFile << "timestamp,t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33,v0,v1,v2" << std::endl;
        for(const StampedState& state: imuTrajectory)
        {
            trajectoryFile << state.timeStamp.time_since_epoch().count() << "," << state.pose(0, 0) << "," << state.pose(0, 1) << "," << state.pose(0, 2) << "," << state.pose(0, 3)
                           << "," << state.pose(1, 0) << "," << state.pose(1, 1) << "," << state.pose(1, 2) << "," << state.pose(1, 3)
                           << "," << state.pose(2, 0) << "," << state.pose(2, 1) << "," << state.pose(2, 2) << "," << state.pose(2, 3)
                           << "," << state.pose(3, 0) << "," << state.pose(3, 1) << "," << state.pose(3, 2) << "," << state.pose(3, 3)
                           << "," << state.velocity(0) << "," << state.velocity(1) << "," << state.velocity(2) << std::endl;
        }
        trajectoryFile.close();
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

    Eigen::Vector3f findOrthogonalVector(const Eigen::Vector3f& vector)
    {
        return Eigen::Vector3f(vector(1) + vector(2), vector(2) - vector(0), -vector(0) - vector(1));
    }

    PM::TransformationParameters findTransform(const std::string& sourceFrame, const std::string& targetFrame, const rclcpp::Time& time, const int& transformDimension)
    {
        geometry_msgs::msg::TransformStamped tf = tfBuffer->lookupTransform(targetFrame, sourceFrame, time, std::chrono::milliseconds(100));
        return PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(tf, transformDimension);
    }

    void imuCallback(const sensor_msgs::msg::Imu& msg)
    {
        try
        {
            if(!initialRobotPoseIsSet.load())
            {
                Eigen::Vector3f linearAcceleration(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
                Eigen::Matrix3f mapToImuOrientation;
                mapToImuOrientation.col(2) = linearAcceleration.normalized();
                mapToImuOrientation.col(1) = findOrthogonalVector(mapToImuOrientation.col(2)).normalized();
                mapToImuOrientation.col(0) = mapToImuOrientation.col(1).cross(mapToImuOrientation.col(2)).normalized();
                PM::TransformationParameters imuToRobot = findTransform(msg.header.frame_id, params->robotFrame, msg.header.stamp, 4);
                mapTfLock.lock();
                robotToMap = Eigen::Matrix4f::Identity();
                robotToMap.topLeftCorner<3, 3>() = (imuToRobot.topLeftCorner<3, 3>() * mapToImuOrientation).inverse();
                mapTfLock.unlock();
                initialRobotPoseIsSet.store(true);
            }

            imuMeasurementsLock.lock();
            imuMeasurements.emplace_back(msg);
            imuMeasurementsLock.unlock();
        }
        catch(const tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
    }

    void gotInput(const PM::DataPoints& input, const std::string& sensorFrame, const rclcpp::Time& timeStampAtStartOfScan, const rclcpp::Time& timeStampAtEndOfScan)
    {
        while(!initialRobotPoseIsSet.load())
        {
            std::this_thread::sleep_for(std::chrono::duration<float>(0.1));
        }

        try
        {
            imuMeasurementsLock.lock();
            rclcpp::Time latestImuMeasurementTime = imuMeasurements.back().header.stamp;
            imuMeasurementsLock.unlock();
            while(rclcpp::ok() && latestImuMeasurementTime < timeStampAtEndOfScan)
            {
                this->get_clock()->sleep_for(rclcpp::Duration(std::chrono::milliseconds(10)));
                imuMeasurementsLock.lock();
                latestImuMeasurementTime = imuMeasurements.back().header.stamp;
                imuMeasurementsLock.unlock();
            }

            std::vector<ImuMeasurement> cloudImuMeasurements; // contains the IMU measurements ranging from just before this cloud to just before the next
            imuMeasurementsLock.lock();
            while(imuMeasurements.size() >= 2 && rclcpp::Time((++imuMeasurements.begin())->header.stamp) <= timeStampAtStartOfScan)
            {
                imuMeasurements.pop_front();
            }
            for(auto it = imuMeasurements.begin(); it != imuMeasurements.end(); it++)
            {
                if(rclcpp::Time(it->header.stamp) < timeStampAtEndOfScan)
                {
                    cloudImuMeasurements.push_back({std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(rclcpp::Time(it->header.stamp).nanoseconds())),
                                                    Eigen::Matrix<float, 3, 1>(it->angular_velocity.x, it->angular_velocity.y, it->angular_velocity.z),
                                                    Eigen::Matrix<float, 3, 1>(it->linear_acceleration.x, it->linear_acceleration.y, it->linear_acceleration.z)});
                }
            }
            imuMeasurementsLock.unlock();

            if(cloudImuMeasurements.size() == 0)
            {
                return;
            }

            PM::TransformationParameters robotToMapAtStartOfScan = robotToMap;
            PM::TransformationParameters sensorToRobot = findTransform(sensorFrame, params->robotFrame, timeStampAtStartOfScan, input.getHomogeneousDim());
            PM::TransformationParameters sensorToMapAtStartOfScan = robotToMapAtStartOfScan * sensorToRobot;
            if(hasToSetRobotPose)
            {
                sensorToMapAtStartOfScan = robotPoseToSet * sensorToRobot;
                hasToSetRobotPose = false;
            }

            try
            {
                mapper->processInput(input, sensorToMapAtStartOfScan, sensorVelocity, cloudImuMeasurements,
                                     std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStampAtStartOfScan.nanoseconds())),
                                     std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStampAtEndOfScan.nanoseconds())));
            }
            catch(const PM::ConvergenceError& convergenceError)
            {
                RCLCPP_ERROR(this->get_logger(), "Unable to process input: %s", convergenceError.what());
                try
                {
                    if(!params->finalTrajectoryFileName.empty())
                    {
                        saveTrajectory(appendToFilePath(params->finalTrajectoryFileName, "_convergence_error"));
                    }
                    if(!params->finalMapFileName.empty())
                    {
                        saveMap(appendToFilePath(params->finalMapFileName, "_convergence_error"));
                    }
                }
                catch(const std::runtime_error& runtimeError)
                {
                    RCLCPP_ERROR(this->get_logger(), "Unable to save: %s", runtimeError.what());
                }
                throw;
            }
            const PM::TransformationParameters& sensorToMapAtEndOfScan = mapper->getPose();
            sensorVelocity = mapper->getVelocity();
            std::vector<StampedState> intraScanLidarTrajectory = mapper->getIntraScanTrajectory();

            PM::TransformationParameters robotToMapAtEndOfScan = transformation->correctParameters(sensorToMapAtEndOfScan * sensorToRobot.inverse());
            mapTfLock.lock();
            robotToMap = robotToMapAtEndOfScan;
            mapTfLock.unlock();

            appendToImuTrajectory(intraScanLidarTrajectory, params->imuToLidar, cloudImuMeasurements);

            nav_msgs::msg::Odometry odomMsgOut = PointMatcher_ROS::pointMatcherTransformationToOdomMsg<float>(robotToMapAtEndOfScan, "map", params->robotFrame,
                                                                                                              timeStampAtEndOfScan);
            Eigen::Vector3f linearDisplacement = robotToMapAtEndOfScan.topRightCorner(input.getEuclideanDim(), 1) -
                                                 robotToMapAtStartOfScan.topRightCorner(input.getEuclideanDim(), 1);
            float deltaTime = (float)(timeStampAtEndOfScan - timeStampAtStartOfScan).seconds();
            Eigen::Vector3f linearVelocity = linearDisplacement / deltaTime;
            odomMsgOut.twist.twist.linear.x = linearVelocity(0);
            odomMsgOut.twist.twist.linear.y = linearVelocity(1);
            odomMsgOut.twist.twist.linear.z = linearVelocity(2);
            odomPublisher->publish(odomMsgOut);

            if(!params->publishTfsBetweenRegistrations)
            {
                geometry_msgs::msg::TransformStamped currentRobotToMapTf = PointMatcher_ROS::pointMatcherTransformationToRosTf<float>(robotToMapAtEndOfScan, "map",
                                                                                                                                      params->robotFrame,
                                                                                                                                      timeStampAtEndOfScan);
                tfBroadcaster->sendTransform(currentRobotToMapTf);
            }

            idleTimeLock.lock();
            lastTimeInputWasProcessed = std::chrono::steady_clock::now();
            idleTimeLock.unlock();
        }
        catch(const tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
    }

    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2& cloudMsgIn)
    {
        if(previousPointCloud2.header.stamp.sec != 0 || previousPointCloud2.header.stamp.nanosec != 0)
        {
            gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(previousPointCloud2), previousPointCloud2.header.frame_id, previousPointCloud2.header.stamp,
                     cloudMsgIn.header.stamp);
        }
        previousPointCloud2 = cloudMsgIn;
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan& scanMsgIn)
    {
        if(previousLaserScan.header.stamp.sec != 0 || previousLaserScan.header.stamp.nanosec != 0)
        {
            gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(previousLaserScan), previousLaserScan.header.frame_id, previousLaserScan.header.stamp,
                     scanMsgIn.header.stamp);
        }
        previousLaserScan = scanMsgIn;
    }

    void mapPublisherLoop()
    {
        rclcpp::Rate publishRate(params->mapPublishRate);

        PM::DataPoints newMap;
        while(rclcpp::ok())
        {
            if(mapper->getNewLocalMap(newMap))
            {
                sensor_msgs::msg::PointCloud2 mapMsgOut = PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(newMap, "map", this->get_clock()->now());
                mapPublisher->publish(mapMsgOut);
            }

            publishRate.sleep();
        }
    }

    void mapTfPublisherLoop()
    {
        rclcpp::Rate publishRate(params->mapTfPublishRate);

        auto lastTime = this->get_clock()->now();

        while(rclcpp::ok())
        {
            mapTfLock.lock();
            PM::TransformationParameters currentRobotToMap = robotToMap;
            mapTfLock.unlock();

            auto currTime = this->get_clock()->now();

            geometry_msgs::msg::TransformStamped currentRobotToMapTf = PointMatcher_ROS::pointMatcherTransformationToRosTf<float>(currentRobotToMap, "map",
                                                                                                                                  params->robotFrame,
                                                                                                                                  currTime);
            if(lastTime != currTime)
            {
                tfBroadcaster->sendTransform(currentRobotToMapTf);
            }

            lastTime = currTime;
            publishRate.sleep();
        }
    }

    void reloadYamlConfigCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "Reloading YAML config");
        mapper->loadYamlConfig(params->inputFiltersConfig, params->icpConfig, params->mapPostFiltersConfig);
    }

    void saveMapCallback(const std::shared_ptr<norlab_icp_mapper_ros::srv::SaveMap::Request> req, std::shared_ptr<norlab_icp_mapper_ros::srv::SaveMap::Response> res)
    {
        try
        {
            saveMap(req->map_file_name.data);
        }
        catch(const std::runtime_error& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to save: %s", e.what());
        }
    }

    void loadMapCallback(const std::shared_ptr<norlab_icp_mapper_ros::srv::LoadMap::Request> req, std::shared_ptr<norlab_icp_mapper_ros::srv::LoadMap::Response> res)
    {
        try
        {
            loadMap(req->map_file_name.data);
            int homogeneousDim = params->is3D ? 4 : 3;
            setRobotPose(PointMatcher_ROS::rosMsgToPointMatcherTransformation<float>(req->pose, homogeneousDim));
            imuTrajectory.clear();
        }
        catch(const std::runtime_error& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to load: %s", e.what());
        }
    }

    void saveTrajectoryCallback(const std::shared_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory::Request> req,
                                std::shared_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory::Response> res)
    {
        try
        {
            saveTrajectory(req->trajectory_file_name.data);
        }
        catch(const std::runtime_error& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to save: %s", e.what());
        }
    }

    void enableMappingCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "Enabling mapping");
        mapper->setIsMapping(true);
    }

    void disableMappingCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "Disabling mapping");
        mapper->setIsMapping(false);
    }
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
