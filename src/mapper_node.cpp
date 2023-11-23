#include "NodeParameters.h"
#include <rclcpp/rclcpp.hpp>
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

class MapperNode : public rclcpp::Node
{
public:
    MapperNode() :
            Node("mapper_node")
    {
        params = std::unique_ptr<NodeParameters>(new NodeParameters(*this));

        transformation = PM::get().TransformationRegistrar.create("RigidTransformation");

        mapper = std::unique_ptr<norlab_icp_mapper::Mapper>(new norlab_icp_mapper::Mapper(params->configFilePath,
                                                                                          params->sensorMaxRange, params->priorDynamic, params->thresholdDynamic,
                                                                                          params->beamHalfAngle, params->epsilonA, params->epsilonD, params->alpha,
                                                                                          params->beta, params->is3D, params->isOnline, params->computeProbDynamic,
                                                                                          params->isMapping, params->saveMapCellsOnHardDrive));

        if(!params->initialMapFileName.empty())
        {
            loadMap(params->initialMapFileName);
        }
        if(!params->initialRobotPoseString.empty())
        {
            setRobotPose(params->initialRobotPose);
        }

        int messageQueueSize;
        if(params->isOnline)
        {
            tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(this->get_clock()));
            messageQueueSize = 1;
        }
        else
        {
            mapperShutdownThread = std::thread(&MapperNode::mapperShutdownLoop, this);
            tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(this->get_clock(), std::chrono::seconds(1000000)));
            messageQueueSize = 0;
        }

        tfListener = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(*tfBuffer));
        tfBroadcaster = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster(*this));

        mapPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 2);
        odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("icp_odom", 50);

        if(params->is3D)
        {
            robotTrajectory = std::unique_ptr<Trajectory>(new Trajectory(3));
            odomToMap = PM::Matrix::Identity(4, 4);
            pointCloud2Subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>("points_in", messageQueueSize,
                                                                                               std::bind(&MapperNode::pointCloud2Callback, this,
                                                                                                         std::placeholders::_1));
        }
        else
        {
            robotTrajectory = std::unique_ptr<Trajectory>(new Trajectory(2));
            odomToMap = PM::Matrix::Identity(3, 3);
            laserScanSubscription = this->create_subscription<sensor_msgs::msg::LaserScan>("points_in", messageQueueSize,
                                                                                               std::bind(&MapperNode::laserScanCallback, this,
                                                                                                         std::placeholders::_1));
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
    std::unique_ptr<Trajectory> robotTrajectory;
    std::mutex mapTfLock;
    PM::TransformationParameters odomToMap;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud2Subscription;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScanSubscription;
    PM::TransformationParameters previousRobotToMap;
    rclcpp::Time previousTimeStamp;
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
        std::string extension = filePath.substr(extensionPosition, filePath.length()-1);

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

    void saveTrajectory(const std::string& trajectoryFileName)
    {
        RCLCPP_INFO(this->get_logger(), "Saving trajectory to %s", trajectoryFileName.c_str());
        robotTrajectory->save(trajectoryFileName);
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
                saveMap(params->finalMapFileName);
                saveTrajectory(params->finalTrajectoryFileName);
                RCLCPP_INFO(this->get_logger(), "Shutting down ROS");
                rclcpp::shutdown();
            }

            std::this_thread::sleep_for(std::chrono::duration<float>(0.1));
        }
    }

    PM::TransformationParameters findTransform(const std::string& sourceFrame, const std::string& targetFrame, const rclcpp::Time& time, const int& transformDimension)
    {
        geometry_msgs::msg::TransformStamped tf = tfBuffer->lookupTransform(targetFrame, sourceFrame, time, std::chrono::milliseconds(100));
        return PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(tf, transformDimension);
    }

    void gotInput(const PM::DataPoints& input, const std::string& sensorFrame, const rclcpp::Time& timeStamp)
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
            try
            {
                mapper->processInput(input, sensorToMapBeforeUpdate,
                                     std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.nanoseconds())));
            }
            catch (const PM::ConvergenceError& convergenceError)
            {
                RCLCPP_ERROR(this->get_logger(), "Unable to process input: %s", convergenceError.what());
                try
                {
                    saveTrajectory(appendToFilePath(params->finalTrajectoryFileName, "_convergence_error"));
                    saveMap(appendToFilePath(params->finalMapFileName, "_convergence_error"));
                }
                catch(const std::runtime_error& runtimeError)
                {
                    RCLCPP_ERROR(this->get_logger(), "Unable to save: %s", runtimeError.what());
                }
                throw;
            }
            const PM::TransformationParameters& sensorToMapAfterUpdate = mapper->getPose();

            PM::TransformationParameters currentOdomToMap = transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());
            mapTfLock.lock();
            odomToMap = currentOdomToMap;
            mapTfLock.unlock();

            PM::TransformationParameters robotToSensor = findTransform(params->robotFrame, sensorFrame, timeStamp, input.getHomogeneousDim());
            PM::TransformationParameters robotToMap = sensorToMapAfterUpdate * robotToSensor;

            robotTrajectory->addPoint(robotToMap.topRightCorner(input.getEuclideanDim(), 1));
            nav_msgs::msg::Odometry odomMsgOut = PointMatcher_ROS::pointMatcherTransformationToOdomMsg<float>(robotToMap, "map", params->robotFrame, timeStamp);

            if(previousTimeStamp.nanoseconds() != 0)
            {
                Eigen::Vector3f linearDisplacement = robotToMap.topRightCorner(input.getEuclideanDim(), 1) - previousRobotToMap.topRightCorner(input.getEuclideanDim(), 1);
                float deltaTime = (float) (timeStamp - previousTimeStamp).seconds();
                Eigen::Vector3f linearVelocity = linearDisplacement / deltaTime;
                odomMsgOut.twist.twist.linear.x = linearVelocity(0);
                odomMsgOut.twist.twist.linear.y = linearVelocity(1);
                odomMsgOut.twist.twist.linear.z = linearVelocity(2);
            }
            previousTimeStamp = timeStamp;
            previousRobotToMap = robotToMap;

            odomPublisher->publish(odomMsgOut);

            if(!params->publishTfsBetweenRegistrations)
            {
                geometry_msgs::msg::TransformStamped currentOdomToMapTf = PointMatcher_ROS::pointMatcherTransformationToRosTf<float>(currentOdomToMap, "map", params->odomFrame, timeStamp);
                tfBroadcaster->sendTransform(currentOdomToMapTf);
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
        gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(cloudMsgIn), cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp);
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan& scanMsgIn)
    {
        gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(scanMsgIn), scanMsgIn.header.frame_id, scanMsgIn.header.stamp);
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
            PM::TransformationParameters currentOdomToMap = odomToMap;
            mapTfLock.unlock();

            auto currTime = this->get_clock()->now();

            geometry_msgs::msg::TransformStamped currentOdomToMapTf = PointMatcher_ROS::pointMatcherTransformationToRosTf<float>(currentOdomToMap, "map",
                                                                                                                            params->odomFrame,
                                                                                                                            currTime);
            if (lastTime != currTime)
                tfBroadcaster->sendTransform(currentOdomToMapTf);

            lastTime = currTime;
            publishRate.sleep();
        }
    }

    void reloadYamlConfigCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
    	RCLCPP_INFO(this->get_logger(), "Reloading YAML config");
    	mapper->loadYamlConfig(params->configFilePath);
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
    		robotTrajectory->clearPoints();
    	}
    	catch(const std::runtime_error& e)
    	{
    		RCLCPP_ERROR(this->get_logger(), "Unable to load: %s", e.what());
    	}
    }

    void saveTrajectoryCallback(const std::shared_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory::Request> req, std::shared_ptr<norlab_icp_mapper_ros::srv::SaveTrajectory::Response> res)
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
    rclcpp::spin(std::make_shared<MapperNode>());
    rclcpp::shutdown();
    return 0;
}
