#include "Deskewer.h"
#include "NodeParameters.h"
#include "norlab_icp_mapper/Mapper.h"
#include <chrono>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <norlab_icp_mapper/Trajectory.h>
#include <norlab_icp_mapper_ros/srv/save.hpp>
#include <norlab_icp_mapper_ros/srv/load_map.hpp>
#include <norlab_icp_mapper_ros/srv/set_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/empty.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <memory>
#include <mutex>
#include <thread>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pointmatcher/PointMatcher.h>

class MapperNode : public rclcpp::Node
{
public:
    MapperNode() :
            Node("mapper_node")
    {
        params = std::unique_ptr<NodeParameters>(new NodeParameters(*this));

        transformation = PM::get().TransformationRegistrar.create("RigidTransformation");

        mapper = std::make_unique<norlab_icp_mapper::Mapper>(params->mappingConfig, params->is3D, params->isOnline,
                                               params->isMapping, params->saveMapCellsOnHardDrive);

        deskewer = std::make_unique<Deskewer>(this->get_logger(), this->get_clock(), params->expectedUniqueDeskewingTFNumber, params->deskewingRoundToNanoSecs);

        if(!params->initialMapFileName.empty())
        {
            loadMap(params->initialMapFileName);
        }
        if(!params->initialRobotPoseString.empty())
        {
            setRobotPose(params->initialRobotPose);
        }
        else
        {
            hasToSetRobotPose = false;
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
        inputFiltersScanPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_after_input_filters", 1);
        deskewingScanPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_after_deskew", 1);
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

        relocalizePoseSubscription = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_in", messageQueueSize,
                                                                                               std::bind(&MapperNode::relocalizePoseCallback, this,
                                                                                                         std::placeholders::_1));

        reloadYamlConfigService = this->create_service<std_srvs::srv::Empty>("reload_yaml_config",
                                                                             std::bind(&MapperNode::reloadYamlConfigCallback, this, std::placeholders::_1,
                                                                                       std::placeholders::_2));
        saveMapService = this->create_service<norlab_icp_mapper_ros::srv::Save>("save_map",
                                                                                   std::bind(&MapperNode::saveMapCallback, this, std::placeholders::_1,
                                                                                             std::placeholders::_2));
        loadMapService = this->create_service<norlab_icp_mapper_ros::srv::LoadMap>("load_map",
                                                                                   std::bind(&MapperNode::loadMapCallback, this, std::placeholders::_1,
                                                                                             std::placeholders::_2));
        saveTrajectoryService = this->create_service<norlab_icp_mapper_ros::srv::Save>("save_trajectory",
                                                                                                 std::bind(&MapperNode::saveTrajectoryCallback, this,
                                                                                                           std::placeholders::_1, std::placeholders::_2));
        enableMappingService = this->create_service<norlab_icp_mapper_ros::srv::SetState>("set_mapping_state",
                                                                            std::bind(&MapperNode::setMappingStateCallback, this, std::placeholders::_1,
                                                                                    std::placeholders::_2));
        enableLocalizationService = this->create_service<norlab_icp_mapper_ros::srv::SetState>("set_loc_state",
                                                                          std::bind(&MapperNode::setLocStateCallback, this, std::placeholders::_1,
                                                                                    std::placeholders::_2));
        mapPublisherThread = std::thread(&MapperNode::mapPublisherLoop, this);
        if(params->publishTfsBetweenRegistrations)
        {
            mapTfPublisherThread = std::thread(&MapperNode::mapTfPublisherLoop, this);
        }

        // Ensure proper localization and mapping states.
        isLocalizingLock.lock();
        isLocalizing = params->localizing;
        if(!isLocalizing)
        {
    	    mapper->setIsMapping(false);
        }
        if(mapper->getIsMapping())
        {
            isLocalizing = true;
        }
        isLocalizingLock.unlock();

        // Initialize parameter callback handle
        paramCallbackHandle = this->get_node_parameters_interface()->add_on_set_parameters_callback(
            std::bind(&MapperNode::updateCompressionVoxelSize, this, std::placeholders::_1));

        // Initial map voxel subsampling
        outputMapSubsamplingFilter =
            PM::get().DataPointsFilterRegistrar.create(
				"OctreeGridDataPointsFilter",
				{
				    {"maxSizeByNode", PointMatcherSupport::toParam(params->compressionVoxelSize)}
				}
            );
        diagnosticsPub = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

        // init mapping and localization status
        mappingStatus.name = "Norlab ICP Mapper Status";
        mappingStatus.hardware_id = "norlab_icp_mapper";
        mappingStatus.message = "Processing time and state of norlab icp mapper";
        mappingStatus.values.clear();
        mappingDurationValue.key = "Duration [ms]";
        mappingStateValue.key = "State";
    }

private:
    typedef PointMatcher<float> PM;

    std::unique_ptr<NodeParameters> params;
    std::shared_ptr<PM::Transformation> transformation;
    std::unique_ptr<norlab_icp_mapper::Mapper> mapper;
    PM::TransformationParameters robotPoseToSet;
    bool hasToSetRobotPose = false;
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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inputFiltersScanPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewingScanPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticsPub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud2Subscription;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScanSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr relocalizePoseSubscription;
    PM::TransformationParameters previousRobotToMap;
    rclcpp::Time previousTimeStamp;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reloadYamlConfigService;
    rclcpp::Service<norlab_icp_mapper_ros::srv::Save>::SharedPtr saveMapService;
    rclcpp::Service<norlab_icp_mapper_ros::srv::LoadMap>::SharedPtr loadMapService;
    rclcpp::Service<norlab_icp_mapper_ros::srv::Save>::SharedPtr saveTrajectoryService;
    rclcpp::Service<norlab_icp_mapper_ros::srv::SetState>::SharedPtr enableMappingService;
    rclcpp::Service<norlab_icp_mapper_ros::srv::SetState>::SharedPtr disableMappingService;
    rclcpp::Service<norlab_icp_mapper_ros::srv::SetState>::SharedPtr enableLocalizationService;
    rclcpp::Service<norlab_icp_mapper_ros::srv::SetState>::SharedPtr disableLocalizationService;

    diagnostic_msgs::msg::DiagnosticStatus mappingStatus;
    diagnostic_msgs::msg::KeyValue mappingDurationValue;
    diagnostic_msgs::msg::KeyValue mappingStateValue;

    std::thread mapPublisherThread;
    std::thread mapTfPublisherThread;

    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> paramCallbackHandle;

    std::shared_ptr<PM::DataPointsFilter> outputMapSubsamplingFilter;

    bool isLocalizing;
    std::mutex isLocalizingLock;

    std::unique_ptr<Deskewer> deskewer;

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

    void gotInput(PM::DataPoints& input, const std::string& sensorFrame, const rclcpp::Time& cloudStamp)
    {
        mappingStatus.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        rclcpp::Time timeStamp = cloudStamp;
        std::chrono::steady_clock::time_point processingStartTime = std::chrono::steady_clock::now();
        long processingDuration(0);
        try
        {
            norlab_icp_mapper::MapperState state = norlab_icp_mapper::MapperState::FAILURE;
            mapper->applyInputFilters(input);
            std::chrono::steady_clock::time_point filterEndTime = std::chrono::steady_clock::now();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Applied input filters in " << std::chrono::duration_cast<std::chrono::milliseconds>(filterEndTime - processingStartTime).count() << " [ms]");
            publishAfterInputFilters(input, sensorFrame, cloudStamp);

            if (params->deskew)
            {
                bool deskewSuccessuful = deskewer->deskewCloud(input, sensorFrame);

                // if deskewing was successful, update the cloud timestamp to match the last point in the cloud
                if (deskewSuccessuful)
                {
                    publishAfterDeskew(input, sensorFrame, cloudStamp);
                    timeStamp = rclcpp::Time(input.times(input.getNbPoints() - 1), timeStamp.get_clock_type());
                }
            }


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
                std::chrono::steady_clock::time_point mappingStartTime = std::chrono::steady_clock::now();
                state = mapper->processInput(input, sensorToMapBeforeUpdate,
                                     std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.nanoseconds())));
                std::chrono::steady_clock::time_point mappingEndTime = std::chrono::steady_clock::now();
                RCLCPP_DEBUG_STREAM(this->get_logger(), "Mapper call executed in: " << std::chrono::duration_cast<std::chrono::milliseconds>(mappingEndTime - mappingStartTime).count() << " [ms]");
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

            robotTrajectory->addPose(robotToMap, std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(timeStamp.nanoseconds())));
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

            std::chrono::steady_clock::time_point processingEndTime = std::chrono::steady_clock::now();
            processingDuration = std::chrono::duration_cast<std::chrono::milliseconds>(processingEndTime - processingStartTime).count();
            std::string message;

            if (state == norlab_icp_mapper::MapperState::LOCALIZING)
            {
                mappingStateValue.value = "LOCALIZING";
                // If the mapping takes more than 100ms, we are not real time anymore
                if (processingDuration > 100)
                {
                    mappingStatus.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                    RCLCPP_WARN_STREAM(this->get_logger(), "Localization finished in " << processingDuration << " [ms]");
                }
                else
                {
                    mappingStatus.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                    RCLCPP_DEBUG_STREAM(this->get_logger(), "Localization finished in " << processingDuration << " [ms]");
                }
            }
            else if (state == norlab_icp_mapper::MapperState::MAPPING)
            {
                mappingStateValue.value = "MAPPING";
                // If the mapping takes more than 100ms, we are not real time anymore
                if (processingDuration > 100)
                {
                    mappingStatus.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                    RCLCPP_WARN_STREAM(this->get_logger(), "Mapping finished in " << processingDuration << " [ms]");
                }
                else
                {
                    mappingStatus.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                    RCLCPP_DEBUG_STREAM(this->get_logger(), "Mapping finished in " << processingDuration << " [ms]");
                }
            }
            else
            {
                mappingStateValue.value = "FAILURE";
                mappingStatus.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                RCLCPP_ERROR_STREAM(this->get_logger(), "Mapping finished with failure in " << processingDuration << " [ms]");
            }

        }
        catch(const tf2::TransformException& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            mappingStatus.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            mappingStateValue.value = "FAILURE";
        }

        mappingDurationValue.value = std::to_string(processingDuration);
        mappingStatus.values.push_back(mappingDurationValue);
        mappingStatus.values.push_back(mappingStateValue);
        publishDiagnosticStatus();
    }

    void publishDiagnosticStatus()
    {
        diagnostic_msgs::msg::DiagnosticArray diagArrayMsg;
        diagArrayMsg.header.stamp = now();
        diagArrayMsg.status.push_back(mappingStatus);
        diagnosticsPub->publish(diagArrayMsg);
    }

    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2& cloudMsgIn)
    {
        RCLCPP_DEBUG(this->get_logger(), "----POINT CLOUD RECEIVED----");
        isLocalizingLock.lock();
        if(isLocalizing)
        {
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            auto input = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(cloudMsgIn);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Input converted in " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " [ms]");
            gotInput(input, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp);
        }
        isLocalizingLock.unlock();
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan& scanMsgIn)
    {
        RCLCPP_DEBUG(this->get_logger(), "----LASER SCAN RECEIVED----");
        isLocalizingLock.lock();
        if(isLocalizing)
        {
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            auto input = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(scanMsgIn);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Input converted in " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " [ms]");
            gotInput(input, scanMsgIn.header.frame_id, scanMsgIn.header.stamp);
        }
        isLocalizingLock.unlock();
    }

    void publishAfterInputFilters(const PM::DataPoints& input, const std::string& sensorFrame, const rclcpp::Time& timeStamp)
    {
        if (inputFiltersScanPublisher->get_subscription_count() > 0)
        {
            sensor_msgs::msg::PointCloud2 filteredInputMsgOut = PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(input, sensorFrame, timeStamp);
            inputFiltersScanPublisher->publish(filteredInputMsgOut);
        }
    }

    void publishAfterDeskew(const PM::DataPoints& input, const std::string& sensorFrame, const rclcpp::Time& timeStamp)
    {
        if (deskewingScanPublisher->get_subscription_count() > 0)
        {
            sensor_msgs::msg::PointCloud2 deskewedCloudMsgOut = PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(input, sensorFrame, timeStamp);
            deskewingScanPublisher->publish(deskewedCloudMsgOut);
        }
    }

    void mapPublisherLoop()
    {
        rclcpp::Rate publishRate(params->mapPublishRate);

        PM::DataPoints newMap;
        while(rclcpp::ok())
        {
            if(mapper->getNewLocalMap(newMap) && mapPublisher->get_subscription_count() > 0)
            {
                if (params->compressionVoxelSize > 0)
                {
                    int origNumPoints = newMap.getNbPoints();
                    std::chrono::steady_clock::time_point mapMessageSubsamplingStartTime = std::chrono::steady_clock::now();
                    outputMapSubsamplingFilter->inPlaceFilter(newMap);
                    std::chrono::steady_clock::time_point mapMessageSubsamplingEndTime = std::chrono::steady_clock::now();
                    RCLCPP_DEBUG_STREAM(this->get_logger(), "Output map subsampled to: " << 100.0*(newMap.getNbPoints() / (double) origNumPoints)
                        << " % in " << std::chrono::duration_cast<std::chrono::milliseconds>(mapMessageSubsamplingEndTime - mapMessageSubsamplingStartTime).count() << " [ms]");
                }

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
    	mapper->loadYamlConfig(params->mappingConfig);
    }

    void saveMapCallback(const std::shared_ptr<norlab_icp_mapper_ros::srv::Save::Request> req, std::shared_ptr<norlab_icp_mapper_ros::srv::Save::Response> res)
    {
    	try
    	{
    		saveMap(req->file_name.data);
            res->success = true;
            res->message = "Map saved successfully to " + std::string(req->file_name.data);
    	}
    	catch(const std::runtime_error& e)
    	{
    		RCLCPP_ERROR(this->get_logger(), "Unable to save: %s", e.what());
            res->success = false;
            res->message = e.what();
    	}
    }

    void loadMapCallback(const std::shared_ptr<norlab_icp_mapper_ros::srv::LoadMap::Request> req, std::shared_ptr<norlab_icp_mapper_ros::srv::LoadMap::Response> res)
    {
    	try
    	{
    		loadMap(req->map_file_name.data);
            int homogeneousDim = params->is3D ? 4 : 3;
            setRobotPose(PointMatcher_ROS::rosMsgToPointMatcherTransformation<float>(req->pose, homogeneousDim));
    		robotTrajectory->clear();
    	}
    	catch(const std::runtime_error& e)
    	{
    		RCLCPP_ERROR(this->get_logger(), "Unable to load: %s", e.what());
    	}
    }

    void saveTrajectoryCallback(const std::shared_ptr<norlab_icp_mapper_ros::srv::Save::Request> req, std::shared_ptr<norlab_icp_mapper_ros::srv::Save::Response> res)
    {
    	try
    	{
    		saveTrajectory(req->file_name.data);
            res->success = true;
            res->message = "Trajectory saved successfully to " + std::string(req->file_name.data);
    	}
    	catch(const std::runtime_error& e)
    	{
    		RCLCPP_ERROR(this->get_logger(), "Unable to save: %s", e.what());
            res->success = false;
            res->message = e.what();
    	}
    }

    void setMappingStateCallback(const std::shared_ptr<norlab_icp_mapper_ros::srv::SetState::Request> req,
                                std::shared_ptr<norlab_icp_mapper_ros::srv::SetState::Response> res)
    {
        res->success = true;
        if (req->state == true)
        {
           	RCLCPP_INFO(this->get_logger(), "Enabling mapping");
            isLocalizingLock.lock();
            if(!isLocalizing)
            {
                isLocalizing = true;
            }
            isLocalizingLock.unlock();
           	mapper->setIsMapping(true);
            res->message = "Mapping enabled";
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Disabling mapping");
           	mapper->setIsMapping(false);
            res->message = "Mapping disabled";
        }
    }

    void setLocStateCallback(const std::shared_ptr<norlab_icp_mapper_ros::srv::SetState::Request> req,
                                std::shared_ptr<norlab_icp_mapper_ros::srv::SetState::Response> res)
    {
        res->success = true;
        if (req->state == true)
        {
           	RCLCPP_INFO(this->get_logger(), "Enabling localization");
            isLocalizingLock.lock();
           	isLocalizing = true;
            isLocalizingLock.unlock();
            res->message = "Localization enabled";
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Disabling localization");
            if(mapper->getIsMapping())
            {
           	    mapper->setIsMapping(false);
            }
            isLocalizingLock.lock();
            isLocalizing = false;
            isLocalizingLock.unlock();
            res->message = "Localization disabled";
        }
    }

    void relocalizePoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& poseMsgIn)
    {
        if (mapper->getIsMapping())
        {
            RCLCPP_WARN(this->get_logger(), "Can not relocalize the robot if mapping is active.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Using 2D pose estimate given.");
            int homogeneousDim = params->is3D ? 4 : 3;
            setRobotPose(PointMatcher_ROS::rosMsgToPointMatcherTransformation<float>(poseMsgIn.pose.pose, homogeneousDim));
        }
    }

    rcl_interfaces::msg::SetParametersResult updateCompressionVoxelSize(const std::vector<rclcpp::Parameter>& updatedParams)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : updatedParams)
        {
            // TODO find a way to move this to NodeParameters.cpp or sync the param name across files
            if (param.get_name() == "compression_voxel_size" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                double voxelSize = param.as_double();

                if (voxelSize < 0)
                {
                    RCLCPP_WARN_STREAM(this->get_logger(), "Invalid voxel size. Must be non-negative: " << voxelSize);
                    result.successful = false;
                    result.reason = "Invalid voxel size. Must be non-negative.";
                }
                else
                {
                    RCLCPP_DEBUG_STREAM(this->get_logger(), "Setting voxel size to: " << voxelSize);
                    params->compressionVoxelSize = voxelSize;

                    outputMapSubsamplingFilter =
                        PM::get().DataPointsFilterRegistrar.create(
           					"OctreeGridDataPointsFilter",
           					{
          						{"maxSizeByNode", PointMatcherSupport::toParam(params->compressionVoxelSize)}
           					}
                        );
                    result.reason = "Voxel size updated successfully.";
                }
            }
        }
        return result;
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapperNode>());
    rclcpp::shutdown();
    return 0;
}
