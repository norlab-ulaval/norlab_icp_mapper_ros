#include "Deskewer.h"
#include "NodeParameters.h"
#include "RegistrationQualityGate.h"
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <unordered_set>
#include <thread>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <norlab_icp_mapper/Trajectory.h>
#include <norlab_icp_mapper/MapperModules/PointDistanceMapperModule.h>
#include <norlab_icp_mapper_ros/srv/save_map.hpp>
#include <norlab_icp_mapper_ros/srv/load_map.hpp>
#include <norlab_icp_mapper_ros/srv/save_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.hpp>

// ── Frame convention ──
// aToB = T_B_A  (maps points FROM frame A INTO frame B)
// T notation is column-major: T_B_A * p_A = p_B

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
        mappingEnabled_.store(params->isMapping);

        RCLCPP_INFO(this->get_logger(),
            "Map management: cell_storage=%s trimming=%s global_output=%s — %s",
            params->saveMapCellsOnHardDrive ? "disk(/tmp/*.vtk)" : "RAM",
            params->enableMapTrimming ? "ON" : "OFF",
            params->enableGlobalOutputMap ? "ON" : "OFF",
            params->enableGlobalOutputMap && params->enableMapTrimming
                ? "local ICP map capped; global output map preserved for revisits/export"
                : params->saveMapCellsOnHardDrive && !params->enableMapTrimming
                ? "large-area mode (bounded local ICP map, global history preserved)"
                : params->enableMapTrimming
                    ? "short-range mode (map capped at trim radius, global history LOST)"
                    : "RAM mode (full map in RAM, grows with total area)");

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
            tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            messageQueueSize = 1;
        }
        else
        {
            mapperShutdownThread = std::thread(&MapperNode::mapperShutdownLoop, this);
            tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock(), std::chrono::seconds(1000000));
            messageQueueSize = 1;
        }

        tfListener = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(*tfBuffer));
        tfBroadcaster = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster(*this));

        // Deskewer is initialized here, after tfBuffer is ready.
        deskewer = std::make_unique<Deskewer>(
            tfBuffer,
            this->get_logger(),
            params->deskewFixedFrame,
            Deskewer::parseTimeMode(params->deskewTimeMode),
            params->deskewTimeField,
            static_cast<uint32_t>(params->expectedUniqueDeskewingTFNumber),
            static_cast<uint32_t>(params->deskewingRoundToNanoSecs),
            static_cast<uint32_t>(params->tfLookupTimeoutMs));

        // IMU subscription for gyro-based rotation-only deskew.
        // This path replaces TF-odom deskew in replay to avoid the coupling:
        //   bad odom angular rate → per-point twist → swirl/double-tree in map.
        if (params->deskew && params->deskewSource == "imu")
        {
            auto imu_qos = rclcpp::SensorDataQoS().best_effort();
            if (!params->isOnline) { imu_qos.keep_all(); }
            imuDeskewSubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
                params->deskewImuTopic, imu_qos,
                [this](const sensor_msgs::msg::Imu::SharedPtr msg)
                {
                    const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
                    const Eigen::Vector3d omega(
                        msg->angular_velocity.x,
                        msg->angular_velocity.y,
                        msg->angular_velocity.z);
                    std::lock_guard<std::mutex> lk(imuDeskewBufMutex_);
                    // Sim time jumped backward (bag loop/restart): drop the stale
                    // future-time samples so the buffer stays time-sorted.
                    if (!imuDeskewBuf_.empty() &&
                        stamp_ns + 500'000'000LL < imuDeskewBuf_.back().first)
                        imuDeskewBuf_.clear();
                    imuDeskewBuf_.emplace_back(stamp_ns, omega);
                    // Retain up to 5 s of gyro history — covers any scan gap during replay.
                    while (imuDeskewBuf_.size() > 1 &&
                           stamp_ns - imuDeskewBuf_.front().first > 5'000'000'000LL)
                        imuDeskewBuf_.pop_front();
                });
            RCLCPP_INFO(this->get_logger(),
                "[IMU deskew] Subscribed to '%s' for gyro rotation-only deskew. "
                "IMU→sensor extrinsic will be looked up on first scan.",
                params->deskewImuTopic.c_str());
        }

        if (params->enableDynamicTrailerSelfFilter)
        {
            dynamicTrailerArticulationSubscription_ =
                this->create_subscription<std_msgs::msg::Float64>(
                    params->dynamicTrailerArticulationTopic,
                    rclcpp::SensorDataQoS(),
                    [this](const std_msgs::msg::Float64::SharedPtr msg)
                    {
                        std::lock_guard<std::mutex> lk(dynamicTrailerMutex_);
                        latestDynamicTrailerAngleRad_ = msg->data;
                        latestDynamicTrailerAngleTime_ = this->now();
                        hasDynamicTrailerAngle_ = true;
                    });
            RCLCPP_INFO(this->get_logger(),
                "[SELF-FILTER] Dynamic trailer OBB enabled: topic=%s hitch=(%.3f, %.3f) "
                "s=[%.2f, %.2f] half_width=%.2f z=[%.2f, %.2f] yaw=%.3f %+g*phi",
                params->dynamicTrailerArticulationTopic.c_str(),
                params->dynamicTrailerHitchX, params->dynamicTrailerHitchY,
                params->dynamicTrailerFrontOffsetM, params->dynamicTrailerRearOffsetM,
                params->dynamicTrailerHalfWidthM,
                params->dynamicTrailerZMinM, params->dynamicTrailerZMaxM,
                params->dynamicTrailerYawOffsetRad, params->dynamicTrailerYawSign);
        }

        mapPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "map", rclcpp::QoS(1).reliable().transient_local());
        inputFiltersScanPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_after_input_filters", 1);
        deskewingScanPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_after_deskew", 1);
        alignedScanPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "aligned_scan", rclcpp::QoS(1).reliable());
        odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("icp_odom", 50);
        // Pure ICP corrections only. Unlike icp_odom, this topic never contains
        // odom-bridge poses and can therefore be used as an estimator correction
        // measurement without confusing dead reckoning for scan matching.
        icpMeasurementPublisher =
            this->create_publisher<nav_msgs::msg::Odometry>("icp_measurement", 50);
        statusPublisher = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("status", 50);
        trajectoryPathPublisher = this->create_publisher<nav_msgs::msg::Path>(
            "trajectory_path", rclcpp::QoS(1).reliable().transient_local());
        trajectoryPathTimer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MapperNode::publishTrajectoryPathSnapshot, this));
        diagnosticsTimer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&MapperNode::publishDiagnosticsHeartbeat, this));

        if(params->is3D)
        {
            robotTrajectory = std::unique_ptr<Trajectory>(new Trajectory(3));
            odomToMap = PM::Matrix::Identity(4, 4);
            // Default live LiDAR QoS is SensorDataQoS/best_effort. Replay bags may
            // offer reliable only, so the subscription reliability is configurable.
            auto qos = rclcpp::SensorDataQoS();
            if (params->inputQosReliable) { qos.reliable(); }
            else { qos.best_effort(); }
            if (!params->isOnline) { qos.keep_all(); }
            RCLCPP_INFO(this->get_logger(),
                "Subscribing to points_in with %s input QoS.",
                params->inputQosReliable ? "reliable" : "best_effort");
            pointCloud2Subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "points_in", qos,
                std::bind(&MapperNode::pointCloud2Callback, this, std::placeholders::_1));
        }
        else
        {
            robotTrajectory = std::unique_ptr<Trajectory>(new Trajectory(2));
            odomToMap = PM::Matrix::Identity(3, 3);
            auto qos = rclcpp::SensorDataQoS();
            if (params->inputQosReliable) { qos.reliable(); }
            else { qos.best_effort(); }
            if (!params->isOnline) { qos.keep_all(); }
            RCLCPP_INFO(this->get_logger(),
                "Subscribing to points_in with %s input QoS.",
                params->inputQosReliable ? "reliable" : "best_effort");
            laserScanSubscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "points_in", qos,
                std::bind(&MapperNode::laserScanCallback, this, std::placeholders::_1));
        }

        relocalizePoseSubscription = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_in", messageQueueSize,
                                                                                               std::bind(&MapperNode::relocalizePoseCallback, this,
                                                                                                         std::placeholders::_1));

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
        enableLocalizationService = this->create_service<std_srvs::srv::Empty>("enable_loc",
                                                                          std::bind(&MapperNode::enableLocCallback, this, std::placeholders::_1,
                                                                                    std::placeholders::_2));
        disableLocalizationService = this->create_service<std_srvs::srv::Empty>("disable_loc",
                                                                           std::bind(&MapperNode::disableLocCallback, this, std::placeholders::_1,
                                                                                     std::placeholders::_2));
        mapPublisherThread = std::thread(&MapperNode::mapPublisherLoop, this);
        if(params->publishTfsBetweenRegistrations)
        {
            mapTfPublisherThread = std::thread(&MapperNode::mapTfPublisherLoop, this);
        }

        // Initialize localization and mapping state (no mutex — isLocalizing_ is atomic).
        isLocalizing_.store(params->localizing);
        if (!isLocalizing_.load())
        {
            mappingEnabled_.store(false);
            mapper->setIsMapping(false);
        }
        if (mapper->getIsMapping())
        {
            isLocalizing_.store(true);
        }

        // Configure quality gate from parameters.
        RegistrationQualityGate::Config qgConfig;
        qgConfig.min_input_points         = params->minInputPoints;
        qgConfig.max_translation_m        = params->maxTranslationCorrection;
        qgConfig.max_rotation_deg         = params->maxRotationCorrectionDeg;
        qgConfig.max_velocity_ms          = params->maxVelocityMs;
        qgConfig.max_yaw_rate_deg_s       = params->maxYawRateDegS;
        qgConfig.max_registration_time_ms = params->maxRegistrationTimeMs;
        qualityGate_.setConfig(qgConfig);

        RCLCPP_INFO(this->get_logger(),
            "Mapper effective params: frames map=%s odom=%s robot=%s filtering=%s "
            "config=%s deskew=%s source=%s is_online=%s cell_storage=%s trimming=%s "
            "anchor_initial_robot=%s pose_gate=%.2fm/%.1fdeg pose_step=%.1fm/%.1fdeg map_update_gate=%.2fm/%.1fdeg "
            "overlap_pose=%.2f/%.2f overlap_map=%.2f/%.2f",
            params->mapFrame.c_str(),
            params->odomFrame.c_str(),
            params->robotFrame.c_str(),
            params->filteringFrame.c_str(),
            params->mappingConfig.c_str(),
            params->deskew ? "true" : "false",
            params->deskewSource.c_str(),
            params->isOnline ? "true" : "false",
            params->saveMapCellsOnHardDrive ? "true" : "false",
            params->enableMapTrimming ? "true" : "false",
            params->anchorMapAtInitialRobotPose ? "true" : "false",
            params->maxTranslationCorrection,
            params->maxRotationCorrectionDeg,
            params->maxPoseStepM,
            params->maxPoseYawStepDeg,
            params->maxMapUpdateTranslationCorrectionM,
            params->maxMapUpdateRotationCorrectionDeg,
            params->minPoseOverlapNearRatio,
            params->minPoseOverlapLooseRatio,
            params->minMapOverlapNearRatio,
            params->minMapOverlapLooseRatio);
        RCLCPP_INFO(this->get_logger(),
            "Recovery/adaptive params: map_recovery=%s after=%d interval=%d radius=%.1fm pts=%d..%d "
            "snapshot_every=%d snapshot_gate=%.2fm/%.1fdeg adaptive_gate=%s gains(v/a/yaw)=%.2f/%.2f/%.2f "
            "aggressive(speed/yaw)=%.2fmps/%.1fdps pivot(speed/yaw/tr)=%.2fmps/%.1fdps/%.2fm "
            "odom_bridge=%s after=%d min_speed=%.2fmps map_insert=%s planar=%s z_drift=%.2fm",
            params->enableMapRecovery ? "true" : "false",
            params->recoveryReloadAfterRejections,
            params->recoveryAttemptIntervalScans,
            params->recoveryLocalMapRadiusM,
            params->recoveryLocalMapMinPoints,
            params->recoveryLocalMapMaxPoints,
            params->snapshotSaveIntervalScans,
            params->snapshotMaxTranslationCorrectionM,
            params->snapshotMaxRotationCorrectionDeg,
            params->enableMotionAdaptiveGate ? "true" : "false",
            params->adaptiveVelocityGain,
            params->adaptiveAccelerationGain,
            params->adaptiveYawRateGain,
            params->aggressiveSpeedMs,
            params->aggressiveYawRateDegS,
            params->pivotLinearSpeedMs,
            params->pivotYawRateDegS,
            params->pivotMaxTranslationCorrectionM,
            params->enableOdomBridge ? "true" : "false",
            params->odomBridgeAfterRejections,
            params->odomBridgeMinSpeedMs,
            params->allowOdomBridgeMapInsertion ? "true" : "false",
            params->enablePlanarPoseConstraint ? "true" : "false",
            params->planarPoseMaxZDriftM);

        // Register parameter update callback.
        paramCallbackHandle = this->get_node_parameters_interface()->add_on_set_parameters_callback(
            std::bind(&MapperNode::updateCompressionVoxelSize, this, std::placeholders::_1));

        // Initial map voxel subsampling filter.
        outputMapSubsamplingFilter =
            PM::get().DataPointsFilterRegistrar.create(
                "OctreeGridDataPointsFilter",
                {{"maxSizeByNode", PointMatcherSupport::toParam(params->compressionVoxelSize)}}
            );

        inputSurfaceNormalFilter_ =
            PM::get().DataPointsFilterRegistrar.create(
                "SurfaceNormalDataPointsFilter",
                {{"knn", PointMatcherSupport::toParam(5)}}
            );

        mapSurfaceNormalFilter_ =
            PM::get().DataPointsFilterRegistrar.create(
                "SurfaceNormalDataPointsFilter",
                {{"knn", PointMatcherSupport::toParam(10)}}
            );

        deterministicMapVoxelFilter_ =
            PM::get().DataPointsFilterRegistrar.create(
                "VoxelGridDataPointsFilter",
                {{"vSizeX", PointMatcherSupport::toParam(0.08)},
                 {"vSizeY", PointMatcherSupport::toParam(0.08)},
                 {"vSizeZ", PointMatcherSupport::toParam(0.08)}}
            );

        deterministicMapperModule_ = std::make_shared<PointDistanceMapperModule>(
            PM::Parameters{{"minDistNewPoint",
                PointMatcherSupport::toParam(params->deterministicMapMinDistNewPoint)}});

        globalOutputMapperModule_ = std::make_shared<PointDistanceMapperModule>(
            PM::Parameters{{"minDistNewPoint",
                PointMatcherSupport::toParam(params->globalOutputMapMinDistNewPoint)}});
    }

    ~MapperNode()
    {
        running_.store(false);
        if (mapPublisherThread.joinable())    { mapPublisherThread.join(); }
        if (mapTfPublisherThread.joinable())  { mapTfPublisherThread.join(); }
        if (mapperShutdownThread.joinable())  { mapperShutdownThread.join(); }
        // Save map/trajectory on any shutdown (SIGINT, timeout, or error).
        // Guards hasSavedMap_ to avoid double-saving when mapperShutdownLoop
        // already triggered a save before the destructor runs.
        if (!hasSavedMap_)
        {
            try {
                saveMap(params->finalMapFileName);
                saveTrajectory(params->finalTrajectoryFileName);
                hasSavedMap_ = true;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to save on shutdown: %s", e.what());
            }
        }
        RCLCPP_INFO(this->get_logger(), "MapperNode shutdown complete.");
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
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    std::unique_ptr<Trajectory> robotTrajectory;
    std::mutex mapTfLock;
    PM::TransformationParameters odomToMap;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inputFiltersScanPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewingScanPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr alignedScanPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr icpMeasurementPublisher;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr statusPublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectoryPathPublisher;
    rclcpp::TimerBase::SharedPtr trajectoryPathTimer_;
    rclcpp::TimerBase::SharedPtr diagnosticsTimer_;
    nav_msgs::msg::Path trajectoryPath_;   ///< Accumulated path, published at each accepted scan.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud2Subscription;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dynamicTrailerArticulationSubscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScanSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr relocalizePoseSubscription;
    PM::TransformationParameters previousRobotToMap;
    rclcpp::Time previousTimeStamp;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reloadYamlConfigService;
    rclcpp::Service<norlab_icp_mapper_ros::srv::SaveMap>::SharedPtr saveMapService;
    rclcpp::Service<norlab_icp_mapper_ros::srv::LoadMap>::SharedPtr loadMapService;
    rclcpp::Service<norlab_icp_mapper_ros::srv::SaveTrajectory>::SharedPtr saveTrajectoryService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enableMappingService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disableMappingService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enableLocalizationService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disableLocalizationService;
    std::thread mapPublisherThread;
    std::thread mapTfPublisherThread;

    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> paramCallbackHandle;

    std::shared_ptr<PM::DataPointsFilter> outputMapSubsamplingFilter;
    std::shared_ptr<PM::DataPointsFilter> inputSurfaceNormalFilter_;
    std::shared_ptr<PM::DataPointsFilter> mapSurfaceNormalFilter_;
    std::shared_ptr<PM::DataPointsFilter> deterministicMapVoxelFilter_;
    std::shared_ptr<MapperModule> deterministicMapperModule_;
    std::shared_ptr<MapperModule> globalOutputMapperModule_;
    std::mutex globalOutputMapMutex_;
    PM::DataPoints globalOutputMap_;
    bool hasGlobalOutputMap_{false};
    uint64_t globalOutputMapUpdates_{0};
    PM::TransformationParameters lastDeterministicMapUpdatePose_;
    bool hasDeterministicMapUpdatePose_{false};
    struct LastGoodMapSnapshot
    {
        PM::DataPoints map;
        PM::TransformationParameters sensorToMap;
        PM::TransformationParameters robotToMap;
        rclcpp::Time stamp;
        uint64_t acceptedScan{0};
        bool valid{false};
    };
    LastGoodMapSnapshot lastGoodMapSnapshot_;
    int lastRecoveryAttemptRejections_{-1};
    double lastOdomPriorSpeedMs_{0.0};
    bool hasLastOdomPriorSpeed_{false};
    bool forceNextMapUpdate_{false};
    // Node-side deterministic insertion avoids the unsafe second ICP pass that
    // was contaminating the map. Spacing is parameterized because replay and
    // low-speed articulated motion need updates before leaving the first scan.
    // Map publication crop radius is ROS parameter map_publish_radius_m.
    // 0.0 = publish full map. When set (e.g. 40.0 m), reduces a 200K-pt global
    // map to ~20-40K locally visible pts, cutting Foxglove WebSocket bandwidth
    // by 80-90%. Configured per scenario via MAPPING_MAP_PUBLISH_RADIUS_M env var.
    // ICP map trimming is parameterized. Keep it enabled online to bound KDTree
    // cost, but disable it for offline ground-truth map generation.
    // Deskewing is disabled for scans acquired during fast articulated turns.
    // At 60 deg/s yaw rate, one 100ms scan spans 6 deg of rotation. At 10m range that
    // shifts points by ~1.05m — comparable to ICP maxDist. The linear TF interpolation
    // in Deskewer introduces errors during non-linear articulation maneuvers that exceed
    // the correction it provides. Disable deskewing above this threshold.
    static constexpr double maxDeskewYawRateDegS_ = 60.0;
    // Map-update correction limits are ROS parameters
    // (max_map_update_translation_correction_m / _rotation_correction_deg).
    // The translation limit measures PRIOR error, not registration quality:
    // with wheel slip the prior can be 2-3 m off while ICP still aligns the
    // scan correctly. A hard 1.5 m limit froze the map mid-run and caused a
    // rejection cascade once the robot outran the frozen map.
    static constexpr double maxMapUpdateYawStepDeg_ = 45.0;
    static constexpr double maxMapUpdateZStepM_ = 0.50;
    static constexpr double mapOverlapNearVoxelM_ = 0.15;
    static constexpr double mapOverlapLooseVoxelM_ = 0.35;
    // Pose/insertion overlap gates are ROS parameters (NodeParameters):
    // min_pose_overlap_{near,loose}_ratio, min_map_overlap_{near,loose}_ratio.
    // A compile-time insertion gate above the pose gate creates an exploration
    // deadlock: scans in the hysteresis band are tracked but never inserted,
    // the map freezes and every following scan is rejected.
    static constexpr int minMapOverlapSamples_ = 200;

    struct MotionState
    {
        double dtAcceptedS{0.0};
        double odomSpeedMs{0.0};
        double odomAccelMs2{0.0};
        double odomYawRateDegS{0.0};
        double imuYawRateDegS{0.0};
        double dominantYawRateDegS{0.0};
        bool aggressive{false};
        bool pivot{false};
    };

    struct AdaptiveGateLimits
    {
        double translationM{0.0};
        double rotationDeg{0.0};
        bool adaptive{false};
        bool pivot{false};
    };

    // ── Safe map publication buffer ──
    // Instead of calling mapper->getNewLocalMap() from the publisher thread
    // (which may swap Map's internal double-buffer and leave isLocalPointCloudEmpty()=true,
    // causing the next processInput to skip ICP and jump to raw odom), we maintain
    // our own copy of the published map updated only from the gotInput thread.
    // The publisher thread reads from this copy under mapPublishLock_.
    PM::DataPoints latestMapForPublication_;
    bool latestMapReady_{false};
    std::mutex mapPublishLock_;
    // Publisher thread sets this after each publish to request a new snapshot.
    // gotInput only calls mapper->getMap() when set — avoids copying the full map
    // (200k+ pts, 50-200ms) at every accepted scan and blocking the ICP hot path.
    std::atomic<bool> needMapSnapshot_{true};

    // Atomic — no mutex needed for simple bool flag.
    std::atomic<bool> isLocalizing_{true};
    // Authoritative requested mapping state. The mapper itself is temporarily
    // switched off during every scan's localization phase, so getIsMapping()
    // cannot represent the operator/service request.
    std::atomic<bool> mappingEnabled_{true};
    // Shutdown flag for background threads.
    std::atomic<bool> running_{true};
    // Scan acceptance statistics.
    std::atomic<uint64_t> scansAccepted_{0};
    std::atomic<uint64_t> scansRejected_{0};
    std::atomic<uint64_t> pointCloudCallbacksStarted_{0};
    std::atomic<uint64_t> pointCloudCallbacksCompleted_{0};
    std::atomic<int64_t> lastPointCloudStampNs_{0};
    std::atomic<uint64_t> mapVersion_{1};
    // Consecutive rejection counter — reset to 0 on each accepted scan.
    // Updated only from the gotInput thread (serialized), so no atomic needed.
    int consecutiveRejections_{0};
    // Odom-bridge scans are accepted for odometry/path but frozen for map insertion.
    // They therefore reset consecutiveRejections_, so recovery needs its own counter.
    int consecutiveOdomBridgeScans_{0};
    int lastOdomBridgeRecoveryAttempt_{-1};
    // Protects outputMapSubsamplingFilter during runtime param update.
    std::mutex mapFilterMutex_;
    // Protects robotTrajectory access across gotInput and mapperShutdownLoop threads.
    std::mutex trajectoryMutex_;

    RegistrationQualityGate qualityGate_;
    // Timestamp of the last ACCEPTED scan — used for quality gate velocity check.
    // Unlike previousTimeStamp (updated on every scan), this only advances on acceptance.
    // Prevents the cascade: correction / dt_single_scan = spurious high velocity when odom
    // has drifted silently over many rejected scans.
    rclcpp::Time lastAcceptedTimeStamp_;
    // Robot pose at the last ACCEPTED scan — used for publishedPosePlausible check.
    // Unlike previousRobotToMap (updated even on rejection), this only advances on acceptance.
    // Prevents: rejected pose becomes new reference → cascade of gradually drifting poses.
    PM::TransformationParameters lastAcceptedRobotToMap_;
    PM::TransformationParameters initialAcceptedRobotToMap_;
    bool hasInitialAcceptedRobotToMap_{false};
    bool mapAnchoredAtInitialRobotPose_{false};
    bool hasSavedMap_{false};
    std::unique_ptr<Deskewer> deskewer;

    // ── IMU gyro deskew buffer (only used when deskew_source="imu") ──
    // ImuSample = std::pair<int64_t stamp_ns, Eigen::Vector3d omega_imu_frame>
    std::deque<Deskewer::ImuSample> imuDeskewBuf_;
    std::mutex imuDeskewBufMutex_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuDeskewSubscription_;
    // Static rotation: IMU frame → sensor/LiDAR frame.  Looked up once from TF_static.
    Eigen::Matrix3d R_sensor_imu_{Eigen::Matrix3d::Identity()};
    bool imuExtrinsicReady_{false};

    std::mutex dynamicTrailerMutex_;
    double latestDynamicTrailerAngleRad_{0.0};
    rclcpp::Time latestDynamicTrailerAngleTime_{0, 0, RCL_ROS_TIME};
    bool hasDynamicTrailerAngle_{false};

    static double yawFromTransform(const PM::TransformationParameters& transform)
    {
        return std::atan2(
            static_cast<double>(transform(1, 0)),
            static_cast<double>(transform(0, 0)));
    }

    static double wrapToPi(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    static std::string transformSummary(const PM::TransformationParameters& transform)
    {
        const int dim = static_cast<int>(transform.rows()) - 1;
        std::ostringstream ss;
        ss << "x=" << transform(0, dim)
           << " y=" << transform(1, dim);
        if (dim >= 3) {
            ss << " z=" << transform(2, dim);
        }
        ss << " yaw_deg=" << yawFromTransform(transform) * 180.0 / M_PI;
        return ss.str();
    }

    bool getFreshDynamicTrailerAngle(const rclcpp::Time& scanStamp, double& angleRad, double& ageS)
    {
        if (!params->enableDynamicTrailerSelfFilter) return false;
        std::lock_guard<std::mutex> lk(dynamicTrailerMutex_);
        if (!hasDynamicTrailerAngle_) return false;
        angleRad = latestDynamicTrailerAngleRad_;
        ageS = std::abs((scanStamp - latestDynamicTrailerAngleTime_).seconds());
        return ageS <= params->dynamicTrailerStaleTimeoutS;
    }

    int removeDynamicTrailerPoints(PM::DataPoints& cloud, const rclcpp::Time& scanStamp)
    {
        if (!params->enableDynamicTrailerSelfFilter) return 0;

        double phi = 0.0;
        double ageS = 0.0;
        if (!getFreshDynamicTrailerAngle(scanStamp, phi, ageS))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[SELF-FILTER] Dynamic trailer OBB skipped: no fresh articulation on %s "
                "(timeout %.2fs). Static YAML bbox still applies.",
                params->dynamicTrailerArticulationTopic.c_str(),
                params->dynamicTrailerStaleTimeoutS);
            return 0;
        }

        const int nbPoints = static_cast<int>(cloud.getNbPoints());
        if (nbPoints <= 0 || cloud.getEuclideanDim() < 3) return 0;

        const double yaw = params->dynamicTrailerYawOffsetRad + params->dynamicTrailerYawSign * phi;
        const double ux = std::cos(yaw);
        const double uy = std::sin(yaw);
        const double lx = -uy;
        const double ly = ux;
        const double hx = params->dynamicTrailerHitchX;
        const double hy = params->dynamicTrailerHitchY;
        const double sMin = params->dynamicTrailerFrontOffsetM;
        const double sMax = params->dynamicTrailerRearOffsetM;
        const double halfWidth = params->dynamicTrailerHalfWidthM;
        const double zMin = params->dynamicTrailerZMinM;
        const double zMax = params->dynamicTrailerZMaxM;

        int writeCol = 0;
        int removed = 0;
        for (int readCol = 0; readCol < nbPoints; ++readCol)
        {
            const double dx = static_cast<double>(cloud.features(0, readCol)) - hx;
            const double dy = static_cast<double>(cloud.features(1, readCol)) - hy;
            const double z = static_cast<double>(cloud.features(2, readCol));
            const double s = dx * ux + dy * uy;
            const double l = dx * lx + dy * ly;
            const bool inside =
                s >= sMin && s <= sMax &&
                std::abs(l) <= halfWidth &&
                z >= zMin && z <= zMax;

            if (inside)
            {
                ++removed;
            }
            else
            {
                if (writeCol != readCol)
                {
                    cloud.setColFrom(writeCol, cloud, readCol);
                }
                ++writeCol;
            }
        }
        cloud.conservativeResize(writeCol);

        if (removed > 0)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[SELF-FILTER] dynamic trailer removed=%d/%d phi=%.3f rad age=%.3fs yaw=%.1f deg",
                removed, nbPoints, phi, ageS, yaw * 180.0 / M_PI);
        }
        return removed;
    }

    // Publish per-scan status to /mapping/status (diagnostic_msgs/DiagnosticStatus).
    // No-op when no subscribers — avoids allocation overhead on the ICP hot path.
    void publishScanStatus(
        const rclcpp::Time& stamp,
        bool accepted,
        const std::string& reason,
        int input_points,
        float translation_m,
        float rotation_deg,
        float reg_ms,
        float dt_since_accepted_s,
        bool turn_recovery = false)
    {
        if (statusPublisher->get_subscription_count() == 0) return;

        using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
        using KV = diagnostic_msgs::msg::KeyValue;

        DiagStatus msg;
        msg.level   = accepted ? DiagStatus::OK : DiagStatus::WARN;
        msg.name    = "mapper_scan";
        msg.message = reason.empty() ? "accepted" : reason;
        msg.hardware_id = "norlab_icp_mapper";

        auto kv = [](const char* k, const std::string& v) {
            KV pair;
            pair.key   = k;
            pair.value = v;
            return pair;
        };

        msg.values.push_back(kv("stamp_s",              std::to_string(static_cast<double>(stamp.nanoseconds()) * 1e-9)));
        msg.values.push_back(kv("accepted",              accepted ? "1" : "0"));
        msg.values.push_back(kv("input_points",          std::to_string(input_points)));
        msg.values.push_back(kv("translation_m",         std::to_string(translation_m)));
        msg.values.push_back(kv("rotation_deg",          std::to_string(rotation_deg)));
        msg.values.push_back(kv("registration_ms",       std::to_string(reg_ms)));
        msg.values.push_back(kv("turn_recovery",         turn_recovery ? "1" : "0"));
        msg.values.push_back(kv("consecutive_rejections",std::to_string(consecutiveRejections_)));
        msg.values.push_back(kv("dt_since_accepted_s",   std::to_string(dt_since_accepted_s)));
        msg.values.push_back(kv("scans_accepted",        std::to_string(scansAccepted_.load())));
        msg.values.push_back(kv("scans_rejected",        std::to_string(scansRejected_.load())));
        msg.values.push_back(kv("map_points",            std::to_string(mapper->getMap().getNbPoints())));

        statusPublisher->publish(msg);
    }

    struct VoxelKey
    {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const VoxelKey& other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash
    {
        std::size_t operator()(const VoxelKey& key) const noexcept
        {
            std::size_t seed = 0;
            const auto combine = [&seed](int value)
            {
                const std::size_t h = std::hash<int>{}(value);
                seed ^= h + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
            };
            combine(key.x);
            combine(key.y);
            combine(key.z);
            return seed;
        }
    };

    struct MapOverlapStats
    {
        int sampled{0};
        int nearHits{0};
        int looseHits{0};
        double nearRatio{1.0};
        double looseRatio{1.0};
    };

    struct MapOverlapVoxelCache
    {
        uint64_t mapVersion{0};
        int mapPoints{0};
        std::unordered_set<VoxelKey, VoxelKeyHash> nearVoxels;
        std::unordered_set<VoxelKey, VoxelKeyHash> looseVoxels;
    };

    mutable std::mutex overlapVoxelCacheMutex_;
    mutable MapOverlapVoxelCache overlapVoxelCache_;

    static bool pointFinite(const PM::DataPoints& cloud, const int col)
    {
        const int dim = cloud.getEuclideanDim();
        for (int row = 0; row < dim; ++row)
        {
            if (!std::isfinite(static_cast<double>(cloud.features(row, col))))
            {
                return false;
            }
        }
        return true;
    }

    static VoxelKey voxelKeyForPoint(const PM::DataPoints& cloud, const int col, const double voxelSize)
    {
        const int dim = cloud.getEuclideanDim();
        const auto quantize = [voxelSize](const float value)
        {
            return static_cast<int>(std::floor(static_cast<double>(value) / voxelSize));
        };
        return VoxelKey{
            quantize(cloud.features(0, col)),
            dim >= 2 ? quantize(cloud.features(1, col)) : 0,
            dim >= 3 ? quantize(cloud.features(2, col)) : 0};
    }

    static std::unordered_set<VoxelKey, VoxelKeyHash> buildVoxelSet(
        const PM::DataPoints& cloud,
        const double voxelSize,
        const int targetSamples = 80000)
    {
        std::unordered_set<VoxelKey, VoxelKeyHash> voxels;
        const int nbPoints = static_cast<int>(cloud.getNbPoints());
        const int stride = std::max(1, nbPoints / std::max(1, targetSamples));
        const int reservePoints = (nbPoints + stride - 1) / stride;
        voxels.reserve(static_cast<std::size_t>(reservePoints) * 2U);
        for (int col = 0; col < nbPoints; col += stride)
        {
            if (pointFinite(cloud, col))
            {
                voxels.insert(voxelKeyForPoint(cloud, col, voxelSize));
            }
        }
        return voxels;
    }

    static bool voxelNeighborhoodOccupied(
        const std::unordered_set<VoxelKey, VoxelKeyHash>& voxels,
        const VoxelKey& key)
    {
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dz = -1; dz <= 1; ++dz)
                {
                    if (voxels.find(VoxelKey{key.x + dx, key.y + dy, key.z + dz}) != voxels.end())
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void setMapperMap(PM::DataPoints map)
    {
        mapper->setMap(map);
        mapVersion_.fetch_add(1, std::memory_order_relaxed);
    }

    static PM::TransformationParameters yawOnlyRobotPose(
        const PM::TransformationParameters& robotToMap,
        const double z)
    {
        PM::TransformationParameters constrained =
            PM::TransformationParameters::Identity(robotToMap.rows(), robotToMap.cols());
        const int dim = static_cast<int>(robotToMap.rows()) - 1;
        const double yaw = yawFromTransform(robotToMap);
        constrained(0, 0) = static_cast<float>(std::cos(yaw));
        constrained(0, 1) = static_cast<float>(-std::sin(yaw));
        constrained(1, 0) = static_cast<float>(std::sin(yaw));
        constrained(1, 1) = static_cast<float>(std::cos(yaw));
        constrained(0, dim) = robotToMap(0, dim);
        constrained(1, dim) = robotToMap(1, dim);
        if (dim >= 3)
        {
            constrained(2, 2) = 1.0f;
            constrained(2, dim) = static_cast<float>(z);
        }
        return constrained;
    }

    bool maybeConstrainPlanarPose(
        PM::TransformationParameters& sensorToMap,
        const PM::TransformationParameters& robotToSensor,
        const PM::TransformationParameters& odomPredictedRobotToMap,
        const char* context)
    {
        if (!params->is3D)
        {
            return true;
        }

        const int dim = static_cast<int>(sensorToMap.rows()) - 1;
        PM::TransformationParameters robotToMap = sensorToMap * robotToSensor;
        const double referenceZ = hasInitialAcceptedRobotToMap_
            ? static_cast<double>(initialAcceptedRobotToMap_(2, dim))
            : static_cast<double>(odomPredictedRobotToMap(2, dim));
        const double zDrift = std::abs(static_cast<double>(robotToMap(2, dim)) - referenceZ);

        if (!params->enablePlanarPoseConstraint)
        {
            if (hasInitialAcceptedRobotToMap_ && zDrift > params->planarPoseMaxZDriftM)
            {
                RCLCPP_WARN(this->get_logger(),
                    "Rejecting non-planar pose in %s: z_drift=%.3fm > %.3fm pose={%s}",
                    context,
                    zDrift,
                    params->planarPoseMaxZDriftM,
                    transformSummary(robotToMap).c_str());
                return false;
            }
            return true;
        }

        const double zBefore = static_cast<double>(robotToMap(2, dim));
        PM::TransformationParameters constrainedRobotToMap =
            yawOnlyRobotPose(robotToMap, referenceZ);
        sensorToMap = constrainedRobotToMap * robotToSensor.inverse();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[PLANAR] %s constrained pose: z %.3f -> %.3f yaw=%.1fdeg",
            context,
            zBefore,
            referenceZ,
            yawFromTransform(constrainedRobotToMap) * 180.0 / M_PI);
        return true;
    }

    double latestImuYawRateDegS(const rclcpp::Time& stamp)
    {
        if (params->deskewImuTopic.empty())
        {
            return 0.0;
        }

        std::lock_guard<std::mutex> lk(imuDeskewBufMutex_);
        if (imuDeskewBuf_.empty())
        {
            return 0.0;
        }

        const int64_t target = stamp.nanoseconds();
        double bestAbsRate = 0.0;
        int64_t bestDt = std::numeric_limits<int64_t>::max();
        for (const auto& sample : imuDeskewBuf_)
        {
            const int64_t dt = std::abs(sample.first - target);
            if (dt < bestDt)
            {
                bestDt = dt;
                bestAbsRate = std::abs(sample.second.z()) * 180.0 / M_PI;
            }
        }
        // Ignore stale IMU when replay timing leaves a gap.
        return bestDt <= 250'000'000LL ? bestAbsRate : 0.0;
    }

    MotionState estimateMotionState(
        const PM::TransformationParameters& odomPredictedRobotToMap,
        const rclcpp::Time& stamp,
        const double dtSecAccepted)
    {
        MotionState motion;
        motion.dtAcceptedS = dtSecAccepted;
        if (!hasInitialAcceptedRobotToMap_ || dtSecAccepted <= 1e-6)
        {
            motion.imuYawRateDegS = latestImuYawRateDegS(stamp);
            motion.dominantYawRateDegS = motion.imuYawRateDegS;
            return motion;
        }

        const int dim = static_cast<int>(odomPredictedRobotToMap.rows()) - 1;
        const Eigen::VectorXf odomDelta =
            odomPredictedRobotToMap.topRightCorner(dim, 1) -
            lastAcceptedRobotToMap_.topRightCorner(dim, 1);
        const double xy = std::hypot(static_cast<double>(odomDelta(0)), static_cast<double>(odomDelta(1)));
        const double dt = std::max(1e-4, std::min(dtSecAccepted, params->adaptiveMaxDtS));
        motion.odomSpeedMs = xy / dt;
        if (hasLastOdomPriorSpeed_)
        {
            motion.odomAccelMs2 =
                std::abs(motion.odomSpeedMs - lastOdomPriorSpeedMs_) / dt;
        }
        const double yawStepDeg =
            std::abs(wrapToPi(
                yawFromTransform(odomPredictedRobotToMap) -
                yawFromTransform(lastAcceptedRobotToMap_))) * 180.0 / M_PI;
        motion.odomYawRateDegS = yawStepDeg / dt;
        motion.imuYawRateDegS = latestImuYawRateDegS(stamp);
        motion.dominantYawRateDegS = std::max(motion.odomYawRateDegS, motion.imuYawRateDegS);
        motion.aggressive =
            motion.odomSpeedMs >= params->aggressiveSpeedMs ||
            motion.dominantYawRateDegS >= params->aggressiveYawRateDegS ||
            motion.odomAccelMs2 >= params->aggressiveSpeedMs;
        motion.pivot =
            motion.odomSpeedMs <= params->pivotLinearSpeedMs &&
            motion.dominantYawRateDegS >= params->pivotYawRateDegS;
        return motion;
    }

    AdaptiveGateLimits adaptiveGateLimits(
        const MotionState& motion,
        const double dtSecAccepted) const
    {
        AdaptiveGateLimits limits;
        const double dt = std::max(0.0, std::min(dtSecAccepted, params->adaptiveMaxDtS));
        limits.translationM =
            params->maxTranslationCorrection +
            params->adaptiveVelocityGain * motion.odomSpeedMs * dt +
            params->adaptiveAccelerationGain * motion.odomAccelMs2 * dt * dt;
        limits.rotationDeg =
            params->maxRotationCorrectionDeg +
            params->adaptiveYawRateGain * motion.dominantYawRateDegS * dt;
        limits.adaptive = params->enableMotionAdaptiveGate;
        limits.pivot = motion.pivot;
        if (motion.pivot)
        {
            limits.translationM =
                std::min(limits.translationM, params->pivotMaxTranslationCorrectionM);
            limits.rotationDeg =
                std::max(limits.rotationDeg, params->maxRotationCorrectionDeg);
        }
        return limits;
    }

    bool adaptiveGateAccepts(
        const RegistrationQualityGate::Result& qgResult,
        const MotionState& motion,
        const double dtSecAccepted,
        std::string& reason) const
    {
        if (!params->enableMotionAdaptiveGate)
        {
            return false;
        }
        const bool rejectionIsFundamental =
            qgResult.rejection_reason.find("few") != std::string::npos ||
            qgResult.rejection_reason.find("NaN") != std::string::npos ||
            qgResult.rejection_reason.find("Inf") != std::string::npos ||
            qgResult.rejection_reason.find("too long") != std::string::npos;
        if (rejectionIsFundamental)
        {
            return false;
        }

        const AdaptiveGateLimits limits = adaptiveGateLimits(motion, dtSecAccepted);
        const bool withinTranslation =
            qgResult.translation_correction_m <= limits.translationM;
        const bool withinRotation =
            qgResult.rotation_correction_deg <= limits.rotationDeg;
        if (withinTranslation && withinRotation)
        {
            std::ostringstream ss;
            ss << "adaptive gate accepted: correction="
               << qgResult.translation_correction_m << "m/"
               << qgResult.rotation_correction_deg << "deg within limits "
               << limits.translationM << "m/" << limits.rotationDeg
               << "deg motion speed=" << motion.odomSpeedMs
               << "m/s accel=" << motion.odomAccelMs2
               << "m/s2 yaw_rate=" << motion.dominantYawRateDegS
               << "deg/s pivot=" << motion.pivot;
            reason = ss.str();
            return true;
        }

        std::ostringstream ss;
        ss << "adaptive gate rejected: correction="
           << qgResult.translation_correction_m << "m/"
           << qgResult.rotation_correction_deg << "deg > limits "
           << limits.translationM << "m/" << limits.rotationDeg
           << "deg motion speed=" << motion.odomSpeedMs
           << "m/s accel=" << motion.odomAccelMs2
           << "m/s2 yaw_rate=" << motion.dominantYawRateDegS
           << "deg/s pivot=" << motion.pivot;
        reason = ss.str();
        return false;
    }

    bool saveGoodMapSnapshot(
        const PM::TransformationParameters& sensorToMap,
        const PM::TransformationParameters& robotToMap,
        const RegistrationQualityGate::Result& qgResult,
        const rclcpp::Time& stamp,
        const char* reason)
    {
        if (qgResult.translation_correction_m > params->snapshotMaxTranslationCorrectionM ||
            qgResult.rotation_correction_deg > params->snapshotMaxRotationCorrectionDeg)
        {
            return false;
        }
        const uint64_t accepted = scansAccepted_.load();
        if (lastGoodMapSnapshot_.valid &&
            accepted - lastGoodMapSnapshot_.acceptedScan <
                static_cast<uint64_t>(params->snapshotSaveIntervalScans))
        {
            return false;
        }

        PM::DataPoints snapshotMap = mapper->getMap();
        if (snapshotMap.getNbPoints() == 0)
        {
            return false;
        }

        lastGoodMapSnapshot_.map = std::move(snapshotMap);
        lastGoodMapSnapshot_.sensorToMap = sensorToMap;
        lastGoodMapSnapshot_.robotToMap = robotToMap;
        lastGoodMapSnapshot_.stamp = stamp;
        lastGoodMapSnapshot_.acceptedScan = accepted;
        lastGoodMapSnapshot_.valid = true;
        RCLCPP_INFO(this->get_logger(),
            "[SNAPSHOT] saved reason=%s scan=%lu map_pts=%d correction=%.3fm/%.1fdeg pose={%s}",
            reason,
            static_cast<unsigned long>(accepted),
            static_cast<int>(lastGoodMapSnapshot_.map.getNbPoints()),
            qgResult.translation_correction_m,
            qgResult.rotation_correction_deg,
            transformSummary(robotToMap).c_str());
        return true;
    }

    bool recoveryDue() const
    {
        if (!params->enableMapRecovery || params->recoveryReloadAfterRejections <= 0)
        {
            return false;
        }
        if (consecutiveRejections_ < params->recoveryReloadAfterRejections)
        {
            return false;
        }
        if (lastRecoveryAttemptRejections_ < 0)
        {
            return true;
        }
        return consecutiveRejections_ - lastRecoveryAttemptRejections_ >=
            params->recoveryAttemptIntervalScans;
    }

    bool recoverLocalMapForPrior(
        const PM::TransformationParameters& odomPredictedRobotToMap,
        const PM::TransformationParameters& sensorToMapPrior,
        const char* trigger)
    {
        if (!recoveryDue())
        {
            return false;
        }

        lastRecoveryAttemptRejections_ = consecutiveRejections_;
        const Eigen::Vector2f centerXY = odomPredictedRobotToMap.topRightCorner(2, 1);

        if (params->enableGlobalOutputMap)
        {
            PM::DataPoints sourceMap;
            {
                std::lock_guard<std::mutex> lock(globalOutputMapMutex_);
                if (hasGlobalOutputMap_ && globalOutputMap_.getNbPoints() > 0)
                {
                    sourceMap = globalOutputMap_;
                }
            }

            if (sourceMap.getNbPoints() > 0)
            {
                PM::DataPoints recoveredMap = cropPointsToRadius(
                    sourceMap,
                    centerXY,
                    static_cast<float>(params->recoveryLocalMapRadiusM));
                const int croppedPts = static_cast<int>(recoveredMap.getNbPoints());
                capCloudPointsDeterministically(recoveredMap, params->recoveryLocalMapMaxPoints);
                const int recoveredPts = static_cast<int>(recoveredMap.getNbPoints());
                if (recoveredPts >= params->recoveryLocalMapMinPoints)
                {
                    normalizeMapNormals(recoveredMap);
                    setMapperMap(recoveredMap);
                    lastDeterministicMapUpdatePose_ = sensorToMapPrior;
                    hasDeterministicMapUpdatePose_ = true;
                    RCLCPP_WARN(this->get_logger(),
                        "[RECOVERY] rebuilt local map from global output: trigger=%s rejects=%d "
                        "global_pts=%d cropped_pts=%d local_pts=%d cap=%d radius=%.1fm center=(%.2f,%.2f) prior={%s}",
                        trigger,
                        consecutiveRejections_,
                        static_cast<int>(sourceMap.getNbPoints()),
                        croppedPts,
                        recoveredPts,
                        params->recoveryLocalMapMaxPoints,
                        params->recoveryLocalMapRadiusM,
                        static_cast<double>(centerXY(0)),
                        static_cast<double>(centerXY(1)),
                        transformSummary(sensorToMapPrior).c_str());
                    return true;
                }
                RCLCPP_WARN(this->get_logger(),
                    "[RECOVERY] global crop too small: trigger=%s pts=%d < %d radius=%.1fm center=(%.2f,%.2f)",
                    trigger,
                    recoveredPts,
                    params->recoveryLocalMapMinPoints,
                    params->recoveryLocalMapRadiusM,
                    static_cast<double>(centerXY(0)),
                    static_cast<double>(centerXY(1)));
            }
        }

        if (lastGoodMapSnapshot_.valid && lastGoodMapSnapshot_.map.getNbPoints() > 0)
        {
            PM::DataPoints recoveredMap = lastGoodMapSnapshot_.map;
            setMapperMap(recoveredMap);
            lastDeterministicMapUpdatePose_ = lastGoodMapSnapshot_.sensorToMap;
            hasDeterministicMapUpdatePose_ = true;
            RCLCPP_WARN(this->get_logger(),
                "[RECOVERY] reloaded last-good local map: trigger=%s rejects=%d snapshot_scan=%lu "
                "map_pts=%d snapshot_pose={%s} current_prior={%s}",
                trigger,
                consecutiveRejections_,
                static_cast<unsigned long>(lastGoodMapSnapshot_.acceptedScan),
                static_cast<int>(lastGoodMapSnapshot_.map.getNbPoints()),
                transformSummary(lastGoodMapSnapshot_.robotToMap).c_str(),
                transformSummary(sensorToMapPrior).c_str());
            return true;
        }

        RCLCPP_WARN(this->get_logger(),
            "[RECOVERY] failed: no usable global crop or last-good snapshot. trigger=%s rejects=%d",
            trigger,
            consecutiveRejections_);
        return false;
    }

    static PM::TransformationParameters yawOffsetPrior(
        const PM::TransformationParameters& prior,
        const double yawOffsetRad)
    {
        PM::TransformationParameters candidate = prior;
        const int dim = static_cast<int>(prior.rows()) - 1;
        PM::TransformationParameters yawOffset =
            PM::TransformationParameters::Identity(prior.rows(), prior.cols());
        yawOffset(0, 0) = static_cast<float>(std::cos(yawOffsetRad));
        yawOffset(0, 1) = static_cast<float>(-std::sin(yawOffsetRad));
        yawOffset(1, 0) = static_cast<float>(std::sin(yawOffsetRad));
        yawOffset(1, 1) = static_cast<float>(std::cos(yawOffsetRad));
        candidate.topLeftCorner(dim, dim) =
            yawOffset.topLeftCorner(dim, dim) * prior.topLeftCorner(dim, dim);
        return candidate;
    }

    bool tryRecoveryYawHypotheses(
        PM::DataPoints& input,
        const PM::TransformationParameters& sensorToMapPrior,
        const std::chrono::time_point<std::chrono::steady_clock>& steadyTs,
        PM::TransformationParameters& bestSensorToMap,
        double& bestCost,
        const char* trigger)
    {
        if (!params->enableMapRecovery ||
            consecutiveRejections_ < params->recoveryReloadAfterRejections ||
            mapper->getMap().getNbPoints() == 0)
        {
            return false;
        }

        const double offsetsDeg[] = {0.0, 5.0, -5.0, 10.0, -10.0, 20.0, -20.0};
        bool found = false;
        bestCost = std::numeric_limits<double>::infinity();
        for (const double offsetDeg : offsetsDeg)
        {
            PM::TransformationParameters candidatePrior =
                yawOffsetPrior(sensorToMapPrior, offsetDeg * M_PI / 180.0);
            try
            {
                mapper->processInput(input, candidatePrior, steadyTs);
                PM::TransformationParameters candidatePose = mapper->getPose();
                PM::DataPoints inputInMapFrame =
                    transformation->compute(input, candidatePose);
                const MapOverlapStats overlap =
                    estimateMapOverlap(inputInMapFrame, mapper->getMap());
                const Eigen::MatrixXf correction = candidatePose * sensorToMapPrior.inverse();
                const int dim = static_cast<int>(candidatePose.rows()) - 1;
                const double tr = correction.topRightCorner(dim, 1).norm();
                const double yawResidualDeg =
                    std::abs(wrapToPi(
                        yawFromTransform(candidatePose) -
                        yawFromTransform(sensorToMapPrior))) * 180.0 / M_PI;
                const double cost =
                    8.0 * (1.0 - overlap.looseRatio) +
                    4.0 * (1.0 - overlap.nearRatio) +
                    0.25 * tr +
                    0.02 * yawResidualDeg +
                    0.01 * std::abs(offsetDeg);
                if (overlap.sampled >= minMapOverlapSamples_ &&
                    overlap.looseRatio >= std::max(0.25, params->minPoseOverlapLooseRatio - 0.20) &&
                    cost < bestCost)
                {
                    bestCost = cost;
                    bestSensorToMap = candidatePose;
                    found = true;
                }
                RCLCPP_INFO(this->get_logger(),
                    "[RECOVERY] hypothesis trigger=%s yaw_offset=%.1fdeg overlap=%.3f/%.3f tr=%.2fm yaw_res=%.1fdeg cost=%.3f accepted_candidate=%d",
                    trigger,
                    offsetDeg,
                    overlap.nearRatio,
                    overlap.looseRatio,
                    tr,
                    yawResidualDeg,
                    cost,
                    found && bestCost == cost);
            }
            catch (const std::exception& e)
            {
                RCLCPP_DEBUG(this->get_logger(),
                    "[RECOVERY] hypothesis failed: trigger=%s yaw_offset=%.1fdeg error=%s",
                    trigger,
                    offsetDeg,
                    e.what());
            }
        }

        if (found)
        {
            RCLCPP_WARN(this->get_logger(),
                "[RECOVERY] selected yaw hypothesis: trigger=%s cost=%.3f pose={%s}",
                trigger,
                bestCost,
                transformSummary(bestSensorToMap).c_str());
        }
        return found;
    }

    const MapOverlapVoxelCache& overlapCacheForMap(const PM::DataPoints& currentMap) const
    {
        const uint64_t version = mapVersion_.load(std::memory_order_relaxed);
        const int mapPoints = static_cast<int>(currentMap.getNbPoints());
        std::lock_guard<std::mutex> lock(overlapVoxelCacheMutex_);
        if (overlapVoxelCache_.mapVersion != version ||
            overlapVoxelCache_.mapPoints != mapPoints)
        {
            const auto cacheStart = std::chrono::steady_clock::now();
            overlapVoxelCache_.nearVoxels =
                buildVoxelSet(currentMap, mapOverlapNearVoxelM_);
            overlapVoxelCache_.looseVoxels =
                buildVoxelSet(currentMap, mapOverlapLooseVoxelM_);
            overlapVoxelCache_.mapVersion = version;
            overlapVoxelCache_.mapPoints = mapPoints;
            const double cacheMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - cacheStart).count();
            RCLCPP_DEBUG(this->get_logger(),
                "Rebuilt map-overlap voxel cache: version=%lu map_pts=%d near_voxels=%zu loose_voxels=%zu time=%.1fms",
                static_cast<unsigned long>(version),
                mapPoints,
                overlapVoxelCache_.nearVoxels.size(),
                overlapVoxelCache_.looseVoxels.size(),
                cacheMs);
        }
        return overlapVoxelCache_;
    }

    MapOverlapStats estimateMapOverlap(
        const PM::DataPoints& scanInMapFrame,
        const PM::DataPoints& currentMap) const
    {
        MapOverlapStats stats;
        if (scanInMapFrame.getNbPoints() == 0 || currentMap.getNbPoints() == 0)
        {
            stats.nearRatio = 0.0;
            stats.looseRatio = 0.0;
            return stats;
        }

        const MapOverlapVoxelCache& cache = overlapCacheForMap(currentMap);
        const int targetSamples = 1500;
        const int scanPoints = static_cast<int>(scanInMapFrame.getNbPoints());
        const int stride = std::max(1, scanPoints / targetSamples);

        for (int col = 0; col < scanPoints; col += stride)
        {
            if (!pointFinite(scanInMapFrame, col))
            {
                continue;
            }

            ++stats.sampled;
            if (voxelNeighborhoodOccupied(
                    cache.nearVoxels,
                    voxelKeyForPoint(scanInMapFrame, col, mapOverlapNearVoxelM_)))
            {
                ++stats.nearHits;
            }
            if (voxelNeighborhoodOccupied(
                    cache.looseVoxels,
                    voxelKeyForPoint(scanInMapFrame, col, mapOverlapLooseVoxelM_)))
            {
                ++stats.looseHits;
            }
        }

        if (stats.sampled > 0)
        {
            stats.nearRatio = static_cast<double>(stats.nearHits) / static_cast<double>(stats.sampled);
            stats.looseRatio = static_cast<double>(stats.looseHits) / static_cast<double>(stats.sampled);
        }
        else
        {
            stats.nearRatio = 0.0;
            stats.looseRatio = 0.0;
        }
        return stats;
    }

    bool mapOverlapTooLow(const MapOverlapStats& stats) const
    {
        return stats.sampled >= minMapOverlapSamples_ &&
               (stats.nearRatio < params->minMapOverlapNearRatio ||
                stats.looseRatio < params->minMapOverlapLooseRatio);
    }

    bool poseOverlapTooLow(const MapOverlapStats& stats) const
    {
        if (stats.sampled < minMapOverlapSamples_)
        {
            return false;
        }

        const bool looseGood =
            stats.looseRatio >= params->minPoseOverlapLooseRatio;
        const bool strongNear =
            stats.nearRatio >= std::min(1.0, params->minPoseOverlapNearRatio + 0.12);
        const bool nearGoodAndAlmostLoose =
            stats.nearRatio >= params->minPoseOverlapNearRatio &&
            stats.looseRatio >= std::max(0.0, params->minPoseOverlapLooseRatio - 0.08);

        // Pose publication must be less brittle than map insertion. During
        // articulated turns or right after a trim, the loose ratio can dip
        // while the local/near geometry is still consistent. Rejecting the
        // pose freezes map->odom, lets the wheel prior run away from the local
        // map, and causes a permanent no-match cascade.
        return !(looseGood || strongNear || nearGoodAndAlmostLoose);
    }

    bool registrationPoseOverlapsCurrentMap(
        const PM::DataPoints& inputInSensorFrame,
        const PM::TransformationParameters& acceptedSensorToMap,
        MapOverlapStats& overlap,
        double& overlapMs)
    {
        PM::DataPoints currentMap = mapper->getMap();
        if (currentMap.getNbPoints() == 0) {
            overlap = {};
            overlapMs = 0.0;
            return true;
        }

        const auto overlapStart = std::chrono::steady_clock::now();
        PM::DataPoints inputInMapFrame =
            transformation->compute(inputInSensorFrame, acceptedSensorToMap);
        overlap = estimateMapOverlap(inputInMapFrame, currentMap);
        overlapMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - overlapStart).count();
        return !poseOverlapTooLow(overlap);
    }

    // ── IMU deskew helpers ──

    // Look up the static rotation from the IMU frame to the sensor/LiDAR frame.
    // Called lazily on first scan to avoid TF-static race at startup.
    // Returns true and sets R_sensor_imu_ on success; false leaves previous value.
    bool tryLookupImuSensorExtrinsic(const std::string& sensorFrame)
    {
        try
        {
            geometry_msgs::msg::TransformStamped tf =
                tfBuffer->lookupTransform(sensorFrame, params->deskewImuFrame, rclcpp::Time(0));
            const auto& q = tf.transform.rotation;
            R_sensor_imu_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
            imuExtrinsicReady_ = true;
            RCLCPP_INFO(this->get_logger(),
                "[IMU deskew] %s → %s extrinsic acquired (R_sensor_imu norm check: %.4f).",
                params->deskewImuFrame.c_str(), sensorFrame.c_str(),
                R_sensor_imu_.determinant());
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[IMU deskew] Cannot look up %s → %s: %s. "
                "Deskew will retry on the next scan.",
                params->deskewImuFrame.c_str(), sensorFrame.c_str(), ex.what());
            return false;
        }
    }

    // ── publishAlignedScan ──

    void publishAlignedScan(
        const PM::DataPoints& inputInSensorFrame,
        const PM::TransformationParameters& acceptedSensorToMap,
        const rclcpp::Time& timeStamp)
    {
        if (alignedScanPublisher->get_subscription_count() == 0)
        {
            return;
        }

        PM::DataPoints inputInMapFrame =
            transformation->compute(inputInSensorFrame, acceptedSensorToMap);
        sensor_msgs::msg::PointCloud2 alignedMsg =
            PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(
                inputInMapFrame, params->mapFrame, timeStamp);
        alignedScanPublisher->publish(alignedMsg);
    }

    bool mapUpdateMotionSafe(
        const PM::TransformationParameters& acceptedSensorToMap,
        std::string& reason) const
    {
        if (!hasDeterministicMapUpdatePose_)
        {
            return true;
        }

        const int dim = static_cast<int>(acceptedSensorToMap.rows()) - 1;
        const double yawStepDeg =
            std::abs(wrapToPi(
                yawFromTransform(acceptedSensorToMap) -
                yawFromTransform(lastDeterministicMapUpdatePose_))) * 180.0 / M_PI;
        if (yawStepDeg > maxMapUpdateYawStepDeg_)
        {
            std::ostringstream ss;
            ss << "yaw step " << yawStepDeg << "deg > " << maxMapUpdateYawStepDeg_ << "deg";
            reason = ss.str();
            return false;
        }

        if (dim >= 3)
        {
            const double zStep = std::abs(
                static_cast<double>(acceptedSensorToMap(2, dim) - lastDeterministicMapUpdatePose_(2, dim)));
            if (zStep > maxMapUpdateZStepM_)
            {
                std::ostringstream ss;
                ss << "z step " << zStep << "m > " << maxMapUpdateZStepM_ << "m";
                reason = ss.str();
                return false;
            }
        }

        return true;
    }

    void ensureInputNormals(PM::DataPoints& input)
    {
        if (!params->is3D) {
            return;
        }

        const unsigned normalDim = static_cast<unsigned>(input.getEuclideanDim());
        if (input.descriptorExists("normals", normalDim)) {
            RCLCPP_DEBUG(this->get_logger(),
                "Input normals already present: pts=%d dim=%u descriptors=%ldx%ld",
                static_cast<int>(input.getNbPoints()), normalDim,
                static_cast<long>(input.descriptors.rows()),
                static_cast<long>(input.descriptors.cols()));
            return;
        }
        if (input.descriptorExists("normals")) {
            input.removeDescriptor("normals");
        }

        const auto normalStart = std::chrono::steady_clock::now();
        inputSurfaceNormalFilter_->inPlaceFilter(input);
        const double normalMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - normalStart).count();

        RCLCPP_DEBUG(this->get_logger(),
            "Computed input normals: pts=%d dim=%u normal_ok=%d time=%.1fms descriptors=%ldx%ld",
            static_cast<int>(input.getNbPoints()), normalDim,
            input.descriptorExists("normals", normalDim),
            normalMs,
            static_cast<long>(input.descriptors.rows()),
            static_cast<long>(input.descriptors.cols()));
    }

    void recomputeMapNormals(PM::DataPoints& map)
    {
        if (!params->is3D || map.getNbPoints() == 0) {
            return;
        }

        const unsigned normalDim = static_cast<unsigned>(map.getEuclideanDim());
        if (map.descriptorExists("normals")) {
            map.removeDescriptor("normals");
        }
        mapSurfaceNormalFilter_->inPlaceFilter(map);
        if (!map.descriptorExists("normals", normalDim)) {
            RCLCPP_WARN(this->get_logger(),
                "Map normal recomputation did not create a valid normals descriptor.");
        }
    }

    void normalizeMapNormals(PM::DataPoints& map)
    {
        const unsigned normalDim = static_cast<unsigned>(map.getEuclideanDim());
        if (!map.descriptorExists("normals", normalDim)) {
            recomputeMapNormals(map);
            return;
        }

        auto normals = map.getDescriptorViewByName("normals");
        for (int col = 0; col < normals.cols(); ++col) {
            const float norm = normals.col(col).norm();
            if (norm > 1e-6f) {
                normals.col(col) /= norm;
            }
        }
    }

    bool deterministicMapUpdateDue(const PM::TransformationParameters& sensorToMap) const
    {
        if (forceNextMapUpdate_) {
            return true;
        }
        if (!hasDeterministicMapUpdatePose_) {
            return true;
        }

        const int dim = static_cast<int>(sensorToMap.rows()) - 1;
        const Eigen::VectorXf delta =
            sensorToMap.topRightCorner(dim, 1) -
            lastDeterministicMapUpdatePose_.topRightCorner(dim, 1);
        const double dist = delta.norm();
        const double yawStepDeg =
            std::abs(wrapToPi(
                yawFromTransform(sensorToMap) -
                yawFromTransform(lastDeterministicMapUpdatePose_))) * 180.0 / M_PI;

        return dist >= params->deterministicMapUpdateDistanceM ||
               yawStepDeg >= params->deterministicMapUpdateYawDeg;
    }

    bool updateMapDeterministically(
        const PM::DataPoints& inputInSensorFrame,
        const PM::TransformationParameters& acceptedSensorToMap)
    {
        if (hasDeterministicMapUpdatePose_ &&
            !deterministicMapUpdateDue(acceptedSensorToMap)) {
            return false;
        }

        const auto updateStart = std::chrono::steady_clock::now();
        PM::DataPoints currentMap = mapper->getMap();
        if (!hasDeterministicMapUpdatePose_ && currentMap.getNbPoints() > 0) {
            lastDeterministicMapUpdatePose_ = acceptedSensorToMap;
            hasDeterministicMapUpdatePose_ = true;
            RCLCPP_INFO(this->get_logger(),
                "Initial map was seeded by mapper processInput: map_pts=%d pose={%s}. "
                "Skipping duplicate insertion.",
                static_cast<int>(currentMap.getNbPoints()),
                transformSummary(acceptedSensorToMap).c_str());
            return true;
        }

        PM::DataPoints inputInMapFrame =
            transformation->compute(inputInSensorFrame, acceptedSensorToMap);
        const bool creatingMap = currentMap.getNbPoints() == 0;

        RCLCPP_INFO(this->get_logger(),
            "Deterministic map update started: mode=%s current_map_pts=%d scan_pts=%d pose={%s}",
            creatingMap ? "create" : "point_distance",
            static_cast<int>(currentMap.getNbPoints()),
            static_cast<int>(inputInSensorFrame.getNbPoints()),
            transformSummary(acceptedSensorToMap).c_str());

        if (!creatingMap)
        {
            const auto overlapStart = std::chrono::steady_clock::now();
            const MapOverlapStats overlap = estimateMapOverlap(inputInMapFrame, currentMap);
            const double overlapMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - overlapStart).count();
            RCLCPP_INFO(this->get_logger(),
                "Scan-map overlap before insertion: near_voxel=%.2fm ratio=%.3f loose_voxel=%.2fm ratio=%.3f sampled=%d time=%.1fms",
                mapOverlapNearVoxelM_,
                overlap.nearRatio,
                mapOverlapLooseVoxelM_,
                overlap.looseRatio,
                overlap.sampled,
                overlapMs);

            if (mapOverlapTooLow(overlap))
            {
                forceNextMapUpdate_ = true;
                RCLCPP_WARN(this->get_logger(),
                    "Skipping map insertion: aligned scan does not overlap current map enough "
                    "(near %.3f threshold %.3f, loose %.3f threshold %.3f, sampled=%d). "
                    "Keeping the previous map-update baseline and forcing a retry on the next accepted scan. "
                    "Check /mapping/aligned_scan in Foxglove: if it is also misaligned, the issue is ICP/odom/TF; "
                    "if it is aligned while /mapping/map is broken, the map insertion path is at fault.",
                    overlap.nearRatio,
                    params->minMapOverlapNearRatio,
                    overlap.looseRatio,
                    params->minMapOverlapLooseRatio,
                    overlap.sampled);
                return false;
            }
        }

        PM::DataPoints updatedMap = creatingMap ? inputInMapFrame : currentMap;
        if (!creatingMap) {
            deterministicMapperModule_->inPlaceUpdateMap(
                inputInMapFrame, updatedMap, acceptedSensorToMap);
        }
        normalizeMapNormals(updatedMap);
        setMapperMap(updatedMap);

        if (params->enableGlobalOutputMap)
        {
            std::lock_guard<std::mutex> lock(globalOutputMapMutex_);
            if (!hasGlobalOutputMap_)
            {
                globalOutputMap_ = inputInMapFrame;
                hasGlobalOutputMap_ = true;
            }
            else
            {
                globalOutputMapperModule_->inPlaceUpdateMap(
                    inputInMapFrame, globalOutputMap_, acceptedSensorToMap);
            }
            ++globalOutputMapUpdates_;
        }

        lastDeterministicMapUpdatePose_ = acceptedSensorToMap;
        hasDeterministicMapUpdatePose_ = true;
        forceNextMapUpdate_ = false;

        const double updateMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - updateStart).count();
        if (params->enableGlobalOutputMap)
        {
            std::lock_guard<std::mutex> lock(globalOutputMapMutex_);
            RCLCPP_INFO(this->get_logger(),
                "Deterministic map update done: mode=%s scan_pts=%d local_map_pts=%d global_map_pts=%d time=%.1fms pose={%s}",
                creatingMap ? "create" : "update",
                static_cast<int>(inputInSensorFrame.getNbPoints()),
                static_cast<int>(updatedMap.getNbPoints()),
                static_cast<int>(globalOutputMap_.getNbPoints()),
                updateMs,
                transformSummary(acceptedSensorToMap).c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                "Deterministic map update done: mode=%s scan_pts=%d map_pts=%d time=%.1fms pose={%s}",
                creatingMap ? "create" : "update",
                static_cast<int>(inputInSensorFrame.getNbPoints()),
                static_cast<int>(updatedMap.getNbPoints()),
                updateMs,
                transformSummary(acceptedSensorToMap).c_str());
        }
        return true;
    }

    bool updateMapFromOdomBridge(
        const PM::DataPoints& inputInSensorFrame,
        const PM::TransformationParameters& odomSensorToMap)
    {
        if (hasDeterministicMapUpdatePose_ &&
            !deterministicMapUpdateDue(odomSensorToMap)) {
            return false;
        }

        const auto updateStart = std::chrono::steady_clock::now();
        PM::DataPoints currentMap = mapper->getMap();
        PM::DataPoints inputInMapFrame =
            transformation->compute(inputInSensorFrame, odomSensorToMap);
        const bool creatingMap = currentMap.getNbPoints() == 0;
        PM::DataPoints updatedMap = creatingMap ? inputInMapFrame : currentMap;
        if (!creatingMap)
        {
            deterministicMapperModule_->inPlaceUpdateMap(
                inputInMapFrame, updatedMap, odomSensorToMap);
        }

        normalizeMapNormals(updatedMap);
        setMapperMap(updatedMap);

        if (params->enableGlobalOutputMap)
        {
            std::lock_guard<std::mutex> lock(globalOutputMapMutex_);
            if (!hasGlobalOutputMap_)
            {
                globalOutputMap_ = inputInMapFrame;
                hasGlobalOutputMap_ = true;
            }
            else
            {
                globalOutputMapperModule_->inPlaceUpdateMap(
                    inputInMapFrame, globalOutputMap_, odomSensorToMap);
            }
            ++globalOutputMapUpdates_;
        }

        lastDeterministicMapUpdatePose_ = odomSensorToMap;
        hasDeterministicMapUpdatePose_ = true;
        forceNextMapUpdate_ = false;

        const double updateMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - updateStart).count();
        if (params->enableGlobalOutputMap)
        {
            std::lock_guard<std::mutex> lock(globalOutputMapMutex_);
            RCLCPP_WARN(this->get_logger(),
                "[ODOM_BRIDGE] map update done: mode=%s scan_pts=%d local_map_pts=%d global_map_pts=%d time=%.1fms pose={%s}",
                creatingMap ? "create" : "dead_reckoning_update",
                static_cast<int>(inputInSensorFrame.getNbPoints()),
                static_cast<int>(updatedMap.getNbPoints()),
                static_cast<int>(globalOutputMap_.getNbPoints()),
                updateMs,
                transformSummary(odomSensorToMap).c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                "[ODOM_BRIDGE] map update done: mode=%s scan_pts=%d map_pts=%d time=%.1fms pose={%s}",
                creatingMap ? "create" : "dead_reckoning_update",
                static_cast<int>(inputInSensorFrame.getNbPoints()),
                static_cast<int>(updatedMap.getNbPoints()),
                updateMs,
                transformSummary(odomSensorToMap).c_str());
        }
        return true;
    }

    bool mapUpdateQualityGood(const RegistrationQualityGate::Result& qgResult) const
    {
        return qgResult.translation_correction_m <= params->maxMapUpdateTranslationCorrectionM &&
               qgResult.rotation_correction_deg <= params->maxMapUpdateRotationCorrectionDeg;
    }

    bool mapUpdateCorrectionAllowed(
        const RegistrationQualityGate::Result& qgResult,
        const MotionState& motion,
        const double dtSecAccepted,
        std::string& reason) const
    {
        if (mapUpdateQualityGood(qgResult))
        {
            reason = "static_map_update_gate";
            return true;
        }

        if (!params->enableMotionAdaptiveGate || motion.pivot)
        {
            reason = "static_gate_failed";
            return false;
        }

        const AdaptiveGateLimits limits = adaptiveGateLimits(motion, dtSecAccepted);
        const double fastTranslationLimit =
            std::min(limits.translationM, 2.0 * params->maxMapUpdateTranslationCorrectionM);
        const bool allowed =
            motion.aggressive &&
            qgResult.translation_correction_m <= fastTranslationLimit &&
            qgResult.rotation_correction_deg <= params->maxMapUpdateRotationCorrectionDeg;
        std::ostringstream ss;
        ss << (allowed ? "adaptive_fast_translation_gate" : "adaptive_fast_translation_rejected")
           << " correction=" << qgResult.translation_correction_m << "m/"
           << qgResult.rotation_correction_deg << "deg limit="
           << fastTranslationLimit << "m/"
           << params->maxMapUpdateRotationCorrectionDeg << "deg speed="
           << motion.odomSpeedMs << "m/s accel=" << motion.odomAccelMs2
           << "m/s2 yaw_rate=" << motion.dominantYawRateDegS << "deg/s";
        reason = ss.str();
        return allowed;
    }

    bool seedInitialMapIfNeeded(
        const PM::DataPoints& inputInSensorFrame,
        const PM::TransformationParameters& sensorToMap)
    {
        if (hasDeterministicMapUpdatePose_) {
            return false;
        }

        PM::DataPoints currentMap = mapper->getMap();
        if (currentMap.getNbPoints() > 0) {
            lastDeterministicMapUpdatePose_ = sensorToMap;
            hasDeterministicMapUpdatePose_ = true;
            RCLCPP_WARN(this->get_logger(),
                "Mapper already had an initial map before deterministic seeding: map_pts=%d pose={%s}. "
                "Keeping it, but this can indicate an unexpected internal seed.",
                static_cast<int>(currentMap.getNbPoints()),
                transformSummary(sensorToMap).c_str());
            return false;
        }

        PM::DataPoints initialMap = transformation->compute(inputInSensorFrame, sensorToMap);
        recomputeMapNormals(initialMap);
        setMapperMap(initialMap);

        if (params->enableGlobalOutputMap)
        {
            std::lock_guard<std::mutex> lock(globalOutputMapMutex_);
            globalOutputMap_ = initialMap;
            hasGlobalOutputMap_ = true;
            globalOutputMapUpdates_ = 1;
        }
        lastDeterministicMapUpdatePose_ = sensorToMap;
        hasDeterministicMapUpdatePose_ = true;
        RCLCPP_INFO(this->get_logger(),
            "Initial map deterministically seeded in map frame: scan_pts=%d map_pts=%d pose={%s}",
            static_cast<int>(inputInSensorFrame.getNbPoints()),
            static_cast<int>(initialMap.getNbPoints()),
            transformSummary(sensorToMap).c_str());
        return true;
    }

    // ── cropPointsToRadius ──
    // Returns a new DataPoints containing only columns within radiusM (XY) of center.
    // Uses 2D distance only to avoid Z artifacts from slope/pitch.
    static PM::DataPoints cropPointsToRadius(
        const PM::DataPoints& map,
        const Eigen::Vector2f& centerXY,
        float radiusM)
    {
        const float radiusSq = radiusM * radiusM;
        const int n = static_cast<int>(map.getNbPoints());

        std::vector<int> keep;
        keep.reserve(n);
        for (int i = 0; i < n; ++i)
        {
            const float dx = map.features(0, i) - centerXY(0);
            const float dy = map.features(1, i) - centerXY(1);
            if (dx * dx + dy * dy <= radiusSq)
            {
                keep.push_back(i);
            }
        }

        if (static_cast<int>(keep.size()) == n)
        {
            return map;  // Nothing to trim — avoid an unnecessary copy.
        }

        PM::DataPoints result = map.createSimilarEmpty();
        result.conservativeResize(static_cast<int>(keep.size()));
        for (int j = 0; j < static_cast<int>(keep.size()); ++j)
        {
            result.setColFrom(j, map, keep[j]);
        }
        return result;
    }

    static void capCloudPointsDeterministically(PM::DataPoints& cloud, const int maxPoints)
    {
        const int n = static_cast<int>(cloud.getNbPoints());
        if (maxPoints <= 0 || n <= maxPoints)
        {
            return;
        }

        PM::DataPoints result = cloud.createSimilarEmpty();
        result.conservativeResize(maxPoints);
        for (int j = 0; j < maxPoints; ++j)
        {
            const int idx = static_cast<int>(
                std::llround(static_cast<double>(j) * static_cast<double>(n - 1) /
                             static_cast<double>(maxPoints - 1)));
            result.setColFrom(j, cloud, std::min(idx, n - 1));
        }
        cloud = std::move(result);
    }

    bool rebuildLocalMapFromGlobalOutput(
        const Eigen::Vector2f& robotXY,
        const char* reason)
    {
        if (!params->enableGlobalOutputMap)
        {
            return false;
        }

        PM::DataPoints sourceMap;
        {
            std::lock_guard<std::mutex> lock(globalOutputMapMutex_);
            if (!hasGlobalOutputMap_ || globalOutputMap_.getNbPoints() == 0)
            {
                return false;
            }
            sourceMap = globalOutputMap_;
        }

        PM::DataPoints localMap = cropPointsToRadius(
            sourceMap,
            robotXY,
            static_cast<float>(params->mapTrimRadiusM));
        const int croppedPts = static_cast<int>(localMap.getNbPoints());
        capCloudPointsDeterministically(localMap, params->maxMapPointsBeforeTrim);
        if (localMap.getNbPoints() == 0)
        {
            return false;
        }

        normalizeMapNormals(localMap);
        setMapperMap(localMap);
        overlapVoxelCache_ = MapOverlapVoxelCache{};

        RCLCPP_INFO(this->get_logger(),
            "Local ICP map rebuilt from global output after %s: global_pts=%d cropped_pts=%d local_pts=%d radius=%.0fm cap=%d.",
            reason,
            static_cast<int>(sourceMap.getNbPoints()),
            croppedPts,
            static_cast<int>(localMap.getNbPoints()),
            params->mapTrimRadiusM,
            params->maxMapPointsBeforeTrim);
        return true;
    }

    bool odomBridgeRecoveryDue() const
    {
        if (!params->enableMapRecovery || params->recoveryReloadAfterRejections <= 0)
        {
            return false;
        }
        if (consecutiveOdomBridgeScans_ < params->recoveryReloadAfterRejections)
        {
            return false;
        }
        if (lastOdomBridgeRecoveryAttempt_ < 0)
        {
            return true;
        }
        return consecutiveOdomBridgeScans_ - lastOdomBridgeRecoveryAttempt_ >=
            params->recoveryAttemptIntervalScans;
    }

    bool recoverLocalMapForOdomBridge(
        const PM::TransformationParameters& robotToMap,
        const char* reason)
    {
        if (!odomBridgeRecoveryDue())
        {
            return false;
        }

        lastOdomBridgeRecoveryAttempt_ = consecutiveOdomBridgeScans_;
        const Eigen::Vector2f robotXY = robotToMap.topRightCorner(2, 1);
        const bool recovered = rebuildLocalMapFromGlobalOutput(robotXY, reason);
        if (recovered)
        {
            refreshMapPublicationSnapshot(reason);
            RCLCPP_WARN(this->get_logger(),
                "[ODOM_BRIDGE_RECOVERY] recentered local ICP map after %d bridge scans at robot_xy=(%.2f,%.2f). "
                "Bridge scans still frozen: no dead-reckoning insertion into map.",
                consecutiveOdomBridgeScans_,
                static_cast<double>(robotXY(0)),
                static_cast<double>(robotXY(1)));
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                "[ODOM_BRIDGE_RECOVERY] local-map recenter failed after %d bridge scans at robot_xy=(%.2f,%.2f).",
                consecutiveOdomBridgeScans_,
                static_cast<double>(robotXY(0)),
                static_cast<double>(robotXY(1)));
        }
        return recovered;
    }

    void refreshMapPublicationSnapshot(const char* reason)
    {
        PM::DataPoints mapSnapshot;
        bool usingGlobalOutputMap = false;
        const bool preferGlobal =
            params->mapPublicationSource == "global" ||
            (params->mapPublicationSource == "auto" && params->enableGlobalOutputMap);
        if (preferGlobal && params->enableGlobalOutputMap)
        {
            std::lock_guard<std::mutex> lock(globalOutputMapMutex_);
            if (hasGlobalOutputMap_ && globalOutputMap_.getNbPoints() > 0)
            {
                mapSnapshot = globalOutputMap_;
                usingGlobalOutputMap = true;
            }
        }
        if (mapSnapshot.getNbPoints() == 0)
        {
            mapSnapshot = mapper->getMap();
        }
        if (mapSnapshot.getNbPoints() == 0) {
            RCLCPP_WARN(this->get_logger(),
                "Map snapshot requested after %s, but mapper map is empty.",
                reason);
            return;
        }

        // View-frustum culling: when map_publish_radius_m > 0, publish only points
        // within that XY radius of the robot. Reduces message size by 80-90% for
        // large maps, keeping Foxglove WebSocket bandwidth within WiFi budget.
        // When map_publish_radius_m == 0 the full map is published (bag replay,
        // offline ground-truth generation).
        const int fullMapPts = static_cast<int>(mapSnapshot.getNbPoints());
        if (hasInitialAcceptedRobotToMap_ && params->mapPublishRadiusM > 0.0)
        {
            const Eigen::Vector2f robotXY = lastAcceptedRobotToMap_.topRightCorner(2, 1);
            mapSnapshot = cropPointsToRadius(
                mapSnapshot, robotXY, static_cast<float>(params->mapPublishRadiusM));
            const int culledPts = static_cast<int>(mapSnapshot.getNbPoints());
            if (culledPts < fullMapPts)
            {
                RCLCPP_DEBUG(this->get_logger(),
                    "Map publication: culled %d → %d pts (radius=%.0fm).",
                    fullMapPts, culledPts, params->mapPublishRadiusM);
            }
        }

        if (params->compressionVoxelSize > 0)
        {
            std::lock_guard<std::mutex> lk(mapFilterMutex_);
            outputMapSubsamplingFilter->inPlaceFilter(mapSnapshot);
        }
        const int snapshotPoints = static_cast<int>(mapSnapshot.getNbPoints());
        {
            std::lock_guard<std::mutex> lk(mapPublishLock_);
            latestMapForPublication_ = std::move(mapSnapshot);
            latestMapReady_ = true;
        }
        needMapSnapshot_.store(false);
        if (hasInitialAcceptedRobotToMap_)
        {
            const Eigen::Vector2f robotXY = lastAcceptedRobotToMap_.topRightCorner(2, 1);
            RCLCPP_INFO(this->get_logger(),
                "Refreshed map publication snapshot after %s: source=%s policy=%s frame=%s robot_xy=(%.2f,%.2f) full_pts=%d published_pts=%d.",
                reason,
                usingGlobalOutputMap ? "global_output" : "local_icp",
                params->mapPublicationSource.c_str(),
                params->mapFrame.c_str(),
                robotXY.x(),
                robotXY.y(),
                fullMapPts,
                snapshotPoints);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                "Refreshed map publication snapshot after %s: source=%s policy=%s frame=%s full_pts=%d published_pts=%d.",
                reason,
                usingGlobalOutputMap ? "global_output" : "local_icp",
                params->mapPublicationSource.c_str(),
                params->mapFrame.c_str(),
                fullMapPts,
                snapshotPoints);
        }
    }

    bool publishedPosePlausible(
        const PM::TransformationParameters& robotToMap,
        const PM::TransformationParameters& odomPredictedRobotToMap,
        double dtSecAccepted,
        std::string& reason,
        bool* priorConsistentLargeYawStep = nullptr) const
    {
        // Compare against last ACCEPTED pose, not previousRobotToMap (which is updated
        // even on rejection). Using a rejected pose as reference allows a cascade of
        // gradually drifting poses to pass the per-step check.
        if (lastAcceptedTimeStamp_.nanoseconds() == 0 || dtSecAccepted <= 1e-6) {
            return true;
        }

        // Absolute yaw step check — independent of dt.
        // dtSecAccepted grows after rejection cascades, making the yaw rate check
        // ineffective (90°/10s < 90 deg/s). A per-scan absolute cap catches any
        // sudden rotation regardless of how long the cascade lasted.
        const double yawStepDeg =
            std::abs(wrapToPi(yawFromTransform(robotToMap) - yawFromTransform(lastAcceptedRobotToMap_))) *
            180.0 / M_PI;
        if (yawStepDeg > params->maxPoseYawStepDeg)
        {
            const double yawOdomResidualDeg =
                std::abs(wrapToPi(
                    yawFromTransform(robotToMap) -
                    yawFromTransform(odomPredictedRobotToMap))) * 180.0 / M_PI;
            if (yawOdomResidualDeg <= params->maxPoseYawOdomResidualDeg)
            {
                if (priorConsistentLargeYawStep)
                {
                    *priorConsistentLargeYawStep = true;
                }
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Large yaw step accepted because ICP agrees with odom prior: "
                    "yaw_step=%.1fdeg > %.1fdeg, yaw_odom_residual=%.1fdeg <= %.1fdeg, dt=%.3fs. "
                    "Map insertion will be frozen for this scan.",
                    yawStepDeg,
                    params->maxPoseYawStepDeg,
                    yawOdomResidualDeg,
                    params->maxPoseYawOdomResidualDeg,
                    dtSecAccepted);
            }
            else
            {
                std::ostringstream ss;
                ss << "published pose yaw step too high: " << yawStepDeg << " deg > "
                   << params->maxPoseYawStepDeg << " deg and ICP-vs-odom yaw residual "
                   << yawOdomResidualDeg << " deg > "
                   << params->maxPoseYawOdomResidualDeg
                   << " deg (dt=" << dtSecAccepted << " s)";
                reason = ss.str();
                return false;
            }
        }

        const int dim = static_cast<int>(robotToMap.rows()) - 1;
        const Eigen::VectorXf delta =
            robotToMap.topRightCorner(dim, 1) -
            lastAcceptedRobotToMap_.topRightCorner(dim, 1);
        const double xy = std::hypot(static_cast<double>(delta(0)), static_cast<double>(delta(1)));
        // dtSecAccepted = temps depuis la dernière acceptation — cohérent avec la
        // référence lastAcceptedRobotToMap_. Évite speed=xy/50ms qui rejette les bons
        // scans après cascade de rejets, et laisse passer des pas backwards <0.4m.
        // Clamped to 10 s max so the rate-based checks don't degrade after long cascades.
        const double dtClamped = std::min(dtSecAccepted, 10.0);
        const double speed = xy / dtClamped;
        const double z = (dim >= 3) ? std::abs(static_cast<double>(delta(2))) : 0.0;
        const double yawRateDegS = yawStepDeg / dtClamped;
        const Eigen::VectorXf odomDelta =
            odomPredictedRobotToMap.topRightCorner(dim, 1) -
            lastAcceptedRobotToMap_.topRightCorner(dim, 1);
        const double odomXy = std::hypot(
            static_cast<double>(odomDelta(0)),
            static_cast<double>(odomDelta(1)));
        const double residualXy = std::hypot(
            static_cast<double>(delta(0) - odomDelta(0)),
            static_cast<double>(delta(1) - odomDelta(1)));

        // Do not assume distance from the initial pose is monotonic. The robot can
        // reverse, return along the same aisle, or make a tight articulated turn.
        // Instead, reject only when the ICP-published motion disagrees strongly with
        // the odometry-predicted local motion over the same accepted-scan gap.
        const double maxOdomResidual =
            std::max(1.0, std::min(params->maxPoseStepM, 0.35 * odomXy + 0.5));
        if (odomXy > 0.25 && residualXy > maxOdomResidual)
        {
            std::ostringstream ss;
            ss << "published pose odom residual too high: residual="
               << residualXy << " m > " << maxOdomResidual
               << " m (icp_step=" << xy << " m odom_step=" << odomXy
               << " m dt=" << dtSecAccepted << " s)";
            reason = ss.str();
            return false;
        }

        const double maxStepForGap =
            std::max(params->maxPoseStepM, params->maxVelocityMs * dtClamped * 1.25);
        if (xy > maxStepForGap) {
            std::ostringstream ss;
            ss << "published pose xy step too high: " << xy << " m > "
               << maxStepForGap << " m (base_limit=" << params->maxPoseStepM
               << " m dt=" << dtClamped << " s)";
            reason = ss.str();
            return false;
        }
        if (speed > params->maxVelocityMs) {
            std::ostringstream ss;
            ss << "published pose speed too high: " << speed << " m/s > "
               << params->maxVelocityMs << " m/s (xy_step=" << xy << " m dt=" << dtClamped << " s)";
            reason = ss.str();
            return false;
        }
        if (yawRateDegS > params->maxYawRateDegS) {
            std::ostringstream ss;
            ss << "published pose yaw rate too high: " << yawRateDegS << " deg/s > "
               << params->maxYawRateDegS << " deg/s (yaw_step=" << yawStepDeg
               << " deg dt_clamped=" << dtClamped << " s)";
            reason = ss.str();
            return false;
        }
        if (z > params->maxZJumpM) {
            std::ostringstream ss;
            ss << "published pose z jump too high: " << z << " m > "
               << params->maxZJumpM << " m";
            reason = ss.str();
            return false;
        }
        return true;
    }

    bool odomBridgeAllowed(
        const MotionState& motion,
        double dtSecAccepted,
        const PM::TransformationParameters& odomPredictedRobotToMap,
        const std::string& rejectedReason,
        std::string& bridgeReason) const
    {
        if (!params->enableOdomBridge || lastAcceptedTimeStamp_.nanoseconds() == 0)
        {
            return false;
        }

        const bool triggeredBySpeed =
            motion.odomSpeedMs >= params->odomBridgeMinSpeedMs;
        const bool triggeredByRejects =
            consecutiveRejections_ >= params->odomBridgeAfterRejections;
        const bool triggeredByGap = dtSecAccepted > 0.5;
        if (!triggeredBySpeed && !motion.aggressive && !triggeredByRejects && !triggeredByGap)
        {
            bridgeReason = "odom bridge inactive: trigger not reached";
            return false;
        }

        std::string odomPlausibilityReason;
        if (!publishedPosePlausible(
                odomPredictedRobotToMap,
                odomPredictedRobotToMap,
                dtSecAccepted,
                odomPlausibilityReason,
                nullptr))
        {
            bridgeReason = "odom bridge refused: odom prior implausible: " +
                odomPlausibilityReason;
            return false;
        }

        std::ostringstream ss;
        ss << "odom_bridge localization-only: " << rejectedReason
           << " speed=" << motion.odomSpeedMs
           << "m/s dt=" << dtSecAccepted
           << "s rejects=" << consecutiveRejections_;
        bridgeReason = ss.str();
        return true;
    }

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
        if (params->enableGlobalOutputMap && hasGlobalOutputMap_)
        {
            std::lock_guard<std::mutex> lock(globalOutputMapMutex_);
            PM::DataPoints mapToSave = globalOutputMap_;
            normalizeMapNormals(mapToSave);
            RCLCPP_INFO(this->get_logger(),
                "Saving untrimmed global output map: pts=%d updates=%lu",
                static_cast<int>(mapToSave.getNbPoints()),
                static_cast<unsigned long>(globalOutputMapUpdates_));
            mapToSave.save(mapFileName);
            return;
        }
        mapper->getMap().save(mapFileName);
    }

    void loadMap(const std::string& mapFileName)
    {
        RCLCPP_INFO(this->get_logger(), "Loading map from %s", mapFileName.c_str());
        PM::DataPoints map = PM::DataPoints::load(mapFileName);
        int euclideanDim = params->is3D ? 3 : 2;
        if(map.getEuclideanDim() != static_cast<unsigned int>(euclideanDim))
        {
            throw std::runtime_error("Invalid map dimension");
        }
        setMapperMap(map);
        if (params->enableGlobalOutputMap)
        {
            std::lock_guard<std::mutex> lock(globalOutputMapMutex_);
            globalOutputMap_ = map;
            hasGlobalOutputMap_ = true;
            globalOutputMapUpdates_ = 0;
        }
    }

    void setRobotPose(const PM::TransformationParameters& robotPose)
    {
        robotPoseToSet = robotPose;
        hasToSetRobotPose = true;

        // A pose reset changes the reference used by every temporal/plausibility
        // gate.  Keeping the previous map's accepted pose caused the first valid
        // ICP result after LoadMap to be rejected as a multi-metre "jump"; the
        // rejected pose then remained the prior forever.  Clear all history that
        // is expressed in the old localization epoch.  The next accepted scan
        // becomes the new baseline and is still checked by the ICP quality gate.
        const auto clockType = this->get_clock()->get_clock_type();
        previousTimeStamp = rclcpp::Time(0, 0, clockType);
        lastAcceptedTimeStamp_ = rclcpp::Time(0, 0, clockType);
        previousRobotToMap = robotPose;
        lastAcceptedRobotToMap_ = robotPose;
        initialAcceptedRobotToMap_ = robotPose;
        hasInitialAcceptedRobotToMap_ = false;
        consecutiveRejections_ = 0;
        consecutiveOdomBridgeScans_ = 0;
        lastOdomBridgeRecoveryAttempt_ = -1;
        lastRecoveryAttemptRejections_ = -1;
        hasLastOdomPriorSpeed_ = false;
        hasDeterministicMapUpdatePose_ = false;
        forceNextMapUpdate_ = true;
        lastGoodMapSnapshot_.valid = false;
        // An explicit pose seed is authoritative; do not subsequently overwrite
        // it with the generic "anchor map at first odom pose" startup path.
        mapAnchoredAtInitialRobotPose_ = true;
        needMapSnapshot_.store(true);

        RCLCPP_INFO(this->get_logger(),
            "Localization epoch reset with robot pose seed {%s}; temporal gates will re-baseline on the next accepted ICP scan.",
            transformSummary(robotPose).c_str());
    }

    void saveTrajectory(const std::string& trajectoryFileName)
    {
        RCLCPP_INFO(this->get_logger(), "Saving trajectory to %s", trajectoryFileName.c_str());
        std::lock_guard<std::mutex> lk(trajectoryMutex_);
        robotTrajectory->save(trajectoryFileName);
    }

    void mapperShutdownLoop()
    {
        std::chrono::duration<float> idleTime = std::chrono::duration<float>::zero();

        while (rclcpp::ok() && running_.load())
        {
            {
                std::lock_guard<std::mutex> lk(idleTimeLock);
                if (lastTimeInputWasProcessed.time_since_epoch().count())
                {
                    idleTime = std::chrono::steady_clock::now() - lastTimeInputWasProcessed;
                }
            }

            if (idleTime > std::chrono::duration<float>(params->maxIdleTime))
            {
                saveMap(params->finalMapFileName);
                saveTrajectory(params->finalTrajectoryFileName);
                RCLCPP_INFO(this->get_logger(), "Max idle time reached. Shutting down.");
                rclcpp::shutdown();
            }

            std::this_thread::sleep_for(std::chrono::duration<float>(0.1f));
        }
    }

    // allowLatestFallback=true  : OK pour TF statiques (hesai→base_link, robot→sensor)
    //                             car l'erreur d'interpolation est negligeable (<1mm).
    // allowLatestFallback=false : Pour TF dynamiques (sensor→odom). Tente d'abord une
    //                             recherche exacte au timestamp du scan. Si absente, essaie
    //                             le dernier TF disponible SEULEMENT si son timestamp est
    //                             dans les 50ms du scan (jitter réseau/scheduling normal).
    //                             Au-delà de 50ms: skip le scan — un prior périmé est pire
    //                             qu'un scan manqué (risque de convergence sur un mur voisin).
    //
    // Threshold 50ms justification: at 1.5 m/s, 50ms = 7.5 cm of robot motion.
    // The ICP convergence basin (maxDist=1.0m) absorbs 7.5 cm without issue.
    // Beyond 50ms the odometry TF publisher has a real problem and the scan should
    // be dropped rather than using a stale transform silently.
    static constexpr double kMaxDynamicTfStalenessMs_ = 50.0;

    PM::TransformationParameters findTransform(
        const std::string& sourceFrame, const std::string& targetFrame,
        const rclcpp::Time& time, const int& transformDimension,
        bool allowLatestFallback = true)
    {
        const auto timeout = std::chrono::milliseconds(params->tfLookupTimeoutMs);
        try
        {
            geometry_msgs::msg::TransformStamped tf =
                tfBuffer->lookupTransform(targetFrame, sourceFrame, time, timeout);
            return PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(tf, transformDimension);
        }
        catch (const tf2::ExtrapolationException& ex)
        {
            if (!allowLatestFallback)
            {
                // Dynamic TF: try time(0) fallback with staleness validation.
                try
                {
                    geometry_msgs::msg::TransformStamped latest =
                        tfBuffer->lookupTransform(targetFrame, sourceFrame, rclcpp::Time(0), timeout);
                    const double staleness_ms =
                        std::abs((time - rclcpp::Time(latest.header.stamp)).seconds()) * 1000.0;
                    if (staleness_ms <= kMaxDynamicTfStalenessMs_)
                    {
                        RCLCPP_DEBUG(this->get_logger(),
                            "TF %s->%s: exact lookup missed, using latest (staleness=%.1fms <= %.0fms).",
                            sourceFrame.c_str(), targetFrame.c_str(),
                            staleness_ms, kMaxDynamicTfStalenessMs_);
                        return PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(
                            latest, transformDimension);
                    }
                    RCLCPP_WARN(this->get_logger(),
                        "TF %s->%s: latest TF too stale (%.1fms > %.0fms) — skipping scan.",
                        sourceFrame.c_str(), targetFrame.c_str(),
                        staleness_ms, kMaxDynamicTfStalenessMs_);
                }
                catch (const tf2::TransformException&) {}
                // Staleness exceeded or second lookup failed: re-throw original.
                // A stale prior sends ICP to the wrong place and can converge
                // on a neighbouring wall — worse than a dropped scan.
                throw;
            }
            // Static TF at startup: startup race condition < 50ms, error negligible.
            RCLCPP_WARN_ONCE(this->get_logger(),
                "TF extrapolation for %s->%s at t=%.3fs: using latest available TF (startup race).",
                sourceFrame.c_str(), targetFrame.c_str(), time.seconds());
            geometry_msgs::msg::TransformStamped tf =
                tfBuffer->lookupTransform(targetFrame, sourceFrame, rclcpp::Time(0), timeout);
            return PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(tf, transformDimension);
        }
    }

    void gotInput(PM::DataPoints& input, const std::string& sensorFrame, const rclcpp::Time& cloudStamp)
    {
        // ── Timestamp validation ──
        if (cloudStamp.nanoseconds() == 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Received cloud with zero timestamp. Skipping.");
            return;
        }

        rclcpp::Time timeStamp = cloudStamp;
        const auto processingStart = std::chrono::steady_clock::now();

        // ── Reset idle timer on every arriving scan ──
        // Must be updated here (not just on accepted scans) so the mapper does
        // not shut down during rejection cascades (e.g. broken odometry producing
        // Z jumps that reject every scan for >10s).
        {
            std::lock_guard<std::mutex> lk(idleTimeLock);
            lastTimeInputWasProcessed = std::chrono::steady_clock::now();
        }

        try
        {
            // ── Transform to filtering frame (self-filter bboxes in base_link) ──
            // filtering_frame est typiquement base_link: les bboxes dans _config.yaml
            // sont exprimees dans ce frame (invariant par rapport a l'orientation du capteur).
            // Le TF hesai_lidar→base_link est statique (URDF fixed joint), toujours disponible.
            const auto t1_ff = std::chrono::steady_clock::now();
            PM::TransformationParameters sensorToFilteringFrame;
            bool usingFilteringFrame = false;
            if (!params->filteringFrame.empty() && params->filteringFrame != sensorFrame)
            {
                try
                {
                    sensorToFilteringFrame = findTransform(
                        sensorFrame, params->filteringFrame, timeStamp, input.getHomogeneousDim());
                    input.features = sensorToFilteringFrame * input.features;
                    usingFilteringFrame = true;
                }
                catch (const tf2::TransformException& ex)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "filtering_frame '%s' TF unavailable (%s). Falling back to sensor frame.",
                        params->filteringFrame.c_str(), ex.what());
                }
            }
            const double ffMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t1_ff).count();

            // Remove articulated trailer/operator points in filtering_frame before
            // static YAML filters and before deskew/ICP. Static axis-aligned bboxes
            // cannot cover high-articulation pivot turns without deleting too much
            // useful environment.
            if (usingFilteringFrame || params->filteringFrame == sensorFrame)
            {
                removeDynamicTrailerPoints(input, timeStamp);
            }
            else if (params->enableDynamicTrailerSelfFilter)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[SELF-FILTER] Dynamic trailer OBB skipped because filtering_frame transform is unavailable.");
            }

            const auto t2_input = std::chrono::steady_clock::now();
            mapper->applyInputFilters(input);
            const double inputMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t2_input).count();

            // Retransformer dans le frame capteur pour l'ICP et le deskew.
            if (usingFilteringFrame)
            {
                input.features = sensorToFilteringFrame.inverse() * input.features;
            }

            publishAfterInputFilters(input, sensorFrame, cloudStamp);

            // ── Deskew on filtered cloud (sensor frame) ──
            // Done AFTER BBox+RandomSampling: the cloud is ~11k pts instead of ~24k,
            // so the TF cache build and OpenMP application loop both run on fewer points.
            // Previously done before filters because VoxelGridDataPointsFilter averaged
            // the times field (int64, ~1.78e18 ns) across voxel points causing int64
            // overflow → negative timestamps → TF lookup failures. RandomSampling does
            // not touch the times field, so the constraint no longer applies.
            // Precondition: cloud must be in sensorFrame (guaranteed by retransform above).
            const auto t0_deskew = std::chrono::steady_clock::now();
            if (params->deskew)
            {
                // Skip only TF/odom deskewing during fast articulated turns. The IMU path is
                // rotation-only gyro integration and is the preferred correction in pivots;
                // suppressing it leaves the scan distorted exactly when it is needed most.
                bool deskewAllowed = true;
                if (params->deskewSource != "imu" &&
                    previousTimeStamp.nanoseconds() != 0 && hasInitialAcceptedRobotToMap_)
                {
                    const double scanDt =
                        std::max(1e-4, (timeStamp - previousTimeStamp).seconds());
                    const double yawDelta = std::abs(wrapToPi(
                        yawFromTransform(lastAcceptedRobotToMap_) -
                        yawFromTransform(previousRobotToMap)));
                    const double yawRateDegS = yawDelta * (180.0 / M_PI) / scanDt;
                    if (yawRateDegS > maxDeskewYawRateDegS_)
                    {
                        deskewAllowed = false;
                        RCLCPP_DEBUG(this->get_logger(),
                            "Deskew suppressed: yaw_rate=%.1f deg/s > %.0f deg/s threshold.",
                            yawRateDegS, maxDeskewYawRateDegS_);
                    }
                }

                bool deskewOk = false;
                if (deskewAllowed)
                {
                    if (params->deskewSource == "imu")
                    {
                        // IMU rotation-only deskew: immune to odom angular-rate errors.
                        // TF deskew would bake wheel-slip / IMU-bias into every inserted
                        // scan, producing the swirl / double-tree artefact.
                        if (!imuExtrinsicReady_)
                            tryLookupImuSensorExtrinsic(sensorFrame);
                        if (imuExtrinsicReady_)
                        {
                            std::vector<Deskewer::ImuSample> imuSnap;
                            {
                                std::lock_guard<std::mutex> lk(imuDeskewBufMutex_);
                                imuSnap.assign(imuDeskewBuf_.begin(), imuDeskewBuf_.end());
                            }
                            // Hybrid deskew: gyro rotation + odom-derived constant
                            // linear velocity. Rotation stays slip-immune (IMU);
                            // translation smear (~v·0.1 m at speed) is compensated
                            // with an error bounded by the odom velocity error over
                            // one scan, not the full displacement. Zero velocity
                            // fallback (rotation-only) when the TF lookup fails.
                            Eigen::Vector3d vSensorEnd = Eigen::Vector3d::Zero();
                            if (input.times.size() > 0)
                            {
                                const int64_t scanEndNs = input.times.maxCoeff();
                                const int64_t scanStartNs = std::max<int64_t>(
                                    input.times.minCoeff(),
                                    scanEndNs - 200'000'000LL);
                                const double spanS = (scanEndNs - scanStartNs) * 1e-9;
                                if (spanS > 0.01)
                                {
                                    try
                                    {
                                        const auto sensorToOdomEnd = findTransform(
                                            sensorFrame, params->odomFrame,
                                            rclcpp::Time(scanEndNs, timeStamp.get_clock_type()),
                                            input.getHomogeneousDim());
                                        const auto sensorToOdomStart = findTransform(
                                            sensorFrame, params->odomFrame,
                                            rclcpp::Time(scanStartNs, timeStamp.get_clock_type()),
                                            input.getHomogeneousDim());
                                        const Eigen::Vector3f vOdom =
                                            (sensorToOdomEnd.topRightCorner(3, 1) -
                                             sensorToOdomStart.topRightCorner(3, 1)) /
                                            static_cast<float>(spanS);
                                        vSensorEnd =
                                            (sensorToOdomEnd.topLeftCorner(3, 3).transpose() * vOdom)
                                                .cast<double>();
                                    }
                                    catch (const tf2::TransformException&)
                                    {
                                        // rotation-only fallback
                                    }
                                }
                            }
                            deskewOk = deskewer->deskewCloudImu(
                                input, imuSnap, R_sensor_imu_, vSensorEnd);
                        }
                    }
                    else
                    {
                        // Legacy TF-odom deskew (default for live robot).
                        deskewOk = deskewer->deskewCloud(input, sensorFrame);
                    }
                }

                if (deskewOk)
                {
                    publishAfterDeskew(input, sensorFrame, cloudStamp);
                    // Deskew rotated point positions; any pre-computed surface normals
                    // in the cloud are now stale. Remove them so ensureInputNormals()
                    // recomputes on the corrected geometry.
                    if (input.descriptorExists("normals"))
                        input.removeDescriptor("normals");
                    // Only update timestamp when using absolute_ns — other modes cannot
                    // give a reliable absolute ROS time from per-point data alone.
                    if (params->deskewTimeMode == "absolute_ns")
                    {
                        if (input.times.size() > 0)
                        {
                            timeStamp = rclcpp::Time(
                                input.times.maxCoeff(),
                                timeStamp.get_clock_type());
                        }
                        else
                        {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Deskew absolute_ns requested but input cloud has no per-point times. "
                                "Keeping header timestamp.");
                        }
                    }
                }
            }
            const double deskewMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0_deskew).count();

            const auto t3_normals = std::chrono::steady_clock::now();
            ensureInputNormals(input);
            const double normalsMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t3_normals).count();

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                "[TIMING] ff=%.0fms input=%.0fms deskew=%.0fms normals=%.0fms",
                ffMs, inputMs, deskewMs, normalsMs);

            const double filterMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - processingStart).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Input filters: " << filterMs << " ms");
            RCLCPP_DEBUG(this->get_logger(),
                "Filtered input ready: frame=%s stamp=%.9f pts=%d normal_ok=%d",
                sensorFrame.c_str(),
                static_cast<double>(timeStamp.nanoseconds()) * 1e-9,
                static_cast<int>(input.getNbPoints()),
                input.descriptorExists("normals", static_cast<unsigned>(input.getEuclideanDim())));

            // Pas de fallback Time(0) pour une TF dynamique: un prior perime
            // envoie ICP au mauvais endroit et peut causer un jump de pose.
            // Si la TF manque → ExtrapolationException → scan skippe via catch exterieur.
            PM::TransformationParameters sensorToOdom =
                findTransform(sensorFrame, params->odomFrame, timeStamp, input.getHomogeneousDim(),
                              /*allowLatestFallback=*/false);
            PM::TransformationParameters robotToSensor =
                findTransform(params->robotFrame, sensorFrame, timeStamp, input.getHomogeneousDim());
            if (params->anchorMapAtInitialRobotPose && !mapAnchoredAtInitialRobotPose_)
            {
                PM::TransformationParameters robotToOdom = sensorToOdom * robotToSensor;
                PM::TransformationParameters initialOdomToMap =
                    transformation->correctParameters(robotToOdom.inverse());
                {
                    std::lock_guard<std::mutex> lk(mapTfLock);
                    odomToMap = initialOdomToMap;
                }
                mapAnchoredAtInitialRobotPose_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "Anchored map at initial robot pose: map->robot(t0)=identity, odom_to_map={%s}",
                    transformSummary(initialOdomToMap).c_str());
            }
            PM::TransformationParameters sensorToMapBeforeUpdate;
            {
                std::lock_guard<std::mutex> lk(mapTfLock);
                sensorToMapBeforeUpdate = odomToMap * sensorToOdom;
            }

            RCLCPP_DEBUG(this->get_logger(),
                "ICP prior: sensorToOdom{%s} sensorToMap{%s}",
                transformSummary(sensorToOdom).c_str(),
                transformSummary(sensorToMapBeforeUpdate).c_str());
            if (hasToSetRobotPose)
            {
                PM::TransformationParameters sensorToRobot = robotToSensor.inverse();
                sensorToMapBeforeUpdate = robotPoseToSet * sensorToRobot;
                hasToSetRobotPose = false;
            }

            const PM::TransformationParameters odomPredictedRobotToMap =
                sensorToMapBeforeUpdate * robotToSensor;
            const float dtSecPre = (previousTimeStamp.nanoseconds() != 0)
                ? static_cast<float>((timeStamp - previousTimeStamp).seconds()) : 0.0f;
            const float dtSecAcceptedPre = (lastAcceptedTimeStamp_.nanoseconds() != 0)
                ? static_cast<float>((timeStamp - lastAcceptedTimeStamp_).seconds()) : dtSecPre;
            MotionState motion = estimateMotionState(
                odomPredictedRobotToMap, timeStamp, dtSecAcceptedPre);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[MOTION] dt=%.3fs odom_speed=%.2fm/s accel=%.2fm/s2 yaw_odom=%.1fdeg/s yaw_imu=%.1fdeg/s aggressive=%d pivot=%d rejects=%d",
                motion.dtAcceptedS,
                motion.odomSpeedMs,
                motion.odomAccelMs2,
                motion.odomYawRateDegS,
                motion.imuYawRateDegS,
                motion.aggressive,
                motion.pivot,
                consecutiveRejections_);

            if (mappingEnabled_.load())
            {
                recoverLocalMapForPrior(
                    odomPredictedRobotToMap,
                    sensorToMapBeforeUpdate,
                    "pre_icp");
            }

            const bool seededInitialMap =
                mappingEnabled_.load() && seedInitialMapIfNeeded(input, sensorToMapBeforeUpdate);

            // ── ICP Phase 1: localisation (map non modifiee) ──
            // Une seule passe fine avec prior odom. Le M-estimateur Cauchy dans la
            // chaine outlier filters (_config.yaml) cree un paysage de cout lisse
            // qui evite les minima locaux sans necessiter de passe coarse preliminaire.
            // Budget: ~25ms (fine) + ~5ms (Phase 2) = 30ms < 50ms (20Hz).
            // Snapshot the service-controlled state once for this scan. A
            // disable request received during a long registration takes effect
            // no later than the next scan without being overwritten here.
            const bool shouldMap = mappingEnabled_.load();

            mapper->setIsMapping(false);

            const auto icpStart = std::chrono::steady_clock::now();
            const auto steadyTs = std::chrono::time_point<std::chrono::steady_clock>(
                std::chrono::nanoseconds(timeStamp.nanoseconds()));

            // ── Passe fine (prior = odom, mapping OFF) ──
            PM::TransformationParameters sensorToMapAfterUpdate =
                sensorToMapBeforeUpdate;
            bool localizationOnlyOdomBridge = false;
            std::string localizationOnlyReason;
            try
            {
                mapper->processInput(input, sensorToMapBeforeUpdate, steadyTs);
                // Copie valeur: apres Phase 2, mapper->getPose() peut changer legerement.
                sensorToMapAfterUpdate = mapper->getPose();
            }
            catch (const PM::ConvergenceError& e)
            {
                double recoveryCost = std::numeric_limits<double>::infinity();
                if (tryRecoveryYawHypotheses(
                        input,
                        sensorToMapBeforeUpdate,
                        steadyTs,
                        sensorToMapAfterUpdate,
                        recoveryCost,
                        "convergence_error"))
                {
                    RCLCPP_WARN(this->get_logger(),
                        "[RECOVERY] ICP convergence recovered by yaw hypotheses: cost=%.3f pose={%s}",
                        recoveryCost,
                        transformSummary(sensorToMapAfterUpdate).c_str());
                }
                else
                {
                MapOverlapStats priorOverlap;
                double priorOverlapMs = 0.0;
                if (shouldMap && mapper->getMap().getNbPoints() > 0)
                {
                    const auto priorOverlapStart = std::chrono::steady_clock::now();
                    PM::DataPoints inputAtPrior =
                        transformation->compute(input, sensorToMapBeforeUpdate);
                    priorOverlap = estimateMapOverlap(inputAtPrior, mapper->getMap());
                    priorOverlapMs = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - priorOverlapStart).count();
                }

                std::ostringstream convergenceReason;
                convergenceReason
                    << "ICP convergence failure: " << e.what()
                    << " prior_overlap near=" << priorOverlap.nearRatio
                    << " loose=" << priorOverlap.looseRatio;
                if (odomBridgeAllowed(
                        motion,
                        dtSecAcceptedPre,
                        odomPredictedRobotToMap,
                        convergenceReason.str(),
                        localizationOnlyReason))
                {
                    localizationOnlyOdomBridge = true;
                    sensorToMapAfterUpdate = sensorToMapBeforeUpdate;
                    RCLCPP_WARN(this->get_logger(),
                        "[ODOM_BRIDGE] %s (prior overlap near %.3f loose %.3f sampled=%d check=%.1fms). "
                        "Publishing odom prior; map insertion will be marked as bridge/dead-reckoning.",
                        localizationOnlyReason.c_str(),
                        priorOverlap.nearRatio,
                        priorOverlap.looseRatio,
                        priorOverlap.sampled,
                        priorOverlapMs);
                }
                else
                {
                    if (shouldMap && mapper->getMap().getNbPoints() > 0)
                    {
                        RCLCPP_WARN(this->get_logger(),
                            "ICP convergence failure and odom prior fallback refused: "
                            "prior overlap near %.3f (<0.15) loose %.3f (<0.30) sampled=%d check=%.1fms.",
                            priorOverlap.nearRatio,
                            priorOverlap.looseRatio,
                            priorOverlap.sampled,
                            priorOverlapMs);
                    }
                    mapper->setIsMapping(shouldMap);
                    ++consecutiveRejections_;
                    ++scansRejected_;
                    RCLCPP_ERROR(this->get_logger(),
                        "ICP convergence failure (scan rejected, continuing): stamp=%.9f frame=%s pts=%d prior={%s}: %s",
                        static_cast<double>(timeStamp.nanoseconds()) * 1e-9,
                        sensorFrame.c_str(),
                        static_cast<int>(input.getNbPoints()),
                        transformSummary(sensorToMapBeforeUpdate).c_str(),
                        e.what());
                    {
                        const float dtA = (lastAcceptedTimeStamp_.nanoseconds() != 0)
                            ? static_cast<float>((timeStamp - lastAcceptedTimeStamp_).seconds()) : 0.0f;
                        publishScanStatus(timeStamp, false, "convergence_error",
                            static_cast<int>(input.getNbPoints()), 0.0f, 0.0f, 0.0f, dtA);
                    }
                    if (params->enableConvergenceErrorDump)
                    {
                        try {
                            saveTrajectory(appendToFilePath(params->finalTrajectoryFileName, "_conv_error"));
                            saveMap(appendToFilePath(params->finalMapFileName, "_conv_error"));
                        } catch (const std::runtime_error& saveErr) {
                            RCLCPP_ERROR(this->get_logger(), "Emergency dump failed: %s", saveErr.what());
                        }
                    }
                    previousTimeStamp = timeStamp;
                    return;
                }
                }
            }
            catch (const std::exception& e)
            {
                mapper->setIsMapping(shouldMap);
                ++consecutiveRejections_;
                ++scansRejected_;
                RCLCPP_ERROR(this->get_logger(),
                    "ICP exception (scan rejected, continuing): stamp=%.9f frame=%s pts=%d prior={%s}: %s",
                    static_cast<double>(timeStamp.nanoseconds()) * 1e-9,
                    sensorFrame.c_str(),
                    static_cast<int>(input.getNbPoints()),
                    transformSummary(sensorToMapBeforeUpdate).c_str(),
                    e.what());
                {
                    const float dtA = (lastAcceptedTimeStamp_.nanoseconds() != 0)
                        ? static_cast<float>((timeStamp - lastAcceptedTimeStamp_).seconds()) : 0.0f;
                    publishScanStatus(timeStamp, false, "icp_exception",
                        static_cast<int>(input.getNbPoints()), 0.0f, 0.0f, 0.0f, dtA);
                }
                previousTimeStamp = timeStamp;
                return;
            }

            const double icpMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - icpStart).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "ICP fine: " << icpMs << " ms");

            RCLCPP_DEBUG(this->get_logger(),
                "ICP result: pose={%s}",
                transformSummary(sensorToMapAfterUpdate).c_str());

            if (!maybeConstrainPlanarPose(
                    sensorToMapAfterUpdate,
                    robotToSensor,
                    odomPredictedRobotToMap,
                    "post_icp"))
            {
                mapper->setIsMapping(shouldMap);
                ++consecutiveRejections_;
                ++scansRejected_;
                const float dtA = (lastAcceptedTimeStamp_.nanoseconds() != 0)
                    ? static_cast<float>((timeStamp - lastAcceptedTimeStamp_).seconds()) : 0.0f;
                publishScanStatus(timeStamp, false, "non_planar_pose_rejected",
                    static_cast<int>(input.getNbPoints()), 0.0f, 0.0f, 0.0f, dtA);
                previousTimeStamp = timeStamp;
                return;
            }

            // ── Quality gate ──
            // dtSec pour le gate velocity/yaw = temps depuis le DERNIER SCAN ACCEPTE.
            // previousTimeStamp est mis a jour a chaque scan (meme rejects), donc
            // dtSec = ~50ms toujours. Or translation_correction = derive odom
            // accumulee sur N rejects silencieux. Resultat: correction/50ms = vitesse
            // fantome (ex: 1.3m/50ms=26m/s) → cascade de rejets infinie.
            // On utilise lastAcceptedTimeStamp_ pour que dtSec = temps depuis dernier
            // scan accepte → velocity = taux de derive odom (physiquement sense).
            const float dtSec = dtSecPre;
            const float dtSecAccepted = dtSecAcceptedPre;
            const auto qgResult = qualityGate_.check(
                sensorToMapBeforeUpdate, sensorToMapAfterUpdate,
                static_cast<int>(input.getNbPoints()), dtSecAccepted, icpMs);
            RCLCPP_DEBUG(this->get_logger(),
                "ICP gate: accepted=%d reason='%s' correction=%.3fm %.2fdeg dt_scan=%.3fs dt_accepted=%.3fs vel=%.2fm/s",
                qgResult.accepted,
                qgResult.rejection_reason.c_str(),
                qgResult.translation_correction_m,
                qgResult.rotation_correction_deg,
                dtSec,
                dtSecAccepted,
                qgResult.velocity_ms);

            bool gateRelaxedAccept = false;
            if (!qgResult.accepted)
            {
                std::string adaptiveReason;
                const bool adaptiveAccepted =
                    adaptiveGateAccepts(qgResult, motion, dtSecAccepted, adaptiveReason);
                if (adaptiveAccepted)
                {
                    RCLCPP_WARN(this->get_logger(),
                        "[GATE] %s", adaptiveReason.c_str());
                }
                else if (!adaptiveReason.empty())
                {
                    RCLCPP_WARN(this->get_logger(),
                        "[GATE] %s", adaptiveReason.c_str());
                }
                // Cascade recovery — after N consecutive rejections, relax the quality gate
                // proportionally to how long the cascade has lasted.
                // Rationale: a stale odomToMap accumulates ICP-correction drift that makes
                // corrections appear large even when ICP converged correctly.
                // The relaxation factor ramps from 1.0× at threshold to 3.0× at 30+ rejections,
                // giving the system progressively more room to recover as the cascade deepens.
                // Fundamental failures (too few points, NaN/Inf) are never relaxed.
                // publishedPosePlausible() remains the final safety net regardless.
                const bool rejectionIsFundamental =
                    qgResult.rejection_reason.find("few") != std::string::npos ||
                    qgResult.rejection_reason.find("NaN") != std::string::npos ||
                    qgResult.rejection_reason.find("Inf") != std::string::npos;
                const int threshold = params->recoveryAfterRejections;
                // relaxation_factor: 1.0 at recoveryAfterRejections, 3.0 at 30+ rejections.
                const double relaxation_factor =
                    1.0 + std::min(consecutiveRejections_, 30) / 15.0;
                const bool inCascadeRecovery =
                    !adaptiveAccepted &&
                    threshold > 0 &&
                    consecutiveRejections_ >= threshold &&
                    !rejectionIsFundamental &&
                    qgResult.translation_correction_m <=
                        params->maxTranslationCorrection * relaxation_factor &&
                    qgResult.rotation_correction_deg <=
                        params->maxRotationCorrectionDeg * relaxation_factor;

                if (!adaptiveAccepted && !inCascadeRecovery)
                {
                    mapper->setIsMapping(shouldMap);  // Restaurer: map non modifiee
                    ++consecutiveRejections_;
                    ++scansRejected_;
                    RCLCPP_WARN(this->get_logger(),
                        "Scan rejected by quality gate: %s", qgResult.rejection_reason.c_str());
                    publishScanStatus(timeStamp, false, qgResult.rejection_reason,
                        qgResult.input_points,
                        static_cast<float>(qgResult.translation_correction_m),
                        static_cast<float>(qgResult.rotation_correction_deg),
                        static_cast<float>(qgResult.registration_time_ms),
                        dtSecAccepted);
                    // Advance timestamp so the next scan's dt is one scan interval,
                    // not cumulative from the last accepted scan.
                    previousTimeStamp = timeStamp;
                    return;
                }

                if (inCascadeRecovery)
                {
                    RCLCPP_WARN(this->get_logger(),
                        "CASCADE RECOVERY: overriding quality gate after %d rejections"
                        " (relaxation=%.2fx) — reason='%s' correction=%.2fm/%.1fdeg"
                        " (relaxed limits %.2fm/%.1fdeg)",
                        consecutiveRejections_,
                        relaxation_factor,
                        qgResult.rejection_reason.c_str(),
                        qgResult.translation_correction_m,
                        qgResult.rotation_correction_deg,
                        params->maxTranslationCorrection * relaxation_factor,
                        params->maxRotationCorrectionDeg * relaxation_factor);
                }
                // Fall through — plausibility gate (publishedPosePlausible) still applies.
                gateRelaxedAccept = true;
            }

            PM::TransformationParameters currentOdomToMap =
                transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());

            PM::TransformationParameters robotToMap = sensorToMapAfterUpdate * robotToSensor;
            // Pose prédite par l'odom seul (avant ICP) — référence directionnelle pour
            // détecter les convergences vers la zone dense d'origine (backwards).

            std::string publishedGateReason;
            bool priorConsistentLargeYawStep = false;
            if (!publishedPosePlausible(
                    robotToMap,
                    odomPredictedRobotToMap,
                    dtSecAccepted,
                    publishedGateReason,
                    &priorConsistentLargeYawStep))
            {
                if (odomBridgeAllowed(
                        motion,
                        dtSecAccepted,
                        odomPredictedRobotToMap,
                        publishedGateReason,
                        localizationOnlyReason))
                {
                    localizationOnlyOdomBridge = true;
                    sensorToMapAfterUpdate = sensorToMapBeforeUpdate;
                    robotToMap = odomPredictedRobotToMap;
                    currentOdomToMap =
                        transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());
                    priorConsistentLargeYawStep = false;
                    RCLCPP_WARN(this->get_logger(),
                        "[ODOM_BRIDGE] %s. Publishing odom prior and freezing map insertion.",
                        localizationOnlyReason.c_str());
                }
                else
                {
                    mapper->setIsMapping(shouldMap);  // Restaurer: map non modifiee
                    ++consecutiveRejections_;
                    ++scansRejected_;
                    RCLCPP_WARN(this->get_logger(),
                        "Scan rejected after ICP because published robot pose would jump: %s",
                        publishedGateReason.c_str());
                    publishScanStatus(timeStamp, false, publishedGateReason,
                        qgResult.input_points,
                        static_cast<float>(qgResult.translation_correction_m),
                        static_cast<float>(qgResult.rotation_correction_deg),
                        static_cast<float>(qgResult.registration_time_ms),
                        dtSecAccepted);
                    // Avancer previousTimeStamp pour que le prochain scan ait dt=1 scan,
                    // pas dt accumule depuis le dernier accepte.
                    // NE PAS mettre a jour lastAcceptedRobotToMap_ : la pose rejetee ne
                    // doit pas devenir la reference du prochain check de plausibilite.
                    previousTimeStamp = timeStamp;
                    return;
                }
            }

            if (!localizationOnlyOdomBridge && shouldMap && mapper->getMap().getNbPoints() > 0)
            {
                MapOverlapStats registrationOverlap;
                double registrationOverlapMs = 0.0;
                if (!registrationPoseOverlapsCurrentMap(
                        input, sensorToMapAfterUpdate, registrationOverlap, registrationOverlapMs))
                {
                    std::ostringstream reason;
                    reason << "registration overlap too low: near "
                           << registrationOverlap.nearRatio << " < " << params->minPoseOverlapNearRatio
                           << " or loose " << registrationOverlap.looseRatio << " < " << params->minPoseOverlapLooseRatio;
                    if (odomBridgeAllowed(
                            motion,
                            dtSecAccepted,
                            odomPredictedRobotToMap,
                            reason.str(),
                            localizationOnlyReason))
                    {
                        localizationOnlyOdomBridge = true;
                        sensorToMapAfterUpdate = sensorToMapBeforeUpdate;
                        robotToMap = odomPredictedRobotToMap;
                        currentOdomToMap =
                            transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());
                        RCLCPP_WARN(this->get_logger(),
                            "[ODOM_BRIDGE] %s (overlap near %.3f/%.3f loose %.3f/%.3f sampled=%d check=%.1fms). "
                            "Publishing odom prior and freezing map insertion.",
                            localizationOnlyReason.c_str(),
                            registrationOverlap.nearRatio,
                            params->minPoseOverlapNearRatio,
                            registrationOverlap.looseRatio,
                            params->minPoseOverlapLooseRatio,
                            registrationOverlap.sampled,
                            registrationOverlapMs);
                    }
                    else
                    {
                        mapper->setIsMapping(shouldMap);
                        ++consecutiveRejections_;
                        ++scansRejected_;
                        RCLCPP_WARN(this->get_logger(),
                            "Scan rejected after ICP because aligned scan does not overlap current map enough "
                            "(near %.3f threshold %.3f, loose %.3f threshold %.3f, sampled=%d, time=%.1fms). "
                            "Rejecting odom pose before publication to prevent map/odom cascade.",
                            registrationOverlap.nearRatio,
                            params->minPoseOverlapNearRatio,
                            registrationOverlap.looseRatio,
                            params->minPoseOverlapLooseRatio,
                            registrationOverlap.sampled,
                            registrationOverlapMs);
                        publishScanStatus(timeStamp, false, reason.str(),
                            qgResult.input_points,
                            static_cast<float>(qgResult.translation_correction_m),
                            static_cast<float>(qgResult.rotation_correction_deg),
                            static_cast<float>(qgResult.registration_time_ms),
                            dtSecAccepted);
                        previousTimeStamp = timeStamp;
                        return;
                    }
                }
            }

            // ── Map update: deterministic insertion at the accepted pose ──
            // Never rerun processInput() with mapping enabled here. A second ICP
            // pass can converge to a different local minimum while also inserting
            // points, and the mapper API has no rollback. That exact mismatch
            // creates double walls even when the published trajectory looks good.
            // Instead, insert the accepted scan in the map at Phase-1 pose.
            consecutiveRejections_ = 0;
            ++scansAccepted_;
            publishScanStatus(timeStamp, true,
                localizationOnlyOdomBridge ? localizationOnlyReason : "",
                qgResult.input_points,
                static_cast<float>(qgResult.translation_correction_m),
                static_cast<float>(qgResult.rotation_correction_deg),
                static_cast<float>(qgResult.registration_time_ms),
                dtSecAccepted,
                priorConsistentLargeYawStep);
            publishAlignedScan(input, sensorToMapAfterUpdate, timeStamp);
            if (shouldMap)
            {
                bool mapChanged = false;
                const int mapPtsBeforeDecision =
                    static_cast<int>(mapper->getMap().getNbPoints());
                const char* mapDecision = "not_evaluated";
                std::string mapGateReason;
                const bool mapCorrectionAllowed =
                    mapUpdateCorrectionAllowed(qgResult, motion, dtSecAccepted, mapGateReason);
                const bool motionInsertionRisk =
                    motion.pivot &&
                    (qgResult.translation_correction_m > params->pivotMaxTranslationCorrectionM ||
                     qgResult.rotation_correction_deg > params->maxMapUpdateRotationCorrectionDeg);
                if (localizationOnlyOdomBridge)
                {
                    ++consecutiveOdomBridgeScans_;
                    if (params->allowOdomBridgeMapInsertion)
                    {
                        mapChanged = updateMapFromOdomBridge(input, sensorToMapAfterUpdate);
                        mapDecision = mapChanged ? "odom_bridge_inserted" : "odom_bridge_not_due";
                        if (!mapChanged)
                        {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Skipping map insertion for odom-bridge scan: deterministic update not due. "
                                "Odom/path updated only.");
                        }
                    }
                    else
                    {
                        mapDecision = "odom_bridge_freeze";
                        if (recoverLocalMapForOdomBridge(robotToMap, "odom_bridge_freeze"))
                        {
                            mapDecision = "odom_bridge_recentered";
                            mapChanged = true;
                        }
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Skipping map insertion for odom-bridge scan. Odom/path updated only; "
                            "dead-reckoning scans are not inserted into the ICP map.");
                    }
                }
                else if (mapCorrectionAllowed)
                {
                    consecutiveOdomBridgeScans_ = 0;
                    lastOdomBridgeRecoveryAttempt_ = -1;
                    if (priorConsistentLargeYawStep)
                    {
                        mapDecision = "turn_recovery_freeze";
                        RCLCPP_WARN(this->get_logger(),
                            "Skipping map insertion for accepted turn-recovery scan: large yaw step is "
                            "consistent with odom prior, but scan distortion/low overlap risk is high. "
                            "Odom/path updated; map waits for the next stable scan.");
                    }
                    else if (motionInsertionRisk)
                    {
                        mapDecision = "motion_risk_freeze";
                        RCLCPP_WARN(this->get_logger(),
                            "Skipping map insertion during aggressive motion: speed=%.2fm/s accel=%.2fm/s2 "
                            "yaw_rate=%.1fdeg/s pivot=%d correction=%.3fm/%.1fdeg. Odom/path updated.",
                            motion.odomSpeedMs,
                            motion.odomAccelMs2,
                            motion.dominantYawRateDegS,
                            motion.pivot,
                            qgResult.translation_correction_m,
                            qgResult.rotation_correction_deg);
                    }
                    else
                    {
                        mapChanged = updateMapDeterministically(input, sensorToMapAfterUpdate);
                        mapDecision = mapChanged ? "inserted_or_seeded" : "not_due_or_overlap_skip";
                    }
                }
                else
                {
                    consecutiveOdomBridgeScans_ = 0;
                    lastOdomBridgeRecoveryAttempt_ = -1;
                    mapDecision = "correction_limit_skip";
                    RCLCPP_WARN(this->get_logger(),
                        "Skipping map insertion for accepted scan: %s. Odom/path still published.",
                        mapGateReason.c_str());
                    RCLCPP_DEBUG(this->get_logger(),
                        "Map-update correction details: correction %.3fm %.1fdeg static_limits %.3fm %.1fdeg.",
                        qgResult.translation_correction_m,
                        qgResult.rotation_correction_deg,
                        params->maxMapUpdateTranslationCorrectionM,
                        params->maxMapUpdateRotationCorrectionDeg);
                }
                const int mapPtsAfterDecision =
                    static_cast<int>(mapper->getMap().getNbPoints());
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[MAP_DECISION] decision=%s changed=%d pts=%d->%d correction=%.3fm/%.1fdeg pose={%s}",
                    mapDecision,
                    mapChanged,
                    mapPtsBeforeDecision,
                    mapPtsAfterDecision,
                    qgResult.translation_correction_m,
                    qgResult.rotation_correction_deg,
                    transformSummary(sensorToMapAfterUpdate).c_str());

                // ── Periodic map trimming (legacy, disabled for large-area mapping) ──
                // Permanently crops the global map — discards data outside mapTrimRadiusM_.
                // Disabled (enable_map_trimming=false) in favour of the library's cell-based
                // management: cells outside BUFFER_SIZE are unloaded to disk and reloaded
                // when the robot returns, so ICP always runs on a bounded local map without
                // losing global history. Enable only for short-range sessions where revisiting
                // is not needed and RAM is critically limited.
                if (mapChanged &&
                    params->enableMapTrimming &&
                    scansAccepted_.load() % static_cast<uint64_t>(params->mapTrimIntervalScans) == 0)
                {
                    PM::DataPoints currentMap = mapper->getMap();
                    const int mapPts = static_cast<int>(currentMap.getNbPoints());
                    if (mapPts > params->maxMapPointsBeforeTrim)
                    {
                        const Eigen::Vector2f robotXY = robotToMap.topRightCorner(2, 1);
                        const bool rebuiltFromGlobal =
                            rebuildLocalMapFromGlobalOutput(robotXY, "periodic trim");
                        if (!rebuiltFromGlobal)
                        {
                            PM::DataPoints trimmedMap =
                                cropPointsToRadius(
                                    currentMap,
                                    robotXY,
                                    static_cast<float>(params->mapTrimRadiusM));
                            const int radiusTrimPts = static_cast<int>(trimmedMap.getNbPoints());
                            capCloudPointsDeterministically(trimmedMap, params->maxMapPointsBeforeTrim);
                            setMapperMap(trimmedMap);
                            overlapVoxelCache_ = MapOverlapVoxelCache{};
                            RCLCPP_INFO(this->get_logger(),
                                "Map trimmed: %d → %d → %d pts (radius=%.0fm, cap=%d).",
                                mapPts, radiusTrimPts, static_cast<int>(trimmedMap.getNbPoints()),
                                params->mapTrimRadiusM, params->maxMapPointsBeforeTrim);
                        }
                    }
                }

                if (mapChanged && !localizationOnlyOdomBridge &&
                    !motionInsertionRisk && !priorConsistentLargeYawStep)
                {
                    saveGoodMapSnapshot(
                        sensorToMapAfterUpdate,
                        robotToMap,
                        qgResult,
                        timeStamp,
                        "accepted_map_update");
                }

                mapper->setIsMapping(true);

                // ── Snapshot de la map pour publication — DANS le thread gotInput ──
                // On n'appelle PAS mapper->getNewLocalMap() depuis mapPublisherLoop:
                // getNewLocalPointCloud() peut swapper les buffers internes de Map,
                // ce qui rend isLocalPointCloudEmpty()=true et casse le scan suivant
                // (ICP sauté → saut à l'odom brut → murs doublés).
                // Solution: copier la map ici (thread gotInput, après Phase 2),
                // et laisser mapPublisherLoop lire depuis cette copie sous mutex.
                //
                // Snapshot only when the publisher thread asks for a fresh copy.
                // Copying/subsampling a large map on every insertion is expensive
                // enough to drop ICP odom frequency and create artificial gaps.
                if (seededInitialMap || (mapChanged && needMapSnapshot_.load()))
                {
                    refreshMapPublicationSnapshot(
                        seededInitialMap ? "initial seed" :
                        "deterministic map update");
                }
            }
            else
            {
                mapper->setIsMapping(false);
            }

            // ── Update odom → map transform only after both gates accepted ──
            if (!hasInitialAcceptedRobotToMap_)
            {
                initialAcceptedRobotToMap_ = robotToMap;
                hasInitialAcceptedRobotToMap_ = true;
            }
            lastAcceptedTimeStamp_ = timeStamp;    // advance only on acceptance
            lastAcceptedRobotToMap_ = robotToMap;  // advance only on acceptance
            lastOdomPriorSpeedMs_ = motion.odomSpeedMs;
            hasLastOdomPriorSpeed_ = true;
            lastRecoveryAttemptRejections_ = -1;
            {
                std::lock_guard<std::mutex> lk(mapTfLock);
                odomToMap = currentOdomToMap;
            }

            {
                std::lock_guard<std::mutex> lk(trajectoryMutex_);
                robotTrajectory->addPose(
                    robotToMap,
                    std::chrono::time_point<std::chrono::steady_clock>(
                        std::chrono::nanoseconds(timeStamp.nanoseconds())));
            }

            // ── Publish odometry ──
            nav_msgs::msg::Odometry odomMsgOut =
                PointMatcher_ROS::pointMatcherTransformationToOdomMsg<float>(
                    robotToMap, params->mapFrame, params->robotFrame, timeStamp);

            // ── Honest pose covariance ──
            // Downstream consumers (imu_odom z-correction, fusion backends) need
            // to know how much to trust this pose; an all-zero covariance claims
            // perfection even while dead-reckoning. Coarse heuristic from the
            // registration evidence of this very scan.
            double sigmaXY;
            double sigmaYawRad;
            if (localizationOnlyOdomBridge)
            {
                // Odom-bridge scan: pure odom prior, no ICP evidence.
                sigmaXY = 0.30 + 0.50 * motion.odomSpeedMs * std::max(0.0f, dtSecAccepted);
                sigmaYawRad = 8.0 * M_PI / 180.0;
            }
            else
            {
                sigmaXY = 0.02 + 0.25 * qgResult.translation_correction_m;
                sigmaYawRad = (0.2 + 0.25 * qgResult.rotation_correction_deg) * M_PI / 180.0;
                if (gateRelaxedAccept)
                {
                    // Accepted through adaptive-gate/cascade relaxation only.
                    sigmaXY *= 3.0;
                    sigmaYawRad *= 3.0;
                }
            }
            sigmaXY = std::min(sigmaXY, 5.0);
            sigmaYawRad = std::min(sigmaYawRad, M_PI);
            const double sigmaZ = 2.0 * sigmaXY;              // weakly observed on flat ground
            const double sigmaRollPitch = 2.0 * sigmaYawRad;  // follows prior under force4DOF
            odomMsgOut.pose.covariance[0]  = sigmaXY * sigmaXY;
            odomMsgOut.pose.covariance[7]  = sigmaXY * sigmaXY;
            odomMsgOut.pose.covariance[14] = sigmaZ * sigmaZ;
            odomMsgOut.pose.covariance[21] = sigmaRollPitch * sigmaRollPitch;
            odomMsgOut.pose.covariance[28] = sigmaRollPitch * sigmaRollPitch;
            odomMsgOut.pose.covariance[35] = sigmaYawRad * sigmaYawRad;
            // Angular twist is never estimated here — mark it untrusted.
            odomMsgOut.twist.covariance[21] = 1e3;
            odomMsgOut.twist.covariance[28] = 1e3;
            odomMsgOut.twist.covariance[35] = 1e3;

            if (previousTimeStamp.nanoseconds() != 0)
            {
                const float deltaTime = static_cast<float>((timeStamp - previousTimeStamp).seconds());
                if (deltaTime > 1e-6f)  // Guard against division by zero.
                {
                    Eigen::Vector3f disp =
                        robotToMap.topRightCorner(input.getEuclideanDim(), 1) -
                        previousRobotToMap.topRightCorner(input.getEuclideanDim(), 1);
                    Eigen::Vector3f vel = disp / deltaTime;
                    odomMsgOut.twist.twist.linear.x = vel(0);
                    odomMsgOut.twist.twist.linear.y = vel(1);
                    odomMsgOut.twist.twist.linear.z = vel(2);
                    // Finite difference of two pose draws → var = 2·σ²/dt².
                    const double velVar =
                        2.0 * sigmaXY * sigmaXY /
                        (static_cast<double>(deltaTime) * static_cast<double>(deltaTime));
                    odomMsgOut.twist.covariance[0]  = velVar;
                    odomMsgOut.twist.covariance[7]  = velVar;
                    odomMsgOut.twist.covariance[14] = velVar;
                }
            }
            previousTimeStamp = timeStamp;
            previousRobotToMap = robotToMap;

            odomPublisher->publish(odomMsgOut);
            if (!localizationOnlyOdomBridge)
            {
                icpMeasurementPublisher->publish(odomMsgOut);
            }

            // ── Trajectoire (nav_msgs/Path) pour Foxglove ──
            nav_msgs::msg::Path pathToPublish;
            {
                std::lock_guard<std::mutex> lk(trajectoryMutex_);
                geometry_msgs::msg::PoseStamped poseStamped;
                poseStamped.header.stamp = timeStamp;
                poseStamped.header.frame_id = params->mapFrame;
                poseStamped.pose = odomMsgOut.pose.pose;

                trajectoryPath_.header.stamp = timeStamp;
                trajectoryPath_.header.frame_id = params->mapFrame;
                trajectoryPath_.poses.push_back(poseStamped);

                // Cap: keep the most recent MAX_TRAJECTORY_POSES entries (~83 min at 10 Hz).
                constexpr size_t MAX_TRAJECTORY_POSES = 50000;
                if (trajectoryPath_.poses.size() > MAX_TRAJECTORY_POSES)
                {
                    trajectoryPath_.poses.erase(
                        trajectoryPath_.poses.begin(),
                        trajectoryPath_.poses.begin() +
                            static_cast<long>(trajectoryPath_.poses.size() - MAX_TRAJECTORY_POSES));
                }

                pathToPublish = trajectoryPath_;
            }
            trajectoryPathPublisher->publish(pathToPublish);

            if (!params->publishTfsBetweenRegistrations)
            {
                geometry_msgs::msg::TransformStamped tfMsg =
                    PointMatcher_ROS::pointMatcherTransformationToRosTf<float>(
                        currentOdomToMap, params->mapFrame, params->odomFrame, timeStamp);
                tfBroadcaster->sendTransform(tfMsg);
            }

            {
                std::lock_guard<std::mutex> lk(idleTimeLock);
                lastTimeInputWasProcessed = std::chrono::steady_clock::now();
            }

            // ── Log diagnostique detaille (scan accepte uniquement) ──
            {
                const double totalMs = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - processingStart).count();
                const uint64_t accepted = scansAccepted_.load();
                const uint64_t rejected = scansRejected_.load();
                const uint64_t total = accepted + rejected;
                const int pct = (total > 0) ? static_cast<int>(100 * accepted / total) : 0;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[ICP] pts=%d filter=%.0fms icp=%.0fms total=%.0fms tr=%.3fm rot=%.1fdeg vel=%.1fm/s pose=(%.2f,%.2f,%.1fdeg) map=%lu | accepted=%lu rejected=%lu (%d%%)",
                    static_cast<int>(input.getNbPoints()),
                    filterMs,
                    icpMs,
                    totalMs,
                    qgResult.translation_correction_m,
                    qgResult.rotation_correction_deg,
                    qgResult.velocity_ms,
                    static_cast<double>(robotToMap(0, input.getEuclideanDim())),
                    static_cast<double>(robotToMap(1, input.getEuclideanDim())),
                    yawFromTransform(robotToMap) * 180.0 / M_PI,
                    static_cast<unsigned long>(mapper->getMap().getNbPoints()),
                    accepted, rejected, pct);
                if (totalMs > static_cast<double>(params->maxRegistrationTimeMs))
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "[ICP] slow scan: total=%.0fms > budget=%.0fms (filter=%.0fms icp=%.0fms)",
                        totalMs, static_cast<double>(params->maxRegistrationTimeMs),
                        filterMs, icpMs);
                }
            }
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(),
                "TF lookup failed (scan skipped): %s", ex.what());
        }
        catch (const std::exception& ex)
        {
            ++scansRejected_;
            RCLCPP_ERROR(this->get_logger(),
                "Unexpected mapper exception (scan rejected): stamp=%.9f frame=%s pts=%d: %s",
                static_cast<double>(timeStamp.nanoseconds()) * 1e-9,
                sensorFrame.c_str(),
                static_cast<int>(input.getNbPoints()),
                ex.what());
            previousTimeStamp = timeStamp;
        }
    }

    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2& cloudMsgIn)
    {
        const uint64_t callbackCount = ++pointCloudCallbacksStarted_;
        lastPointCloudStampNs_.store(rclcpp::Time(cloudMsgIn.header.stamp).nanoseconds());
        if (callbackCount <= 5 || callbackCount % 10 == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                "[POINTS_IN] callback start #%lu frame=%s stamp=%.9f size=%ux%u bytes=%zu",
                static_cast<unsigned long>(callbackCount),
                cloudMsgIn.header.frame_id.c_str(),
                static_cast<double>(rclcpp::Time(cloudMsgIn.header.stamp).nanoseconds()) * 1e-9,
                cloudMsgIn.width,
                cloudMsgIn.height,
                cloudMsgIn.data.size());
        }
        if (!isLocalizing_.load()) { return; }
        auto input = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(cloudMsgIn);
        if (callbackCount <= 5 || callbackCount % 10 == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                "[POINTS_IN] converted #%lu pts=%d descriptors=%zu",
                static_cast<unsigned long>(callbackCount),
                static_cast<int>(input.getNbPoints()),
                input.descriptorLabels.size());
        }
        gotInput(input, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp);
        const uint64_t completedCount = ++pointCloudCallbacksCompleted_;
        if (completedCount <= 5 || completedCount % 10 == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                "[POINTS_IN] callback done #%lu accepted=%lu rejected=%lu",
                static_cast<unsigned long>(completedCount),
                static_cast<unsigned long>(scansAccepted_.load()),
                static_cast<unsigned long>(scansRejected_.load()));
        }
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan& scanMsgIn)
    {
        RCLCPP_DEBUG(this->get_logger(), "----LASER SCAN RECEIVED----");
        if (!isLocalizing_.load()) { return; }
        auto input = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(scanMsgIn);
        gotInput(input, scanMsgIn.header.frame_id, scanMsgIn.header.stamp);
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

    void publishDiagnosticsHeartbeat()
    {
        const uint64_t started = pointCloudCallbacksStarted_.load();
        const uint64_t completed = pointCloudCallbacksCompleted_.load();
        const uint64_t accepted = scansAccepted_.load();
        const uint64_t rejected = scansRejected_.load();
        const int64_t stampNs = lastPointCloudStampNs_.load();
        PM::TransformationParameters currentOdomToMap;
        {
            std::lock_guard<std::mutex> lk(mapTfLock);
            currentOdomToMap = odomToMap;
        }
        const std::string lastAccepted = hasInitialAcceptedRobotToMap_
            ? transformSummary(lastAcceptedRobotToMap_)
            : "none";
        RCLCPP_INFO(this->get_logger(),
            "[MAPPER_HEARTBEAT] callbacks=%lu/%lu in_flight=%ld accepted=%lu rejected=%lu "
            "last_cloud_stamp=%.9f map=%lu odom_to_map={%s} last_accepted_robot={%s}",
            static_cast<unsigned long>(completed),
            static_cast<unsigned long>(started),
            static_cast<long>(started) - static_cast<long>(completed),
            static_cast<unsigned long>(accepted),
            static_cast<unsigned long>(rejected),
            static_cast<double>(stampNs) * 1e-9,
            static_cast<unsigned long>(mapper->getMap().getNbPoints()),
            transformSummary(currentOdomToMap).c_str(),
            lastAccepted.c_str());
    }

    void publishTrajectoryPathSnapshot()
    {
        nav_msgs::msg::Path pathToPublish;
        {
            std::lock_guard<std::mutex> lk(trajectoryMutex_);
            if (trajectoryPath_.poses.empty())
            {
                return;
            }
            pathToPublish = trajectoryPath_;
        }
        trajectoryPathPublisher->publish(pathToPublish);
    }

    void mapPublisherLoop()
    {
        // This thread publishes the map snapshot prepared by gotInput() after each
        // accepted Phase-2 update. It NEVER calls mapper->getNewLocalMap() because
        // that function may swap Map's internal double-buffer, making
        // isLocalPointCloudEmpty()=true in the ICP thread (race condition that causes
        // the trajectory to jump to raw odom and corrupts the map with double walls).
        if (params->mapPublishRate <= 0.0f)
        {
            RCLCPP_INFO(this->get_logger(),
                "Map topic publication disabled (map_publish_rate=0). "
                "ICP/map updates still run; use save_map service or final map file for the full map.");
            return;
        }
        rclcpp::Rate publishRate(params->mapPublishRate);
        while (rclcpp::ok() && running_.load())
        {
            PM::DataPoints mapToPublish;
            bool hasMap = false;
            if (mapPublisher->get_subscription_count() > 0)
            {
                std::lock_guard<std::mutex> lk(mapPublishLock_);
                if (latestMapReady_)
                {
                    mapToPublish = latestMapForPublication_;  // copy
                }
                hasMap = (mapToPublish.getNbPoints() > 0);
            }
            if (hasMap)
            {
                sensor_msgs::msg::PointCloud2 mapMsgOut =
                    PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(
                        mapToPublish, params->mapFrame, this->get_clock()->now());
                mapPublisher->publish(mapMsgOut);
                // Demander un nouveau snapshot pour la prochaine publication.
                needMapSnapshot_.store(true);
            }
            publishRate.sleep();
        }
    }

    void mapTfPublisherLoop()
    {
        rclcpp::Rate publishRate(params->mapTfPublishRate);
        auto lastTime = this->get_clock()->now();
        while (rclcpp::ok() && running_.load())
        {
            PM::TransformationParameters currentOdomToMap;
            {
                std::lock_guard<std::mutex> lk(mapTfLock);
                currentOdomToMap = odomToMap;
            }
            auto currTime = this->get_clock()->now();
            if (lastTime != currTime)
            {
                geometry_msgs::msg::TransformStamped tf =
                    PointMatcher_ROS::pointMatcherTransformationToRosTf<float>(
                        currentOdomToMap, params->mapFrame, params->odomFrame, currTime);
                tfBroadcaster->sendTransform(tf);
            }
            lastTime = currTime;
            publishRate.sleep();
        }
    }

    void reloadYamlConfigCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
    	RCLCPP_INFO(this->get_logger(), "Reloading YAML config");
    	mapper->loadYamlConfig(params->mappingConfig);
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
		    const bool haveLocalizedPose = hasInitialAcceptedRobotToMap_;
		    const PM::TransformationParameters liveRobotPose = haveLocalizedPose
		        ? lastAcceptedRobotToMap_
		        : PM::TransformationParameters();
    		loadMap(req->map_file_name.data);
            int homogeneousDim = params->is3D ? 4 : 3;
            PM::TransformationParameters requestedPose =
                PointMatcher_ROS::rosMsgToPointMatcherTransformation<float>(
                    req->pose, homogeneousDim);
            if (params->preserveRobotPoseOnMapLoad && haveLocalizedPose)
            {
                RCLCPP_INFO(this->get_logger(),
                    "Hot map load: preserving live localized robot pose {%s} instead of forcing requested seed {%s}.",
                    transformSummary(liveRobotPose).c_str(),
                    transformSummary(requestedPose).c_str());
                setRobotPose(liveRobotPose);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),
                    "Cold map load: using requested robot pose seed {%s}.",
                    transformSummary(requestedPose).c_str());
                setRobotPose(requestedPose);
            }
    		{
    		    std::lock_guard<std::mutex> lk(trajectoryMutex_);
    		    robotTrajectory->clear();
    		}
    		trajectoryPath_.poses.clear();
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

    void enableMappingCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        RCLCPP_INFO(this->get_logger(), "Enabling mapping");
        isLocalizing_.store(true);
        mappingEnabled_.store(true);
        mapper->setIsMapping(true);
    }

    void disableMappingCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        RCLCPP_INFO(this->get_logger(), "Disabling mapping");
        mappingEnabled_.store(false);
        mapper->setIsMapping(false);
    }

    void enableLocCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        RCLCPP_INFO(this->get_logger(), "Enabling localization");
        isLocalizing_.store(true);
    }

    void disableLocCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        RCLCPP_INFO(this->get_logger(), "Disabling localization");
        mappingEnabled_.store(false);
        if (mapper->getIsMapping()) { mapper->setIsMapping(false); }
        isLocalizing_.store(false);
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
                    {
                        std::lock_guard<std::mutex> lk(mapFilterMutex_);
                        outputMapSubsamplingFilter =
                            PM::get().DataPointsFilterRegistrar.create(
                                "OctreeGridDataPointsFilter",
                                {{"maxSizeByNode", PointMatcherSupport::toParam(voxelSize)}}
                            );
                    }
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
