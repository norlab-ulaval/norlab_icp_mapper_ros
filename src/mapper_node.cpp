#include "Deskewer.h"
#include "NodeParameters.h"
#include "RegistrationQualityGate.h"
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_set>
#include <thread>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
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
#include <std_srvs/srv/empty.hpp>

// ══════════════════════════════════════════════════════════════════════════════
// FRAME CONVENTION
//
// Variable naming: aToB = T_B_A  (maps points FROM frame A INTO frame B).
//
//   sensorToOdom            = T_odom_sensor   ← tf2 lookupTransform(odom, sensor)
//   odomToMap               = T_map_odom      ← ICP correction (frozen between accepted scans)
//   sensorToMapBeforeUpdate = T_map_sensor    = T_map_odom * T_odom_sensor
//   sensorToMapAfterUpdate  = T_map_sensor'   (result of ICP optimisation)
//   robotToMap              = T_map_robot      = sensorToMapAfterUpdate * robotToSensor
//   robotToSensor           = T_sensor_robot   (static TF from URDF)
//
// ICP prior:  sensorToMapBeforeUpdate = odomToMap(t_last_accepted) * sensorToOdom(t_now)
// After ICP:  odomToMap_new = sensorToMapAfterUpdate * sensorToOdom^{-1}
//             (= T_map_sensor' * T_sensor_odom = T_map_odom_new)
//
// T notation is column-major: T_B_A * p_A = p_B
// ══════════════════════════════════════════════════════════════════════════════

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

        mapPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", rclcpp::QoS(1).reliable());
        inputFiltersScanPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_after_input_filters", 1);
        deskewingScanPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_after_deskew", 1);
        alignedScanPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "aligned_scan", rclcpp::QoS(1).reliable());
        odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("icp_odom", 50);
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
                {{"knn", PointMatcherSupport::toParam(12)}}
            );

        mapSurfaceNormalFilter_ =
            PM::get().DataPointsFilterRegistrar.create(
                "SurfaceNormalDataPointsFilter",
                {{"knn", PointMatcherSupport::toParam(12)}}
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
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr statusPublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectoryPathPublisher;
    rclcpp::TimerBase::SharedPtr trajectoryPathTimer_;
    rclcpp::TimerBase::SharedPtr diagnosticsTimer_;
    nav_msgs::msg::Path trajectoryPath_;   ///< Accumulated path, published at each accepted scan.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud2Subscription;
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
    // Node-side deterministic insertion avoids the unsafe second ICP pass that
    // was contaminating the map. Spacing is parameterized because replay and
    // low-speed articulated motion need updates before leaving the first scan.
    // Map publication: only publish points within this XY radius of the robot.
    // Reduces a 200K-point global map to ~20-40K points locally visible,
    // cutting Foxglove WebSocket bandwidth by 80-90%.
    static constexpr float mapPublishRadiusM_ = 40.0f;
    // ICP map trimming is parameterized. Keep it enabled online to bound KDTree
    // cost, but disable it for offline ground-truth map generation.
    // Deskewing is disabled for scans acquired during fast articulated turns.
    // At 60 deg/s yaw rate, one 100ms scan spans 6 deg of rotation. At 10m range that
    // shifts points by ~1.05m — comparable to ICP maxDist. The linear TF interpolation
    // in Deskewer introduces errors during non-linear articulation maneuvers that exceed
    // the correction it provides. Disable deskewing above this threshold.
    static constexpr double maxDeskewYawRateDegS_ = 60.0;
    static constexpr double maxMapUpdateTranslationCorrectionM_ = 1.50;
    static constexpr double maxMapUpdateRotationCorrectionDeg_ = 12.0;
    static constexpr double maxMapUpdateYawStepDeg_ = 45.0;
    static constexpr double maxMapUpdateZStepM_ = 0.50;
    static constexpr double mapOverlapNearVoxelM_ = 0.15;
    static constexpr double mapOverlapLooseVoxelM_ = 0.35;
    static constexpr double minPoseOverlapNearRatio_ = 0.25;
    static constexpr double minPoseOverlapLooseRatio_ = 0.45;
    static constexpr double minMapOverlapNearRatio_ = 0.30;
    static constexpr double minMapOverlapLooseRatio_ = 0.50;
    static constexpr int minMapOverlapSamples_ = 200;

    // ── Safe map publication buffer ────────────────────────────────────────────
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
    // Shutdown flag for background threads.
    std::atomic<bool> running_{true};
    // Scan acceptance statistics.
    std::atomic<uint64_t> scansAccepted_{0};
    std::atomic<uint64_t> scansRejected_{0};
    std::atomic<uint64_t> pointCloudCallbacksStarted_{0};
    std::atomic<uint64_t> pointCloudCallbacksCompleted_{0};
    std::atomic<int64_t> lastPointCloudStampNs_{0};
    // Consecutive rejection counter — reset to 0 on each accepted scan.
    // Updated only from the gotInput thread (serialized), so no atomic needed.
    int consecutiveRejections_{0};
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
    std::unique_ptr<Deskewer> deskewer;

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
        float dt_since_accepted_s)
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
        msg.values.push_back(kv("consecutive_rejections",std::to_string(consecutiveRejections_)));
        msg.values.push_back(kv("dt_since_accepted_s",   std::to_string(dt_since_accepted_s)));
        msg.values.push_back(kv("scans_accepted",        std::to_string(scansAccepted_.load())));
        msg.values.push_back(kv("scans_rejected",        std::to_string(scansRejected_.load())));

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
        const double voxelSize)
    {
        std::unordered_set<VoxelKey, VoxelKeyHash> voxels;
        const int nbPoints = static_cast<int>(cloud.getNbPoints());
        voxels.reserve(static_cast<std::size_t>(nbPoints) * 2U);
        for (int col = 0; col < nbPoints; ++col)
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

        const auto nearVoxels = buildVoxelSet(currentMap, mapOverlapNearVoxelM_);
        const auto looseVoxels = buildVoxelSet(currentMap, mapOverlapLooseVoxelM_);
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
                    nearVoxels,
                    voxelKeyForPoint(scanInMapFrame, col, mapOverlapNearVoxelM_)))
            {
                ++stats.nearHits;
            }
            if (voxelNeighborhoodOccupied(
                    looseVoxels,
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
               (stats.nearRatio < minMapOverlapNearRatio_ ||
                stats.looseRatio < minMapOverlapLooseRatio_);
    }

    bool poseOverlapTooLow(const MapOverlapStats& stats) const
    {
        return stats.sampled >= minMapOverlapSamples_ &&
               (stats.nearRatio < minPoseOverlapNearRatio_ ||
                stats.looseRatio < minPoseOverlapLooseRatio_);
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
                lastDeterministicMapUpdatePose_ = acceptedSensorToMap;
                hasDeterministicMapUpdatePose_ = true;
                RCLCPP_WARN(this->get_logger(),
                    "Skipping map insertion: aligned scan does not overlap current map enough "
                    "(near %.3f threshold %.3f, loose %.3f threshold %.3f, sampled=%d). "
                    "The map update baseline is advanced to avoid retrying every scan. "
                    "Check /mapping/aligned_scan in Foxglove: if it is also misaligned, the issue is ICP/odom/TF; "
                    "if it is aligned while /mapping/map is broken, the map insertion path is at fault.",
                    overlap.nearRatio,
                    minMapOverlapNearRatio_,
                    overlap.looseRatio,
                    minMapOverlapLooseRatio_,
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
        mapper->setMap(updatedMap);

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

    bool mapUpdateQualityGood(const RegistrationQualityGate::Result& qgResult) const
    {
        return qgResult.translation_correction_m <= maxMapUpdateTranslationCorrectionM_ &&
               qgResult.rotation_correction_deg <= maxMapUpdateRotationCorrectionDeg_;
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
        mapper->setMap(initialMap);
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

    // ── cropPointsToRadius ────────────────────────────────────────────────────
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

    void refreshMapPublicationSnapshot(const char* reason)
    {
        PM::DataPoints mapSnapshot = mapper->getMap();
        if (mapSnapshot.getNbPoints() == 0) {
            RCLCPP_WARN(this->get_logger(),
                "Map snapshot requested after %s, but mapper map is empty.",
                reason);
            return;
        }

        // View-frustum culling: publish only points within mapPublishRadiusM_ of the
        // current robot position. Reduces message size by 80-90% for large maps,
        // keeping Foxglove WebSocket bandwidth within WiFi budget (~2-3 MB/s vs 12 MB/s).
        const int fullMapPts = static_cast<int>(mapSnapshot.getNbPoints());
        if (hasInitialAcceptedRobotToMap_)
        {
            const Eigen::Vector2f robotXY = lastAcceptedRobotToMap_.topRightCorner(2, 1);
            mapSnapshot = cropPointsToRadius(mapSnapshot, robotXY, mapPublishRadiusM_);
            const int culledPts = static_cast<int>(mapSnapshot.getNbPoints());
            if (culledPts < fullMapPts)
            {
                RCLCPP_DEBUG(this->get_logger(),
                    "Map publication: culled %d → %d pts (radius=%.0fm).",
                    fullMapPts, culledPts, static_cast<double>(mapPublishRadiusM_));
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
        RCLCPP_INFO(this->get_logger(),
            "Refreshed map publication snapshot after %s: full_pts=%d published_pts=%d.",
            reason, fullMapPts, snapshotPoints);
    }

    bool publishedPosePlausible(
        const PM::TransformationParameters& robotToMap,
        const PM::TransformationParameters& odomPredictedRobotToMap,
        double dtSecAccepted,
        std::string& reason) const
    {
        // Compare against last ACCEPTED pose, not previousRobotToMap (which is updated
        // even on rejection). Using a rejected pose as reference allows a cascade of
        // gradually drifting poses to pass the per-step check.
        if (lastAcceptedTimeStamp_.nanoseconds() == 0 || dtSecAccepted <= 1e-6) {
            return true;
        }

        const int dim = static_cast<int>(robotToMap.rows()) - 1;
        const Eigen::VectorXf delta =
            robotToMap.topRightCorner(dim, 1) -
            lastAcceptedRobotToMap_.topRightCorner(dim, 1);
        const double xy = std::hypot(static_cast<double>(delta(0)), static_cast<double>(delta(1)));
        // dtSecAccepted = temps depuis la dernière acceptation — cohérent avec la
        // référence lastAcceptedRobotToMap_. Évite speed=xy/50ms qui rejette les bons
        // scans après cascade de rejets, et laisse passer des pas backwards <0.4m.
        const double speed = xy / dtSecAccepted;
        const double z = (dim >= 3) ? std::abs(static_cast<double>(delta(2))) : 0.0;
        const double yawRateDegS =
            std::abs(wrapToPi(yawFromTransform(robotToMap) - yawFromTransform(lastAcceptedRobotToMap_))) *
            180.0 / M_PI / dtSecAccepted;
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
            std::max(params->maxPoseStepM, params->maxVelocityMs * dtSecAccepted * 1.25);
        if (xy > maxStepForGap) {
            std::ostringstream ss;
            ss << "published pose xy step too high: " << xy << " m > "
               << maxStepForGap << " m (base_limit=" << params->maxPoseStepM
               << " m dt=" << dtSecAccepted << " s)";
            reason = ss.str();
            return false;
        }
        if (speed > params->maxVelocityMs) {
            std::ostringstream ss;
            ss << "published pose speed too high: " << speed << " m/s > "
               << params->maxVelocityMs << " m/s (xy_step=" << xy << " m dt=" << dtSecAccepted << " s)";
            reason = ss.str();
            return false;
        }
        if (yawRateDegS > params->maxYawRateDegS) {
            std::ostringstream ss;
            ss << "published pose yaw rate too high: " << yawRateDegS << " deg/s > "
               << params->maxYawRateDegS << " deg/s";
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
        mapper->setMap(map);
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
        // ── Timestamp validation ───────────────────────────────────────────────
        if (cloudStamp.nanoseconds() == 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Received cloud with zero timestamp. Skipping.");
            return;
        }

        rclcpp::Time timeStamp = cloudStamp;
        const auto processingStart = std::chrono::steady_clock::now();

        try
        {
            // ── Deskew FIRST on raw cloud ─────────────────────────────────────────
            // Doit etre fait AVANT applyInputFilters: VoxelGridDataPointsFilter calcule
            // la moyenne des timestamps (int64) sur les points d'un voxel. Avec des
            // timestamps absolus Hesai ~1.779e18 ns, la somme de >=6 points depasse
            // INT64_MAX (9.22e18) → overflow → valeurs negatives → TF lookup echoue.
            // Sur le nuage brut (~24k points), les timestamps viennent du driver et sont
            // valides (par paquet UDP, ~20-100 valeurs distinctes). Aucun overflow.
            if (params->deskew)
            {
                // Skip deskewing during fast articulated turns. The linear TF interpolation
                // in the Deskewer assumes the robot moves linearly between odom samples (20ms
                // apart at 50Hz). During fast articulation, yaw changes non-linearly and the
                // linear interpolation introduces more distortion than it removes.
                bool deskewAllowed = true;
                if (previousTimeStamp.nanoseconds() != 0 && hasInitialAcceptedRobotToMap_)
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

                if (deskewAllowed && deskewer->deskewCloud(input, sensorFrame))
                {
                    publishAfterDeskew(input, sensorFrame, cloudStamp);
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

            // ── Transform to filtering frame (self-filter bboxes in base_link) ──────
            // filtering_frame est typiquement base_link: les bboxes dans _config.yaml
            // sont exprimees dans ce frame (invariant par rapport a l'orientation du capteur).
            // Le TF hesai_lidar→base_link est statique (URDF fixed joint), toujours disponible.
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

            mapper->applyInputFilters(input);

            // Retransformer dans le frame capteur pour l'ICP.
            if (usingFilteringFrame)
            {
                input.features = sensorToFilteringFrame.inverse() * input.features;
            }

            ensureInputNormals(input);

            RCLCPP_DEBUG_STREAM(this->get_logger(), "Input filters: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - processingStart).count() << " ms");
            RCLCPP_DEBUG(this->get_logger(),
                "Filtered input ready: frame=%s stamp=%.9f pts=%d normal_ok=%d",
                sensorFrame.c_str(),
                static_cast<double>(timeStamp.nanoseconds()) * 1e-9,
                static_cast<int>(input.getNbPoints()),
                input.descriptorExists("normals", static_cast<unsigned>(input.getEuclideanDim())));
            publishAfterInputFilters(input, sensorFrame, cloudStamp);

            // Pas de fallback Time(0) pour une TF dynamique: un prior perime
            // envoie ICP au mauvais endroit et peut causer un jump de pose.
            // Si la TF manque → ExtrapolationException → scan skippe via catch exterieur.
            PM::TransformationParameters sensorToOdom =
                findTransform(sensorFrame, params->odomFrame, timeStamp, input.getHomogeneousDim(),
                              /*allowLatestFallback=*/false);
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
                PM::TransformationParameters sensorToRobot =
                    findTransform(sensorFrame, params->robotFrame, timeStamp, input.getHomogeneousDim());
                sensorToMapBeforeUpdate = robotPoseToSet * sensorToRobot;
                hasToSetRobotPose = false;
            }

            const bool seededInitialMap =
                params->isMapping && seedInitialMapIfNeeded(input, sensorToMapBeforeUpdate);

            // ── ICP Phase 1: localisation (map non modifiee) ────────────────────
            // Une seule passe fine avec prior odom. Le M-estimateur Cauchy dans la
            // chaine outlier filters (_config.yaml) cree un paysage de cout lisse
            // qui evite les minima locaux sans necessiter de passe coarse preliminaire.
            // Budget: ~25ms (fine) + ~5ms (Phase 2) = 30ms < 50ms (20Hz).
            const bool shouldMap = params->isMapping;
            mapper->setIsMapping(false);

            const auto icpStart = std::chrono::steady_clock::now();
            const auto steadyTs = std::chrono::time_point<std::chrono::steady_clock>(
                std::chrono::nanoseconds(timeStamp.nanoseconds()));

            // ── Passe fine (prior = odom, mapping OFF) ──────────────────────────
            PM::TransformationParameters sensorToMapAfterUpdate =
                sensorToMapBeforeUpdate;
            bool usedOdomPriorFallback = false;
            try
            {
                mapper->processInput(input, sensorToMapBeforeUpdate, steadyTs);
                // Copie valeur: apres Phase 2, mapper->getPose() peut changer legerement.
                sensorToMapAfterUpdate = mapper->getPose();
            }
            catch (const PM::ConvergenceError& e)
            {
                bool priorOverlapGood = false;
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
                    // This fallback is intentionally looser than the normal map
                    // update gate. It is used only when ICP fails before returning
                    // a pose; accepting the odom prior for one scan is safer than
                    // letting a corridor/garage aliasing failure freeze map growth.
                    priorOverlapGood =
                        priorOverlap.nearRatio >= 0.15 ||
                        priorOverlap.looseRatio >= 0.30;
                }

                if (priorOverlapGood)
                {
                    usedOdomPriorFallback = true;
                    sensorToMapAfterUpdate = sensorToMapBeforeUpdate;
                    RCLCPP_WARN(this->get_logger(),
                        "ICP convergence failure, but odom prior overlaps current map "
                        "(near %.3f, loose %.3f, sampled=%d, check=%.1fms). "
                        "Accepting odom prior for this scan to keep map growth alive: %s",
                        priorOverlap.nearRatio,
                        priorOverlap.looseRatio,
                        priorOverlap.sampled,
                        priorOverlapMs,
                        e.what());
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
                "%s result: pose={%s}",
                usedOdomPriorFallback ? "Odom prior fallback" : "ICP",
                transformSummary(sensorToMapAfterUpdate).c_str());

            // ── Quality gate ─────────────────────────────────────────────────
            // dtSec pour le gate velocity/yaw = temps depuis le DERNIER SCAN ACCEPTE.
            // previousTimeStamp est mis a jour a chaque scan (meme rejects), donc
            // dtSec = ~50ms toujours. Or translation_correction = derive odom
            // accumulee sur N rejects silencieux. Resultat: correction/50ms = vitesse
            // fantome (ex: 1.3m/50ms=26m/s) → cascade de rejets infinie.
            // On utilise lastAcceptedTimeStamp_ pour que dtSec = temps depuis dernier
            // scan accepte → velocity = taux de derive odom (physiquement sense).
            const float dtSec = (previousTimeStamp.nanoseconds() != 0)
                ? static_cast<float>((timeStamp - previousTimeStamp).seconds()) : 0.0f;
            const float dtSecAccepted = (lastAcceptedTimeStamp_.nanoseconds() != 0)
                ? static_cast<float>((timeStamp - lastAcceptedTimeStamp_).seconds()) : dtSec;
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

            if (!qgResult.accepted)
            {
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
                    threshold > 0 &&
                    consecutiveRejections_ >= threshold &&
                    !rejectionIsFundamental &&
                    qgResult.translation_correction_m <=
                        params->maxTranslationCorrection * relaxation_factor &&
                    qgResult.rotation_correction_deg <=
                        params->maxRotationCorrectionDeg * relaxation_factor;

                if (!inCascadeRecovery)
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
                // Fall through — plausibility gate (publishedPosePlausible) still applies.
            }

            PM::TransformationParameters currentOdomToMap =
                transformation->correctParameters(sensorToMapAfterUpdate * sensorToOdom.inverse());

            PM::TransformationParameters robotToSensor =
                findTransform(params->robotFrame, sensorFrame, timeStamp, input.getHomogeneousDim());
            PM::TransformationParameters robotToMap = sensorToMapAfterUpdate * robotToSensor;
            // Pose prédite par l'odom seul (avant ICP) — référence directionnelle pour
            // détecter les convergences vers la zone dense d'origine (backwards).
            const PM::TransformationParameters odomPredictedRobotToMap =
                sensorToMapBeforeUpdate * robotToSensor;

            std::string publishedGateReason;
            if (!publishedPosePlausible(robotToMap, odomPredictedRobotToMap, dtSecAccepted, publishedGateReason))
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

            if (shouldMap && mapper->getMap().getNbPoints() > 0)
            {
                MapOverlapStats registrationOverlap;
                double registrationOverlapMs = 0.0;
                if (!registrationPoseOverlapsCurrentMap(
                        input, sensorToMapAfterUpdate, registrationOverlap, registrationOverlapMs))
                {
                    mapper->setIsMapping(shouldMap);
                    ++consecutiveRejections_;
                    ++scansRejected_;
                    std::ostringstream reason;
                    reason << "registration overlap too low: near "
                           << registrationOverlap.nearRatio << " < " << minPoseOverlapNearRatio_
                           << " or loose " << registrationOverlap.looseRatio << " < " << minPoseOverlapLooseRatio_;
                    RCLCPP_WARN(this->get_logger(),
                        "Scan rejected after ICP because aligned scan does not overlap current map enough "
                        "(near %.3f threshold %.3f, loose %.3f threshold %.3f, sampled=%d, time=%.1fms). "
                        "Rejecting odom pose before publication to prevent map/odom cascade.",
                        registrationOverlap.nearRatio,
                        minPoseOverlapNearRatio_,
                        registrationOverlap.looseRatio,
                        minPoseOverlapLooseRatio_,
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

            // ── Map update: deterministic insertion at the accepted pose ───────
            // Never rerun processInput() with mapping enabled here. A second ICP
            // pass can converge to a different local minimum while also inserting
            // points, and the mapper API has no rollback. That exact mismatch
            // creates double walls even when the published trajectory looks good.
            // Instead, insert the accepted scan in the map at Phase-1 pose.
            consecutiveRejections_ = 0;
            ++scansAccepted_;
            publishScanStatus(timeStamp, true, "",
                qgResult.input_points,
                static_cast<float>(qgResult.translation_correction_m),
                static_cast<float>(qgResult.rotation_correction_deg),
                static_cast<float>(qgResult.registration_time_ms),
                dtSecAccepted);
            publishAlignedScan(input, sensorToMapAfterUpdate, timeStamp);
            if (shouldMap)
            {
                bool mapChanged = false;
                if (mapUpdateQualityGood(qgResult))
                {
                    mapChanged = updateMapDeterministically(input, sensorToMapAfterUpdate);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(),
                        "Skipping map insertion for accepted scan: ICP correction %.3fm %.1fdeg exceeds map-update limits %.3fm %.1fdeg. Odom/path still published.",
                        qgResult.translation_correction_m,
                        qgResult.rotation_correction_deg,
                        maxMapUpdateTranslationCorrectionM_,
                        maxMapUpdateRotationCorrectionDeg_);
                }

                // ── Periodic map trimming: bound KDTree size for ICP ─────────────
                // Every mapTrimIntervalScans_ accepted scans, trim the global map to
                // mapTrimRadiusM_ around the robot if it has grown too large.
                // This keeps ICP registration time bounded regardless of total mapped area.
                // Note: the trimmed map is still used for ICP; the full history is lost.
                // Acceptable for online mapping where the robot moves forward.
                if (mapChanged &&
                    params->enableMapTrimming &&
                    scansAccepted_.load() % static_cast<uint64_t>(params->mapTrimIntervalScans) == 0)
                {
                    PM::DataPoints currentMap = mapper->getMap();
                    const int mapPts = static_cast<int>(currentMap.getNbPoints());
                    if (mapPts > params->maxMapPointsBeforeTrim)
                    {
                        const Eigen::Vector2f robotXY = robotToMap.topRightCorner(2, 1);
                        PM::DataPoints trimmedMap =
                            cropPointsToRadius(
                                currentMap,
                                robotXY,
                                static_cast<float>(params->mapTrimRadiusM));
                        mapper->setMap(trimmedMap);
                        RCLCPP_INFO(this->get_logger(),
                            "Map trimmed: %d → %d pts (radius=%.0fm, threshold=%d).",
                            mapPts, static_cast<int>(trimmedMap.getNbPoints()),
                            params->mapTrimRadiusM, params->maxMapPointsBeforeTrim);
                    }
                }

                mapper->setIsMapping(true);

                // ── Snapshot de la map pour publication — DANS le thread gotInput ───
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

            // ── Update odom → map transform only after both gates accepted ───
            if (!hasInitialAcceptedRobotToMap_)
            {
                initialAcceptedRobotToMap_ = robotToMap;
                hasInitialAcceptedRobotToMap_ = true;
            }
            lastAcceptedTimeStamp_ = timeStamp;    // advance only on acceptance
            lastAcceptedRobotToMap_ = robotToMap;  // advance only on acceptance
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

            // ── Publish odometry ──────────────────────────────────────────────
            nav_msgs::msg::Odometry odomMsgOut =
                PointMatcher_ROS::pointMatcherTransformationToOdomMsg<float>(
                    robotToMap, params->mapFrame, params->robotFrame, timeStamp);

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
                }
            }
            previousTimeStamp = timeStamp;
            previousRobotToMap = robotToMap;

            odomPublisher->publish(odomMsgOut);

            // ── Trajectoire (nav_msgs/Path) pour Foxglove ──────────────────────
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

            // ── Log diagnostique detaille (scan accepte uniquement) ───────────
            {
                const uint64_t accepted = scansAccepted_.load();
                const uint64_t rejected = scansRejected_.load();
                const uint64_t total = accepted + rejected;
                const int pct = (total > 0) ? static_cast<int>(100 * accepted / total) : 0;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[ICP] pts=%d icp=%.0fms tr=%.3fm rot=%.1fdeg vel=%.1fm/s pose=(%.2f,%.2f,%.1fdeg) map=%lu | accepted=%lu rejected=%lu (%d%%)",
                    static_cast<int>(input.getNbPoints()),
                    icpMs,
                    qgResult.translation_correction_m,
                    qgResult.rotation_correction_deg,
                    qgResult.velocity_ms,
                    static_cast<double>(robotToMap(0, input.getEuclideanDim())),
                    static_cast<double>(robotToMap(1, input.getEuclideanDim())),
                    yawFromTransform(robotToMap) * 180.0 / M_PI,
                    static_cast<unsigned long>(mapper->getMap().getNbPoints()),
                    accepted, rejected, pct);
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
        RCLCPP_INFO(this->get_logger(),
            "[MAPPER_HEARTBEAT] callbacks=%lu/%lu in_flight=%ld accepted=%lu rejected=%lu last_cloud_stamp=%.9f map=%lu",
            static_cast<unsigned long>(completed),
            static_cast<unsigned long>(started),
            static_cast<long>(started) - static_cast<long>(completed),
            static_cast<unsigned long>(accepted),
            static_cast<unsigned long>(rejected),
            static_cast<double>(stampNs) * 1e-9,
            static_cast<unsigned long>(mapper->getMap().getNbPoints()));
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
    		loadMap(req->map_file_name.data);
            int homogeneousDim = params->is3D ? 4 : 3;
            setRobotPose(PointMatcher_ROS::rosMsgToPointMatcherTransformation<float>(req->pose, homogeneousDim));
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
        mapper->setIsMapping(true);
    }

    void disableMappingCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        RCLCPP_INFO(this->get_logger(), "Disabling mapping");
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
