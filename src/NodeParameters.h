#ifndef NODE_PARAMETERS_H
#define NODE_PARAMETERS_H

#include <rclcpp/rclcpp.hpp>
#include <norlab_icp_mapper/Mapper.h>
#include <string>

// ──────────────────────────────────────────────────────────────────────────────
// NodeParameters
//
// All ROS 2 node parameters declared, retrieved, and validated in one place.
// Parameters are declared with default values and documented here.
// ──────────────────────────────────────────────────────────────────────────────
class NodeParameters
{
private:
    typedef PointMatcher<float> PM;

    void declareParameters(rclcpp::Node& node);
    void retrieveParameters(rclcpp::Node& node);
    void validateParameters() const;
    void parseComplexParameters();
    void parseInitialRobotPose();

public:
    // ── Frame names ───────────────────────────────────────────────────────────
    std::string mapFrame;         ///< Frame of the published map and odom TF. Default: "map"
    std::string odomFrame;        ///< Odometry frame. Default: "odom"
    std::string robotFrame;       ///< Robot center frame (odom child). Default: "base_link"
    std::string filteringFrame;   ///< Frame in which self-filter bboxes are defined. Default: "base_link"

    // ── Map / file I/O ────────────────────────────────────────────────────────
    std::string mappingConfig;
    std::string initialMapFileName;
    std::string initialRobotPoseString;
    PM::TransformationParameters initialRobotPose;
    std::string finalMapFileName;
    std::string finalTrajectoryFileName;

    // ── Publish rates ─────────────────────────────────────────────────────────
    float mapPublishRate;         ///< Hz at which map is published. Default: 1.0
    float mapTfPublishRate;       ///< Hz at which map TF is republished between registrations. Default: 50.0

    // ── Offline shutdown ──────────────────────────────────────────────────────
    float maxIdleTime;            ///< Seconds of inactivity before shutdown (offline mode). Default: 10.0

    // ── Mode flags ────────────────────────────────────────────────────────────
    bool is3D;
    bool isMapping;
    bool isOnline;
    bool saveMapCellsOnHardDrive;
    bool publishTfsBetweenRegistrations;
    bool localizing;
    bool inputQosReliable;

    // ── Deskew ────────────────────────────────────────────────────────────────
    bool deskew;
    int expectedUniqueDeskewingTFNumber;  ///< Reserve this many slots in TF cache. Default: 4000
    int deskewingRoundToNanoSecs;         ///< Bin point timestamps to this resolution. Default: 50000
    std::string deskewFixedFrame;         ///< Fixed frame for deskew TF interpolation. Default: "odom"
    std::string deskewTimeMode;           ///< absolute_ns | relative_ns | relative_s | auto. Default: "absolute_ns"
    std::string deskewTimeField;          ///< Name of the time field in the DataPoints. Default: "time"
    // IMU-driven rotation-only deskew (avoids odom-error→swirl coupling).
    std::string deskewSource;    ///< "tf" (default, uses odom) | "imu" (gyro rotation-only, replay).
    std::string deskewImuTopic;  ///< IMU topic for gyro deskew. Default: "/mti100/data"
    std::string deskewImuFrame;  ///< IMU link frame for static extrinsic lookup. Default: "imu_link"

    // ── TF ────────────────────────────────────────────────────────────────────
    int tfLookupTimeoutMs;        ///< Timeout for TF lookups in milliseconds. Default: 200

    // ── Map output compression ────────────────────────────────────────────────
    double compressionVoxelSize;  ///< Octree voxel size for published map. 0=disabled. Default: 0.5

    // ── Map publication crop ──────────────────────────────────────────────────
    double mapPublishRadiusM;  ///< Crop published cloud to this XY radius around robot. 0=full map. Default: 0.0

    // ── Quality gate ──────────────────────────────────────────────────────────
    int minInputPoints;                ///< Reject scans with fewer points. Default: 100
    double maxTranslationCorrection;   ///< Max ICP translation correction in meters. Default: 2.0
    double maxRotationCorrectionDeg;   ///< Max ICP rotation correction in degrees. Default: 30.0
    double maxVelocityMs;              ///< Max robot velocity in m/s. Default: 20.0
    double maxYawRateDegS;             ///< Max yaw rate in deg/s. Default: 90.0
    double maxPoseYawStepDeg;          ///< Max absolute yaw step per accepted scan regardless of dt. Default: 30.0
    double maxPoseStepM;               ///< Max published XY step between accepted scans. Default: 2.0
    double maxZJumpM;                  ///< Max published robot Z jump between accepted scans. Default: 0.75
    double maxRegistrationTimeMs;      ///< Max tolerated ICP wall time in ms. Default: 5000.0
    bool enableConvergenceErrorDump;   ///< Save map/traj on convergence error. Default: false
    int recoveryAfterRejections;       ///< Override quality gate after this many consecutive rejections (0=disabled). Default: 10
    double deterministicMapUpdateDistanceM; ///< Deterministic map insertion spacing in meters. Default: 0.10
    double deterministicMapUpdateYawDeg;    ///< Deterministic map insertion yaw spacing in degrees. Default: 3.0
    double deterministicMapMinDistNewPoint; ///< New-map-point spacing for deterministic insertion. Default: 0.05
    bool enableGlobalOutputMap;             ///< Keep an untrimmed output map while ICP uses the local internal map. Default: false
    double globalOutputMapMinDistNewPoint;  ///< New-point spacing for the untrimmed output map. Default: 0.05
    bool enableMapTrimming;                 ///< Destructively trim internal map to bound ICP cost. Default: true
    int mapTrimIntervalScans;               ///< Accepted-scan interval between map trim checks. Default: 10
    double mapTrimRadiusM;                  ///< XY radius kept around robot during trimming. Default: 40.0
    int maxMapPointsBeforeTrim;             ///< Only trim when internal map exceeds this point count. Default: 120000
    double minPoseOverlapNearRatio;         ///< Reject odom pose when aligned-scan near overlap is below this. Default: 0.25
    double minPoseOverlapLooseRatio;        ///< Reject odom pose when aligned-scan loose overlap is below this. Default: 0.45
    double minMapOverlapNearRatio;          ///< Skip map insertion when aligned-scan near overlap is below this. Default: 0.30
    double minMapOverlapLooseRatio;         ///< Skip map insertion when aligned-scan loose overlap is below this. Default: 0.50
    double maxMapUpdateTranslationCorrectionM; ///< Skip map insertion when ICP translation correction exceeds this. Default: 1.50
    double maxMapUpdateRotationCorrectionDeg;  ///< Skip map insertion when ICP rotation correction exceeds this. Default: 12.0

    explicit NodeParameters(rclcpp::Node& node);
};

#endif  // NODE_PARAMETERS_H
