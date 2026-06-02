#include "NodeParameters.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <set>

NodeParameters::NodeParameters(rclcpp::Node& node)
{
    declareParameters(node);
    retrieveParameters(node);
    parseComplexParameters();
    validateParameters();
}

void NodeParameters::declareParameters(rclcpp::Node& node)
{
    // ── Frame names ───────────────────────────────────────────────────────────
    node.declare_parameter<std::string>("map_frame", "map");
    node.declare_parameter<std::string>("odom_frame", "odom");
    node.declare_parameter<std::string>("robot_frame", "base_link");
    node.declare_parameter<std::string>("filtering_frame", "base_link");

    // ── Map / file I/O ────────────────────────────────────────────────────────
    node.declare_parameter<std::string>("mapping_config", "");
    node.declare_parameter<std::string>("initial_map_file_name", "");
    node.declare_parameter<std::string>("initial_robot_pose", "");
    node.declare_parameter<std::string>("final_map_file_name", "map.vtk");
    node.declare_parameter<std::string>("final_trajectory_file_name", "trajectory.vtk");

    // ── Publish rates ─────────────────────────────────────────────────────────
    node.declare_parameter<float>("map_publish_rate", 0.05f);
    node.declare_parameter<float>("map_tf_publish_rate", 50.0f);

    // ── Offline shutdown ──────────────────────────────────────────────────────
    node.declare_parameter<float>("max_idle_time", 10.0f);

    // ── Mode flags ────────────────────────────────────────────────────────────
    node.declare_parameter<bool>("is_3D", true);
    node.declare_parameter<bool>("is_mapping", true);
    node.declare_parameter<bool>("is_online", true);
    node.declare_parameter<bool>("save_map_cells_on_hard_drive", false);
    node.declare_parameter<bool>("publish_tfs_between_registrations", true);
    node.declare_parameter<bool>("localizing", true);
    node.declare_parameter<bool>("input_qos_reliable", false);

    // ── Deskew ────────────────────────────────────────────────────────────────
    node.declare_parameter<bool>("deskew", false);
    node.declare_parameter<int>("expected_unique_deskewing_TF_number", 4000);
    node.declare_parameter<int>("deskewing_round_to_nanosecs", 50000);
    node.declare_parameter<std::string>("deskew_fixed_frame", "odom");
    node.declare_parameter<std::string>("deskew_time_mode", "absolute_ns");
    node.declare_parameter<std::string>("deskew_time_field", "time");

    // ── TF ────────────────────────────────────────────────────────────────────
    node.declare_parameter<int>("tf_lookup_timeout_ms", 200);

    // ── Map output compression ────────────────────────────────────────────────
    node.declare_parameter<double>("compression_voxel_size", 0.5);

    // ── Quality gate ──────────────────────────────────────────────────────────
    node.declare_parameter<int>("min_input_points", 100);
    node.declare_parameter<double>("max_translation_correction", 2.0);
    node.declare_parameter<double>("max_rotation_correction_deg", 30.0);
    node.declare_parameter<double>("max_velocity_ms", 20.0);
    node.declare_parameter<double>("max_yaw_rate_deg_s", 90.0);
    node.declare_parameter<double>("max_pose_step_m", 2.0);
    node.declare_parameter<double>("max_z_jump_m", 0.75);
    node.declare_parameter<double>("max_registration_time_ms", 5000.0);
    node.declare_parameter<bool>("enable_convergence_error_dump", false);
    node.declare_parameter<int>("recovery_after_rejections", 10);
    node.declare_parameter<double>("deterministic_map_update_distance_m", 0.03);
    node.declare_parameter<double>("deterministic_map_update_yaw_deg", 0.5);
    node.declare_parameter<double>("deterministic_map_min_dist_new_point", 0.03);
    node.declare_parameter<bool>("enable_global_output_map", false);
    node.declare_parameter<double>("global_output_map_min_dist_new_point", 0.05);
    node.declare_parameter<bool>("enable_map_trimming", true);
    node.declare_parameter<int>("map_trim_interval_scans", 10);
    node.declare_parameter<double>("map_trim_radius_m", 40.0);
    node.declare_parameter<int>("max_map_points_before_trim", 120000);
}

void NodeParameters::retrieveParameters(rclcpp::Node& node)
{
    // ── Frame names ───────────────────────────────────────────────────────────
    node.get_parameter("map_frame", mapFrame);
    node.get_parameter("odom_frame", odomFrame);
    node.get_parameter("robot_frame", robotFrame);
    node.get_parameter("filtering_frame", filteringFrame);

    // ── Map / file I/O ────────────────────────────────────────────────────────
    node.get_parameter("mapping_config", mappingConfig);
    node.get_parameter("initial_map_file_name", initialMapFileName);
    node.get_parameter("initial_robot_pose", initialRobotPoseString);
    node.get_parameter("final_map_file_name", finalMapFileName);
    node.get_parameter("final_trajectory_file_name", finalTrajectoryFileName);

    // ── Publish rates ─────────────────────────────────────────────────────────
    node.get_parameter("map_publish_rate", mapPublishRate);
    node.get_parameter("map_tf_publish_rate", mapTfPublishRate);

    // ── Offline shutdown ──────────────────────────────────────────────────────
    node.get_parameter("max_idle_time", maxIdleTime);

    // ── Mode flags ────────────────────────────────────────────────────────────
    node.get_parameter("is_3D", is3D);
    node.get_parameter("is_mapping", isMapping);
    node.get_parameter("is_online", isOnline);
    node.get_parameter("save_map_cells_on_hard_drive", saveMapCellsOnHardDrive);
    node.get_parameter("publish_tfs_between_registrations", publishTfsBetweenRegistrations);
    node.get_parameter("localizing", localizing);
    node.get_parameter("input_qos_reliable", inputQosReliable);

    // ── Deskew ────────────────────────────────────────────────────────────────
    node.get_parameter("deskew", deskew);
    node.get_parameter("expected_unique_deskewing_TF_number", expectedUniqueDeskewingTFNumber);
    node.get_parameter("deskewing_round_to_nanosecs", deskewingRoundToNanoSecs);
    node.get_parameter("deskew_fixed_frame", deskewFixedFrame);
    node.get_parameter("deskew_time_mode", deskewTimeMode);
    node.get_parameter("deskew_time_field", deskewTimeField);

    // ── TF ────────────────────────────────────────────────────────────────────
    node.get_parameter("tf_lookup_timeout_ms", tfLookupTimeoutMs);

    // ── Map output compression ────────────────────────────────────────────────
    node.get_parameter("compression_voxel_size", compressionVoxelSize);

    // ── Quality gate ──────────────────────────────────────────────────────────
    node.get_parameter("min_input_points", minInputPoints);
    node.get_parameter("max_translation_correction", maxTranslationCorrection);
    node.get_parameter("max_rotation_correction_deg", maxRotationCorrectionDeg);
    node.get_parameter("max_velocity_ms", maxVelocityMs);
    node.get_parameter("max_yaw_rate_deg_s", maxYawRateDegS);
    node.get_parameter("max_pose_step_m", maxPoseStepM);
    node.get_parameter("max_z_jump_m", maxZJumpM);
    node.get_parameter("max_registration_time_ms", maxRegistrationTimeMs);
    node.get_parameter("enable_convergence_error_dump", enableConvergenceErrorDump);
    node.get_parameter("recovery_after_rejections", recoveryAfterRejections);
    node.get_parameter("deterministic_map_update_distance_m", deterministicMapUpdateDistanceM);
    node.get_parameter("deterministic_map_update_yaw_deg", deterministicMapUpdateYawDeg);
    node.get_parameter("deterministic_map_min_dist_new_point", deterministicMapMinDistNewPoint);
    node.get_parameter("enable_global_output_map", enableGlobalOutputMap);
    node.get_parameter("global_output_map_min_dist_new_point", globalOutputMapMinDistNewPoint);
    node.get_parameter("enable_map_trimming", enableMapTrimming);
    node.get_parameter("map_trim_interval_scans", mapTrimIntervalScans);
    node.get_parameter("map_trim_radius_m", mapTrimRadiusM);
    node.get_parameter("max_map_points_before_trim", maxMapPointsBeforeTrim);
}

void NodeParameters::validateParameters() const
{
    // ── File existence checks ─────────────────────────────────────────────────
    if (!initialMapFileName.empty())
    {
        std::ifstream ifs(initialMapFileName.c_str());
        if (!ifs.good())
        {
            throw std::runtime_error("Initial map file does not exist: " + initialMapFileName);
        }
    }

    if (!mappingConfig.empty())
    {
        std::ifstream ifs(mappingConfig.c_str());
        if (!ifs.good())
        {
            throw std::runtime_error("Mapping config file does not exist: " + mappingConfig);
        }
    }

    // ── Offline-mode output file checks ───────────────────────────────────────
    if (!isOnline)
    {
        std::ofstream mapOfs(finalMapFileName.c_str(), std::ios_base::app);
        if (!mapOfs.good())
        {
            throw std::runtime_error("Cannot write to final map file: " + finalMapFileName);
        }

        std::ofstream trajectoryOfs(finalTrajectoryFileName.c_str(), std::ios_base::app);
        if (!trajectoryOfs.good())
        {
            throw std::runtime_error("Cannot write to final trajectory file: " + finalTrajectoryFileName);
        }

        if (maxIdleTime < 0.0f)
        {
            throw std::runtime_error("max_idle_time must be non-negative: " + std::to_string(maxIdleTime));
        }
    }

    // ── Rate checks ───────────────────────────────────────────────────────────
    if (mapPublishRate < 0.0f)
    {
        throw std::runtime_error("map_publish_rate must be non-negative: " + std::to_string(mapPublishRate));
    }
    if (publishTfsBetweenRegistrations && mapTfPublishRate <= 0.0f)
    {
        throw std::runtime_error("map_tf_publish_rate must be positive: " + std::to_string(mapTfPublishRate));
    }

    // ── Logic consistency ─────────────────────────────────────────────────────
    if (!isMapping && initialMapFileName.empty())
    {
        throw std::runtime_error(
            "is_mapping=false requires initial_map_file_name to be set.");
    }

    // ── Deskew ────────────────────────────────────────────────────────────────
    if (deskew)
    {
        if (expectedUniqueDeskewingTFNumber <= 0)
        {
            throw std::runtime_error(
                "expected_unique_deskewing_TF_number must be positive: " +
                std::to_string(expectedUniqueDeskewingTFNumber));
        }
        if (deskewingRoundToNanoSecs <= 0)
        {
            throw std::runtime_error(
                "deskewing_round_to_nanosecs must be positive: " +
                std::to_string(deskewingRoundToNanoSecs));
        }
        const std::set<std::string> valid_modes{"absolute_ns", "relative_ns", "relative_s", "auto"};
        if (valid_modes.find(deskewTimeMode) == valid_modes.end())
        {
            throw std::runtime_error(
                "deskew_time_mode must be one of: absolute_ns | relative_ns | relative_s | auto. "
                "Got: " + deskewTimeMode);
        }
    }

    // ── TF ────────────────────────────────────────────────────────────────────
    if (tfLookupTimeoutMs <= 0)
    {
        throw std::runtime_error(
            "tf_lookup_timeout_ms must be positive: " + std::to_string(tfLookupTimeoutMs));
    }

    // ── Compression ───────────────────────────────────────────────────────────
    if (compressionVoxelSize < 0.0)
    {
        throw std::runtime_error(
            "compression_voxel_size must be non-negative: " + std::to_string(compressionVoxelSize));
    }

    // ── Quality gate ──────────────────────────────────────────────────────────
    if (minInputPoints < 0)
    {
        throw std::runtime_error(
            "min_input_points must be non-negative: " + std::to_string(minInputPoints));
    }
    if (maxTranslationCorrection <= 0.0)
    {
        throw std::runtime_error(
            "max_translation_correction must be positive: " + std::to_string(maxTranslationCorrection));
    }
    if (maxRotationCorrectionDeg <= 0.0 || maxRotationCorrectionDeg > 180.0)
    {
        throw std::runtime_error(
            "max_rotation_correction_deg must be in (0, 180]: " + std::to_string(maxRotationCorrectionDeg));
    }
    if (maxVelocityMs <= 0.0)
    {
        throw std::runtime_error(
            "max_velocity_ms must be positive: " + std::to_string(maxVelocityMs));
    }
    if (maxYawRateDegS <= 0.0)
    {
        throw std::runtime_error(
            "max_yaw_rate_deg_s must be positive: " + std::to_string(maxYawRateDegS));
    }
    if (maxPoseStepM <= 0.0)
    {
        throw std::runtime_error(
            "max_pose_step_m must be positive: " + std::to_string(maxPoseStepM));
    }
    if (maxZJumpM <= 0.0)
    {
        throw std::runtime_error(
            "max_z_jump_m must be positive: " + std::to_string(maxZJumpM));
    }
    if (deterministicMapUpdateDistanceM <= 0.0)
    {
        throw std::runtime_error(
            "deterministic_map_update_distance_m must be positive: " +
            std::to_string(deterministicMapUpdateDistanceM));
    }
    if (deterministicMapUpdateYawDeg <= 0.0)
    {
        throw std::runtime_error(
            "deterministic_map_update_yaw_deg must be positive: " +
            std::to_string(deterministicMapUpdateYawDeg));
    }
    if (deterministicMapMinDistNewPoint <= 0.0)
    {
        throw std::runtime_error(
            "deterministic_map_min_dist_new_point must be positive: " +
            std::to_string(deterministicMapMinDistNewPoint));
    }
    if (globalOutputMapMinDistNewPoint <= 0.0)
    {
        throw std::runtime_error(
            "global_output_map_min_dist_new_point must be positive: " +
            std::to_string(globalOutputMapMinDistNewPoint));
    }
    if (mapTrimIntervalScans <= 0)
    {
        throw std::runtime_error(
            "map_trim_interval_scans must be positive: " +
            std::to_string(mapTrimIntervalScans));
    }
    if (mapTrimRadiusM <= 0.0)
    {
        throw std::runtime_error(
            "map_trim_radius_m must be positive: " +
            std::to_string(mapTrimRadiusM));
    }
    if (maxMapPointsBeforeTrim <= 0)
    {
        throw std::runtime_error(
            "max_map_points_before_trim must be positive: " +
            std::to_string(maxMapPointsBeforeTrim));
    }
}

void NodeParameters::parseComplexParameters()
{
    parseInitialRobotPose();
}

void NodeParameters::parseInitialRobotPose()
{
    if (initialRobotPoseString.empty())
    {
        return;
    }

    const int homogeneousDim = is3D ? 4 : 3;
    initialRobotPose = PM::TransformationParameters::Identity(homogeneousDim, homogeneousDim);

    std::string s = initialRobotPoseString;
    s.erase(std::remove(s.begin(), s.end(), '['), s.end());
    s.erase(std::remove(s.begin(), s.end(), ']'), s.end());
    std::replace(s.begin(), s.end(), ',', ' ');
    std::replace(s.begin(), s.end(), ';', ' ');

    // Use std::vector instead of VLA.
    const int dim = homogeneousDim * homogeneousDim;
    std::vector<float> poseMatrix(dim, 0.0f);

    std::istringstream ss(s);
    for (int i = 0; i < dim; ++i)
    {
        if (!(ss >> poseMatrix[i]))
        {
            throw std::runtime_error(
                "Failed to parse initial_robot_pose at element " + std::to_string(i) + ".");
        }
    }

    float extra = 0.0f;
    if (ss >> extra)
    {
        throw std::runtime_error(
            "initial_robot_pose has too many elements for a " +
            std::to_string(homogeneousDim) + "x" + std::to_string(homogeneousDim) + " matrix.");
    }

    for (int i = 0; i < dim; ++i)
    {
        initialRobotPose(i / homogeneousDim, i % homogeneousDim) = poseMatrix[i];
    }
}
