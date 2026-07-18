#include "NodeParameters.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <set>
#include <cmath>

NodeParameters::NodeParameters(rclcpp::Node& node)
{
    declareParameters(node);
    retrieveParameters(node);
    parseComplexParameters();
    validateParameters();
}

void NodeParameters::declareParameters(rclcpp::Node& node)
{
    // ── Frame names ──
    node.declare_parameter<std::string>("map_frame", "map");
    node.declare_parameter<std::string>("odom_frame", "odom");
    node.declare_parameter<std::string>("robot_frame", "base_link");
    node.declare_parameter<std::string>("filtering_frame", "base_link");

    // ── Map / file I/O ──
    node.declare_parameter<std::string>("mapping_config", "");
    node.declare_parameter<std::string>("initial_map_file_name", "");
    node.declare_parameter<std::string>("initial_robot_pose", "");
    node.declare_parameter<std::string>("final_map_file_name", "map.vtk");
    node.declare_parameter<std::string>("final_trajectory_file_name", "trajectory.vtk");

    // ── Publish rates ──
    node.declare_parameter<float>("map_publish_rate", 0.05f);
    node.declare_parameter<float>("map_tf_publish_rate", 50.0f);

    // ── Offline shutdown ──
    node.declare_parameter<float>("max_idle_time", 10.0f);

    // ── Mode flags ──
    node.declare_parameter<bool>("is_3D", true);
    node.declare_parameter<bool>("is_mapping", true);
    node.declare_parameter<bool>("is_online", true);
    node.declare_parameter<bool>("save_map_cells_on_hard_drive", false);
    node.declare_parameter<bool>("publish_tfs_between_registrations", true);
    node.declare_parameter<bool>("localizing", true);
    node.declare_parameter<bool>("input_qos_reliable", false);
    node.declare_parameter<bool>("anchor_map_at_initial_robot_pose", false);
    // Teach-and-repeat normally reloads a snapshot of the map currently in use.
    // Preserve the live localized pose in that case; forcing the recorded route
    // start pose made a robot 2-6 m away appear to be exactly at the start.
    // If no accepted pose exists yet (cold start), the LoadMap request pose is
    // still used as the initial localization prior.
    node.declare_parameter<bool>("preserve_robot_pose_on_map_load", true);

    // ── Deskew ──
    node.declare_parameter<bool>("deskew", false);
    node.declare_parameter<int>("expected_unique_deskewing_TF_number", 4000);
    node.declare_parameter<int>("deskewing_round_to_nanosecs", 50000);
    node.declare_parameter<std::string>("deskew_fixed_frame", "odom");
    node.declare_parameter<std::string>("deskew_time_mode", "absolute_ns");
    node.declare_parameter<std::string>("deskew_time_field", "time");
    // IMU-driven rotation-only deskew. "tf" = legacy odom-based; "imu" = gyro-based.
    // Replay should use "imu" to avoid odom-error→swirl in the map.
    node.declare_parameter<std::string>("deskew_source", "tf");
    node.declare_parameter<std::string>("deskew_imu_topic", "/mti100/data");
    node.declare_parameter<std::string>("deskew_imu_frame", "imu_link");

    // ── TF ──
    node.declare_parameter<int>("tf_lookup_timeout_ms", 200);

    // ── Map output compression ──
    node.declare_parameter<double>("compression_voxel_size", 0.5);

    // ── Map publication crop ──
    // 0.0 = publish full map. Set to e.g. 40.0 to crop to a 40m bubble around
    // the robot — reduces Foxglove WebSocket bandwidth by ~80-90% on large maps.
    node.declare_parameter<std::string>("map_publication_source", "auto");
    node.declare_parameter<double>("map_publish_radius_m", 0.0);

    // ── Quality gate ──
    node.declare_parameter<int>("min_input_points", 100);
    node.declare_parameter<double>("max_translation_correction", 2.0);
    node.declare_parameter<double>("max_rotation_correction_deg", 30.0);
    node.declare_parameter<double>("max_velocity_ms", 20.0);
    node.declare_parameter<double>("max_yaw_rate_deg_s", 90.0);
    node.declare_parameter<double>("max_pose_yaw_step_deg", 30.0);
    node.declare_parameter<double>("max_pose_yaw_odom_residual_deg", 12.0);
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
    node.declare_parameter<double>("min_pose_overlap_near_ratio", 0.25);
    node.declare_parameter<double>("min_pose_overlap_loose_ratio", 0.45);
    node.declare_parameter<double>("min_map_overlap_near_ratio", 0.30);
    node.declare_parameter<double>("min_map_overlap_loose_ratio", 0.50);
    node.declare_parameter<double>("max_map_update_translation_correction_m", 1.50);
    node.declare_parameter<double>("max_map_update_rotation_correction_deg", 12.0);
    node.declare_parameter<bool>("enable_map_recovery", true);
    node.declare_parameter<int>("recovery_reload_after_rejections", 4);
    node.declare_parameter<int>("recovery_attempt_interval_scans", 5);
    node.declare_parameter<double>("recovery_local_map_radius_m", 45.0);
    node.declare_parameter<int>("recovery_local_map_min_points", 5000);
    node.declare_parameter<int>("recovery_local_map_max_points", 60000);
    node.declare_parameter<int>("snapshot_save_interval_scans", 20);
    node.declare_parameter<double>("snapshot_max_translation_correction_m", 1.0);
    node.declare_parameter<double>("snapshot_max_rotation_correction_deg", 5.0);
    node.declare_parameter<bool>("enable_motion_adaptive_gate", true);
    node.declare_parameter<double>("adaptive_max_dt_s", 2.0);
    node.declare_parameter<double>("adaptive_velocity_gain", 1.25);
    node.declare_parameter<double>("adaptive_acceleration_gain", 0.50);
    node.declare_parameter<double>("adaptive_yaw_rate_gain", 1.25);
    node.declare_parameter<double>("aggressive_speed_ms", 2.0);
    node.declare_parameter<double>("aggressive_yaw_rate_deg_s", 35.0);
    node.declare_parameter<double>("pivot_linear_speed_ms", 0.75);
    node.declare_parameter<double>("pivot_yaw_rate_deg_s", 35.0);
    node.declare_parameter<double>("pivot_max_translation_correction_m", 1.25);
    node.declare_parameter<bool>("enable_odom_bridge", true);
    node.declare_parameter<int>("odom_bridge_after_rejections", 0);
    node.declare_parameter<double>("odom_bridge_min_speed_ms", 1.5);
    node.declare_parameter<bool>("allow_odom_bridge_map_insertion", false);
    node.declare_parameter<bool>("enable_planar_pose_constraint", false);
    node.declare_parameter<double>("planar_pose_max_z_drift_m", 2.0);

    // ── Dynamic trailer self-filter ──
    node.declare_parameter<bool>("enable_dynamic_trailer_self_filter", true);
    node.declare_parameter<std::string>("dynamic_trailer_articulation_topic", "/mtt_articulation_angle");
    node.declare_parameter<double>("dynamic_trailer_stale_timeout_s", 0.5);
    node.declare_parameter<double>("dynamic_trailer_yaw_offset_rad", M_PI);
    node.declare_parameter<double>("dynamic_trailer_yaw_sign", -1.0);
    node.declare_parameter<double>("dynamic_trailer_hitch_x", -1.45);
    node.declare_parameter<double>("dynamic_trailer_hitch_y", -0.085);
    node.declare_parameter<double>("dynamic_trailer_front_offset_m", -0.15);
    node.declare_parameter<double>("dynamic_trailer_rear_offset_m", 2.20);
    node.declare_parameter<double>("dynamic_trailer_half_width_m", 1.20);
    node.declare_parameter<double>("dynamic_trailer_z_min_m", -0.35);
    node.declare_parameter<double>("dynamic_trailer_z_max_m", 2.50);
}

void NodeParameters::retrieveParameters(rclcpp::Node& node)
{
    // ── Frame names ──
    node.get_parameter("map_frame", mapFrame);
    node.get_parameter("odom_frame", odomFrame);
    node.get_parameter("robot_frame", robotFrame);
    node.get_parameter("filtering_frame", filteringFrame);

    // ── Map / file I/O ──
    node.get_parameter("mapping_config", mappingConfig);
    node.get_parameter("initial_map_file_name", initialMapFileName);
    node.get_parameter("initial_robot_pose", initialRobotPoseString);
    node.get_parameter("final_map_file_name", finalMapFileName);
    node.get_parameter("final_trajectory_file_name", finalTrajectoryFileName);

    // ── Publish rates ──
    node.get_parameter("map_publish_rate", mapPublishRate);
    node.get_parameter("map_tf_publish_rate", mapTfPublishRate);

    // ── Offline shutdown ──
    node.get_parameter("max_idle_time", maxIdleTime);

    // ── Mode flags ──
    node.get_parameter("is_3D", is3D);
    node.get_parameter("is_mapping", isMapping);
    node.get_parameter("is_online", isOnline);
    node.get_parameter("save_map_cells_on_hard_drive", saveMapCellsOnHardDrive);
    node.get_parameter("publish_tfs_between_registrations", publishTfsBetweenRegistrations);
    node.get_parameter("localizing", localizing);
    node.get_parameter("input_qos_reliable", inputQosReliable);
    node.get_parameter("anchor_map_at_initial_robot_pose", anchorMapAtInitialRobotPose);
    node.get_parameter("preserve_robot_pose_on_map_load", preserveRobotPoseOnMapLoad);

    // ── Deskew ──
    node.get_parameter("deskew", deskew);
    node.get_parameter("expected_unique_deskewing_TF_number", expectedUniqueDeskewingTFNumber);
    node.get_parameter("deskewing_round_to_nanosecs", deskewingRoundToNanoSecs);
    node.get_parameter("deskew_fixed_frame", deskewFixedFrame);
    node.get_parameter("deskew_time_mode", deskewTimeMode);
    node.get_parameter("deskew_time_field", deskewTimeField);
    node.get_parameter("deskew_source", deskewSource);
    node.get_parameter("deskew_imu_topic", deskewImuTopic);
    node.get_parameter("deskew_imu_frame", deskewImuFrame);

    // ── TF ──
    node.get_parameter("tf_lookup_timeout_ms", tfLookupTimeoutMs);

    // ── Map output compression ──
    node.get_parameter("compression_voxel_size", compressionVoxelSize);

    // ── Map publication crop ──
    node.get_parameter("map_publication_source", mapPublicationSource);
    node.get_parameter("map_publish_radius_m", mapPublishRadiusM);

    // ── Quality gate ──
    node.get_parameter("min_input_points", minInputPoints);
    node.get_parameter("max_translation_correction", maxTranslationCorrection);
    node.get_parameter("max_rotation_correction_deg", maxRotationCorrectionDeg);
    node.get_parameter("max_velocity_ms", maxVelocityMs);
    node.get_parameter("max_yaw_rate_deg_s", maxYawRateDegS);
    node.get_parameter("max_pose_yaw_step_deg", maxPoseYawStepDeg);
    node.get_parameter("max_pose_yaw_odom_residual_deg", maxPoseYawOdomResidualDeg);
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
    node.get_parameter("min_pose_overlap_near_ratio", minPoseOverlapNearRatio);
    node.get_parameter("min_pose_overlap_loose_ratio", minPoseOverlapLooseRatio);
    node.get_parameter("min_map_overlap_near_ratio", minMapOverlapNearRatio);
    node.get_parameter("min_map_overlap_loose_ratio", minMapOverlapLooseRatio);
    node.get_parameter("max_map_update_translation_correction_m", maxMapUpdateTranslationCorrectionM);
    node.get_parameter("max_map_update_rotation_correction_deg", maxMapUpdateRotationCorrectionDeg);
    node.get_parameter("enable_map_recovery", enableMapRecovery);
    node.get_parameter("recovery_reload_after_rejections", recoveryReloadAfterRejections);
    node.get_parameter("recovery_attempt_interval_scans", recoveryAttemptIntervalScans);
    node.get_parameter("recovery_local_map_radius_m", recoveryLocalMapRadiusM);
    node.get_parameter("recovery_local_map_min_points", recoveryLocalMapMinPoints);
    node.get_parameter("recovery_local_map_max_points", recoveryLocalMapMaxPoints);
    node.get_parameter("snapshot_save_interval_scans", snapshotSaveIntervalScans);
    node.get_parameter("snapshot_max_translation_correction_m", snapshotMaxTranslationCorrectionM);
    node.get_parameter("snapshot_max_rotation_correction_deg", snapshotMaxRotationCorrectionDeg);
    node.get_parameter("enable_motion_adaptive_gate", enableMotionAdaptiveGate);
    node.get_parameter("adaptive_max_dt_s", adaptiveMaxDtS);
    node.get_parameter("adaptive_velocity_gain", adaptiveVelocityGain);
    node.get_parameter("adaptive_acceleration_gain", adaptiveAccelerationGain);
    node.get_parameter("adaptive_yaw_rate_gain", adaptiveYawRateGain);
    node.get_parameter("aggressive_speed_ms", aggressiveSpeedMs);
    node.get_parameter("aggressive_yaw_rate_deg_s", aggressiveYawRateDegS);
    node.get_parameter("pivot_linear_speed_ms", pivotLinearSpeedMs);
    node.get_parameter("pivot_yaw_rate_deg_s", pivotYawRateDegS);
    node.get_parameter("pivot_max_translation_correction_m", pivotMaxTranslationCorrectionM);
    node.get_parameter("enable_odom_bridge", enableOdomBridge);
    node.get_parameter("odom_bridge_after_rejections", odomBridgeAfterRejections);
    node.get_parameter("odom_bridge_min_speed_ms", odomBridgeMinSpeedMs);
    node.get_parameter("allow_odom_bridge_map_insertion", allowOdomBridgeMapInsertion);
    node.get_parameter("enable_planar_pose_constraint", enablePlanarPoseConstraint);
    node.get_parameter("planar_pose_max_z_drift_m", planarPoseMaxZDriftM);
    node.get_parameter("enable_dynamic_trailer_self_filter", enableDynamicTrailerSelfFilter);
    node.get_parameter("dynamic_trailer_articulation_topic", dynamicTrailerArticulationTopic);
    node.get_parameter("dynamic_trailer_stale_timeout_s", dynamicTrailerStaleTimeoutS);
    node.get_parameter("dynamic_trailer_yaw_offset_rad", dynamicTrailerYawOffsetRad);
    node.get_parameter("dynamic_trailer_yaw_sign", dynamicTrailerYawSign);
    node.get_parameter("dynamic_trailer_hitch_x", dynamicTrailerHitchX);
    node.get_parameter("dynamic_trailer_hitch_y", dynamicTrailerHitchY);
    node.get_parameter("dynamic_trailer_front_offset_m", dynamicTrailerFrontOffsetM);
    node.get_parameter("dynamic_trailer_rear_offset_m", dynamicTrailerRearOffsetM);
    node.get_parameter("dynamic_trailer_half_width_m", dynamicTrailerHalfWidthM);
    node.get_parameter("dynamic_trailer_z_min_m", dynamicTrailerZMinM);
    node.get_parameter("dynamic_trailer_z_max_m", dynamicTrailerZMaxM);
}

void NodeParameters::validateParameters() const
{
    // ── File existence checks ──
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

    // ── Offline-mode output file checks ──
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

    // ── Rate checks ──
    if (mapPublishRate < 0.0f)
    {
        throw std::runtime_error("map_publish_rate must be non-negative: " + std::to_string(mapPublishRate));
    }
    if (publishTfsBetweenRegistrations && mapTfPublishRate <= 0.0f)
    {
        throw std::runtime_error("map_tf_publish_rate must be positive: " + std::to_string(mapTfPublishRate));
    }
    if (mapPublicationSource != "auto" &&
        mapPublicationSource != "local" &&
        mapPublicationSource != "global")
    {
        throw std::runtime_error(
            "map_publication_source must be one of: auto, local, global. Got: " +
            mapPublicationSource);
    }

    // ── Logic consistency ──
    if (!isMapping && initialMapFileName.empty())
    {
        throw std::runtime_error(
            "is_mapping=false requires initial_map_file_name to be set.");
    }

    // ── Deskew ──
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
        const std::set<std::string> valid_sources{"tf", "imu"};
        if (valid_sources.find(deskewSource) == valid_sources.end())
        {
            throw std::runtime_error(
                "deskew_source must be 'tf' or 'imu'. Got: " + deskewSource);
        }
        if (deskewSource == "imu" && deskewImuTopic.empty())
        {
            throw std::runtime_error(
                "deskew_imu_topic must be set when deskew_source=imu.");
        }
    }

    // ── TF ──
    if (tfLookupTimeoutMs <= 0)
    {
        throw std::runtime_error(
            "tf_lookup_timeout_ms must be positive: " + std::to_string(tfLookupTimeoutMs));
    }

    // ── Compression ──
    if (compressionVoxelSize < 0.0)
    {
        throw std::runtime_error(
            "compression_voxel_size must be non-negative: " + std::to_string(compressionVoxelSize));
    }

    // ── Quality gate ──
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
    if (maxPoseYawStepDeg <= 0.0 || maxPoseYawStepDeg > 180.0)
    {
        throw std::runtime_error(
            "max_pose_yaw_step_deg must be in (0, 180]: " + std::to_string(maxPoseYawStepDeg));
    }
    if (maxPoseYawOdomResidualDeg <= 0.0 || maxPoseYawOdomResidualDeg > 180.0)
    {
        throw std::runtime_error(
            "max_pose_yaw_odom_residual_deg must be in (0, 180]: " +
            std::to_string(maxPoseYawOdomResidualDeg));
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
    if (recoveryReloadAfterRejections < 0)
    {
        throw std::runtime_error(
            "recovery_reload_after_rejections must be non-negative: " +
            std::to_string(recoveryReloadAfterRejections));
    }
    if (recoveryAttemptIntervalScans <= 0)
    {
        throw std::runtime_error(
            "recovery_attempt_interval_scans must be positive: " +
            std::to_string(recoveryAttemptIntervalScans));
    }
    if (recoveryLocalMapRadiusM <= 0.0)
    {
        throw std::runtime_error(
            "recovery_local_map_radius_m must be positive: " +
            std::to_string(recoveryLocalMapRadiusM));
    }
    if (recoveryLocalMapMinPoints <= 0)
    {
        throw std::runtime_error(
            "recovery_local_map_min_points must be positive: " +
            std::to_string(recoveryLocalMapMinPoints));
    }
    if (recoveryLocalMapMaxPoints < recoveryLocalMapMinPoints)
    {
        throw std::runtime_error(
            "recovery_local_map_max_points must be >= recovery_local_map_min_points: " +
            std::to_string(recoveryLocalMapMaxPoints));
    }
    if (snapshotSaveIntervalScans <= 0)
    {
        throw std::runtime_error(
            "snapshot_save_interval_scans must be positive: " +
            std::to_string(snapshotSaveIntervalScans));
    }
    if (snapshotMaxTranslationCorrectionM <= 0.0)
    {
        throw std::runtime_error(
            "snapshot_max_translation_correction_m must be positive: " +
            std::to_string(snapshotMaxTranslationCorrectionM));
    }
    if (snapshotMaxRotationCorrectionDeg <= 0.0 || snapshotMaxRotationCorrectionDeg > 180.0)
    {
        throw std::runtime_error(
            "snapshot_max_rotation_correction_deg must be in (0, 180]: " +
            std::to_string(snapshotMaxRotationCorrectionDeg));
    }
    if (adaptiveMaxDtS <= 0.0)
    {
        throw std::runtime_error(
            "adaptive_max_dt_s must be positive: " + std::to_string(adaptiveMaxDtS));
    }
    if (adaptiveVelocityGain < 0.0 || adaptiveAccelerationGain < 0.0 || adaptiveYawRateGain < 0.0)
    {
        throw std::runtime_error(
            "adaptive gains must be non-negative.");
    }
    if (aggressiveSpeedMs <= 0.0 || aggressiveYawRateDegS <= 0.0 ||
        pivotLinearSpeedMs < 0.0 || pivotYawRateDegS <= 0.0 ||
        pivotMaxTranslationCorrectionM <= 0.0)
    {
        throw std::runtime_error(
            "aggressive/pivot thresholds must be positive except pivot_linear_speed_ms which can be zero.");
    }
    if (odomBridgeAfterRejections < 0)
    {
        throw std::runtime_error(
            "odom_bridge_after_rejections must be non-negative: " +
            std::to_string(odomBridgeAfterRejections));
    }
    if (odomBridgeMinSpeedMs < 0.0)
    {
        throw std::runtime_error(
            "odom_bridge_min_speed_ms must be non-negative: " +
            std::to_string(odomBridgeMinSpeedMs));
    }
    if (planarPoseMaxZDriftM <= 0.0)
    {
        throw std::runtime_error(
            "planar_pose_max_z_drift_m must be positive: " +
            std::to_string(planarPoseMaxZDriftM));
    }
    if (enableDynamicTrailerSelfFilter)
    {
        if (dynamicTrailerArticulationTopic.empty())
        {
            throw std::runtime_error(
                "dynamic_trailer_articulation_topic must be non-empty when dynamic trailer self-filter is enabled.");
        }
        if (dynamicTrailerStaleTimeoutS <= 0.0)
        {
            throw std::runtime_error(
                "dynamic_trailer_stale_timeout_s must be positive: " +
                std::to_string(dynamicTrailerStaleTimeoutS));
        }
        if (dynamicTrailerRearOffsetM <= dynamicTrailerFrontOffsetM)
        {
            throw std::runtime_error(
                "dynamic_trailer_rear_offset_m must be > dynamic_trailer_front_offset_m.");
        }
        if (dynamicTrailerHalfWidthM <= 0.0)
        {
            throw std::runtime_error(
                "dynamic_trailer_half_width_m must be positive: " +
                std::to_string(dynamicTrailerHalfWidthM));
        }
        if (dynamicTrailerZMaxM <= dynamicTrailerZMinM)
        {
            throw std::runtime_error(
                "dynamic_trailer_z_max_m must be > dynamic_trailer_z_min_m.");
        }
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
