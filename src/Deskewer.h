#ifndef DESKEWER_H
#define DESKEWER_H

#include <pointmatcher/PointMatcher.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// ──────────────────────────────────────────────────────────────────────────────
// Deskewer — motion compensation for spinning LiDAR scanners.
//
// Each point in a scan is captured at a slightly different time while the
// robot is moving. Without deskewing, the assembled cloud appears smeared.
//
// This class corrects each point to the frame at the scan end-time by
// looking up the robot pose at the per-point timestamp via TF.
//
// Time modes:
//   absolute_ns  — point time is an absolute Unix timestamp in nanoseconds.
//                  Correct for Hesai XT-32.
//   relative_ns  — point time is nanoseconds since the first point in the scan.
//   relative_s   — point time is seconds since the first point in the scan.
//   auto         — heuristic detection (debug only, logs a warning).
//
// Thread safety: deskewCloud() is not thread-safe. Call from one thread only.
// ──────────────────────────────────────────────────────────────────────────────
class Deskewer
{
public:
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

    // ── Time mode ─────────────────────────────────────────────────────────────
    enum class TimeMode
    {
        ABSOLUTE_NS,  ///< Per-point time is absolute Unix epoch in nanoseconds.
        RELATIVE_NS,  ///< Per-point time is nanoseconds relative to first point.
        RELATIVE_S,   ///< Per-point time is seconds relative to first point.
        AUTO,         ///< Heuristic detection — debug only, not for production.
    };

    static TimeMode parseTimeMode(const std::string& mode_str);

    // ── Constructor ───────────────────────────────────────────────────────────
    // tf_buffer     : Shared TF buffer from the parent node (not owned here).
    // logger        : ROS logger from parent node.
    // fixed_frame   : Frame considered fixed for TF interpolation (e.g. "odom").
    // time_mode     : How to interpret per-point timestamps.
    // time_field    : Name of the time field in DataPoints (default: "time").
    // cache_slots   : Number of unique time slots to reserve in the cache.
    // round_to_ns   : Round point timestamps to this granularity (ns) to reduce lookups.
    // timeout_ms    : TF lookup timeout in milliseconds.
    Deskewer(
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        const rclcpp::Logger& logger,
        const std::string& fixed_frame,
        TimeMode time_mode,
        const std::string& time_field,
        uint32_t cache_slots,
        uint32_t round_to_ns,
        uint32_t timeout_ms);

    // ── deskewCloud ───────────────────────────────────────────────────────────
    // Modifies cloud in-place. Returns true on success, false if deskew was
    // skipped (missing time field, TF failure, etc.).
    // On failure the cloud is left unmodified.
    bool deskewCloud(DP& cloud, const std::string& sensor_frame);

private:
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    rclcpp::Logger logger_;
    std::string fixedFrame_;
    TimeMode timeMode_;
    std::string timeFieldName_;
    uint32_t cacheSlots_;
    uint32_t roundToNs_;
    uint32_t timeoutMs_;

    // Cache: binned timestamp (int64) → TF transform for that time.
    // Rebuilt from scratch each call — cleared at start of deskewCloud().
    std::unordered_map<int64_t, geometry_msgs::msg::TransformStamped> tfsCache_;

    // Convert raw point time (as stored in DataPoints.times) to an absolute
    // nanosecond ROS timestamp, given the scan header stamp.
    int64_t toAbsoluteNs(int64_t raw_time, int64_t first_point_raw, int64_t header_stamp_ns) const;

    // Detect time mode heuristically (auto mode only, logs a warning).
    TimeMode detectTimeMode(int64_t sample_raw, int64_t header_stamp_ns) const;
};

#endif  // DESKEWER_H
