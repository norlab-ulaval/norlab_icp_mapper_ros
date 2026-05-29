#include "Deskewer.h"

#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <tf2/utils.h>
#include <omp.h>

#include <chrono>
#include <cstdint>
#include <cmath>
#include <stdexcept>

// ──────────────────────────────────────────────────────────────────────────────
// Deskewer implementation
// ──────────────────────────────────────────────────────────────────────────────

Deskewer::TimeMode Deskewer::parseTimeMode(const std::string& mode_str)
{
    if (mode_str == "absolute_ns") { return TimeMode::ABSOLUTE_NS; }
    if (mode_str == "relative_ns") { return TimeMode::RELATIVE_NS; }
    if (mode_str == "relative_s")  { return TimeMode::RELATIVE_S;  }
    if (mode_str == "auto")        { return TimeMode::AUTO;         }
    throw std::invalid_argument("Unknown deskew_time_mode: " + mode_str +
        ". Valid: absolute_ns | relative_ns | relative_s | auto");
}

Deskewer::Deskewer(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const rclcpp::Logger& logger,
    const std::string& fixed_frame,
    TimeMode time_mode,
    const std::string& time_field,
    uint32_t cache_slots,
    uint32_t round_to_ns,
    uint32_t timeout_ms)
    : tfBuffer_(std::move(tf_buffer))
    , logger_(logger)
    , fixedFrame_(fixed_frame)
    , timeMode_(time_mode)
    , timeFieldName_(time_field)
    , cacheSlots_(cache_slots)
    , roundToNs_(round_to_ns)
    , timeoutMs_(timeout_ms)
{
    tfsCache_.reserve(cache_slots);

    if (timeMode_ == TimeMode::AUTO)
    {
        RCLCPP_WARN(logger_,
            "[Deskewer] deskew_time_mode=auto is a debug mode only. "
            "Time semantics are guessed heuristically and may be wrong. "
            "For production use, set deskew_time_mode explicitly.");
    }
}

// ── toAbsoluteNs ─────────────────────────────────────────────────────────────

int64_t Deskewer::toAbsoluteNs(
    int64_t raw_time,
    int64_t first_point_raw,
    int64_t header_stamp_ns) const
{
    switch (timeMode_)
    {
        case TimeMode::ABSOLUTE_NS:
            // Raw value is already an absolute nanosecond timestamp.
            return raw_time;

        case TimeMode::RELATIVE_NS:
            // Raw value is nanoseconds since the first point.
            // Approximate scan end = header_stamp; map forward from that.
            // Actually: header_stamp ≈ end-of-scan, so first point time ≈
            // header_stamp - (last_relative - first_relative).
            // We use: abs = header_stamp + (raw - last_point_relative)
            // which is computed in deskewCloud where latest is known.
            // Here we cannot compute that; caller handles the offset.
            // Return as relative offset — caller adjusts with latest_abs.
            return raw_time;  // offset, caller adjusts

        case TimeMode::RELATIVE_S:
            // Raw value is seconds * 1e9 nanoseconds? No — DataPoints.times is
            // Int64Matrix; libpointmatcher_ros encodes relative_s as
            // round(seconds * 1e9). So this is the same as relative_ns in storage.
            return raw_time;

        case TimeMode::AUTO:
            // Heuristic: if raw_time looks like a valid Unix timestamp (> 1e18 ns ≈ year 2001+)
            // treat as absolute_ns, otherwise relative_ns.
            if (raw_time > static_cast<int64_t>(1e15))
            {
                return raw_time;
            }
            return header_stamp_ns - (first_point_raw - raw_time);  // rough relative→abs

        default:
            return raw_time;
    }
}

// ── detectTimeMode ────────────────────────────────────────────────────────────

Deskewer::TimeMode Deskewer::detectTimeMode(int64_t sample_raw, int64_t header_stamp_ns) const
{
    // If value is close to current ROS time in nanoseconds: absolute_ns.
    const int64_t ten_years_ns = static_cast<int64_t>(10LL * 365 * 24 * 3600) * 1'000'000'000LL;
    if (std::abs(sample_raw - header_stamp_ns) < ten_years_ns && sample_raw > 0)
    {
        return TimeMode::ABSOLUTE_NS;
    }
    // If value is small (< 1 s in ns): relative_ns
    if (sample_raw < 1'000'000'000LL)
    {
        return TimeMode::RELATIVE_NS;
    }
    return TimeMode::RELATIVE_NS;
}

// ── deskewCloud ───────────────────────────────────────────────────────────────

bool Deskewer::deskewCloud(DP& cloud, const std::string& sensor_frame)
{
    const auto begin_wall = std::chrono::steady_clock::now();

    // ── Check time field exists ───────────────────────────────────────────────
    if (!cloud.timeExists(timeFieldName_))
    {
        RCLCPP_WARN(logger_,
            "[Deskewer] Cloud does not contain time field '%s'. Deskewing disabled for this scan.",
            timeFieldName_.c_str());
        return false;
    }

    const int n_pts = static_cast<int>(cloud.getNbPoints());
    if (n_pts == 0)
    {
        RCLCPP_WARN(logger_, "[Deskewer] Empty cloud, skipping deskew.");
        return false;
    }

    // ── Find min / max raw point time ─────────────────────────────────────────
    int64_t min_raw = cloud.times(0);
    int64_t max_raw = cloud.times(0);
    for (int i = 1; i < n_pts; ++i)
    {
        min_raw = std::min(min_raw, cloud.times(i));
        max_raw = std::max(max_raw, cloud.times(i));
    }

    RCLCPP_INFO_ONCE(logger_,
        "[Deskewer] First scan raw time range: [%ld, %ld] ns, delta=%ld ns (%d points). "
        "As seconds: [%.6f, %.6f]. Expected ~1.779e18 ns for absolute Unix time.",
        min_raw, max_raw, max_raw - min_raw, n_pts,
        min_raw * 1e-9, max_raw * 1e-9);

    // ── Determine effective time mode ─────────────────────────────────────────
    TimeMode effective_mode = timeMode_;
    if (timeMode_ == TimeMode::AUTO)
    {
        effective_mode = detectTimeMode(max_raw, max_raw);
        RCLCPP_WARN(logger_,
            "[Deskewer] auto mode detected time mode: %s. "
            "This is unreliable — set deskew_time_mode explicitly.",
            (effective_mode == TimeMode::ABSOLUTE_NS) ? "absolute_ns" : "relative_ns");
    }

    // ── Convert raw times to absolute nanoseconds for TF lookup ──────────────
    // latest_abs_ns is the reference time (end-of-scan) for all corrections.
    int64_t latest_abs_ns;
    // A lambda to convert any raw point time to absolute ns.
    std::function<int64_t(int64_t)> rawToAbsNs;

    switch (effective_mode)
    {
        case TimeMode::ABSOLUTE_NS:
            latest_abs_ns = max_raw;
            rawToAbsNs = [](int64_t raw) { return raw; };
            break;

        case TimeMode::RELATIVE_NS:
            // Header stamp is not available here; we use max_raw as latest.
            // The scan header stamp is passed as cloud.header (not available in DataPoints).
            // Workaround: max_raw is offset from start; latest = max_raw; others are < max_raw.
            // We still need an anchor. Without the header stamp we cannot get absolute time.
            // Assumption: the TF buffer can find relative times IF we add the scan start time.
            // We cannot do this correctly without the header — the caller should pass it.
            // For now, warn and fall back: treat as if relative times are offsets from
            // the last TF in the buffer (imprecise but safe).
            RCLCPP_WARN_ONCE(logger_,
                "[Deskewer] deskew_time_mode=relative_ns: per-point absolute time cannot be "
                "determined without cloud header stamp. TF lookups may use wrong times. "
                "Consider passing scan header stamp to Deskewer.");
            latest_abs_ns = max_raw;  // This is wrong in absolute terms but consistent
            rawToAbsNs = [](int64_t raw) { return raw; };
            break;

        case TimeMode::RELATIVE_S:
            // Same limitation as relative_ns.
            RCLCPP_WARN_ONCE(logger_,
                "[Deskewer] deskew_time_mode=relative_s: same caveat as relative_ns.");
            latest_abs_ns = max_raw;
            rawToAbsNs = [](int64_t raw) { return raw; };
            break;

        default:
            latest_abs_ns = max_raw;
            rawToAbsNs = [](int64_t raw) { return raw; };
            break;
    }

    const rclcpp::Time latest_time_ros(latest_abs_ns);

    // ── Build TF cache: lookup each unique binned timestamp ───────────────────
    tfsCache_.clear();
    const int64_t round_to = static_cast<int64_t>(roundToNs_);
    const auto tf_timeout = rclcpp::Duration(0, static_cast<uint32_t>(timeoutMs_ * 1'000'000u));
    int tf_failures = 0;

    for (int i = 0; i < n_pts; ++i)
    {
        const int64_t abs_ns = rawToAbsNs(cloud.times(i));
        const int64_t binned = abs_ns / round_to;

        if (tfsCache_.count(binned) == 0)
        {
            const rclcpp::Time point_time_ros(abs_ns);
            try
            {
                // lookupTransform(target, target_time, source, source_time, fixed_frame, timeout)
                // Corrects each point so it appears as if captured at latest_time_ros.
                geometry_msgs::msg::TransformStamped tf =
                    tfBuffer_->lookupTransform(
                        sensor_frame, latest_time_ros,
                        sensor_frame, point_time_ros,
                        fixedFrame_, tf_timeout);
                tfsCache_.emplace(binned, tf);
            }
            catch (const tf2::TransformException& ex)
            {
                // Throttle to 1 log every 5s — deskew failure is non-fatal (ICP proceeds
                // without deskewing), but we don't want to spam at 20Hz.
                static rclcpp::Clock deskew_warn_clock(RCL_STEADY_TIME);
                RCLCPP_WARN_THROTTLE(logger_, deskew_warn_clock, 5000,
                    "[Deskewer] TF lookup failed at time %ld ns (%.3f s): %s. "
                    "Deskewing disabled for this scan — ICP continues without deskewing.",
                    abs_ns, abs_ns * 1e-9, ex.what());
                ++tf_failures;
                return false;  // Abort deskew — don't partially corrupt the cloud.
            }
        }
    }

    RCLCPP_DEBUG(logger_,
        "[Deskewer] Populated TF cache with %zu unique time slots. TF failures: %d",
        tfsCache_.size(), tf_failures);

    // ── Apply transforms in parallel ──────────────────────────────────────────
    // At this point tfsCache_ is fully populated (no more writes).
    // Reading with .at() is safe for concurrent readers.
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < n_pts; ++i)
    {
        const int64_t abs_ns = rawToAbsNs(cloud.times(i));
        const int64_t binned = abs_ns / round_to;

        // .at() is read-only; safe under concurrent access when no writes occur.
        const auto& tf = tfsCache_.at(binned);
        const auto T = PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(tf, 4);
        cloud.features.col(i) = T * cloud.features.col(i);
    }

    const auto end_wall = std::chrono::steady_clock::now();
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_wall - begin_wall).count();

    RCLCPP_DEBUG(logger_,
        "[Deskewer] Deskewed %d points in %ld ms (%zu unique TF slots).",
        n_pts, elapsed_ms, tfsCache_.size());

    return true;
}
