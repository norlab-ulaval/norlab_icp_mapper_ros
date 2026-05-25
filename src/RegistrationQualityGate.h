#ifndef REGISTRATION_QUALITY_GATE_H
#define REGISTRATION_QUALITY_GATE_H

// ──────────────────────────────────────────────────────────────────────────────
// RegistrationQualityGate
//
// Decides whether an ICP registration result is plausible enough to accept.
// Inputs: prior pose, optimized pose, cloud size, time delta, wall time.
// Output: accept/reject + human-readable reason + metrics.
//
// No ROS dependency — fully unit-testable without a ROS environment.
// ──────────────────────────────────────────────────────────────────────────────

#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <sstream>
#include <limits>

class RegistrationQualityGate
{
public:
    // ── Configuration ─────────────────────────────────────────────────────────
    struct Config
    {
        int    min_input_points           = 100;
        double max_translation_m          = 2.0;    ///< Max correction magnitude in meters
        double max_rotation_deg           = 30.0;   ///< Max correction magnitude in degrees
        double max_velocity_ms            = 20.0;   ///< Max robot speed in m/s
        double max_yaw_rate_deg_s         = 90.0;   ///< Max yaw rate in deg/s
        double max_registration_time_ms   = 5000.0; ///< Max ICP wall-clock time
    };

    // ── Result ────────────────────────────────────────────────────────────────
    struct Result
    {
        bool        accepted            = false;
        std::string rejection_reason    = "";

        // Computed metrics — always filled, even when rejected.
        double translation_correction_m = 0.0;
        double rotation_correction_deg  = 0.0;
        double velocity_ms              = 0.0;
        double yaw_rate_deg_s           = 0.0;
        double registration_time_ms     = 0.0;
        int    input_points             = 0;

        std::string summary() const
        {
            std::ostringstream ss;
            ss << (accepted ? "ACCEPTED" : "REJECTED")
               << " | trans=" << translation_correction_m << " m"
               << " rot=" << rotation_correction_deg << " deg"
               << " vel=" << velocity_ms << " m/s"
               << " yaw=" << yaw_rate_deg_s << " deg/s"
               << " icp=" << registration_time_ms << " ms"
               << " pts=" << input_points;
            if (!accepted)
            {
                ss << " | reason: " << rejection_reason;
            }
            return ss.str();
        }
    };

    // ── Constructor ───────────────────────────────────────────────────────────
    RegistrationQualityGate() = default;
    explicit RegistrationQualityGate(const Config& config) : config_(config) {}

    // ── check ─────────────────────────────────────────────────────────────────
    // prior            : pose before ICP (sensor-to-map), homogeneous (4x4 or 3x3).
    // optimized        : pose after ICP (sensor-to-map).
    // input_points     : number of points in the filtered input cloud.
    // dt_seconds       : time since last accepted scan (0 on first scan).
    // registration_ms  : wall-clock time ICP took in milliseconds.
    Result check(
        const Eigen::MatrixXf& prior,
        const Eigen::MatrixXf& optimized,
        int input_points,
        double dt_seconds,
        double registration_ms) const
    {
        Result r;
        r.input_points        = input_points;
        r.registration_time_ms = registration_ms;

        // ── 1. Minimum point count ────────────────────────────────────────────
        if (input_points < config_.min_input_points)
        {
            r.rejection_reason = "Too few input points: " + std::to_string(input_points) +
                                 " < " + std::to_string(config_.min_input_points);
            return r;
        }

        // ── 2. Finite transform check ─────────────────────────────────────────
        if (!optimized.allFinite())
        {
            r.rejection_reason = "Optimized transform contains NaN or Inf.";
            return r;
        }
        if (!prior.allFinite())
        {
            r.rejection_reason = "Prior transform contains NaN or Inf.";
            return r;
        }

        // ── 3. Correction magnitude ───────────────────────────────────────────
        // correction = optimized * prior^{-1}
        const Eigen::MatrixXf correction = optimized * prior.inverse();

        const int dim = static_cast<int>(prior.rows()) - 1;  // euclidean dim (2 or 3)
        r.translation_correction_m = correction.topRightCorner(dim, 1).norm();

        // Rotation angle from rotation submatrix via trace formula.
        const Eigen::MatrixXf R = correction.topLeftCorner(dim, dim);
        double cos_angle = 0.0;
        if (dim == 3)
        {
            cos_angle = (R.trace() - 1.0) / 2.0;
        }
        else  // dim == 2
        {
            cos_angle = static_cast<double>(R(0, 0));
        }
        // Clamp to [-1, 1] to guard against numerical noise.
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
        r.rotation_correction_deg = std::acos(cos_angle) * (180.0 / M_PI);

        // Dynamic translation threshold: grows with dt so accumulated odom drift
        // during a rejection cascade doesn't trigger permanent rejection.
        // effective = max(config, max_velocity * dt) — allows up to max_velocity_ms
        // per second of accumulated drift before rejecting.
        const double effective_max_translation_m =
            (dt_seconds > 1e-6)
            ? std::max(config_.max_translation_m, config_.max_velocity_ms * dt_seconds)
            : config_.max_translation_m;

        if (r.translation_correction_m > effective_max_translation_m)
        {
            r.rejection_reason = "Translation correction too large: " +
                std::to_string(r.translation_correction_m) + " m > " +
                std::to_string(effective_max_translation_m) + " m (dt=" +
                std::to_string(dt_seconds) + "s, static_max=" +
                std::to_string(config_.max_translation_m) + "m)";
            return r;
        }

        // Dynamic rotation threshold: same principle.
        const double effective_max_rotation_deg =
            (dt_seconds > 1e-6)
            ? std::max(config_.max_rotation_deg, config_.max_yaw_rate_deg_s * dt_seconds)
            : config_.max_rotation_deg;

        if (r.rotation_correction_deg > effective_max_rotation_deg)
        {
            r.rejection_reason = "Rotation correction too large: " +
                std::to_string(r.rotation_correction_deg) + " deg > " +
                std::to_string(effective_max_rotation_deg) + " deg (dt=" +
                std::to_string(dt_seconds) + "s)";
            return r;
        }

        // ── 4. Velocity / yaw-rate bounds (only when dt > 0) ─────────────────
        if (dt_seconds > 1e-6)
        {
            r.velocity_ms = r.translation_correction_m / dt_seconds;
            if (r.velocity_ms > config_.max_velocity_ms)
            {
                r.rejection_reason = "Implied velocity too high: " +
                    std::to_string(r.velocity_ms) + " m/s > " +
                    std::to_string(config_.max_velocity_ms) + " m/s";
                return r;
            }

            r.yaw_rate_deg_s = r.rotation_correction_deg / dt_seconds;
            if (r.yaw_rate_deg_s > config_.max_yaw_rate_deg_s)
            {
                r.rejection_reason = "Implied yaw rate too high: " +
                    std::to_string(r.yaw_rate_deg_s) + " deg/s > " +
                    std::to_string(config_.max_yaw_rate_deg_s) + " deg/s";
                return r;
            }
        }

        // ── 5. Registration time ──────────────────────────────────────────────
        if (registration_ms > config_.max_registration_time_ms)
        {
            r.rejection_reason = "ICP took too long: " +
                std::to_string(registration_ms) + " ms > " +
                std::to_string(config_.max_registration_time_ms) + " ms";
            return r;
        }

        r.accepted = true;
        return r;
    }

    // Allow updating config at runtime (e.g. via parameter callback).
    void setConfig(const Config& config) { config_ = config; }
    const Config& getConfig() const { return config_; }

private:
    Config config_;
};

#endif  // REGISTRATION_QUALITY_GATE_H
