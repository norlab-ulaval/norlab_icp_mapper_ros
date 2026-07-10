#ifndef CONSTANT_VELOCITY_DESKEWER_H
#define CONSTANT_VELOCITY_DESKEWER_H

#include "IDeskewer.h"
#include <pointmatcher/PointMatcher.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

// Motion-compensates a scan without any TF lookups, by assuming the sensor kept moving at the
// same rigid-body velocity it had between the two previous registered scans (as in KISS-ICP's
// constant-velocity deskewing: https://github.com/PRBonn/kiss-icp).
class ConstantVelocityDeskewer : public IDeskewer
{
  private:
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

    const std::string timeFieldName = "time";
    rclcpp::Logger logger;

    // Relative transform observed between the two most recently registered scans, i.e.
    // sensorToMap(new)^-1 * sensorToMap(old), and the time elapsed between them.
    PM::TransformationParameters lastRelativeTransform;
    double lastDeltaTimeSeconds = 0.0;
    bool hasMotionEstimate = false;

  public:
    explicit ConstantVelocityDeskewer(const rclcpp::Logger &logger);

    bool deskewCloud(DP &cloud, const std::string &sensorFrame) override;
    void updateMotion(const PM::TransformationParameters &relativeTransform, double deltaTimeSeconds) override;
};

#endif
