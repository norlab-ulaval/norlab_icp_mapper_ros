#include "ConstantVelocityDeskewer.h"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <omp.h>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

ConstantVelocityDeskewer::ConstantVelocityDeskewer(const rclcpp::Logger &logger)
    : logger(logger)
{
}

void ConstantVelocityDeskewer::updateMotion(const PM::TransformationParameters &relativeTransform, double deltaTimeSeconds)
{
    if(deltaTimeSeconds <= std::numeric_limits<double>::epsilon())
    {
        RCLCPP_WARN(logger, "Non-positive delta time (%f [s]) between scans, ignoring motion update.", deltaTimeSeconds);
        return;
    }

    lastRelativeTransform = relativeTransform;
    lastDeltaTimeSeconds = deltaTimeSeconds;
    hasMotionEstimate = true;
}

bool ConstantVelocityDeskewer::deskewCloud(ConstantVelocityDeskewer::DP &cloud, const std::string & /*sensorFrame*/)
{
    if(!hasMotionEstimate)
    {
        RCLCPP_DEBUG(logger, "No motion estimate yet, skipping deskew.");
        return false;
    }

    if(!cloud.timeExists(timeFieldName))
    {
        RCLCPP_WARN(logger, "The input pointcloud does not contain the 'time' field. Skipping.");
        return false;
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    int64_t latestTime = 0;
    for(int i = 0; i < cloud.getNbPoints(); ++i)
    {
        latestTime = std::max(latestTime, cloud.times(i));
    }

    // lastRelativeTransform is sensorToMap(new)^-1 * sensorToMap(old): the pose of the sensor at
    // the previous scan, expressed in the frame of the sensor at the current scan's latest known
    // pose. Applying an alpha-fraction of that same twist to a point captured "alpha" of the way
    // back through the inter-scan interval expresses it in the latest-pose frame too, under the
    // constant-velocity assumption -- no TF lookup needed.
    if(lastRelativeTransform.rows() == 4)
    {
        Eigen::Matrix3f rotation = lastRelativeTransform.topLeftCorner<3, 3>();
        Eigen::Vector3f translation = lastRelativeTransform.topRightCorner<3, 1>();
        Sophus::SE3f relative(Sophus::SO3f(rotation), translation);
        Sophus::SE3f::Tangent twist = relative.log();

        #pragma omp parallel for
        for(int i = 0; i < cloud.getNbPoints(); ++i)
        {
            double elapsedSeconds = std::max(0.0, static_cast<double>(latestTime - cloud.times(i)) / 1e9);
            float alpha = static_cast<float>(elapsedSeconds / lastDeltaTimeSeconds);
            PM::TransformationParameters fractionalTransform = Sophus::SE3f::exp(alpha * twist).matrix();
            cloud.features.col(i) = fractionalTransform * cloud.features.col(i);
        }
    }
    else
    {
        float angle = std::atan2(lastRelativeTransform(1, 0), lastRelativeTransform(0, 0));
        Eigen::Vector2f translation = lastRelativeTransform.topRightCorner<2, 1>();
        Sophus::SE2f relative(Sophus::SO2f(angle), translation);
        Sophus::SE2f::Tangent twist = relative.log();

        #pragma omp parallel for
        for(int i = 0; i < cloud.getNbPoints(); ++i)
        {
            double elapsedSeconds = std::max(0.0, static_cast<double>(latestTime - cloud.times(i)) / 1e9);
            float alpha = static_cast<float>(elapsedSeconds / lastDeltaTimeSeconds);
            PM::TransformationParameters fractionalTransform = Sophus::SE2f::exp(alpha * twist).matrix();
            cloud.features.col(i) = fractionalTransform * cloud.features.col(i);
        }
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    RCLCPP_DEBUG_STREAM(logger, "Point cloud deskewed (constant velocity) in "
                                     << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " [ms]");
    return true;
}
