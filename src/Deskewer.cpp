#include "Deskewer.h"
#include <cstdint>
#include <chrono>
#include <pointmatcher_ros/PointMatcher_ROS.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <omp.h>

Deskewer::Deskewer(const rclcpp::Logger& logger, rclcpp::Clock::SharedPtr clock)
    : logger(logger)
{
    tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(clock));
    tfListener = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(*tfBuffer));
    tfsCache.reserve(expectedNumberOfPclColumns);
}

bool Deskewer::deskewCloud(Deskewer::DP &cloud, const std::string &sensorFrame)
{
   	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    tfsCache.clear();
    std::unordered_map<int64_t, int64_t> timeCache;

    if (!cloud.timeExists(timeFieldName))
    {
        RCLCPP_WARN(logger, "The input pointcloud does not contain the 'time' field. Skipping.");
        return false;
    }

    int64_t latestTime = 0;
    for (int i=0; i<cloud.getNbPoints(); ++i)
    {
        latestTime = std::max(latestTime, cloud.times(i));
    }
    rclcpp::Time latestTimeRos(latestTime);

    //iterate over the pointcloud, lookup ROS transforms
    // and fill the lookup table with the transforms
    for (int i=0; i<cloud.getNbPoints(); ++i)
    {
        int64_t cachedTfTime = cloud.times(i) / roundToIntervalsOfNanoseconds;
        timeCache[cloud.times(i)] = cachedTfTime;
        if(tfsCache.count(cachedTfTime) == 0)
        {
            rclcpp::Time laserTimeRos(cloud.times(i));
            try{
                geometry_msgs::msg::TransformStamped transform = tfBuffer->lookupTransform(sensorFrame,
                                                        latestTimeRos,
                                                        sensorFrame,
                                                        laserTimeRos,
                                                        fixedFrameForLaser,
                                                        rclcpp::Duration(0, 2.5e8));
                tfsCache[cachedTfTime] = transform;
            }
            catch(tf2::TransformException &ex){
                RCLCPP_ERROR(logger, "Pointcloud callback failed because: %s", ex.what());
                return false;
            }
        }
    }

    // apply the transforms to the pointcloud in parallel
    #pragma omp parallel for
    for (int i=0; i<cloud.getNbPoints(); ++i) {
        // transform the point
        int64_t cachedTfTime = timeCache[cloud.times(i)];
        auto transform = tfsCache[cachedTfTime];
        auto transformationParameters = PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(transform, 4);
        cloud.features.col(i) = transformationParameters * cloud.features.col(i);
    }

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    RCLCPP_DEBUG_STREAM(logger, "Point cloud deskewed in " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " [µs]");
    return true;
}
