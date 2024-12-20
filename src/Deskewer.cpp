#include "Deskewer.h"
#include <cstdint>
#include <chrono>
#include <pointmatcher_ros/PointMatcher_ROS.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

Deskewer::Deskewer(const rclcpp::Logger& logger, rclcpp::Clock::SharedPtr clock)
    : logger(logger)
{
    tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(clock));
    tfListener = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(*tfBuffer));
    tfsCache.reserve(expectedNumberOfPclColumns);
}

void Deskewer::deskew_cloud(Deskewer::DP &cloud, const std::string &sensorFrame)
{
    tfsCache.clear();

   	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    if (!cloud.timeExists(timeFieldName))
    {
        RCLCPP_WARN(logger, "The input pointcloud does not contain 'time' or 't' or 'timestamp' field. Skipping.");
        return;
    }

    int64_t latestTime = 0;
    for (int i=0; i<cloud.getNbPoints(); ++i)
    {
        latestTime = std::max(latestTime, cloud.times(i));
    }
    rclcpp::Time latestTimeRos(latestTime);

    int64_t cachedTfTime = 0;
    //iterate over the pointcloud, lookup tfs and apply them
    for (int i=0; i<cloud.getNbPoints(); ++i)
    {
        cachedTfTime = cloud.times(i) / roundToIntervalsOfNanoseconds;

        geometry_msgs::msg::TransformStamped transform;
        tf2::Stamped<tf2::Transform> stampedTransform;
        if(tfsCache.count(cachedTfTime) == 0)
        {
            rclcpp::Time laserTimeRos(cloud.times(i));

            // RCLCPP_INFO_STREAM(logger, "Point time: " << cloud.times(i) << " [ns] | Cached time: "  << cachedTfTime << " [ns]\n"
            //         << "Seconds: "<< laser_beam_time.seconds() << " [s] | Nanosecs: " << laser_beam_time.nanoseconds() << " [ns]");

            try{
                transform = tfBuffer->lookupTransform(sensorFrame,
                                                        latestTimeRos,
                                                        sensorFrame,
                                                        laserTimeRos,
                                                        fixedFrameForLaser,
                                                        rclcpp::Duration(0, 2.5e8));
            }
            catch(tf2::TransformException &ex){
                RCLCPP_ERROR(logger, "Pointcloud callback failed because: %s", ex.what());
                return;
            }
            tfsCache[cachedTfTime] = transform;
        }
        else
        {
            transform = tfsCache[cachedTfTime];
        }

        // transform the point
        auto transformationParameters = PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(transform, 4);
        cloud.features.col(i) = transformationParameters * cloud.features.col(i);
    }

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    RCLCPP_INFO_STREAM(logger, "Point cloud deskewed in " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " [µs]");
}
