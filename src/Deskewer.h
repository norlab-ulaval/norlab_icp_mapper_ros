#ifndef DESKEWER_H
#define DESKEWER_H

#include <pointmatcher/PointMatcher.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class Deskewer
{
  private:
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

    uint expectedUniqueDeskewingTFNumber;
    uint deskewingRoundToNSecs;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer = nullptr;
    std::unique_ptr<tf2_ros::TransformListener> tfListener = nullptr;
    rclcpp::Logger logger;

    const std::string timeFieldName = "time";
    std::string fixedFrameForLaser = "odom";

    // Dictionary to store already looked up transforms
    std::unordered_map<int64_t, geometry_msgs::msg::TransformStamped> tfsCache;

  public:
    // Constructor
    Deskewer(const rclcpp::Logger &logger, rclcpp::Clock::SharedPtr clock, uint expectedUniqueDeskewingTFNumber, uint deskewingRoundToNSecs);

    bool deskewCloud(DP &cloud, const std::string &sensor_frame);
};

#endif
