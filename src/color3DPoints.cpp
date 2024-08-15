#include "NodeParameters.h"
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"


class color3DPointsNode : public rclcpp::Node
{
    /*
    subscribe to camera topic
    subscribe to lidar topic
    publish colorPoints
    */
    public:
    // Constuctor
    color3DPointsNode() : Node("color3DPointsNode")
    {
        // Create a subscription to the camera topic
        cameraSubscription = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_in", 10, std::bind(&color3DPointsNode::cameraCallback, this, std::placeholders::_1));

        // Create a subscription to the lidar topic
        lidarSubscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points_in", 10, std::bind(&color3DPointsNode::lidarCallback, this, std::placeholders::_1));

        // Create a publisher for the colorPoints
        colorPointsPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colorPoints", 10);
    }

    // Callback function for the camera topic
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Do something with the camera data
        RCLCPP_INFO(this->get_logger(), "Received camera data");
    }

    // Callback function for the lidar topic
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Do something with the lidar data
        RCLCPP_INFO(this->get_logger(), "Received lidar data");
    }

    private:
    // Declare the camera subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cameraSubscription;

    // Declare the lidar subscription
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidarSubscription;

    // Declare the colorPoints publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colorPointsPublisher;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<color3DPointsNode>());
  rclcpp::shutdown();
  return 0;
}