#include "NodeParameters.h"
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>


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
            "points_in", 10, std::bind(&color3DPointsNode::colorlidarCallback, this, std::placeholders::_1));

/*
        lidarSubscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points_in", 10, std::bind(&color3DPointsNode::lidarCallback_nonpcl, this, std::placeholders::_1));
*/
        // Create a publisher for the colorPoints
        colorPointsPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("colorlidarpoints", 10);

    }

    // Callback function for the camera topic
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Do something with the camera data
        //RCLCPP_INFO(this->get_logger(), "Received camera data height=%d weight=%d", msg->height, msg->width);
    }


    void lidarCallback_nonpcl(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Create a copy of the input PointCloud2 message to modify
        sensor_msgs::msg::PointCloud2 colored_msg;

        colored_msg.header = msg->header;
        colored_msg.height = msg->height;
        colored_msg.width = msg->width;
        colored_msg.fields = msg->fields;
        colored_msg.is_bigendian = msg->is_bigendian;
        colored_msg.is_dense = msg->is_dense;

        // Copy original point step and row step
        colored_msg.point_step = msg->point_step;
        colored_msg.row_step = msg->row_step;
        colored_msg.data = msg->data;

        // Add RGB fields to the new message
        sensor_msgs::PointCloud2Modifier modifier(colored_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

        // Resize data buffer
        size_t point_step = colored_msg.point_step;
        size_t new_point_step = point_step + 3; // Adding 3 bytes for RGB
        size_t new_row_step = (colored_msg.row_step / point_step) * new_point_step; // Adjust row_step
        colored_msg.data.resize(new_row_step * colored_msg.height); // Resize buffer to accommodate RGB fields

        // Copy original data to the new buffer
        std::memcpy(colored_msg.data.data(), msg->data.data(), msg->data.size());

        // Iterate through the point cloud and assign colors
        sensor_msgs::PointCloud2Iterator<float> iter_x(colored_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(colored_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(colored_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(colored_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(colored_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(colored_msg, "b");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
        {
            // Assign colors based on the point's position (example)
            *iter_r = static_cast<uint8_t>(std::min(255.0f, *iter_x * 0.0f));
            *iter_g = static_cast<uint8_t>(std::min(255.0f, *iter_y * 255.0f));
            *iter_b = static_cast<uint8_t>(std::min(255.0f, *iter_z * 0.0f));
        }

        // Publish the colored point cloud
        colorPointsPublisher->publish(colored_msg);
    }

    void colorlidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the sensor_msgs::msg::PointCloud2 message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // Create a new point cloud with color information
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_colored;
        pcl_cloud_colored.header = pcl_cloud.header;

        // Add the color information to the point cloud
        for (const auto& point : pcl_cloud.points) {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = 255;  // Red channel
            colored_point.g = 0;    // Green channel
            colored_point.b = 0;    // Blue channel
            pcl_cloud_colored.points.push_back(colored_point);
        }

        // Convert the PCL point cloud with color back to a sensor_msgs::msg::PointCloud2 message
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(pcl_cloud_colored, output_msg);

        // Publish the colored point cloud
        //publisher_->publish(output_msg);
        colorPointsPublisher->publish(output_msg);
    }

    private:
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_;
    sensor_msgs::msg::Image::SharedPtr new_image_;

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