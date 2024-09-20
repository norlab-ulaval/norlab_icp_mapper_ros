#include "NodeParameters.h"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <algorithm>

class color3DPointsNode : public rclcpp::Node
{
    public:
    color3DPointsNode() : Node("color3DPointsNode")
    {
        // Subscription to the camera topic
        cameraSubscription = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_in", 10, std::bind(&color3DPointsNode::cameraCallback, this, std::placeholders::_1));

        // Subscription to the lidar topic
        lidarSubscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points_in", 10, std::bind(&color3DPointsNode::colorlidarPointsCallback, this, std::placeholders::_1));


        // Publisher for the colorPoints
        colorPointsPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("colorlidarpoints", 10);
        tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        params = std::make_unique<NodeParameters>(*this);

    }

private:
    bool getTransform(std::string& target_frame, std::string& source_frame, rclcpp::Time& time_stamp)
    {
        if (target_frame.empty() || source_frame.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Empty target or source frame; target_frame: %s source frame: %s", target_frame.c_str(), source_frame.c_str());
            return false;
        }
        try
        {
            transform = tfBuffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return false;
        }
    }

    cv::Mat getTranslationVector()
    {
        cv::Mat translation_vector = (cv::Mat_<double>(3, 1) << transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
        return translation_vector;
    }

    cv::Mat getRotationVector()
    {
        tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return cv::Mat_<double>(3, 1) << roll, pitch, yaw;
    }

    cv::Mat getRotationMatrix()
    {
        cv::Mat rotation_matrix;
        cv::Rodrigues(getRotationVector(), rotation_matrix);
        return rotation_matrix;
    }

    bool is_point_in_image(const cv::Point2f& point)
    {
        return point.x >= 0 && point.x < image_.cols && point.y >= 0 && point.y < image_.rows;
    }

    bool is_valid_index(int index, std::vector<int> valid_indices)
    {
        return std::binary_search(valid_indices.begin(), valid_indices.end(), index);
    }

    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr camera_msg)
    {
        try {

            image_ = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8)->image;
            image_received = true;
            image_frameId  = camera_msg->header.frame_id;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            image_received = false;
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB> add_color_field(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud)
    {
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_colored;
        pcl_cloud_colored.header = pcl_cloud.header;

        for (std::size_t i = 0; i < pcl_cloud.size(); ++i) {
            pcl::PointXYZ point = pcl_cloud[i];
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = 0;
            colored_point.g = 0;
            colored_point.b = 0;
            //colored_point.intensity = 1.0f;
            pcl_cloud_colored.points.push_back(colored_point);
        }
        return pcl_cloud_colored;
    }

    std::vector<int> get_valid_indices(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud)
    {
        std::vector<int> valid_indices;
        for (std::size_t i = 0; i < pcl_cloud.size(); ++i) {
            pcl::PointXYZ point = pcl_cloud[i];
            if (point.x > 0)
            {
                valid_indices.push_back(i);
            }
        }
        return valid_indices;
    }

    void colorlidarPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
    {

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*lidar_msg, pcl_cloud);
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_colored = add_color_field(pcl_cloud);
        float prob_radiation = params->probRadiation;
        float grey_value = 0.0;
        cv::Vec3b color;


        lidar_frameId = lidar_msg->header.frame_id;
        time_stamp    = lidar_msg->header.stamp;

        bool transform_status = getTransform(image_frameId, lidar_frameId, time_stamp);
        sensor_msgs::msg::PointCloud2 output_msg;

        if ((!transform_status) || (!image_received))
        {
            pcl::toROSMsg(pcl_cloud_colored, output_msg);
            colorPointsPublisher->publish(output_msg);
            return;
        }

        std::vector<int> valid_indices = get_valid_indices(pcl_cloud);
        std::vector<cv::Point2f> image_points = project_point(pcl_cloud_colored);

        //Mpdify color to the points that are in the image
        for (std::size_t index = 0; index < image_points.size(); index++)
        {
            if (is_valid_index(index, valid_indices) && is_point_in_image(image_points[index]))
            {
                color = image_.at<cv::Vec3b>(image_points[index]);
                grey_value = 0.299*color[2]+0.587*color[1]+0.114*color[0];
                prob_radiation = grey_value/255;
                RCLCPP_INFO(this->get_logger(), "Probability of radiation: %f grey value", prob_radiation);
                uint8_t point_color = (prob_radiation > 0.2) ? 255 : 0;
                pcl_cloud_colored.points[index].r = point_color; // color[2];
                pcl_cloud_colored.points[index].g = point_color;
                pcl_cloud_colored.points[index].b = point_color;
            }
        }

        // Convert the PCL point cloud with color back to a sensor_msgs::msg::PointCloud2 message
        pcl::toROSMsg(pcl_cloud_colored, output_msg);

        colorPointsPublisher->publish(output_msg);
    }

    std::vector<cv::Point3f> convert_to_cv_points(const pcl::PointCloud<pcl::PointXYZRGB> pcl_points) {
        std::vector<cv::Point3f> cv_points;
        for (const auto& point : pcl_points.points) {
            cv_points.emplace_back(point.x, point.y, point.z);
        }
        return cv_points;
    }

    std::vector<cv::Point2f> project_point(const pcl::PointCloud<pcl::PointXYZRGB> pcl_points)
    {

        //TBD read the camera info from rosbags
        // Camera intrinsics
        double fx = 1297.672904;
        double fy = 1298.631344;
        double cx = 620.914026;
        double cy = 238.280325;

        // Camera matrix
        cv::Mat camera_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                                                            0, fy, cy,
                                                            0, 0, 1);

        cv::Mat dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

        cv::Mat rMat = getRotationMatrix();
        cv::Mat tvec = getTranslationVector();

        // Convert PCL point to OpenCV point
        std::vector<cv::Point3f> object_points = convert_to_cv_points(pcl_points);
        std::vector<cv::Point2f> image_points;

        // Project the 3D point into the 2D image plane
        cv::projectPoints(object_points, rMat, tvec, camera_matrix_, dist_coeffs_, image_points);

        // Return the 2D image point
        return image_points;
    }

    // Camera Subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cameraSubscription;

    // Lidar Subscription
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidarSubscription;

    // ColorPoints Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colorPointsPublisher;

    std::unique_ptr<NodeParameters> params;
    cv::Mat image_;
    bool image_received;
    std::string image_frameId;
    std::string lidar_frameId;
    rclcpp::Time time_stamp;
    geometry_msgs::msg::TransformStamped transform;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    int count;
};



int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<color3DPointsNode>());
  rclcpp::shutdown();
  return 0;
}


