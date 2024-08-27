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
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class GetTransformVectors: public rclcpp::Node
{
public:
    GetTransformVectors() : Node("GetTransformVectors"){
        tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    }

    GetTransformVectors(std::string& target_frame, std::string& source_frame, rclcpp::Time& time_stamp): Node("GetTransformVectors"){
        tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        transform = getTransform(target_frame, source_frame, time_stamp);
    }

    geometry_msgs::msg::TransformStamped getTransform(std::string& target_frame, std::string& source_frame, rclcpp::Time& time_stamp)
    {
        try {
            //transform = tfBuffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            transform = tfBuffer->lookupTransform(
                "arm_camera_color_frame",  // Target frame
                "lidar_link",   // Source frame
                tf2::TimePointZero // Use the latest available transform
            );
            RCLCPP_INFO(this->get_logger(), "Transform from LiDAR to Camera: [%f, %f, %f]",
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        }
        return transform;
    }

    cv::Mat getTranslationVector()
    {
        cv::Mat translation_vector = (cv::Mat_<double>(3, 1) << transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
        return translation_vector;
    }

    cv::Mat getRotationVector()
    {
        cv::Mat rotation_vector = (cv::Mat_<double>(3, 1) << transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);
        return rotation_vector;
    }

    cv::Mat getRotationMatrix()
    {
        cv::Mat rotation_matrix;
        cv::Rodrigues(getRotationVector(), rotation_matrix);
        return rotation_matrix;
    }

private:
    geometry_msgs::msg::TransformStamped transform;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
};


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
            "points_in", 10, std::bind(&color3DPointsNode::colorlidarPointsCallback, this, std::placeholders::_1));

/*
        lidarSubscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points_in", 10, std::bind(&color3DPointsNode::lidarCallback_nonpcl, this, std::placeholders::_1));
*/
        // Create a publisher for the colorPoints
        colorPointsPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("colorlidarpoints", 10);
        tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    }

private:
    void getTransform(std::string& target_frame, std::string& source_frame, rclcpp::Time& time_stamp)
    {
        try {
            transform = tfBuffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            // transform = tfBuffer->lookupTransform(
            //     "arm_camera_color_frame",  // Target frame
            //     "lidar_link",   // Source frame
            //     tf2::TimePointZero // Use the latest available transform
            //);
            RCLCPP_INFO(this->get_logger(), "Transform from LiDAR to Camera: [%f, %f, %f]",
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
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
    void saveVectorToCSV(const std::vector<cv::Point2f>& points, const std::string& filename) {
        std::ofstream file(filename);

        if (file.is_open()) {
            for (const auto& point : points) {
                file << point.x << "," << point.y << "\n";
            }
            file.close();
        }
        else {
            std::cerr << "Unable to open file for writing: " << filename << std::endl;
        }
    }

    bool is_point_in_image(const cv::Point2f& point) {
        return point.x >= 0 && point.x < image_.cols && point.y >= 0 && point.y < image_.rows;
    }


    // Callback function for the camera topic
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr camera_msg)
    {
        // Do something with the camera data
        RCLCPP_INFO(this->get_logger(), "Received camera data height=%d weight=%d", camera_msg->height, camera_msg->width);
        try {
            // Convert ROS Image message to OpenCV image
            image_ = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8)->image;
            image_received = true;
            image_frameId  = camera_msg->header.frame_id;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }

    }

    void colorlidarPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
    {
        // Convert the sensor_msgs::msg::PointCloud2 message to a PCL point cloud
        RCLCPP_INFO(this->get_logger(), "in colorlidarPointsCallback1");

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*lidar_msg, pcl_cloud);

        /* TBD - replace manual copy to colored cloud with this
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*pointcloud_msg, *colored_cloud);
        */
       RCLCPP_INFO(this->get_logger(), "in colorlidarPointsCallback2");
        if (!image_received) {
            RCLCPP_WARN(this->get_logger(), "No image received yet");
            colorPointsPublisher->publish(*lidar_msg);
            return;
        }

        lidar_frameId = lidar_msg->header.frame_id;
        time_stamp    = lidar_msg->header.stamp;
        count += 1;



        // Create a new point cloud with color information
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_colored;
        pcl_cloud_colored.header = pcl_cloud.header;

        // Add the color information to the point cloud. Make all points black
        for (const auto& point : pcl_cloud.points) {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            // Correctly assign RGB values
            uint8_t r = 0;  // Red channel
            uint8_t g = 0;    // Green channel
            uint8_t b = 0;    // Blue channel
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 |
                    static_cast<uint32_t>(b));

            colored_point.rgb = *reinterpret_cast<float*>(&rgb);

            pcl_cloud_colored.points.push_back(colored_point);
        }
        RCLCPP_INFO(this->get_logger(), "Point cloud colored");
        std::vector<cv::Point2f> image_points = project_point(pcl_cloud_colored);

        if (count <= 1){
            saveVectorToCSV(image_points, "/home/dheeraj/demo/image_points/image_points_2.csv");
        }

        //Add color to the points that are in the image
        for (std::size_t index = 0; index < image_points.size(); index++) {
            //RCLCPP_INFO(this->get_logger(), "Global Point in image index %ld x %f y %f", index, image_points[index].x, image_points[index].y);
            if (is_point_in_image(image_points[index])) {
                //RCLCPP_INFO(this->get_logger(), "Point in image index %ld x %f y %f", index, image_points[index].x, image_points[index].y);
                cv::Vec3b color = image_.at<cv::Vec3b>(image_points[index]);
                pcl_cloud_colored.points[index].r = color[2];
                pcl_cloud_colored.points[index].g = color[1];
                pcl_cloud_colored.points[index].b = color[0];
            }
        }

        // Convert the PCL point cloud with color back to a sensor_msgs::msg::PointCloud2 message
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(pcl_cloud_colored, output_msg);

        // Publish the colored point cloud
        //publisher_->publish(output_msg);
        colorPointsPublisher->publish(output_msg);
    }

    std::vector<cv::Point3f> convert_to_cv_points(const pcl::PointCloud<pcl::PointXYZRGB> pcl_points) {
        std::vector<cv::Point3f> cv_points;
        for (const auto& point : pcl_points.points) {
            cv_points.emplace_back(point.x, point.y, point.z);
        }
        return cv_points;
    }

    std::vector<cv::Point2f> project_point(const pcl::PointCloud<pcl::PointXYZRGB> pcl_points) {

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

        //TBD get the rvec and tvec from transform lookup
        getTransform(image_frameId, lidar_frameId, time_stamp);
        cv::Mat rMat = getRotationMatrix();
        cv::Mat tvec = getTranslationVector();

        // Project the point to the image plane
        // Convert PCL point to OpenCV point
        std::vector<cv::Point3f> object_points = convert_to_cv_points(pcl_points);
        std::vector<cv::Point2f> image_points;

        // Project the 3D point into the 2D image plane
        cv::projectPoints(object_points, rMat, tvec, camera_matrix_, dist_coeffs_, image_points);

        // Return the 2D image point
        return image_points;
    }

    // Declare the camera subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cameraSubscription;

    // Declare the lidar subscription
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidarSubscription;

    // Declare the colorPoints publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colorPointsPublisher;

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


