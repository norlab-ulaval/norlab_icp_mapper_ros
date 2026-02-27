#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>

class WheelVelocityNoiseNode : public rclcpp::Node
{
public:
    WheelVelocityNoiseNode()
    : Node("wheel_velocity_noise_node"),
      generator_(std::random_device{}()),
      distribution_(-0.2, 0.2) // mean = -0.2, stddev = 0.2
    {
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/warthog/platform/odom_noisy", 10);
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/warthog/platform/odom", 10,
            std::bind(&WheelVelocityNoiseNode::odom_callback, this, std::placeholders::_1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto noisy_msg = *msg;

        // Apply random gaussian noise to the x velocity component
        double noise = distribution_(generator_);
        noisy_msg.twist.twist.linear.x += noise;

        publisher_->publish(noisy_msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelVelocityNoiseNode>());
    rclcpp::shutdown();
    return 0;
}
