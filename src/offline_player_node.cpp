#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <map>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

class OfflinePlayerNode : public rclcpp::Node
{
public:
    OfflinePlayerNode() : Node("offline_player_node")
    {
        this->declare_parameter<std::string>("bag_path", "");
        this->declare_parameter<std::string>("scan_topic", "/robosense/points");
        this->declare_parameter<std::string>("odom_topic", "/icp_odom");
        this->declare_parameter<int>("max_buffer_size", 2);

        bag_path_ = this->get_parameter("bag_path").as_string();
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        max_buffer_size_ = this->get_parameter("max_buffer_size").as_int();

        scans_in_flight_ = 0;
        scans_skipped_ = 0;

        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 100);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 100,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                (void)msg;
                // A scan was processed
                if (scans_in_flight_ > 0) {
                    scans_in_flight_--;
                }
            });

        // Start playback thread
        play_thread_ = std::thread(&OfflinePlayerNode::play, this);
    }

    ~OfflinePlayerNode()
    {
        if (play_thread_.joinable()) {
            play_thread_.join();
        }
    }

private:
    void play()
    {
        if (bag_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No bag path provided.");
            rclcpp::shutdown();
            return;
        }

        rosbag2_cpp::Reader reader;
        rosbag2_storage::StorageOptions storage_options{};
        storage_options.uri = bag_path_;
        storage_options.storage_id = "mcap";

        rosbag2_cpp::ConverterOptions converter_options{};
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        try {
            reader.open(storage_options, converter_options);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open bag %s: %s", bag_path_.c_str(), e.what());
            rclcpp::shutdown();
            return;
        }

        auto topics_types = reader.get_all_topics_and_types();
        std::map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers;

        for (const auto& topic_metadata : topics_types) {
            if (topic_metadata.name.find("tf") == std::string::npos &&
                topic_metadata.name.find("vectornav") == std::string::npos &&
                topic_metadata.name.find("robosense") == std::string::npos &&
                topic_metadata.name.find("odom") == std::string::npos) {
                continue;
            }

            rclcpp::QoS qos(100);
            if (topic_metadata.name == "/tf_static" || topic_metadata.name == "tf_static") {
                qos.transient_local();
            }
            publishers[topic_metadata.name] = this->create_generic_publisher(topic_metadata.name, topic_metadata.type, qos);
            RCLCPP_INFO(this->get_logger(), "Discovered topic: %s of type %s", topic_metadata.name.c_str(), topic_metadata.type.c_str());
        }

        auto last_scan_time = std::chrono::steady_clock::now();
        bool first_scan = true;

        while (rclcpp::ok() && reader.has_next()) {
            auto bag_message = reader.read_next();

            // Publish clock
            rosgraph_msgs::msg::Clock clock_msg;
            clock_msg.clock.sec = bag_message->time_stamp / 1000000000LL;
            clock_msg.clock.nanosec = bag_message->time_stamp % 1000000000LL;
            clock_pub_->publish(clock_msg);

            std::string topic_name = bag_message->topic_name;

            if (topic_name == scan_topic_) {
                if (scans_skipped_ < 10) {
                    scans_skipped_++;
                    continue; // Skip the first 10 scans to let TF buffer fill
                }

                while (rclcpp::ok() && scans_in_flight_ >= max_buffer_size_) {
                    std::this_thread::sleep_for(20ms);
                }
                scans_in_flight_++;
            }

            if (publishers.find(topic_name) != publishers.end()) {
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                
                if (topic_name == scan_topic_) {
                    auto current_time = std::chrono::steady_clock::now();
                    if (!first_scan) {
                        auto delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_scan_time).count();
                        std::cout << "Published scan in time: " << delay_ms << " ms" << std::endl;
                    }
                    first_scan = false;
                    last_scan_time = current_time;
                }
                
                publishers[topic_name]->publish(serialized_msg);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Finished reading bag. Waiting for remaining scans...");
        while (rclcpp::ok() && scans_in_flight_ > 0) {
            std::this_thread::sleep_for(100ms);
        }

        RCLCPP_INFO(this->get_logger(), "All scans processed. Shutting down offline player.");
        rclcpp::shutdown();
    }

    std::string bag_path_;
    std::string scan_topic_;
    std::string odom_topic_;
    int max_buffer_size_;
    std::atomic<int> scans_in_flight_;
    int scans_skipped_;

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::thread play_thread_;
};

int main(int argc, char** argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OfflinePlayerNode>();
    
    // Use multi-threaded executor to allow playback and subscription callbacks concurrently
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
