#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "autonav_interfaces/msg/encoders.hpp"

#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <chrono>
#include <iomanip>

/**
 * @brief Data Publisher Node for Automated Testing
 * 
 * This node collects data from multiple topics during automated tests:
 * - Subscribes to "/data/toggle_collect" to enable/disable data collection
 * - Subscribes to "/estop" for emergency stop monitoring
 * - Subscribes to test-specific topics (GPS, odometry, cmd_vel, etc.)
 * - Publishes collected data to "/data/dump" topic
 * 
 * The data is formatted as CSV-style strings for easy logging by the automater scripts.
 */

class DataPublisherNode : public rclcpp::Node {
private:
    // Control subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr toggle_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr estop_sub_;
    
    // Data publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_dump_pub_;
    
    // Timer for periodic data publishing
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // State variables
    bool collecting_data_;
    bool estop_triggered_;
    std::string test_id_;
    
    // Latest data storage
    std::string latest_gps_data_;
    std::string latest_odom_data_;
    std::string latest_cmd_vel_data_;
    std::string latest_encoder_data_;
    std::string latest_imu_data_;
    std::string latest_scan_data_;
    std::string latest_lines_data_;
    std::string latest_motor_cmd_data_;
    std::string latest_motor_velocity_data_;
    
    // Generic subscribers - will be created dynamically based on topics_to_monitor
    std::vector<rclcpp::SubscriptionBase::SharedPtr> dynamic_subscribers_;

public:
    DataPublisherNode() : Node("data_publisher_node"), collecting_data_(false), estop_triggered_(false) {
        // Declare parameters
        this->declare_parameter("test_id", "");
        this->declare_parameter("topics_to_monitor", std::vector<std::string>{});
        this->declare_parameter("publish_rate", 10.0);
        
        // Get parameters
        test_id_ = this->get_parameter("test_id").as_string();
        auto topics = this->get_parameter("topics_to_monitor").as_string_array();
        double rate = this->get_parameter("publish_rate").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Starting Data Publisher Node for test: %s", test_id_.c_str());
        
        // Create control subscriptions
        toggle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/data/toggle_collect", 10,
            std::bind(&DataPublisherNode::toggle_callback, this, std::placeholders::_1));
        
        estop_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/estop", 10,
            std::bind(&DataPublisherNode::estop_callback, this, std::placeholders::_1));
        
        // Create data dump publisher
        data_dump_pub_ = this->create_publisher<std_msgs::msg::String>("/data/dump", 10);
        
        // Subscribe to test-specific topics
        for (const auto& topic : topics) {
            RCLCPP_INFO(this->get_logger(), "Monitoring topic: %s", topic.c_str());
            subscribe_to_topic(topic);
        }
        
        // Create timer for periodic publishing
        auto period = std::chrono::duration<double>(1.0 / rate);
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&DataPublisherNode::publish_data, this));
        
        RCLCPP_INFO(this->get_logger(), "Data Publisher Node initialized");
    }

private:
    void subscribe_to_topic(const std::string& topic) {
        if (topic == "/gps_fix") {
            auto sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                topic, 10,
                [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(8)
                       << msg->latitude << "," << msg->longitude << "," << msg->altitude;
                    latest_gps_data_ = ss.str();
                });
            dynamic_subscribers_.push_back(sub);
        }
        else if (topic == "/imu/data") {
            auto sub = this->create_subscription<sensor_msgs::msg::Imu>(
                topic, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(6)
                       << msg->linear_acceleration.x << ","
                       << msg->linear_acceleration.y << ","
                       << msg->linear_acceleration.z << ","
                       << msg->angular_velocity.x << ","
                       << msg->angular_velocity.y << ","
                       << msg->angular_velocity.z << ","
                       << msg->orientation.x << ","
                       << msg->orientation.y << ","
                       << msg->orientation.z;
                    latest_imu_data_ = ss.str();
                });
            dynamic_subscribers_.push_back(sub);
        }
        else if (topic == "/scan") {
            auto sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                topic, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(3)
                       << msg->range_min << "," << msg->range_max << "," << msg->ranges.size();
                    latest_scan_data_ = ss.str();
                });
            dynamic_subscribers_.push_back(sub);
        }
        else if (topic == "/odom") {
            auto sub = this->create_subscription<nav_msgs::msg::Odometry>(
                topic, 10,
                [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(4)
                       << msg->pose.pose.position.x << ","
                       << msg->pose.pose.position.y << ","
                       << msg->pose.pose.orientation.z;
                    latest_odom_data_ = ss.str();
                });
            dynamic_subscribers_.push_back(sub);
        }
        else if (topic == "/cmd_vel") {
            auto sub = this->create_subscription<geometry_msgs::msg::Twist>(
                topic, 10,
                [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(3)
                       << msg->linear.x << "," << msg->angular.z;
                    latest_cmd_vel_data_ = ss.str();
                });
            dynamic_subscribers_.push_back(sub);
        }
        else if (topic == "/encoders") {
            auto sub = this->create_subscription<autonav_interfaces::msg::Encoders>(
                topic, 10,
                [this](const autonav_interfaces::msg::Encoders::SharedPtr msg) {
                    std::stringstream ss;
                    ss << msg->left_motor_count << "," << msg->right_motor_count;
                    latest_encoder_data_ = ss.str();
                });
            dynamic_subscribers_.push_back(sub);
        }
        else if (topic == "/line_detection/lines") {
            // Fallback: if line detection publishes String payloads
            auto sub = this->create_subscription<std_msgs::msg::String>(
                topic, 10,
                [this](const std_msgs::msg::String::SharedPtr msg) {
                    latest_lines_data_ = msg->data;
                });
            dynamic_subscribers_.push_back(sub);
        }
        else if (topic == "/motor/cmd_value") {
            auto sub = this->create_subscription<std_msgs::msg::Int32>(
                topic, 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) {
                    latest_motor_cmd_data_ = std::to_string(msg->data);
                });
            dynamic_subscribers_.push_back(sub);
        }
        else if (topic == "/motor/linear_velocity") {
            auto sub = this->create_subscription<std_msgs::msg::String>(
                topic, 10,
                [this](const std_msgs::msg::String::SharedPtr msg) {
                    latest_motor_velocity_data_ = msg->data;
                });
            dynamic_subscribers_.push_back(sub);
        }
        // Add more topic types as needed
    }

    void toggle_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        collecting_data_ = msg->data;
        if (collecting_data_) {
            RCLCPP_INFO(this->get_logger(), "Data collection ENABLED");
            // Clear previous data
            latest_gps_data_.clear();
            latest_odom_data_.clear();
            latest_cmd_vel_data_.clear();
            latest_encoder_data_.clear();
            latest_imu_data_.clear();
            latest_scan_data_.clear();
            latest_lines_data_.clear();
            latest_motor_cmd_data_.clear();
            latest_motor_velocity_data_.clear();
        } else {
            RCLCPP_INFO(this->get_logger(), "Data collection DISABLED");
        }
    }

    void estop_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string incoming = msg->data;
        
        if (incoming.empty()) {
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "E-Stop message received: %s", incoming.c_str());
        
        if (incoming == "STOP") {
            RCLCPP_WARN(this->get_logger(), "ESTOP PRESSED: Stopping data collection");
            estop_triggered_ = true;
            collecting_data_ = false;
        }
    }

    void publish_data() {
        if (!collecting_data_ || estop_triggered_) {
            return;
        }
        
        // Publish data for each topic separately in the format expected by base_automator:
        // "topic_name,data_type,data_values"
        
        auto msg = std_msgs::msg::String();
        static int debug_count = 0;
        
        // Publish GPS data
        if (!latest_gps_data_.empty()) {
            msg.data = "/gps_fix,NavSatFix," + latest_gps_data_;
            data_dump_pub_->publish(msg);
            if (debug_count < 3) {
                RCLCPP_INFO(this->get_logger(), "Publishing GPS: %s", msg.data.c_str());
            }
        }
        // Publish IMU data
        if (!latest_imu_data_.empty()) {
            msg.data = "/imu/data,Imu," + latest_imu_data_;
            data_dump_pub_->publish(msg);
            if (debug_count < 3) {
                RCLCPP_INFO(this->get_logger(), "Publishing IMU: %s", msg.data.c_str());
            }
        }
        // Publish LaserScan data summary
        if (!latest_scan_data_.empty()) {
            msg.data = "/scan,LaserScan," + latest_scan_data_;
            data_dump_pub_->publish(msg);
            if (debug_count < 3) {
                RCLCPP_INFO(this->get_logger(), "Publishing Scan: %s", msg.data.c_str());
            }
        }
        
        // Publish Odometry data
        if (!latest_odom_data_.empty()) {
            msg.data = "/odom,Odometry," + latest_odom_data_;
            data_dump_pub_->publish(msg);
            if (debug_count < 3) {
                RCLCPP_INFO(this->get_logger(), "Publishing Odom: %s", msg.data.c_str());
            }
        }
        
        // Publish cmd_vel data
        if (!latest_cmd_vel_data_.empty()) {
            msg.data = "/cmd_vel,Twist," + latest_cmd_vel_data_;
            data_dump_pub_->publish(msg);
            if (debug_count < 3) {
                RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: %s", msg.data.c_str());
            }
        }
        
        // Publish encoder data
        if (!latest_encoder_data_.empty()) {
            msg.data = "/encoders,String," + latest_encoder_data_;
            data_dump_pub_->publish(msg);
            if (debug_count < 3) {
                RCLCPP_INFO(this->get_logger(), "Publishing encoders: %s", msg.data.c_str());
            }
        }
        // Publish line detection data if available
        if (!latest_lines_data_.empty()) {
            msg.data = "/line_detection/lines,String," + latest_lines_data_;
            data_dump_pub_->publish(msg);
            if (debug_count < 3) {
                RCLCPP_INFO(this->get_logger(), "Publishing lines: %s", msg.data.c_str());
            }
        }
        
        // Publish motor cmd data
        if (!latest_motor_cmd_data_.empty()) {
            msg.data = "/motor/cmd_value,Int32," + latest_motor_cmd_data_;
            data_dump_pub_->publish(msg);
            if (debug_count < 3) {
                RCLCPP_INFO(this->get_logger(), "Publishing motor_cmd: %s", msg.data.c_str());
            }
        }
        
        // Publish motor velocity data
        if (!latest_motor_velocity_data_.empty()) {
            msg.data = "/motor/linear_velocity,String," + latest_motor_velocity_data_;
            data_dump_pub_->publish(msg);
            if (debug_count < 3) {
                RCLCPP_INFO(this->get_logger(), "Publishing motor_velocity: %s", msg.data.c_str());
            }
        }
        
        debug_count++;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}