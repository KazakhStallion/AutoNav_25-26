#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstring>
#include <cstdlib>

#include "serialib.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "std_msgs/msg/string.hpp"
#include "autonav_interfaces/msg/encoders.hpp"

using namespace std::chrono_literals;

class WheelOdomPublisher : public rclcpp::Node
{
  public:
    WheelOdomPublisher()
    : Node("wheelodom_publisher"), x_(0.0), y_(0.0), theta_(0.0), linear_velocity_(0.0), angular_velocity_(0.0), 
    wheel_base_(0.6858), wheel_radius_(0.205), prev_left_encoder_count_(0), prev_right_encoder_count_(0), 
    left_encoder_count_(0), right_encoder_count_(0), ticks_per_revolution_(81923)
    {
      encoder_subscription_ = this->create_subscription<autonav_interfaces::msg::Encoders>("encoders", 
      10, std::bind(&WheelOdomPublisher::encoder_callback, this, std::placeholders::_1));

      // TF2 Broadcaster for odom → base_link transform
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
      timer_ = this->create_wall_timer(200ms, std::bind(&WheelOdomPublisher::update_wheel_odom, this));
      last_time_ = this->get_clock()->now();
    }

    // void simulateEncoderCounts(int left, int right)
    // {
    //     left_encoder_count_ = left;
    //     right_encoder_count_ = right;
    // }

  private:
    void encoder_callback(const autonav_interfaces::msg::Encoders::SharedPtr msg)
    {
      int temp_left_encoder_count = left_encoder_count_;
      if (left_encoder_count_ > 0){
      	temp_left_encoder_count = -left_encoder_count_;
      }
      else{
      	temp_left_encoder_count = std::abs(left_encoder_count_);
      }
      int left_delta_count = std::abs(msg->left_motor_count - temp_left_encoder_count);
      int right_delta_count = std::abs(msg->right_motor_count - right_encoder_count_);
      const int MAX_ENCODER_DELTA = 20000;

      if (left_delta_count > MAX_ENCODER_DELTA || right_delta_count > MAX_ENCODER_DELTA){
      	printf("Encountered Bad Encoder Count Reading\n");
      	return;
      }
      // Update encoder values when new data is received
      left_encoder_count_ = msg->left_motor_count;
      right_encoder_count_ = msg->right_motor_count;
      if (left_encoder_count_ > 0) {
      	left_encoder_count_ = -left_encoder_count_;
      }
      else {
      	left_encoder_count_ = std::abs(left_encoder_count_);
      }
    }

    void update_wheel_odom()
    {
      // Get the current time and calculate the time difference (dt)
      auto current_time = this->get_clock()->now();
      double dt = (current_time - last_time_).seconds();  // Time in seconds
      last_time_ = current_time;  // Update last_time_ for the next callback

      // Calculate the change in encoder counts
      int left_delta_ticks = left_encoder_count_ - prev_left_encoder_count_;
      int right_delta_ticks = right_encoder_count_ - prev_right_encoder_count_;

      // Update previous encoder counts for next iteration
      prev_left_encoder_count_ = left_encoder_count_;
      prev_right_encoder_count_ = right_encoder_count_;

      // Convert encoder ticks to linear displacement (in meters)
      double left_displacement = (2 * M_PI * wheel_radius_) * (left_delta_ticks / (double)ticks_per_revolution_);
      double right_displacement = (2 * M_PI * wheel_radius_) * (right_delta_ticks / (double)ticks_per_revolution_);
      
      // Compute robot's linear velocity and angular velocity
      double forward_displacement = (left_displacement + right_displacement) / 2.0;
      double angular_displacement = (right_displacement - left_displacement) / wheel_base_;
      
      // Update robot's position (x, y) and orientation (theta)
      x_ += forward_displacement * cos(theta_);
      y_ += forward_displacement * sin(theta_);
      theta_ += angular_displacement;

      // Normalize theta to be within [-pi, pi]
      if (theta_ > M_PI)
          theta_ -= 2 * M_PI;
      if (theta_ < -M_PI)
          theta_ += 2 * M_PI;

      // Calculate linear and angular velocities
      linear_velocity_ = forward_displacement / dt;  // meters per second (m/s)
      angular_velocity_ = angular_displacement / dt;  // radians per second (rad/s)

      // Print x, y, theta, linear, and angular velocity for debugging
      printf("Position: x = %f, y = %f, theta = %f\n", x_, y_, theta_);
      printf("Velocity: linear = %f, angular = %f\n", linear_velocity_, angular_velocity_);

      // < ----------------------------- Now move onto publishing the wheel odometry topic with type nav_msgs::msg::Odometry ----------------------------- >
      nav_msgs::msg::Odometry wheel_odom_msg;
      // Setup basic header info like timestamp, parent frame, and child frame
      wheel_odom_msg.header.stamp = this->now();  // Timestamp
      wheel_odom_msg.header.frame_id = "odom"; // Parent Frame
      wheel_odom_msg.child_frame_id = "base_link"; // Child Frame (The robot's base frame)
      
      // Fill the nav_msgs::msg::Odometry message with relevant wheel odometry information calculated previously
      // Fill in positional data
      wheel_odom_msg.pose.pose.position.x = x_;
      wheel_odom_msg.pose.pose.position.y = y_;
      wheel_odom_msg.pose.pose.position.z = 0.0;  // We are assuming flat ground for most of the course conditions

      // Fill in orientation data
      tf2::Quaternion q;
      q.setRPY(0, 0, theta_); // Converts Eular Angles to Quaternion(Still not fully sure what a Quaternion is)
      wheel_odom_msg.pose.pose.orientation.x = q.x();
      wheel_odom_msg.pose.pose.orientation.y = q.y();
      wheel_odom_msg.pose.pose.orientation.z = q.z();
      wheel_odom_msg.pose.pose.orientation.w = q.w();

      // Fill in twist data i.e. linear and angular velocities
      wheel_odom_msg.twist.twist.linear.x = linear_velocity_; // units are m/s
      wheel_odom_msg.twist.twist.angular.z = angular_velocity_; // units are rads/s

      // < ----------------------------- Publish the wheel odometry info ----------------------------- >
      publisher_->publish(wheel_odom_msg);
      
      // Now work on broadcasting transform (odom -> base_link)
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();
      transform.header.frame_id = "odom";
      transform.child_frame_id = "base_link";
      
      // Fill in transform.transform.translation
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.translation.z = 0.0;  // Assuming no vertical displacement
      
      // Fill in transform.transform.rotation
      transform.transform.rotation.x = q.x();
      transform.transform.rotation.y = q.y();
      transform.transform.rotation.z = q.z();
      transform.transform.rotation.w = q.w();

      // Send the transform
      #ifdef PUBLISH_TRANSFORM
      tf_broadcaster_->sendTransform(transform);
      #endif
    }

    double x_, y_, theta_;  // Robot's position (x, y) and orientation (theta)
    double linear_velocity_, angular_velocity_; // Robot's linear and angular velocity 
    double wheel_base_;  // Distance between wheels (L)
    double wheel_radius_;  // Radius of the wheels (r)
    int prev_left_encoder_count_, prev_right_encoder_count_;  // Previous encoder counts for delta calculation
    int left_encoder_count_, right_encoder_count_; // Left Encoder and Right Encoder count reading from control topic
    const int ticks_per_revolution_;  // Number of encoder ticks per wheel revolution
    rclcpp::Time last_time_; //Time of the last callback to calculate dt

    serialib motorController;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // TF2 broadcaster for odom → base_link transform
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<autonav_interfaces::msg::Encoders>::SharedPtr encoder_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdomPublisher>());
  // auto odometry_node = std::make_shared<WheelOdomPublisher>();
  // odometry_node->simulateEncoderCounts(80000, 80000);  // First update (left = 1000, right = 1200)
  // std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait 1 second for odometry update
  // odometry_node->simulateEncoderCounts(40000, 120000);  // Second update (left = 2000, right = 2400)
  // std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait 1 second for odometry update
  // rclcpp::spin(odometry_node);
  
  rclcpp::shutdown();
  return 0;
}
