#include <rclcpp/rclcpp.hpp>
#include "serialib.hpp"
#include "std_msgs/msg/string.hpp"

#include <string>

#define DEBUG_ESTOP

// This node is meant to be looking at two main topics:
//  Subscribe "/data/toggle_collect" Topic
//  Publish   "/data/dump" Topic
//
// When a test is initiated through one of the t001_[TEST NAME].launch.py files
// this Node will be started. Then it will look at the "/data/toggle_collect" Topic
//
// Depending on how the Node is launched, it will additionally look at other
// data Topics hosted by a number of other Nodes in the ROS2 stack.
//
// When allowed by seeing a TRUE through the "/data/toggle_collect" Topic
// Will start publishing all data from all Topics to "/data/dump" Topic
//
// This "/data/dump" Topic will then be put into log files by the corresponding
// t001_automater.py file.

// ALL CODE HERE IS Pseudo Code for setting up the purpose of this node.

class ControlNode : public rclcpp::Node {

    public:

    Data_Publisher_Node() : Node("data_publisher_node"){
        // Topics to Subscribe to:
        this->declare_parameter("data_toggle_collect", "/data/toggle_collect");

        // Serial Ports (Always be on the E-Stop for if testing needs to be ended)
        this->declare_parameter("estop_port", "/dev/ttyTHS1");
    }

    // Subscribe to "/data/toggle_collect" Topic (This topic holds a Boolean)
    // This tells the node when to start and stop publishing data to the "/data/dump" Topic
    formatrclcpp::Subscription<data_toggle_msgs::msg::Data>::SharedPtr dataSub;

    // Depends on the test being executed "t001", and what the testing_data_collection_setter.yaml says.
    // Using the testing_data_collection_setter.yaml specific test "t001" for example: {"/gps/fix", "/odom", "/tf", "/encoders", "/cmd_vel"}
    // These are from the parameters upon a .launch.py file launching the Node.
    std::topics_to_subscribe<str> topics = {};
    for (int topic : topics) {
        formatrclcpp::Subscription<TODO::msg::TODO>::SharedPtr TODOSub;
    }

    // Publish to "/data/dump" Topic
    // This is a CSV style data output format using all the Subscribed Topics. May need to be formatted.
    rclcpp::Publisher<data::msg::Data>::SharedPtr dataPub;

    void data_publishing_formater(const std_msgs::msg::String::SharedPtr dataPub){
    }

    void estop_callback(const std_msgs::msg::String::SharedPtr msg){
	    std::string incoming = msg->data; 
        if (incoming.empty()) {
	        #ifdef DEBUG_ESTOP
	        RCLCPP_INFO(this->get_logger(), "incoming string empty");
	        #endif
            return;
            }

            #ifdef DEBUG_ESTOP
            RCLCPP_INFO(this->get_logger(), "incoming string: %s", incoming.c_str());
            #endif

            if (incoming == "STOP") {
                RCLCPP_WARN(this->get_logger(), "ESTOP PRESSED: MOTORS SHUTTING DOWN");
                motors.shutdown();
            }
    }  

    void configure(const std::shared_ptr<autonav_interfaces::srv::ConfigureControl::Request> request, 
                         std::shared_ptr<autonav_interfaces::srv::ConfigureControl::Response> response) {


        // configure serial
        std::string estop_port = this->get_parameter("estop_port").as_string();

        estop_sub_ = this->create_subscription<std_msgs::msg::String>("/estop", 10, std::bind(&ControlNode::estop_callback, this, std::placeholders::_1));
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc,argv);
    rclcpp::shutdown();
    return 0;
}