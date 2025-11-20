#include <rclcpp/rclcpp.hpp>
#include "serialib.hpp"
#include "xbox.hpp"
#include "motor_controller.hpp"
#include "autonomous.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "autonav_interfaces/msg/encoders.hpp"
#include "autonav_interfaces/srv/configure_control.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

#include <iostream>
#include <string>

#define WHEEL_BASE 0.6858

#define DEBUG_ESTOP

class ControlNode : public rclcpp::Node {

    public:


    ControlNode() 
      : Node("control_node")
       {

        // topic names
        this->declare_parameter("controller_topic", "joy");
        this->declare_parameter("encoder_topic", "encoders");
        this->declare_parameter("path_planning_topic", "cmd_vel");

        // serial ports
        this->declare_parameter("motor_port", "/dev/ttyACM0");
        this->declare_parameter("arduino_port", "/dev/ttyACM2");
        this->declare_parameter("estop_port", "/dev/ttyTHS1");

        configure_server = this->create_service<autonav_interfaces::srv::ConfigureControl>
             ("configure_control", std::bind(&ControlNode::configure, this, std::placeholders::_1, std::placeholders::_2));


    }

    serialib arduinoSerial;
    Xbox controller;
    MotorController motors;
    Autonomous currPose;

    private:
    bool autonomousMode = false;
    float linear_move;
    float angular_move;
    float left_wheel_speed;
    float right_wheel_speed;

    // Velocity calculation from encoders
    int prev_left_encoder_ = 0;
    int prev_right_encoder_ = 0;
    rclcpp::Time prev_encoder_time_;
    bool encoder_init_ = false;
    const double wheel_diameter_m_ = 0.254;  // 10 inches
    const double wheel_circumference_m_ = 3.14159 * wheel_diameter_m_;
    const int encoder_counts_per_rev_ = 81923;

    // subscription for joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controllerSub;
    //configuration server
    rclcpp::Service<autonav_interfaces::srv::ConfigureControl>::SharedPtr configure_server;

    // publisher for encoder values
     rclcpp::Publisher<autonav_interfaces::msg::Encoders>::SharedPtr encodersPub;
     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_velocity_pub_;
     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr estop_sub_;
     rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr raw_cmd_sub_;
     rclcpp::TimerBase::SharedPtr encoder_timer_;

    // rclcpp::TimerBase::SharedPtr joy_timer_;
    // subscription for Nav2 pose
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pathPlanningSub;

    bool currX = false;
    bool prevX = false;

    void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        currX = joy_msg->buttons[3];

        // Detect rising edge
        if (!prevX && currX) {
            autonomousMode = !autonomousMode;

            if (autonomousMode) {
                char mode[12] = "AUTONOMOUS\n";
                arduinoSerial.writeString(mode);
            } else {
                char mode[8] = "MANUAL\n";
                arduinoSerial.writeString(mode);
            }
        }

        prevX = currX;

        if(!autonomousMode){
            controller.set_b(joy_msg->buttons[1]);
            controller.set_x(joy_msg->buttons[3]);
            controller.set_y(joy_msg->buttons[2]);

            controller.set_right_bumper(joy_msg->buttons[6]);
            controller.set_left_bumper(joy_msg->buttons[7]);

            controller.set_left_stick_x(joy_msg->axes[0]);
            controller.set_left_stick_y(joy_msg->axes[1]);
            controller.set_right_stick_x(joy_msg->axes[2]);
            controller.set_right_stick_y(joy_msg->axes[3]);

        }
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
    


    void publish_encoder_data() {
        autonav_interfaces::msg::Encoders encoder_msg;
        encoder_msg.left_motor_rpm = 0;
        encoder_msg.right_motor_rpm = 0;
        encoder_msg.left_motor_count = motors.getLeftEncoderCount();
        encoder_msg.right_motor_count = motors.getRightEncoderCount();
        //RCLCPP_INFO(this->get_logger(), "LEC: %s", motors.getLeftEncoderCount());

        // Calculate linear velocity from encoder deltas
        auto current_time = this->get_clock()->now();
        if (encoder_init_) {
            double dt = (current_time - prev_encoder_time_).seconds();
            if (dt > 0) {
                int delta_left = encoder_msg.left_motor_count - prev_left_encoder_;
                int delta_right = encoder_msg.right_motor_count - prev_right_encoder_;
                double delta_avg = (delta_left + delta_right) / 2.0;
                
                double revolutions = delta_avg / encoder_counts_per_rev_;
                double distance_m = revolutions * wheel_circumference_m_;
                double velocity_ms = distance_m / dt;
                double velocity_mph = velocity_ms * 2.23694;  // m/s to MPH
                
                // Publish velocity as string (can be parsed by data_publisher)
                auto vel_msg = std_msgs::msg::String();
                vel_msg.data = std::to_string(velocity_mph);
                motor_velocity_pub_->publish(vel_msg);
            }
        } else {
            encoder_init_ = true;
        }
        prev_left_encoder_ = encoder_msg.left_motor_count;
        prev_right_encoder_ = encoder_msg.right_motor_count;
        prev_encoder_time_ = current_time;

        std::string arduinoEncoderCounts = "L:";
        arduinoEncoderCounts += encoder_msg.left_motor_count;
        arduinoEncoderCounts += " R:";
        arduinoEncoderCounts += encoder_msg.right_motor_count;



        if(!autonomousMode){
            Xbox::CommandData command = controller.calculateCommand();

            if(command.cmd == Xbox::MOVE){
                // Joystick values are -1 to 1, multiply by current speed in MPH
                float right_mph = command.right_motor_speed * motors.getSpeedMPH();
                float left_mph = command.left_motor_speed * motors.getSpeedMPH();
                motors.move(right_mph, left_mph);
            }
            else if(command.cmd == Xbox::SPEED_DOWN){
                motors.decreaseSpeed();
                RCLCPP_INFO(this->get_logger(), "Speed down. New speed: %.2f MPH", motors.getSpeedMPH());
            }
            else if(command.cmd == Xbox::SPEED_UP){
                motors.increaseSpeed();
                RCLCPP_INFO(this->get_logger(), "Speed up! New speed: %.2f MPH", motors.getSpeedMPH());
            }
            else if(command.cmd == Xbox::STOP){
                motors.shutdown();
            }
        }
        else {
           motors.move(left_wheel_speed * 40, right_wheel_speed * 40);

        }

        arduinoEncoderCounts += "\n";
        // arduinoSerial.writeString(arduinoEncoderCounts.c_str());

        encodersPub->publish(encoder_msg);

    }

    void path_planning_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

        if (autonomousMode) {
            /*Move to Pose Way*/
            // currPose.linearX = msg->linear.x;
            // currPose.linearY = msg->linear.y;
            //currPose.linearZ = msg->linear.z;

            // currPose.angularZ = msg->angular.z;

            // currPose.goToPose(linearX, linearY, motors);

            /*Direct Speed to Motor Way*/

            linear_move = msg->linear.x;
            angular_move = msg->angular.z;

            left_wheel_speed = linear_move - ( angular_move * (WHEEL_BASE/2));
            right_wheel_speed = linear_move +( angular_move * (WHEEL_BASE/2));

        }
    }

    void raw_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        // Bypass speed calibration for raw motor commands (used in t007 test)
        // Send the same CMD value to both motors for straight-line motion
        int cmd_value = msg->data;
        motors.moveRaw(cmd_value, cmd_value);
        #ifdef CONTROL_DEBUG
        RCLCPP_INFO(this->get_logger(), "Raw CMD: %d", cmd_value);
        #endif
    }


    void init_serial_arduino(const char * arduino_port) {

        char ret;
        ret = arduinoSerial.openDevice(arduino_port, 9600);

        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "Arduino serial error: %s", arduinoSerial.error_map.at(ret).c_str());

        }
	else {
	    RCLCPP_INFO(this->get_logger(), "arduino connection success!");
	}


        char mode[8] = "MANUAL\n";
        arduinoSerial.writeString(mode);
    }



    void configure(const std::shared_ptr<autonav_interfaces::srv::ConfigureControl::Request> request, 
                         std::shared_ptr<autonav_interfaces::srv::ConfigureControl::Response> response) {


        // configure serial
        std::string motor_port = this->get_parameter("motor_port").as_string();
        std::string arduino_port = this->get_parameter("arduino_port").as_string();
        std::string estop_port = this->get_parameter("estop_port").as_string();


        if (request->arduino) {
            init_serial_arduino(arduino_port.c_str());
        }
        if (request->motors) {
            char ret = motors.configure(motor_port.c_str());
            if (ret != 1) {
                // RCLCPP_ERROR(this->get_logger(), "Motor serial error: %s", .error_map.at(ret).c_str());
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Motor serial connection success!");
            }

        }

	    
        estop_sub_ = this->create_subscription<std_msgs::msg::String>("/estop", 10, std::bind(&ControlNode::estop_callback, this, std::placeholders::_1));


        std::string leftMotorCommand = "!C 1 0\r";
        std::string rightMotorCommand = "!C 2 0 \r";
        motors.motorSerial.writeString(leftMotorCommand.c_str());
        motors.motorSerial.writeString(rightMotorCommand.c_str());

        // configure topics
        std::string controller_topic = this->get_parameter("controller_topic").as_string();
        std::string encoder_topic = this->get_parameter("encoder_topic").as_string();
        std::string path_planning_topic = "cmd_vel";

        //XBOX SUB
        controllerSub = this->create_subscription<sensor_msgs::msg::Joy>(
            controller_topic, 10, std::bind(&ControlNode::joystick_callback, this, std::placeholders::_1));

        // ESTOP CALLBACK
	
        


        //NAVIGATION ENCODER PUB
        encodersPub = this->create_publisher<autonav_interfaces::msg::Encoders>(encoder_topic, 10);

        // MOTOR VELOCITY PUB (calculated from encoder deltas)
        motor_velocity_pub_ = this->create_publisher<std_msgs::msg::String>("/motor/linear_velocity", 10);

        encoder_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&ControlNode::publish_encoder_data, this)
        );

       /* joy_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ControlNode::joy_timer_callback, this)
        );*/
        //GPS PUB
        //PATH PLANNING SUB
        pathPlanningSub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&ControlNode::path_planning_callback, this, std::placeholders::_1));

        // RAW MOTOR COMMAND SUB (for calibration tests)
        raw_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/motor/cmd_value", 10, std::bind(&ControlNode::raw_cmd_callback, this, std::placeholders::_1));

        response->ret = 0;


    }

};


int main(int argc, char** argv) {

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;

}

