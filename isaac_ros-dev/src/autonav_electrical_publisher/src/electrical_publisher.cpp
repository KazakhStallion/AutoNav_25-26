#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"

// I2C includes
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <functional>

class ElectricalPublisherNode : public rclcpp::Node {
private:
    // I2C variables
    int i2c_fd_;
    // int i2c_address_ = 0x48;  // TODO: Set your I2C device address

    // Conversion factors (multiply by raw decimal value from I2C)
    const double bit_2_mVolt = 1.25;   // Convert raw bits to millivolts
    const double bit_2_mAmp  = 1.0;    // Convert raw bits to milliamps
    const double bit_2_mWatt = 25.0;   // Convert raw bits to milliwatts

    // I2C register/pointer addresses for each measurement
    const uint8_t REG_VOLTAGE = 0x02;  // Voltage register
    const uint8_t REG_POWER   = 0x03;  // Power register
    const uint8_t REG_CURRENT = 0x04;  // Current register

    // Create some data storage variables here:

    // Declare publishers here:

    // Timer for fixed-rate I2C polling loop
    rclcpp::TimerBase::SharedPtr i2c_timer_;
    double loop_rate_hz_ = 10.0;  // Default 10 Hz, adjust as needed (1400 Hz should be the limit)

    // ================================================================
    // I2C polling callback - runs at fixed rate (loop_rate_hz_)
    // ================================================================
    void i2c_timer_callback() {
        // This function runs at a fixed rate (e.g., 10 Hz)
        // Add your chain of I2C commands here:

        // Example: Read voltage, current, and power in sequence
        //
        // uint8_t read_buffer[2];
        // int16_t raw_value;
        //
        // // --- Read Voltage from REG_VOLTAGE (0x02) ---
        // uint8_t reg = REG_VOLTAGE;
        // if (write(i2c_fd_, &reg, 1) == 1) {
        //     if (read(i2c_fd_, read_buffer, 2) == 2) {
        //         raw_value = (read_buffer[0] << 8) | read_buffer[1];
        //         double voltage_mV = raw_value * bit_2_mVolt;
        //         RCLCPP_INFO(this->get_logger(), "Voltage: %.2f mV", voltage_mV);
        //     }
        // }
        //
        // // --- Read Power from REG_POWER (0x03) ---
        // reg = REG_POWER;
        // if (write(i2c_fd_, &reg, 1) == 1) {
        //     if (read(i2c_fd_, read_buffer, 2) == 2) {
        //         raw_value = (read_buffer[0] << 8) | read_buffer[1];
        //         double power_mW = raw_value * bit_2_mWatt;
        //         RCLCPP_INFO(this->get_logger(), "Power: %.2f mW", power_mW);
        //     }
        // }
        //
        // // --- Read Current from REG_CURRENT (0x04) ---
        // reg = REG_CURRENT;
        // if (write(i2c_fd_, &reg, 1) == 1) {
        //     if (read(i2c_fd_, read_buffer, 2) == 2) {
        //         raw_value = (read_buffer[0] << 8) | read_buffer[1];
        //         double current_mA = raw_value * bit_2_mAmp;
        //         RCLCPP_INFO(this->get_logger(), "Current: %.2f mA", current_mA);
        //     }
        // }
        //
        // // ================================================================
        // // Publish the three data values to their respective topics
        // // ================================================================
        // //
        // // Create message objects
        // auto voltage_msg = std_msgs::msg::Float32();
        // auto current_msg = std_msgs::msg::Float32();
        // auto power_msg = std_msgs::msg::Float32();
        // //
        // // Set the data (convert mV to V, mA to A, mW to W if desired)
        // voltage_msg.data = voltage_mV / 1000.0;  // Convert mV to V
        // current_msg.data = current_mA / 1000.0;  // Convert mA to A
        // power_msg.data = power_mW / 1000.0;      // Convert mW to W
        // //
        // // Publish to topics
        // voltage_pub_->publish(voltage_msg);
        // current_pub_->publish(current_msg);
        // power_pub_->publish(power_msg);
        // //
        // // ================================================================
    }

    // I2C helper functions
    bool open_i2c(const std::string& device, int address) {
        i2c_fd_ = open(device.c_str(), O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device: %s", device.c_str());
            return false;
        }
        if (ioctl(i2c_fd_, I2C_SLAVE, address) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set I2C address: 0x%02X", address);
            close(i2c_fd_);
            return false;
        }
        return true;
    }

    void close_i2c() {
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
            i2c_fd_ = -1;
        }
    }

    // TODO: Add I2C read/write functions as needed
    // Example:
    // int read_i2c(uint8_t* buffer, size_t length) {
    //     return read(i2c_fd_, buffer, length);
    // }
    // int write_i2c(const uint8_t* data, size_t length) {
    //     return write(i2c_fd_, data, length);
    // }

public:
    ElectricalPublisherNode() : Node("electrical_publisher_node"), i2c_fd_(-1) {
        // TODO: Implement electrical publisher

        // Initialize I2C:
        // if (!open_i2c("/dev/i2c-1", i2c_address_)) {
        //     RCLCPP_ERROR(this->get_logger(), "I2C initialization failed");
        // }

        // ================================================================
        // EXAMPLE: Send "01 AA" to register 0x05 on slave address 0x40
        // ================================================================
        //
        // Step 1: Open I2C bus and set slave address to 0x40
        //     if (!open_i2c("/dev/i2c-7", 0x40)) {  // Use i2c-7 for Jetson pins 3/5
        //         RCLCPP_ERROR(this->get_logger(), "Failed to open I2C for 0x40");
        //         return;
        //     }
        //
        // Step 2: Build the data buffer
        //     The write format is: [register_address, data_byte_1, data_byte_2, ...]
        //     To write "01 AA" to register 0x05:
        //         - First byte: 0x05 (register/pointer address)
        //         - Second byte: 0x01 (first data byte)
        //         - Third byte: 0xAA (second data byte)
        //
        //     uint8_t buffer[3] = {0x05, 0x01, 0xAA};
        //
        // Step 3: Write to the device
        //     if (write(i2c_fd_, buffer, sizeof(buffer)) != sizeof(buffer)) {
        //         RCLCPP_ERROR(this->get_logger(), "I2C write failed");
        //     } else {
        //         RCLCPP_INFO(this->get_logger(), "Sent 01 AA to register 0x05 on slave 0x40");
        //     }
        //
        // ================================================================

        // ================================================================
        // EXAMPLE: Read 2 bytes from register 0x05 on slave address 0x40
        // ================================================================
        //
        // Reading from I2C is a two-step process:
        //   1. Write the register/pointer address you want to read from
        //   2. Read the data bytes
        //
        // Step 1: Open I2C bus and set slave address to 0x40 (if not already open)
        //     if (!open_i2c("/dev/i2c-7", 0x40)) {
        //         RCLCPP_ERROR(this->get_logger(), "Failed to open I2C for 0x40");
        //         return;
        //     }
        //
        // Step 2: Write the register address to set the pointer
        //     uint8_t reg_addr = 0x05;
        //     if (write(i2c_fd_, &reg_addr, 1) != 1) {
        //         RCLCPP_ERROR(this->get_logger(), "Failed to set register pointer");
        //         return;
        //     }
        //
        // Step 3: Read the data from that register
        //     uint8_t read_buffer[2];  // Reading 2 bytes
        //     if (read(i2c_fd_, read_buffer, sizeof(read_buffer)) != sizeof(read_buffer)) {
        //         RCLCPP_ERROR(this->get_logger(), "I2C read failed");
        //     } else {
        //         RCLCPP_INFO(this->get_logger(), "Read from 0x05: 0x%02X 0x%02X",
        //                     read_buffer[0], read_buffer[1]);
        //     }
        //
        
        // ================================================================

        // Create a topic to publish to here:

        // ================================================================
        // Create timer for fixed-rate I2C polling loop
        // ================================================================
        // The timer calls i2c_timer_callback() at the specified rate
        // Adjust loop_rate_hz_ to change the polling frequency
        //
        auto timer_period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
        i2c_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
            std::bind(&ElectricalPublisherNode::i2c_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "I2C polling loop started at %.1f Hz", loop_rate_hz_);
    }

    ~ElectricalPublisherNode() {
        close_i2c();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ElectricalPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
