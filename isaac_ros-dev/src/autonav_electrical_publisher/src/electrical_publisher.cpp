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

    // INA226 conversion factors (per-bit LSB values)
    const double bit_2_mVolt = 1.25;    // 1.25 mV/bit (bus voltage LSB)
    const double bit_2_mAmp  = 0.25;   // 250 µA/bit (current LSB)
    const double bit_2_mWatt = 6.25;   // 6.25 mW/bit (power LSB = 25 * current LSB)

    // I2C register/pointer addresses for each measurement
    const uint8_t REG_VOLTAGE = 0x02;  // Voltage register
    const uint8_t REG_POWER   = 0x03;  // Power register
    const uint8_t REG_CURRENT = 0x04;  // Current register

    // Create some data storage variables here:
    double voltage_mV_ = 0.0;
    double current_mA_ = 0.0;
    double power_mW_   = 0.0;

    bool chip_ready_ = false;

    // Declare publishers here:

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr power_pub_;

    // Timer for fixed-rate I2C polling loop
    rclcpp::TimerBase::SharedPtr i2c_timer_;
    double loop_rate_hz_ = 10.0;  // Default 10 Hz, adjust as needed (1400 Hz should be the limit)

    // ================================================================
    // I2C polling callback - runs at fixed rate (loop_rate_hz_)
    // ================================================================
    void i2c_timer_callback() {
        if (!chip_ready_) {
            return;
        }

        uint8_t buf[2];
        int16_t raw;

        // Voltage
        uint8_t reg = REG_VOLTAGE;
        if (write(i2c_fd_, &reg, 1) == 1 && read(i2c_fd_, buf, 2) == 2) {
            raw = (buf[0] << 8) | buf[1];
            voltage_mV_ = raw * bit_2_mVolt;
        }

        // Current
        reg = REG_CURRENT;
        if (write(i2c_fd_, &reg, 1) == 1 && read(i2c_fd_, buf, 2) == 2) {
            raw = (buf[0] << 8) | buf[1];
            current_mA_ = raw * bit_2_mAmp;
        }

        // Power
        reg = REG_POWER;
        if (write(i2c_fd_, &reg, 1) == 1 && read(i2c_fd_, buf, 2) == 2) {
            raw = (buf[0] << 8) | buf[1];
            power_mW_ = raw * bit_2_mWatt;
        }

        // Publish (convert milli-units to base units)
        std_msgs::msg::Float32 msg;

        msg.data = voltage_mV_ / 1000.0;
        voltage_pub_->publish(msg);

        msg.data = current_mA_ / 1000.0;
        current_pub_->publish(msg);

        msg.data = power_mW_ / 1000.0;
        power_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
            "V=%.2f V | I=%.3f A | P=%.1f W",
            voltage_mV_ / 1000.0, current_mA_ / 1000.0, power_mW_ / 1000.0);
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
            if (!open_i2c("/dev/i2c-7", 0x40)) {
                RCLCPP_ERROR(this->get_logger(), "I2C init failed");
                return;
            }   

        //
        // Step 2: Write the register address to set the pointer
            // Write INA226 calibration register (0x05) with CAL = 2048 (0x0800)
            // CAL = 0.00512 / (CURRENT_LSB * R_SHUNT) = 0.00512 / (0.00025 * 0.010) = 2048
            uint8_t calib_buf[3] = {0x05, 0x08, 0x00};
            if (write(i2c_fd_, calib_buf, sizeof(calib_buf)) != sizeof(calib_buf)) {
                RCLCPP_ERROR(this->get_logger(), "Calibration write failed");
                return;
            }

        // Step 3: Read back calibration register to verify
            uint8_t reg = 0x05;
            uint8_t read_buf[2];

            if (write(i2c_fd_, &reg, 1) == 1 &&
              read(i2c_fd_, read_buf, 2) == 2 &&
              read_buf[0] == 0x08 &&
              read_buf[1] == 0x00) {

              chip_ready_ = true;
              RCLCPP_INFO(this->get_logger(), "INA226 calibrated and ready (CAL=2048)");
            } else {
              RCLCPP_ERROR(this->get_logger(),
                "INA226 calibration readback failed (got 0x%02X%02X, expected 0x0800)",
                read_buf[0], read_buf[1]);
            }
        
        // ================================================================

        // Create a topic to publish to here:

        voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/electrical/voltage", 10);

        current_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/electrical/current", 10);

        power_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/electrical/power", 10);

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
