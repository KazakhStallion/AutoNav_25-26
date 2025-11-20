#pragma once 

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "serialib.hpp"

class MotorController {
private:
    float speed_mph = 3.5;  // Default speed in MPH (3-4 MPH for competition)
    float speed_increment = 0.01;  // MPH increment when pressing bumpers
    std::pair<int, int> right_turn_speeds = {-10, -10};
    std::pair<int, int> left_turn_speeds = {10, 10};
    std::string comPort;
    int prevLeftEncoderCount = 0;
    int prevRightEncoderCount = 0;
    int temp = 0;

public:
    // Constructor
    MotorController();
    serialib motorSerial;
    // configuration
    char configure(const char * port);

    // Moveeeeeee
    void forward();
    void backward();
    void turnLeft();
    void turnRight();
    void move(float right_speed, float left_speed);
    void moveRaw(int right_cmd, int left_cmd);  // Bypass speed calibration
    void stop();
    void shutdown();

    // Get and set
    void setSpeedMPH(float mph);  // Set desired speed in MPH
    float getSpeedMPH();  // Get current speed in MPH
    void increaseSpeed();
    void decreaseSpeed();
    int  getLeftEncoderCount();
    int getRightEncoderCount();
    int getLeftRPM();
    int getRightRPM();
};