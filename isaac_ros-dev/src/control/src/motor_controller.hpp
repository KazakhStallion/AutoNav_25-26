#pragma once 

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "serialib.hpp"

class MotorController {
private:
    int stepSize = 10;
    std::pair<int, int> right_turn_speeds = {-10, -10};
    std::pair<int, int> left_turn_speeds = {10, 10};
    int speed = 11;
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
    void setStepSize(int size);
    int getStepSize();
    void setSpeed(int s);
    int getSpeed();
    int  getLeftEncoderCount();
    int getRightEncoderCount();
    int getLeftRPM();
    int getRightRPM();
};