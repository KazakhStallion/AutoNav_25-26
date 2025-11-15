#include "motor_controller.hpp"
#include <rclcpp/rclcpp.hpp>

// Fixing the encoder reading with regex and climits
#include <regex>
#include <climits>

// constructor
MotorController::MotorController(){
  
}

char MotorController::configure(const char * port){

   char errorOpening = motorSerial.openDevice(port, 115200);
   return errorOpening;
  //motorSerial.write("!MG\r");
  if (errorOpening!=1){
    printf ("Unsuccessful connection to %s\n",port);
  }
  else{
    printf ("Successful connection to %s\n",port);
  }

  std::string rightMotorCommand = "!C 1 0\r";
  std::string leftMotorCommand = "!C 2 0 \r";
  motorSerial.writeString(leftMotorCommand.c_str());
  motorSerial.writeString(rightMotorCommand.c_str());
  
}

// moves the robot forward
void MotorController::forward(){
  if (speed < 0){
    return;
  }
  else{
    int leftMotorSpeed = -1 * (int)(stepSize * speed);
    int rightMotorSpeed = (int)(stepSize * speed);

    std::string leftMotorCommand = "!G 1 " + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2 " + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
  }
}

// moves the robot in backwards
void MotorController::backward(){
  if (speed < 0){
    return;
  }
  else{
    int leftMotorSpeed = -1 * (int)(stepSize * speed);
    int rightMotorSpeed = (int)(stepSize * speed);

    std::string leftMotorCommand = "!G 1 " + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2 " + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
  }
}

// moves the robot left
void MotorController::turnLeft(){
  if (speed < 0){
    return;
  }
  else{
    int leftMotorSpeed = (int)(stepSize * left_turn_speeds.first);
    int rightMotorSpeed = (int)(stepSize * left_turn_speeds.second);

    std::string leftMotorCommand = "!G 1 " + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2 " + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
  }
}

// moves the robot right
void MotorController::turnRight(){
  if (speed < 0){
    return;
  }
  else{
    int leftMotorSpeed = (int)(stepSize * right_turn_speeds.first);
    int rightMotorSpeed = (int)(stepSize * right_turn_speeds.second);

    std::string leftMotorCommand = "!G 1 " + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2 " + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
  }
}

// moves the robot at specific motor speeds
void MotorController::move(float right_speed, float left_speed){
  
    int leftMotorSpeed = (int)(-stepSize * left_speed);
    int rightMotorSpeed = (int)(stepSize * right_speed);

    std::string leftMotorCommand = "!G 1 " + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2 " + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
}

// stops the robot from moving
void MotorController::stop(){
  std::string leftMotorCommand = "!G 1 0\r";
  std::string rightMotorCommand = "!G 2 0 \r";
  motorSerial.writeString(leftMotorCommand.c_str());
  motorSerial.writeString(rightMotorCommand.c_str());
}

// ends the serial connection
void MotorController::shutdown(){
  std::string command = "!EX\r";
  motorSerial.writeString(command.c_str());
  std::this_thread::sleep_for(std::chrono::seconds(1));
  motorSerial.closeDevice();
}

// updates the step size
void MotorController::setStepSize(int size){
  stepSize = size;
}

// gets the step size
int MotorController::getStepSize(){
  return stepSize;
}

// updates the speed
void MotorController::setSpeed(int s){
  speed = s;
}

// gets the speed
int MotorController::getSpeed(){
  return speed;
}

int MotorController::getRightEncoderCount(){
  std::string command = "?C 1\r";
  char readBuffer[41] = {};
  motorSerial.writeString(command.c_str());
  motorSerial.readString(readBuffer, '\n', 40, 10);

  // DEBUG: Print raw buffer
  RCLCPP_INFO(rclcpp::get_logger("control"), "RIGHT ENC RAW: [%s]", readBuffer);

  // Convert to string and, if we see echo of the command, try one quick extra read
  std::string s(readBuffer);
  if (s.find("?C") != std::string::npos || s.find("!G") != std::string::npos || s.find("!C") != std::string::npos) {
    char readBuffer2[41] = {};
    motorSerial.readString(readBuffer2, '\n', 40, 10);
    RCLCPP_INFO(rclcpp::get_logger("control"), "RIGHT ENC EXTRA RAW: [%s]", readBuffer2);
    s = std::string(readBuffer2);
  }

  // Extract first signed integer robustly
  std::smatch m;
  std::regex re(R"((-?\d+))");
  if (std::regex_search(s, m, re)) {
    try {
      long long valLL = std::stoll(m.str(1));
      // clamp to 32-bit signed range before storing/returning
      int val = (valLL > INT_MAX) ? INT_MAX : (valLL < INT_MIN) ? INT_MIN : static_cast<int>(valLL);
      prevRightEncoderCount = val;
      RCLCPP_INFO(rclcpp::get_logger("control"), "RIGHT ENC PARSED: [%d]", val);
      return val;
    } catch (...) {
      // fall through to return previous
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("control"), "RIGHT ENC PARSED: [%d] (fallback)", prevRightEncoderCount);
  return prevRightEncoderCount;
}

int MotorController::getLeftEncoderCount(){
  std::string command = "?C 2\r";
  char readBuffer[41] = {};
  motorSerial.writeString(command.c_str());
  motorSerial.readString(readBuffer, '\n', 40, 10);

  // DEBUG: Print raw buffer
  RCLCPP_INFO(rclcpp::get_logger("control"), "LEFT ENC RAW: [%s]", readBuffer);

  std::string s(readBuffer);
  if (s.find("?C") != std::string::npos || s.find("!G") != std::string::npos || s.find("!C") != std::string::npos) {
    char readBuffer2[41] = {};
    motorSerial.readString(readBuffer2, '\n', 40, 10);
    RCLCPP_INFO(rclcpp::get_logger("control"), "LEFT ENC EXTRA RAW: [%s]", readBuffer2);
    s = std::string(readBuffer2);
  }

  std::smatch m;
  std::regex re(R"((-?\d+))");
  if (std::regex_search(s, m, re)) {
    try {
      long long valLL = std::stoll(m.str(1));
      int val = (valLL > INT_MAX) ? INT_MAX : (valLL < INT_MIN) ? INT_MIN : static_cast<int>(valLL);
      prevLeftEncoderCount = val;
      RCLCPP_INFO(rclcpp::get_logger("control"), "LEFT ENC PARSED: [%d]", val);
      return val;
    } catch (...) {
      // fall through
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("control"), "LEFT ENC PARSED: [%d] (fallback)", prevLeftEncoderCount);
  return prevLeftEncoderCount;
}

int MotorController::getRightRPM(){
  std::string command = "?BS 1\r";
  char readBuffer[41] = {};
  motorSerial.writeString(command.c_str());  

  motorSerial.readString(readBuffer, '\n', 15, 20);

  std::cout << readBuffer << std::endl;
  std::string rpm = "";
    bool equalSign = false;
    for (int i = 0; i < 40; i++) {
        //std::cout << readBuffer[i] << std::endl;

        if ((readBuffer[i] >= '0' && readBuffer[i] <= '9' && equalSign) || (readBuffer[i] == '-' && equalSign)) {
          rpm += readBuffer[i];
        }
        if (readBuffer[i] == 61) {
            equalSign = true;
        }
    }

  return 6;
  //return 5;
}

int MotorController::getLeftRPM(){
  std::string command = "?BS 2\r";
  char readBuffer[41] = {};
  motorSerial.writeString(command.c_str());  

  motorSerial.readString(readBuffer, '\n', 15, 20);


  std::string rpm = "";
    bool equalSign = false;
    for (int i = 0; i < 40; i++) {
        //std::cout << readBuffer[i] << std::endl;

        if ((readBuffer[i] >= '0' && readBuffer[i] <= '9' && equalSign) || (readBuffer[i] == '-' && equalSign)) {
          rpm += readBuffer[i];
        }
        if (readBuffer[i] == 61) {
            equalSign = true;
        }
    }
    //std::cout <<"RIGHT RPM: " << rpm << std::endl;
    //RCLCPP_INFO(rclcpp::get_logger("MotorController"), "RIGHT RPM: %s", rpm.c_str());
    return 6;
    //return 5;
}

