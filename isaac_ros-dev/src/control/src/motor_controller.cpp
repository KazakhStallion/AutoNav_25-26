#include "motor_controller.hpp"
#include <rclcpp/rclcpp.hpp>

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
  char firstBuffer[41] = {};
  char readBuffer[41] = {};
  motorSerial.writeString(command.c_str());

  motorSerial.readString(firstBuffer, '\n', 40, 10);

  bool isEcho = false;
  bool isData = false;
  for (int i = 0; i < 40 && firstBuffer[i] != '\0'; i++) {
    if (firstBuffer[i] == '?') isEcho = true;
    if (firstBuffer[i] == '=') isData = true;
  }

  if (isData) {
    strcpy(readBuffer, firstBuffer);
  } else if (isEcho) {
    motorSerial.readString(readBuffer, '\n', 40, 10);
  } else {
    strcpy(readBuffer, firstBuffer);
  }

  RCLCPP_INFO(rclcpp::get_logger("control"), "RIGHT ENCODER RAW: %s", readBuffer);
  std::string encoderCount = "";
  bool equalSign = false;

  for (int i = 0; i < 40 && readBuffer[i] != '\0' && readBuffer[i] != '\r' && readBuffer[i] != '\n'; ++i) {
    if (!equalSign) {
      if (readBuffer[i] == '=') {
        equalSign = true;
      }
      continue;
    }

    if ((readBuffer[i] >= '0' && readBuffer[i] <= '9') || readBuffer[i] == '-') {
      encoderCount += readBuffer[i];
    } else if (!encoderCount.empty()) {
      // stop once the number is over
      break;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("control"), "RIGHT ENCODER PARSED: %s", encoderCount.c_str());
  #ifdef CONTROL_DEBUG
    RCLCPP_INFO(rclcpp::get_logger("control"), "Rec %s", encoderCount.c_str());
  #endif

  try {
    temp = std::stoi(encoderCount);
    prevRightEncoderCount = temp;
    return std::stoi(encoderCount);
  } catch (...) {
    return prevRightEncoderCount;
  }
}

int MotorController::getLeftEncoderCount(){
  std::string command = "?C 2\r";
  char firstBuffer[41] = {};
  char readBuffer[41] = {};
  motorSerial.writeString(command.c_str());

  motorSerial.readString(firstBuffer, '\n', 40, 10);

  bool isEcho = false;
  bool isData = false;
  for (int i = 0; i < 40 && firstBuffer[i] != '\0'; i++) {
    if (firstBuffer[i] == '?') isEcho = true;
    if (firstBuffer[i] == '=') isData = true;
  }

  if (isData) {
    strcpy(readBuffer, firstBuffer);
  } else if (isEcho) {
    motorSerial.readString(readBuffer, '\n', 40, 10);
  } else {
    strcpy(readBuffer, firstBuffer);
  }

  RCLCPP_INFO(rclcpp::get_logger("control"), "LEFT ENCODER RAW: %s", readBuffer);
  std::string encoderCount = "";
  bool equalSign = false;

  for (int i = 0; i < 40 && readBuffer[i] != '\0' && readBuffer[i] != '\r' && readBuffer[i] != '\n'; ++i) {
    if (!equalSign) {
      if (readBuffer[i] == '=') {
        equalSign = true;
      }
      continue;
    }

    if ((readBuffer[i] >= '0' && readBuffer[i] <= '9') || readBuffer[i] == '-') {
      encoderCount += readBuffer[i];
    } else if (!encoderCount.empty()) {
      // stop once the number is over
      break;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("control"), "LEFT ENCODER PARSED: %s", encoderCount.c_str());
  #ifdef CONTROL_DEBUG
    RCLCPP_INFO(rclcpp::get_logger("control"), "REC %s", encoderCount.c_str());
  #endif

  try {
    temp = std::stoi(encoderCount);
    prevLeftEncoderCount = temp;
    return std::stoi(encoderCount);
  } catch (...) {
    return prevLeftEncoderCount;
  }
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

