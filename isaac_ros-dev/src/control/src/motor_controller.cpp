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
  if (speed_mph < 0){
    return;
  }
  else{
    // Convert MPH to motor CMD using calibrated speedConverter
    int motorCmd = (int)speedConverter(speed_mph);
    int leftMotorSpeed = -1 * motorCmd;
    int rightMotorSpeed = motorCmd;

    std::string leftMotorCommand = "!G 1 " + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2 " + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
  }
}

// moves the robot in backwards
void MotorController::backward(){
  if (speed_mph < 0){
    return;
  }
  else{
    // Convert MPH to motor CMD using calibrated speedConverter
    int motorCmd = (int)speedConverter(speed_mph);
    int leftMotorSpeed = motorCmd;  // Reversed for backward
    int rightMotorSpeed = -1 * motorCmd;

    std::string leftMotorCommand = "!G 1 " + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2 " + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
  }
}

// moves the robot left
void MotorController::turnLeft(){
  if (speed_mph < 0){
    return;
  }
  else{
    // For turning, use a fraction of the forward speed
    float turn_speed_mph = speed_mph * 0.5;  // 50% of forward speed for turns
    int motorCmd = (int)speedConverter(turn_speed_mph);
    int leftMotorSpeed = motorCmd;
    int rightMotorSpeed = motorCmd;

    std::string leftMotorCommand = "!G 1 " + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2 " + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
  }
}

// moves the robot right
void MotorController::turnRight(){
  if (speed_mph < 0){
    return;
  }
  else{
    // For turning, use a fraction of the forward speed
    float turn_speed_mph = speed_mph * 0.5;  // 50% of forward speed for turns
    int motorCmd = (int)speedConverter(turn_speed_mph);
    int leftMotorSpeed = -1 * motorCmd;
    int rightMotorSpeed = -1 * motorCmd;

    std::string leftMotorCommand = "!G 1 " + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2 " + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
  }
}

// moves the robot at specific motor speeds (input in MPH)
void MotorController::move(float right_speed_mph, float left_speed_mph){
    // Convert each wheel's MPH to motor CMD using calibrated speedConverter
    int leftMotorCmd = (int)speedConverter(left_speed_mph);
    int rightMotorCmd = (int)speedConverter(right_speed_mph);
    
    int leftMotorSpeed = -leftMotorCmd;
    int rightMotorSpeed = rightMotorCmd;

    std::string leftMotorCommand = "!G 1 " + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2 " + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
}

// moves the robot with raw CMD values (bypasses speed calibration)
void MotorController::moveRaw(int right_cmd, int left_cmd){
    // Send raw CMD values directly to motors without speed conversion
    // Used for calibration tests (t007) to collect empirical speed data
    std::string leftMotorCommand = "!G 1 " + std::to_string(-left_cmd) + "\r";
    std::string rightMotorCommand = "!G 2 " + std::to_string(right_cmd) + "\r";

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

// updates the desired speed in MPH
void MotorController::setSpeedMPH(float mph){
  speed_mph = mph;
}

// gets the current speed in MPH
float MotorController::getSpeedMPH(){
  return speed_mph;
}

// increases speed by the increment size
void MotorController::increaseSpeed(){
  speed_mph += speed_increment;
  // Cap at max speed of 5 MPH
  if (speed_mph > 5.0) {
    speed_mph = 5.0;
  }
}

// decreases speed by the increment size
void MotorController::decreaseSpeed(){
  speed_mph -= speed_increment;
  // Don't go below 0
  if (speed_mph < 0.0) {
    speed_mph = 0.0;
  }
}

// Speed calibration function:
float speedConverter(float speed_CONTROL){
  // Purpose of this function:
  // Provide conversion from,
  // speedCONTROL [mph] -> ConvertedSpeedCMD [-]
  // that is backed by empirical results.

  // Setup constants for empirical speed conversion:
  // cmd = A0*v^2 + A1*v + A2
  float A0 = 0;
  float A1 = 1;
  float A2 = 0;

  float Kcal = 1;

  float v = speed_CONTROL * Kcal;
  float converted_speed_CMD = A0 * v * v + A1 * v + A2;
  return converted_speed_CMD;
}

int MotorController::getRightEncoderCount(){
  std::string command = "?C 1\r";
  char readBuffer[41] = {};
  motorSerial.writeString(command.c_str());  
  motorSerial.readString(readBuffer, '\n', 40, 10);

  //std::cout << readBuffer << std::endl;
  std::string encoderCount = "";
  bool equalSign = false;
  for (int i = 0; i < 40; i++) {
      //std::cout << readBuffer[i] << std::endl;

      if ((readBuffer[i] >= '0' && readBuffer[i] <= '9' && equalSign) || (readBuffer[i] == '-' && equalSign)) {
        encoderCount += readBuffer[i];
      }
      if (readBuffer[i] == 61) {
          equalSign = true;
      }
  }

  
    #ifdef CONTROL_DEBUG
    RCLCPP_INFO(rclcpp::get_logger("control"), "REC  %s", encoderCount.c_str());
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
  char readBuffer[41] = {};
  motorSerial.writeString(command.c_str());  

  motorSerial.readString(readBuffer, '\n', 40, 10);

  std::string encoderCount = "";
  bool equalSign = false;
  for (int i = 0; i < 40; i++) {
      //std::cout << readBuffer[i] << std::endl;

      if ((readBuffer[i] >= '0' && readBuffer[i] <= '9' && equalSign) || (readBuffer[i] == '-' && equalSign)) {
        encoderCount += readBuffer[i];
      }
      if (readBuffer[i] == 61) {
          equalSign = true;
      }
  }

    #ifdef CONTROL_DEBUG
    RCLCPP_INFO(rclcpp::get_logger("control"), "LEC  %s", encoderCount.c_str());
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

