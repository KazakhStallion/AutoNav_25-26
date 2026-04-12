
#include "xbox.hpp"

Xbox::Xbox() 
  : b_button_state(false), x_button_state(false), y_button_state(false),
    right_bumper_state(false), left_bumper_state(false),
    left_stick_x_pos(0.0f), left_stick_y_pos(0.0f), 
    right_stick_x_pos(0.0f), right_stick_y_pos(0.0f){

}

void Xbox::set_b(bool state){
  b_button_state = state;
}

void Xbox::set_x(bool state){
  x_button_state = state;
}

void Xbox::set_y(bool state){
  y_button_state = state;
}

void Xbox::set_right_bumper(bool state){
  right_bumper_state = state;
}

void Xbox::set_left_bumper(bool state){
  left_bumper_state = state;
}

void Xbox::set_left_stick_x(float pos) {
  left_stick_x_pos = pos;
}

void Xbox::set_left_stick_y(float pos) {
  left_stick_y_pos = pos;
}

void Xbox::set_right_stick_x(float pos) {
  right_stick_x_pos = pos;
}

void Xbox::set_right_stick_y(float pos) {
  right_stick_y_pos = pos;
}

void Xbox::adjust_joysticks(){
  if(left_stick_y_pos < 0.1 && left_stick_y_pos > -0.1){
    left_stick_y_pos = 0;
  }

  if(right_stick_y_pos < 0.1 && right_stick_y_pos > -0.1){
    right_stick_y_pos = 0;
  }

  if(left_stick_x_pos < 0.1 && left_stick_x_pos > -0.1){
    left_stick_x_pos = 0;
  }

  if(right_stick_x_pos < 0.1 && right_stick_x_pos > -0.1){
    right_stick_x_pos = 0;
  }
}

Xbox::CommandData Xbox::calculateCommand(){
  CommandData cmd;
  if(b_button_state){
    cmd.cmd = STOP;
    cmd.left_motor_speed = 0.0f;
    cmd.right_motor_speed = 0.0f;
    return cmd;
  }
  else if(left_bumper_state){
    cmd.cmd = SPEED_UP;
    cmd.left_motor_speed = 0.0f;
    cmd.right_motor_speed = 0.0f;
    return cmd;
  }
  else if(right_bumper_state){
    cmd.cmd = SPEED_DOWN;
    cmd.left_motor_speed = 0.0f;
    cmd.right_motor_speed = 0.0f;
    return cmd;
  }

  adjust_joysticks();
  if(left_stick_y_pos == 0 && right_stick_y_pos == 0){
    cmd.cmd = NONE;
    return cmd;
  }
  if(TANKDRIVE){
    cmd.cmd = MOVE;

    cmd.right_motor_speed = -right_stick_y_pos;
    cmd.left_motor_speed = -left_stick_y_pos;

    return cmd;
  }
  else{
    //fucking trig

    float magnitude = sqrt((left_stick_x_pos * left_stick_x_pos) + (left_stick_y_pos * left_stick_y_pos));

    // calculate the direction (angle in radians) of the joystick input
    float angle = atan2(left_stick_y_pos, left_stick_x_pos); 

    cmd.cmd = MOVE;

    // cosine and sine of the angle to split the movement into two components:
    cmd.left_motor_speed = magnitude * sin(angle + M_PI / 4);  // Adjust for robot orientation
    cmd.right_motor_speed = magnitude * cos(angle + M_PI / 4); // Adjust for robot orientation 
    return cmd;
  }
}

bool Xbox::switchMode(){
  if(x_button_state){
    return true;
  }
  return false;
}
