// ====================
// Author: Cas Damen
// Created on: 31-10-2023
// Description: Arduino code to control the drive of the Agrobot Gantry
// ====================

// Include AccelStepper
#include <AccelStepper.h>
#include <MultiStepper.h>

// Include ROS serial to the code
#include <ros.h>
#include <std_msgs/Int8.h>

// Motor pins explanation:
// Puls pin: Each puls will rotate the motor by one step.
// Direction pin: Setting this pin to LOW or HIGH will change the direction the motor rotates.
// Enable pin: If set to LOW motor is activated. If set to HIGH motor is deactivated.

// Pins stepper motor 1 (Front left)
const int M1_pulsPin = 9;
const int M1_dirPin = 22;
const int M1_enPin = 23;

// Pins stepper motor 2 (Front right)
const int M2_pulsPin = 10;
const int M2_dirPin = 24;
const int M2_enPin = 25;

// Pins stepper motor 3 (Back right)
const int M3_pulsPin = 11;
const int M3_dirPin = 26;
const int M3_enPin = 27;

// Pins stepper motor 4 (Back left)
const int M4_pulsPin = 12;
const int M4_dirPin = 28;
const int M4_enPin = 29;

// Create stepper motor objects from AccelStepper library
AccelStepper motor_1(1,M1_pulsPin,M1_dirPin);
AccelStepper motor_2(1,M2_pulsPin,M2_dirPin);
AccelStepper motor_3(1,M3_pulsPin,M3_dirPin);
AccelStepper motor_4(1,M4_pulsPin,M4_dirPin);

// Initialise global variables for controlling the motor
const int motor_default_speed = 200;
int current_state = 0;

// Create ROS node handle and include messages
ros::NodeHandle nh;
std_msgs::Int8 int8_msg;

// Subscriber callback arduino command. This is the action the robot has to execute
void arduino_command_callback(const std_msgs::Int8 &message) {
  current_state = message.data;
}

// Create ROS subsriber
ros::Subscriber<std_msgs::Int8> arduino_cmd_subscriber("agrobot_drive/arduino_command", &arduino_command_callback);

void setup() {
  // Set baudraute for serial
  Serial.begin(57600);

  // Initialise ROS node and subscribe to topics
  nh.initNode();
  nh.subscribe(M1_subscriber);
  nh.subscribe(M2_subscriber);
  nh.subscribe(M3_subscriber);
  nh.subscribe(M4_subscriber);
  nh.subscribe(arduino_cmd_subscriber);

  // Set parameters stepper motors
  motor_1.setMaxSpeed(1000);
  motor_2.setMaxSpeed(1000);
  motor_3.setMaxSpeed(1000);
  motor_4.setMaxSpeed(1000);
}

void loop() {
  // State 0 is the idle state where the motors are turned off and the Arduino waits for a ROS command
  if(current_state == 1) {
    // Drive the Agrobot Gantry forward
    agrobot_drive_forward();
  } else if(current_state == 2) {
    // Drive the Agrobot Gant;ry backward
    agrobot_drive_backward();
  } else if(current_state == 3) {
    // Turn the Agrobot Gantry left
    agrobot_turn_left();
  } else if(current_state == 4) {
    // Turn the Agrobot Gantry right
    agrobot_turn_right();
  }
  
  nh.spinOnce();
}

// Drive the Agrobot Gantry forward
void agrobot_drive_forward() {
  // Turn the four motors on, all in the same direction
  motor_1.setSpeed(motor_default_speed);
  motor_2.setSpeed(motor_default_speed);
  motor_3.setSpeed(motor_default_speed);
  motor_4.setSpeed(motor_default_speed);

  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

// Drive the Agrobot Gantry backward
void agrobot_drive_backward() {
  // Turn the four motors on, all in the same direction
  motor_1.setSpeed(-motor_default_speed);
  motor_2.setSpeed(-motor_default_speed);
  motor_3.setSpeed(-motor_default_speed);
  motor_4.setSpeed(-motor_default_speed);

  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

// Turn the Agrobot Gantry to the left
void agrobot_turn_left() {
  // Turn the right motors on in the positive direction and the left motors in the negative direction
  motor_1.setSpeed(-motor_default_speed);
  motor_2.setSpeed(motor_default_speed);
  motor_3.setSpeed(motor_default_speed);
  motor_4.setSpeed(-motor_default_speed);

  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

// Turn the Agrobot Gantry to the right
void agrobot_turn_right() {
  // Turn the left motors on in the positive direction and the right motors in the positive direction
  motor_1.setSpeed(motor_default_speed);
  motor_2.setSpeed(-motor_default_speed);
  motor_3.setSpeed(-motor_default_speed);
  motor_4.setSpeed(motor_default_speed);

  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}
