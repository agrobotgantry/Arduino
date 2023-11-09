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
#include <std_msgs/Bool.h>

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
// NOTE: a motor only runs when the runSpeed() function is called. Because of that, variables are used to indicate if a motor should be runnning or not
const int motor_default_speed = 200;
bool motor_1_active = false;
bool motor_2_active = false;
bool motor_3_active = false;
bool motor_4_active = false;
int motor_1_speed = 0;
int motor_2_speed = 0;
int motor_3_speed = 0;
int motor_4_speed = 0;
bool object_detected = false;

// Create ROS node handle and include messages
ros::NodeHandle nh;
std_msgs::Int8 int8_msg;
std_msgs::Bool bool_msg;


// Subscriber callback stepper motor 1. This is for testing the individual motor
void motor_1_callback(const std_msgs::Int8 &message) {
  // Control the motor
  motor_control(1,message.data);
}


// Subscriber callback stepper motor 2. This is for testing the individual motor
void motor_2_callback(const std_msgs::Int8 &message) {
  // Control the motor
  motor_control(2,message.data);
}


// Subscriber callback stepper motor 3. This is for testing the individual motor
void motor_3_callback(const std_msgs::Int8 &message) {
  // Control the motor
  motor_control(3,message.data);
}


// Subscriber callback stepper motor 4. This is for testing the individual motor
void motor_4_callback(const std_msgs::Int8 &message) {
  // Control the motor
  motor_control(4,message.data);
}


// Subscriber callback object detected
void object_detected_callback(const std_msgs::Bool &message) {
  object_detected = message.data;
}


// Subscriber callback arduino command. This is the action the robot has to execute
void arduino_command_callback(const std_msgs::Int8 &message) {
  // Select the correct state the robot should execute
  if(message.data == 0) {
    // Stop the Agrobot Gantry
    agrobot_stop();
  } else if(message.data == 1) {
    // Drive the Agrobot Gantry forward
    agrobot_drive_forward();
  } else if(message.data == 2) {
    // Drive the Agrobot Gant;ry backward
    agrobot_drive_backward();
  } else if(message.data == 3) {
    // Turn the Agrobot Gantry left
    agrobot_turn_left();
  } else if(message.data == 4) {
    // Turn the Agrobot Gantry right
    agrobot_turn_right();
  }
}


// Create ROS subsribers
ros::Subscriber<std_msgs::Int8> M1_subscriber("agrobot_drive/motor_1", &motor_1_callback);
ros::Subscriber<std_msgs::Int8> M2_subscriber("agrobot_drive/motor_2", &motor_2_callback);
ros::Subscriber<std_msgs::Int8> M3_subscriber("agrobot_drive/motor_3", &motor_3_callback);
ros::Subscriber<std_msgs::Int8> M4_subscriber("agrobot_drive/motor_4", &motor_4_callback);
ros::Subscriber<std_msgs::Int8> object_detected_subscriber("agrobot_object_detection/object_detected", &object_detected_callback);
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
  nh.subscribe(object_detected_subscriber);
  nh.subscribe(arduino_cmd_subscriber);

  // Set parameters stepper motors
  motor_1.setMaxSpeed(1000);
  motor_2.setMaxSpeed(1000);
  motor_3.setMaxSpeed(1000);
  motor_4.setMaxSpeed(1000);
}


void loop() {
  // Run the motors
  run_motors();

  //
  // Add if statement to check if object_detected is false before running the motors
  //

  nh.spinOnce();
}


// Function to turn the stepper motor on. The direction should be positive (HIGH = 1) or negative (LOW = 0)
void motor_on(int motor_number, int motor_direction){
  // Select the correct motor to turn on
  if(motor_number == 1 && motor_direction == 0) {
    // Motor front left - turn in negative direction
    motor_1_active = true;
    motor_1_speed = -motor_default_speed;
  } else if(motor_number == 1 && motor_direction == 1) {
    // Motor front left - turn in positive direction
    motor_1_active = true;
    motor_1_speed = motor_default_speed;
  } else if(motor_number == 2 && motor_direction == 0) {
    // Motor front right - turn in negative direction
    motor_2_active = true;
    motor_2_speed = -motor_default_speed;
  } else if(motor_number == 2 && motor_direction == 1) {
    // Motor front right - turn in positive direction
    motor_2_active = true;
    motor_2_speed = motor_default_speed;
  } else if (motor_number == 3 && motor_direction == 0) {
    // Motor back right - turn in negative direction
    motor_3_active = true;
    motor_3_speed = -motor_default_speed;
  } else if (motor_number == 3 && motor_direction == 1) {
    // Motor back right - turn in positive direction
    motor_3_active = true;
    motor_3_speed = motor_default_speed;
  } else if(motor_number == 4 && motor_direction == 0) {
    // Motor back left - turn in negative direction
    motor_4_active = true;
    motor_4_speed = -motor_default_speed;
  } else if(motor_number == 4 && motor_direction == 1) {
    // Motor back left - turn in positive direction
    motor_4_active = true;
    motor_4_speed = motor_default_speed;
  }
}


// Function to turn the stepper motor off
void motor_off(int motor_number){
  // Select the correct motor to turn off
  if(motor_number == 1) {
    // Motor front left
    motor_1_active = false;
    motor_1_speed = 0;
  } else if(motor_number == 2) {
    // Motor front right
    motor_2_active = false;
    motor_2_speed = 0;
  } else if(motor_number == 3) {
    // Motor back right
    motor_3_active = false;
    motor_3_speed = 0;
  } else if(motor_number == 4) {
    // Motor back left
    motor_4_active = false;
    motor_4_speed = 0;
  }
}


// Function to control the motor after a callback from the subscriber
void motor_control(int motor_number, int motor_state) {
  // Select the correct action for the stepper motor
  if(motor_state == 0) {
    // Turn motor off
    motor_off(motor_number);
  } else if(motor_state == 1) {
    // Turn motor on in positive direction
    motor_on(motor_number,1);
  } else if(motor_state == 2) {
    // Turn motor on in negative direction
    motor_on(motor_number,0);
  }
}


// Run the motors
void run_motors() {
  // Run motor 1
  if (motor_1_active == true) {
    motor_1.setSpeed(motor_1_speed);
    motor_1.runSpeed();
  }
  // Run motor 2
  if (motor_2_active == true) {
    motor_2.setSpeed(motor_2_speed);
    motor_2.runSpeed();
  }
  // Run motor 3
  if (motor_3_active == true) {
    motor_3.setSpeed(motor_3_speed);
    motor_3.runSpeed();
  }
  // Run motor 4
  if (motor_4_active == true) {
    motor_4.setSpeed(motor_4_speed);
    motor_4.runSpeed();
  }
}


// Stop the Agrobot Gantry
void agrobot_stop() {
  // Turn all four motors off
  motor_off(1);
  motor_off(2);
  motor_off(3);
  motor_off(4);
}


// Drive the Agrobot Gantry forward
void agrobot_drive_forward() {
  // Turn the four motors on, all in the same direction
  motor_on(1,1);
  motor_on(2,1);
  motor_on(3,1);
  motor_on(4,1);
}


// Drive the Agrobot Gantry backward
void agrobot_drive_backward() {
  // Turn the four motors on, all in the same direction
  motor_on(1,0);
  motor_on(2,0);
  motor_on(3,0);
  motor_on(4,0);
}


// Turn the Agrobot Gantry to the left
void agrobot_turn_left() {
  // Turn the right motors on in the positive direction and the left motors in the negative direction
  motor_on(1,0);
  motor_on(2,1);
  motor_on(3,1);
  motor_on(4,0);
}


// Turn the Agrobot Gantry to the right
void agrobot_turn_right() {
  // Turn the left motors on in the positive direction and the right motors in the positive direction
  motor_on(1,1);
  motor_on(2,0);
  motor_on(3,0);
  motor_on(4,1);
}
