// ====================
// Author: Cas Damen
// Created on: 17-11-2023
// Description: Arduino code to control the steering of the Agrobot Gantry
// ====================

// Include AccelStepper
#include <AccelStepper.h>
#include <MultiStepper.h>

// Include ROS serial to the code
#include <ros.h>
#include <std_msgs/Int8.h>

// Pins IR sensors
const int S1_Pin = 30;
const int S2_Pin = 31;
const int S3_Pin = 32;
const int S4_Pin = 33;

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
const int motor_default_speed = 50;
int current_state = 0;

// Create ROS node handle and include messages
ros::NodeHandle nh;
std_msgs::Int8 int8_msg;

// Subscriber callback arduino command. This is the action the robot has to execute
void arduino_command_callback(const std_msgs::Int8 &message) {
  current_state = message.data;
}

// Create ROS subsriber and publisher
ros::Subscriber<std_msgs::Int8> arduino_cmd_subscriber("agrobot_steering/arduino_command", &arduino_command_callback);
ros::Publisher arduino_state_publisher("agrobot_steering/arduino_state", &int8_msg);

void setup() {
  // Set baudraute for serial
  Serial.begin(57600);

  // Initialise ROS node, subscribe and initialise publisher to topics
  nh.initNode();
  nh.subscribe(arduino_cmd_subscriber);
  nh.advertise(arduino_state_publisher);

  // Initialise IR sensors
  pinMode(S1_Pin, INPUT);
  pinMode(S2_Pin, INPUT);
  pinMode(S3_Pin, INPUT);
  pinMode(S4_Pin, INPUT);
}

void loop() {
  // Control the motors
  if(current_state == 0) {
    // Idle - Wait for a command from ROS
    publish_state(arduino_state_publisher, 0);
  } else if(current_state == 1) {
    // Initialise the Agrobot Gantry
    publish_state(arduino_state_publisher, 1);
    initialise_wheels();
    current_state = 0;
  } else if(current_state == 2) {
    // Turn wheels to straight position
    publish_state(arduino_state_publisher, 2);
    turn_wheels_straight();
    current_state = 0;
  } else if(current_state == 3) {
    publish_state(arduino_state_publisher, 3);
    // Turn the wheels to the turn position
    turn_wheels_turn();
    current_state = 0;
  }
  
  nh.spinOnce();
}

// Initialise the wheels
void initialise_wheels() {
  bool motor_1_state = false;
  bool motor_2_state = false;
  bool motor_3_state = false;
  bool motor_4_state = false;

  // Turn the four wheels to the turn position
  while(not motor_1_state and not motor_2_state and not motor_3_state and not motor_4_state) {
    motor_1_state = turn_one_wheel(motor_1, S1_Pin, 1);
    motor_2_state = turn_one_wheel(motor_2, S2_Pin, 0);
    motor_3_state = turn_one_wheel(motor_3, S3_Pin, 1);
    motor_4_state = turn_one_wheel(motor_4, S4_Pin, 0);
  }

  // Reset states to false
  motor_1_state = false;
  motor_2_state = false;
  motor_3_state = false;
  motor_4_state = false;

  // Turn the four wheels back to the straight position
  while(not motor_1_state and not motor_2_state and not motor_3_state and not motor_4_state) {
    motor_1_state = turn_one_wheel(motor_1, S1_Pin, 0);
    motor_2_state = turn_one_wheel(motor_2, S2_Pin, 1);
    motor_3_state = turn_one_wheel(motor_3, S3_Pin, 0);
    motor_4_state = turn_one_wheel(motor_4, S4_Pin, 1);
  }

  // Set the current position as the 0 position
  motor_1.setCurrentPosition(motor_1.currentPosition());
  motor_2.setCurrentPosition(motor_2.currentPosition());
  motor_3.setCurrentPosition(motor_3.currentPosition());
  motor_4.setCurrentPosition(motor_4.currentPosition());
}

// Turn the wheels to the straight position
void turn_wheels_straight() {
  bool motor_1_state = false;
  bool motor_2_state = false;
  bool motor_3_state = false;
  bool motor_4_state = false;

  // Turn the four wheels to the correct position
  while(not motor_1_state and not motor_2_state and not motor_3_state and not motor_4_state) {
    motor_1_state = turn_one_wheel(motor_1, S1_Pin, 0);
    motor_2_state = turn_one_wheel(motor_2, S2_Pin, 1);
    motor_3_state = turn_one_wheel(motor_3, S3_Pin, 0);
    motor_4_state = turn_one_wheel(motor_4, S4_Pin, 1);
  }
}

// Turn the wheels to the turn position
void turn_wheels_turn() {
  bool motor_1_state = false;
  bool motor_2_state = false;
  bool motor_3_state = false;
  bool motor_4_state = false;

  // Turn the four wheels to the correct position
  while(not motor_1_state and not motor_2_state and not motor_3_state and not motor_4_state) {
    motor_1_state = turn_one_wheel(motor_1, S1_Pin, 1);
    motor_2_state = turn_one_wheel(motor_2, S2_Pin, 0);
    motor_3_state = turn_one_wheel(motor_3, S3_Pin, 1);
    motor_4_state = turn_one_wheel(motor_4, S4_Pin, 0);
  }
}

// Turn one wheel to the correct position
bool turn_one_wheel(AccelStepper &motor, int IR_sensor, bool motor_direction) {
  int motor_speed = motor_default_speed;
  bool state = false;
  
  // Set the speed of the motor according to the direction
  if(not motor_direction) {
    motor_speed = -motor_default_speed;
  }

  // Turn the wheel
  if(motor.currentPosition() > -5 and motor_1.currentPosition() < 5) {
    // Ignore the IR sensor in the first few steps
    motor.runSpeed();
  } else if(not digitalRead(IR_sensor)) {
    // Run the motor untill the sensor sends a signal the position is reached
    motor.runSpeed();
  } else {
    // Position is reached
    state = true;
  }

  return state;
}

// Funtion to publish the state value to the Arduino
void publish_state(ros::Publisher publisher, int state) {
  int8_msg.data = state;
  publisher.publish(&int8_msg);
}
