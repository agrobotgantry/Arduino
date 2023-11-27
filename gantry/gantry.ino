// ====================
// Author: Cas Damen & Jerome Kemper
// Created on: 27-11-2023
// Description: Arduino code to control the gantry of the Agrobot
// ====================

// Include libaries
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

// Include ROS serial to the code
#include <ros.h>
#include <std_msgs/Bool.h>

// Motor pins explanation:
// Puls pin: Each puls will rotate the motor by one step.
// Direction pin: Setting this pin to LOW or HIGH will change the direction the motor rotates.
// Enable pin: If set to LOW motor is activated. If set to HIGH motor is deactivated.

// Pins stepper motor x-axis
const int MX_pulsPin = 2;
const int MX_dirPin = 22;
const int MX_enPin = 23;

// Pins stepper motor y-axis
const int MY_pulsPin = 3;
const int MY_dirPin = 24;
const int MY_enPin = 25;

// Pins stepper motor z-axis
const int MZ_pulsPin = 4;
const int MZ_dirPin = 26;
const int MZ_enPin = 27;

// Pin servo gripper
const int servo_gripperPin = 42;

// Pins switches x-as
const int Switch_X1_NC = 30;
const int Switch_X1_NO = 31;
const int Switch_X2_NC = 32;
const int Switch_X2_NO = 33;

// Pins reed switches y-as
const int Reed_Y1 = 34;
const int Reed_Y2 = 35;

// Pins reed switches z-as
const int Reed_Z1 = 38;
const int Reed_Z2 = 39;

// Create stepper motor objects from AccelStepper library
AccelStepper motor_x(1,MX_pulsPin,MX_dirPin);
AccelStepper motor_y(1,MY_pulsPin,MY_dirPin);
AccelStepper motor_z(1,MZ_pulsPin,MZ_dirPin);

// Create gripper servo object
Servo servo_gripper;

// Create ROS node handle and include messages
ros::NodeHandle nh;
std_msgs::Bool bool_msg;

// Subscriber callback gripper
void gripper_callback(const std_msgs::Bool &message) {
  // Open or close gripper
  if(message.data == false) {
    gripper_open();
  } else if(message.data == true) {
    gripper_close();
  }
}

// Create ROS subsriber
ros::Subscriber<std_msgs::Bool> gripper_subscriber("agrobot_gantry/gripper", &gripper_callback);

void setup() {
  // Set baudraute for serial
  Serial.begin(57600);

  // Initialise servo gripper
  servo_gripper.attach(servo_gripperPin);

  // Initialise the gantry
  initialise();

  // Initialise ROS node and subscribe to topics
  nh.initNode();
  nh.subscribe(gripper_subscriber);
}

void loop() {
  //
  // Add code here
  //

  nh.spinOnce();
}

// Initialise the gantry
void initialise() {
  // Open the gripper
  gripper_open();

  //
  // Add code for initialising the gantry
  //
}

// Open the gripper
void gripper_open() {
  servo_gripper.write(0);
}

// Close the gripper
void gripper_close() {
  servo_gripper.write(20);
}
