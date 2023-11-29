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
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>

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
std_msgs::Empty empty_msg;
geometry_msgs::Point point_msg;

// Initialise the gantry
void arduino_initialise_callback() {
  initialise_x_axis()
  initialise_y_axis()
  initialise_z_axis()
  gripper_open();

  // Set current positions of motors as zero
  motor_x.setCurrentPosition();
  motor_y.setCurrentPosition();
  motor_z.setCurrentPosition();
}

// Subscriber callback coordinates
void arduino_coordinates_callback(const geometry_msgs::Point &message) {
  float position_x = message.x;
  float position_y = message.y;
  float position_z = message.z;

  //
  //
  // Plaats hier de code die de gantry moet uitvoeren nadat de coordinaten zijn ontvangen
  //
  //
}

// Create ROS subsribers
ros::Subscriber<std_msgs::Empty> initialise_subscriber("agrobot_gantry/initialise", &arduino_initialise_callback);
ros::Subscriber<geometry_msgs::Point> coordinates_subscriber("agrobot_gantry/coordinates", &arduino_coordinates_callback);

void setup() {
  // Set baudraute for serial
  Serial.begin(57600);

  // Initialise servo gripper
  servo_gripper.attach(servo_gripperPin);

  // Initialise normal and reed switches
  pinMode(Switch_X1_NC, INPUT);
  pinMode(Switch_X1_NO, INPUT);
  pinMode(Switch_X2_NC, INPUT);
  pinMode(Switch_X2_NO, INPUT);
  
  pinMode(Reed_Y1, INPUT);
  pinMode(Reed_Y2, INPUT);
  pinMode(Reed_Z1, INPUT);
  pinMode(Reed_Z2, INPUT);

  // Set parameters for motors
  /*motor_x.setSpeed(1000);
  motor_y.setSpeed(1000);
  motor_z.setSpeed(1000);*/
  motor_x.setAcceleration(500);
  motor_y.setAcceleration(500);
  motor_z.setAcceleration(500);

  // Initialise ROS node and subscribe to topics
  nh.initNode();
  nh.subscribe(initialise_subscriber);
  nh.subscribe(coordinates_subscriber);
}

void loop() {
  //
  // Add code here
  //

  nh.spinOnce();
}

// Initialise the x-axis
void initialise_x_axis() {
  //
  //
  //
}

// Initialise the y-axis
void initialise_y_axis() {
  //
  //
  //
}

// Initialise the z-axis
void initialise_z_axis() {
  // Move the axis up as long as the reed switch is not triggered
  if(digitalRead(Reed_Z1) == HIGH) {
    motor_z.setSpeed(200);
    motor_z.runSpeed();
  }
}

// Move the z-axis to the up position
void z_axis_up() {
  // Check if reed switch is reached
  if(digitalRead(Reed_Z1) == HIGH) {
    motor_z.runToNewPosition(0);
  }
}

// Move the z-axis to the down position
void z_axis_down() {
  // Check if reed switch is reached
  if(digitalRead(Reed_Z2) == HIGH) {
    motor_z.runToNewPosition(400);
  }
}

// Open the gripper
void gripper_open() {
  servo_gripper.write(180);
}

// Close the gripper
void gripper_close() {
  servo_gripper.write(0);
}
