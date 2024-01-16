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
const int S2_Pin = 32;
const int S3_Pin = 25;
const int S4_Pin = 33;

// Motor pins explanation:
// Puls pin: Each puls will rotate the motor by one step.
// Direction pin: Setting this pin to LOW or HIGH will change the direction the motor rotates.
// Enable pin: If set to LOW motor is activated. If set to HIGH motor is deactivated.

// Pins stepper motor 1 (Front left)
const int M1_pulsPin = 45;
const int M1_dirPin = 36;
const int M1_enPin = 37;

// Pins stepper motor 2 (Front right)
const int M2_pulsPin = 44;
const int M2_dirPin = 38;
const int M2_enPin = 39;

// Pins stepper motor 3 (Back right)
const int M3_pulsPin = 43;
const int M3_dirPin = 26;
const int M3_enPin = 27;

// Pins stepper motor 4 (Back left)
const int M4_pulsPin = 42;
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
unsigned long start_time;

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

  // Set parameters stepper motors
  motor_1.setMaxSpeed(1000);
  motor_2.setMaxSpeed(1000);
  motor_3.setMaxSpeed(1000);
  motor_4.setMaxSpeed(1000);
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
    turn_wheels(1,0,1,0);
    current_state = 0;
  } else if(current_state == 3) {
    publish_state(arduino_state_publisher, 3);
    // Turn the wheels to the turn position
    turn_wheels(0,1,0,1);
    current_state = 0;
  }
  
  nh.spinOnce();
}

// Initialise the wheels
void initialise_wheels() {
  // Turn the wheels to the turned position
  turn_wheels(0,1,0,1);
  delay(1000);

  // Turn the wheels to the straight position
  turn_wheels(1,0,1,0);
}

// Turn the wheels
void turn_wheels(bool M1_direction, bool M2_direction, bool M3_direction, bool M4_direction) {
  bool motor_1_state = false;
  bool motor_2_state = false;
  bool motor_3_state = false;
  bool motor_4_state = false;

  // Set start time
  start_time = millis();

  // Turn the four wheels to the correct position
  while(not motor_1_state or not motor_2_state or not motor_3_state or not motor_4_state) {
    //
    // LET_OP: Sensor motor 2 terugzetten!
    //
    motor_1_state = turn_one_wheel(motor_1, S1_Pin, M1_direction, start_time, motor_default_speed);
    motor_2_state = turn_one_wheel(motor_2, S1_Pin, M2_direction, start_time, motor_default_speed);
    motor_3_state = turn_one_wheel(motor_3, S3_Pin, M3_direction, start_time, motor_default_speed * 2.5);
    motor_4_state = turn_one_wheel(motor_4, S4_Pin, M4_direction, start_time, motor_default_speed * 2.5);
  }
}

// Turn one wheel to the correct position
bool turn_one_wheel(AccelStepper &motor, int IR_sensor, bool motor_direction, unsigned long start_time, int motor_input_speed) {
  int motor_speed = motor_input_speed;
  bool state = false;
  
  // Set the speed of the motor according to the direction
  if(not motor_direction) {
    motor_speed = -motor_input_speed;
  }
  motor.setSpeed(motor_speed);

  // Get the duration of this function
  unsigned long current_time = millis();

  // Turn the wheel
  if(current_time - start_time < 2000) {
    // At the first 2000 miliseconds of this function, just run the motor
    motor.runSpeed();
  } else if(not digitalRead(IR_sensor)) {
    // Run the motor untill the sensor sends a signal the position is reached
    motor.runSpeed();
  } else if(digitalRead(IR_sensor)) {
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
