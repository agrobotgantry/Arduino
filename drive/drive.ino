// ====================
// Author: Cas Damen
// Created on: 31-10-2023
// Description: Arduino code to control the drive of the Agrobot Gantry and the object detection
// ====================

// Include ROS serial to the code
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

// Pins ultrasonic sensor 1 (Front left)
const int US1_trigPin = 30;
const int US1_echoPin = 31;

// Pins ultrasonic sensor 2 (Front right)
const int US2_trigPin = 32;
const int US2_echoPin = 33;

// Pins ultrasonic sensor 3 (Back right)
const int US3_trigPin = 34;
const int US3_echoPin = 35;

// Pins ultrasonic sensor 4 (Back left)
const int US4_trigPin = 36;
const int US4_echoPin = 37;

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

// Create ROS node handle and include messages
ros::NodeHandle  nh;
std_msgs::Float32 float32_msg;
std_msgs::Int8 int8_msg;

// Create ROS publishers
ros::Publisher US1_publisher("agrobot_drive/ultrasonic_1", &float32_msg);
ros::Publisher US2_publisher("agrobot_drive/ultrasonic_2", &float32_msg);
ros::Publisher US3_publisher("agrobot_drive/ultrasonic_3", &float32_msg);
ros::Publisher US4_publisher("agrobot_drive/ultrasonic_4", &float32_msg);


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
ros::Subscriber<std_msgs::Int8> arduino_cmd_subscriber("agrobot_drive/arduino_command", &arduino_command_callback);


void setup() {
  // Set baudraute for serial
  Serial.begin(57600);

  // Initialise ROS node, advertise publishers and subscribe to topics
  nh.initNode();
  nh.advertise(US1_publisher);
  nh.advertise(US2_publisher);
  nh.advertise(US3_publisher);
  nh.advertise(US4_publisher);
  nh.subscribe(M1_subscriber);
  nh.subscribe(M2_subscriber);
  nh.subscribe(M3_subscriber);
  nh.subscribe(M4_subscriber);
  nh.subscribe(arduino_cmd_subscriber);

  // Initialise ultrasonic sensor 1 (Front left)
  pinMode(US1_echoPin, INPUT);
  pinMode(US1_trigPin, OUTPUT);

  // Initialise ultrasonic sensor 2 (Front right)
  pinMode(US2_echoPin, INPUT);
  pinMode(US2_trigPin, OUTPUT);

  // Initialise ultrasonic sensor 3 (Back right)
  pinMode(US3_echoPin, INPUT);
  pinMode(US3_trigPin, OUTPUT);

  // Initialise ultrasonic sensor 4 (Back left)
  pinMode(US4_echoPin, INPUT);
  pinMode(US4_trigPin, OUTPUT);

  // Initialise stepper motor 1 (Front left)
  pinMode(M1_dirPin, OUTPUT); 
  pinMode(M1_pulsPin, OUTPUT);
  pinMode(M1_enPin, OUTPUT);
  digitalWrite(M1_enPin, LOW);

  // Initialise stepper motor 2 (Front right)
  pinMode(M2_dirPin, OUTPUT); 
  pinMode(M2_pulsPin, OUTPUT);
  pinMode(M2_enPin, OUTPUT);
  digitalWrite(M2_enPin, LOW);

  // Initialise stepper motor 3 (Back right)
  pinMode(M3_dirPin, OUTPUT); 
  pinMode(M3_pulsPin, OUTPUT);
  pinMode(M3_enPin, OUTPUT);
  digitalWrite(M3_enPin, LOW);

  // Initialise stepper motor 4 (Back left)
  pinMode(M4_dirPin, OUTPUT); 
  pinMode(M4_pulsPin, OUTPUT);
  pinMode(M4_enPin, OUTPUT);
  digitalWrite(M4_enPin, LOW);
}


void loop() {
  // Get the distances of the ultrasonic sensors
  float US1_distance = ultrasonic_sensor_distance(US1_trigPin,US1_echoPin);
  float US2_distance = ultrasonic_sensor_distance(US2_trigPin,US2_echoPin);
  float US3_distance = ultrasonic_sensor_distance(US3_trigPin,US3_echoPin);
  float US4_distance = ultrasonic_sensor_distance(US4_trigPin,US4_echoPin);

  // Publish the data of the ultrasonic sensor
  publish_ultrasonic_sensor(US1_publisher, US1_distance);
  publish_ultrasonic_sensor(US2_publisher, US2_distance);
  publish_ultrasonic_sensor(US3_publisher, US3_distance);
  publish_ultrasonic_sensor(US4_publisher, US4_distance);
  
  delay(100);
  nh.spinOnce();
}


// Function to turn the stepper motor on. The direction should be positive (HIGH = 1) or negative (LOW = 0)
void motor_on(int motor_number, int motor_direction){
  // Select the correct motor to turn on
  if(motor_number == 1) {
    // Motor front left
    digitalWrite(M1_enPin, LOW);
    digitalWrite(M1_dirPin, motor_direction);
    analogWrite(M1_pulsPin, 128);
    Serial.println("Motor 1 turned on");
  } else if(motor_number == 2) {
    // Motor front right
    digitalWrite(M2_enPin, LOW);
    digitalWrite(M2_dirPin, motor_direction);
    analogWrite(M2_pulsPin, 128);
    Serial.println("Motor 2 turned on");
  } else if(motor_number == 3) {
    // Motor back right
    digitalWrite(M3_enPin, LOW);
    digitalWrite(M3_dirPin, motor_direction);
    analogWrite(M3_pulsPin, 128);
    Serial.println("Motor 3 turned on");
  } else if(motor_number == 4) {
    // Motor back left
    digitalWrite(M4_enPin, LOW);
    digitalWrite(M4_dirPin, motor_direction);
    analogWrite(M4_pulsPin, 128);
    Serial.println("Motor 4 turned on");
  }
}


// Function to turn the stepper motor off
void motor_off(int motor_number){
  // Select the correct motor to turn off
  if(motor_number == 1) {
    // Motor front left
    digitalWrite(M1_enPin, LOW);
    digitalWrite(M1_dirPin, LOW);
    analogWrite(M1_pulsPin, 0);
    Serial.println("Motor 1 turned off");
  } else if(motor_number == 2) {
    // Motor front right
    digitalWrite(M2_enPin, LOW);
    digitalWrite(M2_dirPin, LOW);
    analogWrite(M2_pulsPin, 0);
    Serial.println("Motor 2 turned off");
  } else if(motor_number == 3) {
    // Motor back right
    digitalWrite(M3_enPin, LOW);
    digitalWrite(M3_dirPin, LOW);
    analogWrite(M3_pulsPin, 0);
    Serial.println("Motor 3 turned off");
  } else if(motor_number == 4) {
    // Motor back left
    digitalWrite(M4_enPin, LOW);
    digitalWrite(M4_dirPin, LOW);
    analogWrite(M4_pulsPin, 0);
    Serial.println("Motor 4 turned off");
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


// Function to retrieve the distance the ultrasonic sensor measures
float ultrasonic_sensor_distance(int triggerPin,int echoPin)
{
  // Initialise variable
  float duration, distance;

  // Send a wave to detect the object
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Get the duration and calculate the distance based on the send wave
  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  delay(100);
   
  return (distance);
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


// Funtion to publish the value of a ultrasonic sensor to ROS
void publish_ultrasonic_sensor(ros::Publisher publisher, float distance) {
  float32_msg.data = distance;
  publisher.publish(&float32_msg);
}
