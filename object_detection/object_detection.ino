// ====================
// Author: Cas Damen
// Created on: 09-11-2023
// Description: Arduino code for the object detection of the Agrobot Gantry
// ====================

// Include ROS serial to the code
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

// Pins ultrasonic sensor 1 (Back left)
const int US1_trigPin = 26;
const int US1_echoPin = 27;

// Pins ultrasonic sensor 2 (Back right)
const int US2_trigPin = 28;
const int US2_echoPin = 25; // 29

// Pins ultrasonic sensor 3 (Front left)
const int US3_trigPin = 30;
const int US3_echoPin = 31;

// Pins ultrasonic sensor 4 (Front right)
const int US4_trigPin = 32;
const int US4_echoPin = 33;

// Create ROS node handle and include messages
ros::NodeHandle nh;
std_msgs::Float32 float32_msg;
std_msgs::Bool bool_msg;

// Create ROS publishers
ros::Publisher US1_publisher("agrobot_object_detection/ultrasonic_1", &float32_msg);
ros::Publisher US2_publisher("agrobot_object_detection/ultrasonic_2", &float32_msg);
ros::Publisher US3_publisher("agrobot_object_detection/ultrasonic_3", &float32_msg);
ros::Publisher US4_publisher("agrobot_object_detection/ultrasonic_4", &float32_msg);
ros::Publisher object_detected_publisher("agrobot_object_detection/object_detected", &bool_msg);


void setup() {
  // Set baudraute for serial
  Serial.begin(57600);

  // Initialise ROS node and advertise publishers
  nh.initNode();
  nh.advertise(US1_publisher);
  nh.advertise(US2_publisher);
  nh.advertise(US3_publisher);
  nh.advertise(US4_publisher);
  nh.advertise(object_detected_publisher);

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
}


void loop() {
  // Get the distances of the ultrasonic sensors
  float US1_distance = ultrasonic_sensor_distance(US1_trigPin,US1_echoPin);
  float US2_distance = ultrasonic_sensor_distance(US2_trigPin,US2_echoPin);
  float US3_distance = ultrasonic_sensor_distance(US3_trigPin,US3_echoPin);
  float US4_distance = ultrasonic_sensor_distance(US4_trigPin,US4_echoPin);

  // Check if there is an object detected
  float min_distance = 30.0;
  bool object_detected = false;

  bool US1_detection = US1_distance < min_distance && US1_distance > 0;
  bool US2_detection = US2_distance < min_distance && US2_distance > 0;
  bool US3_detection = US3_distance < min_distance && US3_distance > 0;
  bool US4_detection = US4_distance < min_distance && US4_distance > 0;
  
  if (US1_detection || US2_detection || US3_detection || US4_detection) {
    object_detected = true;
  } else {
    object_detected = false;
  }

  // Publish the data of the ultrasonic sensor
  publish_ultrasonic_sensor(US1_publisher, US1_distance);
  publish_ultrasonic_sensor(US2_publisher, US2_distance);
  publish_ultrasonic_sensor(US3_publisher, US3_distance);
  publish_ultrasonic_sensor(US4_publisher, US4_distance);

  // Publish the value of object detected
  bool_msg.data = object_detected;
  object_detected_publisher.publish(&bool_msg);

  nh.spinOnce();
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


// Funtion to publish the value of a ultrasonic sensor to ROS
void publish_ultrasonic_sensor(ros::Publisher publisher, float distance) {
  float32_msg.data = distance;
  publisher.publish(&float32_msg);
}
