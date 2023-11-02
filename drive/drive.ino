// ====================
// Author: Cas Damen
// Created on: 31-10-2023
// Description: Arduino code to control the drive of the Agrobot Gantry and the object detection
// ====================

// Include ROS serial to the code
#include <ros.h>
#include <std_msgs/Float32.h>

// Create ROS node handle and include messages
ros::NodeHandle  nh;
std_msgs::Float32 float32_msg;

// Create ROS publishers
ros::Publisher US1_publisher("agrobot_drive/ultrasonic_1", &float32_msg);
ros::Publisher US2_publisher("agrobot_drive/ultrasonic_2", &float32_msg);

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


void setup() {
  // Set baudraute for serial
  Serial.begin(57600);

  // Initialise ROS node and advertise publishers
  nh.initNode();
  nh.advertise(US1_publisher);
  nh.advertise(US2_publisher);

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
  float US1_distance = ultrasonic_sensor(US1_trigPin,US1_echoPin);
  float US2_distance = ultrasonic_sensor(US2_trigPin,US2_echoPin);
  //float US3_distance = ultrasonic_sensor(US3_trigPin,US3_echoPin);
  //float US4_distance = ultrasonic_sensor(US4_trigPin,US4_echoPin);

  // Publish the data of the ultrasonic sensor
  float32_msg.data = US1_distance;
  US1_publisher.publish(&float32_msg);

  float32_msg.data = US2_distance;
  US2_publisher.publish(&float32_msg);
  
  nh.spinOnce();

  // Print the distance of the ultrasonic sensor
  Serial.print("Distance sensor 1: ");
  Serial.println(US1_distance);
  Serial.print("Distance sensor 2: ");
  Serial.println(US2_distance);
  delay(100);
}


// Function to retrieve the distance the ultrasonic sensor measures
float ultrasonic_sensor(int triggerPin,int echoPin)
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
