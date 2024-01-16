#include <AccelStepper.h>

// Include ROS serial to the code
#include <ros.h>
#include <std_msgs/Int8.h>

#define STEP_PIN 42
#define DIR_PIN  38

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Create ROS node handle and include messages
ros::NodeHandle nh;
std_msgs::Int8 int8_msg;

// Current state
// 0 = off
// 1 = sluiten
// 2 = openen
int current_state = 0;

// Subscriber callback arduino command
void arduino_command_callback(const std_msgs::Int8 &message) {
  current_state = message.data;
}

// Create ROS subsriber and publisher
ros::Subscriber<std_msgs::Int8> arduino_cmd_subscriber("agrobot_storage/arduino_command", &arduino_command_callback);

void setup() {
  // Set baudraute for serial
  Serial.begin(57600);

  // Initialise ROS node, subscribe and initialise publisher to topics
  nh.initNode();
  nh.subscribe(arduino_cmd_subscriber);
  
  stepper.setMaxSpeed(10000.0);
  stepper.setAcceleration(10000.0);
  
  //stepper.moveTo(1600);  
}

void loop() {

  stepper.run();

  if(current_state == 1) {
    stepper.moveTo(1600);
  } else if (current_state == 2){
    stepper.moveTo(-1600);
  }

  nh.spinOnce();
}
