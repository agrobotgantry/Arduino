// defines pins numbers
#include <ros.h>
#include <drive/drive_output.h>
#include <drive/drive_feedback.h>
#include <drive/drive_sensors.h>

//direction pin
const int M1_dirPin=22;
const int M2_dirPin=24;
const int M3_dirPin=26;
const int M4_dirPin=28;

//puls pin
const int M1_pulsPin=9;
const int M2_pulsPin=10; 
const int M3_pulsPin=11;
const int M4_pulsPin=12;

//enable pin
const int M1_enPin=23;
const int M2_enPin=25;
const int M3_enPin=27;
const int M4_enPin=29;

//variabelen vanuit raspberry
bool  x_moveForward     = false;
bool  x_moveBackwards   = false;
bool  x_moveLeft       = false;
bool  x_moveRight      = false;
int   i_targetSpeed  = 0;

//variabelen naar raspberry
bool  x_movingForward   = false;
bool  x_movingBackwards = false;
bool  x_movingLeft     = false;
bool  x_movingRight    = false;
int   i_currentSpeed = 0;
int   i_distanceSensorFrontLeft     = 0;
int   i_distanceSensorFrontRight    = 0;
int   i_distanceSensorBackLeft   = 0;
int   i_distanceSensorBackRight  = 0;

//ultrasonic sensor
const int trigger1 = 30;
const int trigger2 = 32;
const int trigger3 = 34;
const int trigger4 = 36;

const int echo1 = 31;
const int echo2 = 33;
const int echo3 = 35;
const int echo4 = 37;


//ROS
ros::NodeHandle nh;

void messageCb(const drive::drive_output &driveOutput)
{
  x_moveForward = driveOutput.x_moveForward;
  x_moveBackwards = driveOutput.x_moveBackwards;
  x_moveLeft = driveOutput.x_moveLeft;
  x_moveRight = driveOutput.x_moveRight;
  i_targetSpeed = driveOutput.i_targetSpeed;
}

//ROS subscribe
ros::Subscriber<drive::drive_output> sub("topic_drive_output", &messageCb);

//ROS publish
//initialiseerd headerfile
drive::drive_feedback driveFeedback;
drive::drive_sensors driveSensors;

//initialiseerd publisher+topic
ros::Publisher topic_drive_feedback("topic_drive_feedback", &driveFeedback); 
ros::Publisher topic_drive_sensors("topic_drive_sensors", &driveSensors);

void setup() 
{
nh.initNode();
//advertised topic
nh.advertise(topic_drive_feedback); 
nh.advertise(topic_drive_sensors); 

nh.subscribe(sub);

//seriële monitor
Serial.begin(57600);
Serial.flush();

//initialiseert motor1
  pinMode(M1_dirPin,OUTPUT); 
  pinMode(M1_pulsPin,OUTPUT);
  pinMode(M1_enPin,OUTPUT);
  digitalWrite(M1_enPin,LOW);

//initialiseert motor2
  pinMode(M2_dirPin,OUTPUT); 
  pinMode(M2_pulsPin,OUTPUT);
  pinMode(M2_enPin,OUTPUT);
  digitalWrite(M2_enPin,LOW);

//initialiseert motor3
  pinMode(M3_dirPin,OUTPUT); 
  pinMode(M3_pulsPin,OUTPUT);
  pinMode(M3_enPin,OUTPUT);
  digitalWrite(M3_enPin,LOW);

//initialiseert motor4
  pinMode(M4_dirPin,OUTPUT); 
  pinMode(M4_pulsPin,OUTPUT);
  pinMode(M4_enPin,OUTPUT);
  digitalWrite(M4_enPin,LOW);

//initialiseert ultrasonic sensor1
 pinMode(trigger1, OUTPUT);  
 pinMode(echo1, INPUT);
 
 //initialiseert ultrasonic sensor2
 pinMode(trigger2, OUTPUT);  
 pinMode(echo2, INPUT);
 
 //initialiseert ultrasonic sensr3
 pinMode(trigger3, OUTPUT);  
 pinMode(echo3, INPUT);
 
 //initialiseert ultrasonic sensor4
 pinMode(trigger4, OUTPUT);  
 pinMode(echo4, INPUT);
}

void loop() 
{
  Snelheid(); 
  if(x_moveForward == true)
  {
    AgrobotForward();
  }
  
  else if(x_moveBackwards == true)
  {
    AgrobotBackwards();
  } 
        
  else if(x_moveLeft == true)
  {
    AgrbotLinks();
  } 
       
  else if(x_moveRight == true)
  {
    AgrobotRight();
  }
  else{
    AgrobotStop();
  }
  
  i_distanceSensorFrontLeft = ultrasonicsensor(trigger1,echo1);
  i_distanceSensorFrontRight = ultrasonicsensor(trigger2,echo2);
  i_distanceSensorBackRight = ultrasonicsensor(trigger3,echo3);
  i_distanceSensorBackLeft = ultrasonicsensor(trigger4,echo4);
 
  //publishValues();
  publishValues(); 
  topic_drive_feedback.publish(&driveFeedback);
  topic_drive_sensors.publish(&driveSensors);
  nh.spinOnce();
  delay(100);
}

//functies

//vooruit
void AgrobotForward(void)
{
 digitalWrite(M1_enPin, LOW);
 digitalWrite(M2_enPin, LOW);
 digitalWrite(M3_enPin, LOW);
 digitalWrite(M4_enPin, LOW);
             
 analogWrite(M1_pulsPin, 128);
 analogWrite(M2_pulsPin, 128);
 analogWrite(M3_pulsPin, 128);
 analogWrite(M4_pulsPin, 128);

 digitalWrite(M1_dirPin, HIGH);
 digitalWrite(M2_dirPin, LOW);
 digitalWrite(M3_dirPin, LOW);
 digitalWrite(M4_dirPin, HIGH);

 x_movingRight = false; 
 x_movingLeft = false; 
 x_movingBackwards = false; 
 x_movingForward = true;
}

//achteruit
void AgrobotBackwards(void)
{
 digitalWrite(M1_enPin, LOW);
 digitalWrite(M2_enPin, LOW);
 digitalWrite(M3_enPin, LOW);
 digitalWrite(M4_enPin, LOW);
              
 analogWrite(M1_pulsPin, 128);
 analogWrite(M2_pulsPin, 128);
 analogWrite(M3_pulsPin, 128);
 analogWrite(M4_pulsPin, 128);
  
 digitalWrite(M1_dirPin, LOW);
 digitalWrite(M2_dirPin, HIGH);
 digitalWrite(M3_dirPin, HIGH);
 digitalWrite(M4_dirPin, LOW);
 
 x_movingRight = false; 
 x_movingLeft = false; 
 x_movingBackwards = true;
 x_movingForward = false;
}

//links
void AgrbotLinks(void)
{
 digitalWrite(M1_enPin, LOW);
 digitalWrite(M2_enPin, LOW);
 digitalWrite(M3_enPin, LOW);
 digitalWrite(M4_enPin, LOW);
       
 analogWrite(M1_pulsPin, 128);
 analogWrite(M2_pulsPin, 128);
 analogWrite(M3_pulsPin, 128);
 analogWrite(M4_pulsPin, 128);

 digitalWrite(M1_dirPin, LOW);
 digitalWrite(M2_dirPin, LOW);
 digitalWrite(M3_dirPin, LOW);
 digitalWrite(M4_dirPin, LOW);
 
 x_movingForward = false;
 x_movingBackwards = false; 
 x_movingRight = false; 
 x_movingLeft = true;  
}

//rechts
void AgrobotRight(void)
{
 digitalWrite(M1_enPin, LOW);
 digitalWrite(M2_enPin, LOW);
 digitalWrite(M3_enPin, LOW);
 digitalWrite(M4_enPin, LOW);
       
 analogWrite(M1_pulsPin, 128);
 analogWrite(M2_pulsPin, 128);
 analogWrite(M3_pulsPin, 128);
 analogWrite(M4_pulsPin, 128);

 digitalWrite(M1_dirPin, HIGH);
 digitalWrite(M2_dirPin, HIGH);
 digitalWrite(M3_dirPin, HIGH);
 digitalWrite(M4_dirPin, HIGH);
 
 x_movingForward = false;
 x_movingBackwards = false;
 x_movingLeft = false;
 x_movingRight = true;  
}
void AgrobotStop(void)
{
 digitalWrite(M1_enPin, LOW);
 digitalWrite(M2_enPin, LOW);
 digitalWrite(M3_enPin, LOW);
 digitalWrite(M4_enPin, LOW);
             
 analogWrite(M1_pulsPin, 0);
 analogWrite(M2_pulsPin, 0);
 analogWrite(M3_pulsPin, 0);
 analogWrite(M4_pulsPin, 0);

 digitalWrite(M1_dirPin, LOW);
 digitalWrite(M2_dirPin, LOW);
 digitalWrite(M3_dirPin, LOW);
 digitalWrite(M4_dirPin, LOW);

 x_movingForward = false;
 x_movingBackwards = false;
 x_movingLeft = false;
 x_movingRight = false;
}
void Snelheid(void){
  //snelheid aanpassen
//30,64HZ
  if (i_targetSpeed == 25)
  {
//pin 11-12    
  TCCR1B = TCCR1B & B11111000 | B00000101;
//PIN 9-10 
  TCCR2B = TCCR2B & B11111000 | B00000111; 
  i_currentSpeed=25;
  }
  
//122,55HZ  
  if (i_targetSpeed == 50)
  {
//pin 11-12    
  TCCR1B = TCCR1B & B11111000 | B00000100;
//PIN 9-10  
  TCCR2B = TCCR2B & B11111000 | B00000110;
  i_currentSpeed=50;
  }
  
//490,20HZ  
  if (i_targetSpeed == 75)
  {
//pin 11-12    
  TCCR1B = TCCR1B & B11111000 | B00000011;
//PIN 9-10 
  TCCR2B = TCCR2B & B11111000 | B00000100; 
  i_currentSpeed=75;
  }
  
//3921,16HZ  
  if (i_targetSpeed == 100)
  {
//pin 11-12
  TCCR1B = TCCR1B & B11111000 | B00000010; 
//PIN 9-10  
  TCCR2B = TCCR2B & B11111000 | B00000010;
  i_currentSpeed=100;
  }
}

void publishValues() 
{
//linked header file to variables  
  driveFeedback.x_movingForward = x_movingForward;
  driveFeedback.x_movingBackwards = x_movingBackwards;   
  driveFeedback.x_movingRight = x_movingRight;   
  driveFeedback.x_movingLeft = x_movingLeft;
  driveFeedback.i_currentSpeed = i_currentSpeed;      

  driveSensors.i_distanceSensorFrontLeft = i_distanceSensorFrontLeft;
  driveSensors.i_distanceSensorBackLeft = i_distanceSensorBackLeft;
  driveSensors.i_distanceSensorFrontRight = i_distanceSensorFrontRight;
  driveSensors.i_distanceSensorBackRight = i_distanceSensorBackRight; 

}

float ultrasonicsensor(int triggerPin,int echoPin)
{
  long duration=0;
  float distance=0;
  // Clears the trigger condition
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigger HIGH (ACTIVE) for 10 microseconds
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance,Speed of sound wave divided by 2 (go and back)
  distance = duration * 0.034 / 2;
   
  return (distance);
}
