#include <ros.h> 
#include <std_msgs/String.h>
#include <portal/portal_output.h> 
#include <portal/portal_feedback.h> 
#include <portal/portal_sensors.h> 
//#include <servo.h>

// Direction pinnen 
const int XY_Mot1_Dir = 22  ;
const int XY_Mot2_Dir = 24  ;
const int Z_Mot_Dir = 26    ;
const int Gripper_Dir = 28  ;

// Puls pinnen
const int XY_Mot1_Puls = 2  ;
const int XY_Mot2_Puls = 3  ;
const int Z_Mot_Puls = 4    ;
const int Gripper_Puls = 6  ;

// Enable pinnen
const int XY_Mot1_Ena = 23  ;
const int XY_Mot2_Ena = 25  ;
const int Z_Mot_Ena = 27    ;
const int Gripper_Ena = 29  ;

// Servo pinnen gripper
const int ServoHook = 30    ;

// Inputs 
const int X_as_switch_positive_NC = 30  ;
const int X_as_switch_positive_NO = 31  ;
const int X_as_switch_negative_NC = 32  ;
const int X_as_switch_negative_NO = 33  ; 
const int Y_as_switch_positive_NC = 34  ;
const int Y_as_switch_positive_NO = 35  ;
const int Y_as_switch_negative_NC = 36  ;
const int Y_as_switch_negative_NO = 37  ;
const int z_as_switch_Down = 38         ;
const int z_as_switch_Up = 39           ;
const int Gripper_is_open = 40          ;
const int Gripper_is_cloded = 41        ;

// Variabelen vanuit raspberry
bool x_XgoPositive = false    ;
bool x_XgoNegative = false    ;
bool x_YgoPostive = false     ;
bool x_YgoNegative = false    ;
bool x_ZgoUp = false          ;
bool x_ZgoDown = false        ;
bool x_GripperClose = false   ;
bool x_GripperOpen = false    ;
bool x_goHomePosition = false ;

// Variabelen naar rapsberry
bool x_XgoingPositive = false    ;
bool x_XgoingNegative = false    ;
bool x_YgoingPositive = false    ;
bool x_YgoingNegative = false    ;
bool x_ZgoingUp = false          ;
bool x_ZgoingDown = false        ;
bool x_XendSwitchPositive = false ;
bool x_XendSwitchNegative = false ;
bool x_YendSwitchPositive = false ;
bool x_YendSwitchNegative = false ;
bool x_ZendSwitchUp = false       ;
bool x_ZendSwitchDown = false     ;
bool x_isHomePosition = false     ;
bool x_GripperIsClosed = false    ;
bool x_GripperIsOpen = false      ;

// ROS
ros::NodeHandle nh;

void messageCb(const portal::portal_output &portal_output)
{
  x_XgoPositive = portal_output.x_XgoPositive        ;
  x_XgoNegative = portal_output.x_XgoNegative        ;
  x_YgoPostive = portal_output.x_YgoPositive          ;
  x_YgoNegative = portal_output.x_YgoNegative        ;
  x_ZgoUp = portal_output.x_ZgoUp                    ;
  x_ZgoDown = portal_output.x_ZgoDown                ;
  x_goHomePosition = portal_output.x_goHomePosition  ;
}

//Subscribe 
ros::Subscriber<portal::portal_output> sub("topic_portal_output", &messageCb);

// Publisher
portal::portal_feedback portalFeedback;
portal::portal_sensors portalSensors;

ros::Publisher topic_portal_feedback("topic_portal_feedback", &portalFeedback);
ros::Publisher topic_portal_sensors("topic_portal_sensors", &portalSensors);

void setup() 
{
// change pwm signalspeed
// TCCR3B = TCCR3B & B11111000 | B00000011;  // 490HZ Pin 2 en 3
  TCCR3B = TCCR3B & B11111000 | B00000010;  // 3920HZ Pin 2 en 3 
// TCCR0B = TCCR0B & B11111000 | B00000011;  // 980 HZ Pin 4
  TCCR0B = TCCR0B & B11111000 | B00000010;  // 7810 HZ Pin 4


  // Motor1 xy-portaal
  pinMode(XY_Mot1_Dir, OUTPUT)      ;
  pinMode(XY_Mot1_Puls, OUTPUT)     ;
  pinMode(XY_Mot1_Ena, OUTPUT)      ;
  digitalWrite(XY_Mot1_Ena, HIGH)   ;

  // Motor2 xy-portaal
  pinMode(XY_Mot2_Dir, OUTPUT)      ;
  pinMode(XY_Mot2_Puls, OUTPUT)     ;
  pinMode(XY_Mot2_Ena, OUTPUT)      ;
  digitalWrite(XY_Mot2_Ena, HIGH)   ;

  // Inputs XY-portaal
  pinMode(X_as_switch_negative_NC, INPUT_PULLUP)       ;
  pinMode(X_as_switch_negative_NO, INPUT_PULLUP)       ;
  pinMode(X_as_switch_positive_NC, INPUT_PULLUP)       ;
  pinMode(X_as_switch_positive_NO, INPUT_PULLUP)       ;
  pinMode(Y_as_switch_negative_NC, INPUT_PULLUP)       ;
  pinMode(Y_as_switch_negative_NO, INPUT_PULLUP)       ;
  pinMode(Y_as_switch_positive_NC, INPUT_PULLUP)       ;
  pinMode(Y_as_switch_positive_NO, INPUT_PULLUP)       ;

  // Motor z-portaal
  pinMode(Z_Mot_Dir, OUTPUT)        ;
  pinMode(Z_Mot_Puls, OUTPUT)       ;
  pinMode(Z_Mot_Ena, OUTPUT)        ;
  digitalWrite(Z_Mot_Ena, HIGH)     ;

  // Inputs Z-portaal
  pinMode(z_as_switch_Down, INPUT_PULLUP)  ;
  pinMode(z_as_switch_Up, INPUT_PULLUP)    ;  

  // Gripper
  pinMode(Gripper_Dir, OUTPUT)      ;
  pinMode(Gripper_Puls, OUTPUT)     ;
  pinMode(Gripper_Ena, OUTPUT)      ;
  digitalWrite(Gripper_Ena, HIGH)   ;
  
  //myservo.attach(ServoHook)         ;

  //Inputs Gripper
  pinMode(Gripper_is_open, INPUT_PULLUP)   ;
  pinMode(Gripper_is_cloded, INPUT_PULLUP) ;

  nh.initNode()                     ;
  nh.advertise(topic_portal_feedback) ;
  nh.advertise(topic_portal_sensors) ;
  nh.subscribe(sub)                 ;

  Serial.begin(57600);
  Serial.flush();
}

void loop() 
{

  if (x_XgoPositive == true)
  {
    X_Core_Positive(); 
    if (digitalRead(X_as_switch_positive_NO ) == 1)
    {
      x_XendSwitchPositive  == true ;
      ResetMotoren();
    } 
  }

  else if (x_XgoNegative == true)
  {
    X_Core_Negative();
    if (digitalRead(X_as_switch_negative_NO) == 1)
    {
      x_XendSwitchNegative == true ;
      ResetMotoren();
    }
  }

  else if (x_YgoPostive == true)
  {
      Y_Core_Positive();
    if (digitalRead(Y_as_switch_positive_NO) == 1)
    {
      x_YendSwitchPositive  == true ;
      ResetMotoren();
    }
  }

  else if (x_YgoNegative == true)
  {
    Y_Core_Negative();
    if (digitalRead(Y_as_switch_negative_NO) == 1)
    {
      x_YendSwitchNegative == true ;
      ResetMotoren();      
    }
  }

  else if (x_ZgoUp == true)
  {
    Z_as_Up();
    if (digitalRead(z_as_switch_Up) == 1) 
    {
      x_ZendSwitchUp == true ;
      ResetMotoren();        
    }
  }

  else if (x_ZgoDown == true)
  {
    Z_as_Down();
    if (digitalRead(z_as_switch_Down) == 1) 
    {
      x_ZendSwitchDown == true ;
      ResetMotoren();      
    }
  }

  else if (x_GripperClose == 1)
  {
    GripperStart();
    delay(1500); // hopelijk is dit niet nodig?
  }

  else if (x_GripperOpen == true)
  {
    GripperStop();
    delay(1500); // hopelijk is dit niet nodig?
  }

  else if (x_goHomePosition == true)
  {
    Z_as_Up();
    
    if (digitalRead(z_as_switch_Up) == 1)
    {
      ResetMotoren()   ;
      X_Core_Negative() ;
      
      if (digitalRead(X_as_switch_negative_NO) == 1)
      {
        ResetMotoren()    ;
        Y_Core_Negative() ;

        if (digitalRead(Y_as_switch_negative_NO) == 1)
        {
          ResetMotoren()          ; 
          x_isHomePosition = true ;
        }
      }
    }
  }

  else 
  {
    ResetMotoren() ;
  }
  // Message values 
  portalFeedback.x_XgoingPositive = x_XgoingPositive;
  portalFeedback.x_XgoingNegative = x_XgoingNegative;
  portalFeedback.x_YgoingPositive = x_YgoingPositive;
  portalFeedback.x_YgoingNegative = x_YgoingNegative;
  portalFeedback.x_ZgoingUp = x_ZgoingUp;
  portalFeedback.x_ZgoingDown = x_ZgoingDown;
  portalFeedback.x_isHomePosition = x_isHomePosition;

  portalSensors.x_XendSwitchPostive = digitalRead(X_as_switch_positive_NO);
  portalSensors.x_XendSwitchNegative = digitalRead(X_as_switch_negative_NO);
  portalSensors.x_YendSwitchPositive = digitalRead(Y_as_switch_positive_NO);
  portalSensors.x_YendSwitchNegative = digitalRead(Y_as_switch_negative_NO);
  portalSensors.x_ZendSwitchUp = digitalRead(z_as_switch_Up);
  portalSensors.x_ZendSwitchDown = digitalRead(z_as_switch_Down);

  topic_portal_feedback.publish(&portalFeedback);
  topic_portal_sensors.publish(&portalSensors);
  nh.spinOnce();
  delay(100);

}

//Functies 
void X_Core_Positive(void)
{
  digitalWrite(XY_Mot1_Ena, LOW )   ; // Enable motor 1
  analogWrite(XY_Mot1_Puls, 128 )   ; // Motor 1 Puls
  digitalWrite(XY_Mot1_Dir, HIGH )  ; // Motor 1 Linksom

  digitalWrite(XY_Mot2_Ena, LOW )   ; // Enable motor 2
  analogWrite(XY_Mot2_Puls, 128 )   ; // Motor 2 Puls
  digitalWrite(XY_Mot2_Dir, LOW)    ; // Motor 2 Rechtsom

  x_XgoingPositive = true           ;
}

void X_Core_Negative(void)
{
  digitalWrite(XY_Mot1_Ena, LOW )   ; // Enable motor 1
  analogWrite(XY_Mot1_Puls, 128 )   ; // Motor 1 Puls
  digitalWrite(XY_Mot1_Dir, LOW )   ; // Motor 1 Rechtsom

  digitalWrite(XY_Mot2_Ena, LOW )   ; // Enable motor 2
  analogWrite(XY_Mot2_Puls, 128 )   ; // Motor 2 Puls
  digitalWrite(XY_Mot2_Dir, HIGH)   ; // Motor 2 Linksom

  x_XgoingNegative = true           ;
}

void Y_Core_Positive(void)
{
  digitalWrite(XY_Mot1_Ena, LOW )   ; // Enable motor 1
  analogWrite(XY_Mot1_Puls, 128 )   ; // Motor 1 Puls
  digitalWrite(XY_Mot1_Dir, LOW )   ; // Motor 1 Rechtom
  
  digitalWrite(XY_Mot2_Ena, LOW )   ; // Enable motor 2
  analogWrite(XY_Mot2_Puls, 128 )   ; // Motor 2 Puls
  digitalWrite(XY_Mot2_Dir, LOW)    ; // Motor 2 Rechtsom

  x_YgoingPositive = true           ;
}

void Y_Core_Negative(void)
{
  digitalWrite(XY_Mot1_Ena, LOW )   ; // Enable motor 1
  analogWrite(XY_Mot1_Puls, 128 )   ; // Motor 1 Puls
  digitalWrite(XY_Mot1_Dir, HIGH)   ; // Motor 1 Rechtsom

  digitalWrite(XY_Mot2_Ena, LOW )   ; // Enable motor 2
  analogWrite(XY_Mot2_Puls, 128 )   ; // Motor 2 Puls
  digitalWrite(XY_Mot2_Dir, HIGH)   ; // Motor 2 Linksom

  x_YgoingNegative = true           ;
}

void Z_as_Up(void)
{
  digitalWrite(Z_Mot_Ena, LOW )     ; // Enable Z-as Motor
  analogWrite(Z_Mot_Puls, 128 )     ; // Motor Z Puls
  digitalWrite(Z_Mot_Dir, HIGH )    ; // Motor Z Rechtsom

  x_ZgoingUp = true                 ;
}

void Z_as_Down(void)
{
  digitalWrite(Z_Mot_Ena, LOW )     ; // Enable Z-as Motor
  analogWrite(Z_Mot_Puls, 128 )     ; // Motor Z Puls
  digitalWrite(Z_Mot_Dir, LOW )     ; // Motor Z Linksom

  x_ZgoingDown = true               ; 
}

void ResetMotoren(void)
{
  digitalWrite(XY_Mot1_Ena, HIGH )   ; // Reset motor 1
  digitalWrite(XY_Mot2_Ena, HIGH )   ; // Reset motor 2
  digitalWrite(Z_Mot_Ena, HIGH )     ; // Reset motor Z
  analogWrite(XY_Mot1_Puls, 0)       ; // Reset Puls Motor 1
  analogWrite(XY_Mot2_Puls, 0)       ; // Reset Puls Motor 2
  analogWrite(Z_Mot_Puls, 0)         ; // Reset Puls Motor Z 
  
  x_XgoingPositive = false           ; // Reset Message value
  x_XgoingNegative = false           ; // Reset Message value
  x_YgoingPositive = false           ; // Reset Message value
  x_YgoingNegative = false           ; // Reset Message value
  x_ZgoingUp = false                 ; // Reset Message value
  x_ZgoingDown = false               ; // Reset Message value
}

void GripperStart(void)
{
  //myservo.write(90) ;
  delay(500)        ;
}

void GripperStop(void)
{
  //myservo.write(15) ;
  delay(500)        ;
}
