// ====================
// Author: Cas Damen & Jerome Kemper
// Created on: 27-11-2023
// Description: Arduino code to control the gantry of the Agrobot
// ====================


// include other arduino lib
#include <avr/io.h>
#include <avr/interrupt.h>

// header file met alle pinouts
#include "pinDefine.h"
#include "servo_gantry.h"
#include "stepper_gantry.h"
#include "sorting_gantry.h"
#include "rosserial_gantry.h"

// global var yikes
unsigned long waitTime = 0;
int state = 0;
int* ptr_state = &state;
int* gewas_locatie;

// ====================================== SETUP ======================================


void setup() {
  Serial.begin(57600);

  // while(1);
  
  initializePins(); // pinout
  //setExternalInterrupt(); // kan pas als Alex het overzet

  setupTimer();     // setup at timer2 1ms
  setup_servo();
  setup_stepperXYZ();
  setup_ros_nodes();

  delay(10); // small delay to be sure 
}


// ====================================== LOOP ======================================


void loop() {
  
  // check_werking_sensoren_gantry(); // reading sensor trigger

  //int x_afstand = 2400;   
  //int y_afstand = -2200;


  if (startingpoint){ // global in rossserial
    // if subscribe ros aanzetten van gantry
    // zet state op 0;

    // reset ook id
    *ptr_id = 0;
    *ptr_state = 0;

    // if subscribe LEFT, MIDDLE, RIGHT
    // ga naar locatie en start proces
  }

  // aanvraag gewas locatie
  if (*ptr_state == 0){
    static bool publish_message = false;

    if(!publish_message){
      // ros publish
      start_vision_gewas_locatie_pub.publish(&empty_msg);
  
      publish_message = true;
    }
    if((position_x != 0 && position_y != 0) ){ 
      // ros subscribe - wachten tot de waarden up to date zijn
      *ptr_state += 1;
    }

  }

  // naar gewas locatie gaan met gegeven coordinaten
  else if (*ptr_state == 1){
    gewasPositie(position_x, position_y, ptr_state);
  }

  // gripper sluiten - gewas pakken
  else if (*ptr_state == 2){
    gripper_close(ptr_state);
    waitTime = millis(); 
  }

  // wacht seconde voordat servo goed sluit
  // gewas brengen naar de camera voor herkenning
  else if (*ptr_state == 3){
    if ((millis() - waitTime >= 1000)){
      gewas_naar_camera(ptr_state);    
    }
  }

  // aanvraag object detectie op Jetson
  else if (*ptr_state == 4){

    static bool publish_message = false;

    if(!publish_message){
      // ros publish
      start_object_recognition_pub.publish(&empty_msg);
  
      publish_message = true;
    }
    if (*ptr_id != NULL){
      gewas_locatie = gewas_bak_locatie(ptr_state);
    }

  }

  // breng gewas naar juiste bak
  else if (*ptr_state == 5){
    sorteer_gewas(ptr_state, gewas_locatie);
  }

  // gripper openen - gewas laten vallen
  else if (*ptr_state == 6){
    gripper_open(ptr_state);
    waitTime = millis();
  }

  // controlleer de bakken hoe ze gevuld zijn
  // process bevat geen feedback en wordt incrementeer opgeteld
  else if (*ptr_state == 7){
    // check staat bakken

    if ((millis() - waitTime >= 2000)){
      Serial.println("Check Bakken");
      limiet_bakken(ptr_state); // check limiet, anders ROS publish
    }

  }

  // melding geven dat alle taken zijn doorgelopen
  // wellicht een melding dat de bakken vol zijn zodat ze geleegd kunnen worden
  else if (*ptr_state == 8){
    //ROS MESSAGE KLAAR
    nh.advertise(gewas_verwerkt_pub); 
  }


  nh.spinOnce();
}


// ====================================== FUNCTIONS ======================================

void setupTimer() {
  cli(); // Disable interrupts

  // Timer2 for a 1ms interval at 16MHz clock
  TCCR2A = 0;
  TCCR2B = 0;
  OCR2A = 249;  

  TCCR2A |= (1 << WGM21); // Set CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 64 prescaler
  TIMSK2 |= (1 << OCIE2A); // Enable timer compare interrupt

  sei(); // Enable interrupts
}

ISR(TIMER2_COMPA_vect) {
  // Your code here - will be executed every 1ms
  
  int x = motor_x.isRunning();
  int y = motor_y.isRunning();
  int z = motor_z.isRunning();

  if (x || y || z){
    // check position if running in workspace bounds

  }
}


void setExternalInterrupt(){
  attachInterrupt(digitalPinToInterrupt(Switch_X1_NC_links), x_axis_stop, RISING);
  attachInterrupt(digitalPinToInterrupt(Switch_X2_NC_rechts), x_axis_stop, RISING);
  attachInterrupt(digitalPinToInterrupt(Reed_Y1_voor), y_axis_stop, RISING);
  attachInterrupt(digitalPinToInterrupt(Reed_Y2_achter), y_axis_stop, RISING);
  attachInterrupt(digitalPinToInterrupt(Reed_Z1_boven), z_axis_stop, RISING);
  attachInterrupt(digitalPinToInterrupt(Reed_Z2_onder), z_axis_stop, RISING);
}

void check_werking_sensoren_gantry(){

  if(digitalRead(Reed_Y1_voor) == LOW) { 
      Serial.println("Y PRESSED 1");
  }
  if(digitalRead(Reed_Y2_achter) == LOW) { 
      Serial.println("Y PRESSED 2");
  }
  if(digitalRead(Reed_Z1_boven) == HIGH) { 
      Serial.println("Z PRESSED 1");
  }
  if(digitalRead(Reed_Z2_onder) == LOW) { 
      Serial.println("Z PRESSED 2");
  }
  if(digitalRead(Switch_X1_NC_links) == HIGH) { 
      Serial.println("X PRESSED 1");
  }
  if(digitalRead(Switch_X2_NC_rechts) == HIGH) { 
      Serial.println("X PRESSED 2");
  }

  int debug_pin = 0;
  if (debug_pin){
    int digitalState = digitalRead(Reed_Z2_onder); // Read the digital state of the pin (HIGH or LOW)
    Serial.print("Digital State: ");    Serial.println(digitalState);
    delay(100);
  }
  
}