//====================================================
// TITLE BLOCK
//====================================================
/*  
 *  Project: GR-LRR
 *  Author: Jacob Owens - Vestas Blades America
 *  Date: 07/23/2024
 */
//====================================================
// END TITLE BLOCK
//====================================================
//
//
//
//
//====================================================
// GENERAL LIBRARIES
//====================================================
#include <Arduino.h>
#include <mbed.h>
#include <math.h>
//====================================================
// END GENERAL LIBRARIES
//====================================================
//
//
//
//
//====================================================
// COMMUNICATION DEFINTIONS
//====================================================
#include <ArduinoJson.h>
// json packet allows us to use a 'dict' like structure in the form of "jsonPacket['prop'] = value"
StaticJsonDocument<1024> jsonPacket;
// json message is a string that contains all the info smashed together
char outgoingJsonString [1024] = "";

enum SerialMachineStates {
  STREAMING,
  WAITING,
};
SerialMachineStates serialMachineState;

volatile int serialMachineDelay;

void serialMachine(void);
void serialStream(void);
//====================================================
// END COMMUNICATION DEFINTIONS
//====================================================
//
//
//
//
//====================================================
// MOTOR DEFINITIONS
//====================================================

#include <Arduino_CAN.h>

auto motorSpeed = 0.0; // m/s from the tablet

float motorSpeed1, motorSpeed2, motorSpeed3, motorSpeed4;

auto opMotorSpeed = 0.0;
float erefs = 0.0;
float opErefs = 0.0;
float wheelDiameter = 0.048;
int cr1, cr2, cr3, cr4;


uint8_t EREFS_HEXDATA[4];  
uint8_t OP_EREFS_HEXDATA[4];  


float ms_to_erefs(float ms, float wheelDiameter);
void erefs_to_hexdata(float erefs, uint8_t hexdata[4]);

int motorIndex = 0;

volatile int motorMachineDelay;

uint32_t const M1_EREFS_ID = 0x048020A8;
uint32_t const M2_EREFS_ID = 0x048040A8;
uint32_t const M3_EREFS_ID = 0x048060A8;
uint32_t const M4_EREFS_ID = 0x048080A8;

uint32_t const MOTOR_EREFS_IDS [4] = {M1_EREFS_ID, M2_EREFS_ID, M3_EREFS_ID, M4_EREFS_ID};
float motorSpeeds[4] = {0.0, 0.0, 0.0, 0.0};


//uint8_t const EREFS_HEXDATA[] = {0x00, 0x00, 0x00, 0x00};


void motorMachine(void);

enum MotorMachineStates {
  WAITING_MOTORS,
  WRITING_MOTORS,
};

MotorMachineStates motorMachineState;





//====================================================
// END MOTOR DEFINITIONS
//====================================================



//====================================================
// ULTRASONIC SENSOR DEFINITIONS
//====================================================

//====================================================
// END ULTRASONIC SENSOR DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// BATTERY MONITOR DEFINITIONS
//====================================================

//====================================================
// END BATTERY MONITOR DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// CLAMP DEFINITIONS
//====================================================

//====================================================
// END CLAMP DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// ON BOARD LED DEFINITIONS
//====================================================
#define RED_LED     LEDR
#define GREEN_LED   LEDG
#define BLUE_LED    LEDB
bool redLedFlag = false;
bool blueLedFlag = false;
volatile int redLedDelay = 0;
volatile int blueLedDelay = 0;

enum LedStates {
  LED_OFF,
  LED_ON
};
LedStates RedLedState;
LedStates BlueLedState;

void RedLedMachine(void);
void BlueLedMachine(void);
//====================================================
// END ON BOARD LED DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// STATE LED DEFINITIONS
//====================================================

//====================================================
// END STATE LED DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// ESTOP DEFINITIONS
//====================================================

//====================================================
// END ESTOP DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// ENCODER DEFINITIONS
//====================================================

//====================================================
// END ENCODER DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// TIMER DEFINITIONS
//====================================================
bool testState = false;
volatile int testDelay = 0;


#include "Portenta_H7_TimerInterrupt.h"
volatile int interruptCounter = 0;
void m7timer() { 
  // every 1/10,000 second - 10,000hz - 0.0001 second
  interruptCounter++;

  if(serialMachineDelay) serialMachineDelay--;
  if(motorMachineDelay) motorMachineDelay--;

  if(testDelay) testDelay--;
  // every 10/10,000 second - 1,000hz - 0.001 second
  if ((interruptCounter % 10) == 0) { 
     // this can indicate if something is taking way to long?
  }

  // every 100/10,000 second - 100hz - 0.01 second
  if ((interruptCounter % 100) == 0) { 

  }

  // every 1,000/10,000 second - 10hz - 0.1 second
  if ((interruptCounter % 1000) == 0) { 
    if (redLedDelay) redLedDelay--;
  }

  // every 10,000/10,000 second - 1hz
  if ((interruptCounter % 10000) == 0) {
    if (blueLedDelay) blueLedDelay--;
    interruptCounter = 0;
  }

}
Portenta_H7_Timer M7Timer(TIM7);

//====================================================
// END TIMER DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// BEGIN SETUP FUNCTION PROTOTYPES
//====================================================
void setup(void);
void loop(void);
//====================================================
// END SETUP FUNCTION PROTOTYPES
//====================================================
//
//
//
//
//====================================================
// BEGIN SETUP
//====================================================
void setup() {

  // timer setup
  M7Timer.attachInterruptInterval(100, m7timer);
  // debug setup
  Serial.begin(115200);

  // turn red led on for initialization setup
  digitalWrite(RED_LED, LOW);
  // extra stepper pin
  delay(2000);
  digitalWrite(RED_LED, HIGH);




  // CAN SETUP
  if (!CAN.begin(CanBitRate::BR_250k))
  {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {
      Serial.println("CAN ISSUE");
      delay(1000);
    }
  }





}
//====================================================
// END SETUP
//====================================================
//
//
//
//
//====================================================
// BEGIN LOOP
//====================================================
void loop() {

  BlueLedMachine();

  serialMachine();

}
//====================================================
// END LOOP
//====================================================







//====================================================
// BEGIN FUNCTIONS
//====================================================
//====================================================
// debug test machine
//====================================================
void test() {

  if (!testDelay){
    if (testState){
      digitalWrite(D6, HIGH);    
    }
    else {
      digitalWrite(D6, LOW);  
    }

    testDelay = 1;  // sets the countdown timer to 100ms.  
    testState = !testState;
  }
}

//====================================================
// end debug test machine
//====================================================




//====================================================
// BEGIN BLUE LED MACHINE
//====================================================
void BlueLedMachine() {
  switch(BlueLedState) {
    case LED_OFF:
      if (!blueLedDelay) {
        blueLedDelay = 1;
        BlueLedState = LED_ON;
        digitalWrite(BLUE_LED, LOW);

      }
    break;
    case LED_ON:
      if (!blueLedDelay) {
        blueLedDelay = 1;
        BlueLedState = LED_OFF;
        digitalWrite(BLUE_LED, HIGH);
      }
    break;
    default:
    break;
  }
}
//====================================================
// END BLUE LED MACHINE
//====================================================


//====================================================
// BEGIN SERIAL MACHINE
//====================================================

void serialStream(void){

  
  String incomingJsonString = Serial.readStringUntil('\n');

  deserializeJson(jsonPacket, incomingJsonString);



  if (jsonPacket["type"] == "get"){
    // get messages
    jsonPacket["device"] == "h7";
  }

  if (jsonPacket["type"] == "set"){
    // set variables
    motorSpeed1 = jsonPacket["motorSpeed1"];
    motorSpeed2 = jsonPacket["motorSpeed2"];
    motorSpeed3 = jsonPacket["motorSpeed3"];
    motorSpeed4 = jsonPacket["motorSpeed4"];

  }


  serializeJson(jsonPacket, outgoingJsonString);

  Serial.write(outgoingJsonString);
  Serial.write('\n');
  

}

void serialMachine() {
  switch(serialMachineState) {
    case STREAMING:
      if (!serialMachineDelay) {

        serialStream();
        serialMachineDelay = 10; // 1khz
        serialMachineState = WAITING;

      }
    break;
    case WAITING:
      if (!serialMachineDelay) {
        serialMachineDelay = 10; // 1khz
        serialMachineState = STREAMING;
      }
    break;
    default:
    break;
  }
}
//====================================================
// END SERIAL MACHINE
//====================================================





//====================================================
// BEGIN MOTOR MACHINE
//====================================================

void erefs_to_hexdata(float input_float, uint8_t hexdata[4]) {
        uint32_t initialInt = int(round(input_float*16*16*16*16));
        // Split the number into bytes
        hexdata[3] = (initialInt >> 24) & 0xFF;  // Most significant byte
        hexdata[2] = (initialInt >> 16) & 0xFF;
        hexdata[1] = (initialInt >> 8) & 0xFF;
        hexdata[0] = initialInt & 0xFF;          // Least significant byte
}

float ms_to_erefs(float ms, float wheelDiameter) {
    // meters per second linear speed to erefs
    // wheel diameter in meters 0.048

    float angularVelocity = ms/wheelDiameter; // 0.1 m/s /m = 1/s
    float rpm = angularVelocity/0.10471975057; // convert from rads/s to rpm

    //std::cout<<"EREFS: " << rpm * 18.7187185 << "\n";
    return rpm * 18.7187185; // this was pulled from the eletrocraft setup file
}


void motorMachine(void){

  switch(motorMachineState) {

    case WAITING_MOTORS:
    if (!motorMachineDelay) {
      motorMachineDelay = 10;
      motorMachineState = WRITING_MOTORS;

    }
    break;

    case WRITING_MOTORS:

    if (!motorMachineDelay) {

      motorMachineDelay = 5;
      erefs = ms_to_erefs(motorSpeeds[motorIndex], wheelDiameter);
      erefs_to_hexdata(erefs, EREFS_HEXDATA);

      CanMsg MOTOR_SET_EREFS(CanExtendedId(MOTOR_EREFS_IDS[motorIndex]), sizeof(EREFS_HEXDATA), EREFS_HEXDATA);

      CAN.write(MOTOR_SET_EREFS);

      motorIndex++; // 0 , 1 , 2 , 3 

      if (motorIndex == 4) {
        motorIndex = 0;
        motorMachineDelay = 10;
        motorMachineState = WAITING_MOTORS;
      }
      

    }



    break;
    default:
    break;
  }


}





//====================================================
// END MOTOR MACHINE
//====================================================