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
#include <blueLedModule.cpp>

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
//char outgoingJsonString [1024] = "";

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

float erefValues[4];

float ms_to_erefs(float ms, float wheelDiameter);
void erefs_to_hexdata(float erefs, uint8_t hexdata[4]);

int motorIndex = 0;

volatile int motorMachineDelay;

uint32_t const M1_EREFS_ID = 0x048020A8;
uint32_t const M2_EREFS_ID = 0x048040A8;
uint32_t const M3_EREFS_ID = 0x048060A8;
uint32_t const M4_EREFS_ID = 0x048080A8;
uint32_t const MD_EREFS_ID = 0x049FE0A8; // default ID

uint32_t const MOTOR_EREFS_IDS [4] = {M1_EREFS_ID, M2_EREFS_ID, M3_EREFS_ID, M4_EREFS_ID};
float motorSpeeds[4] = {0.0, 0.0, 0.0, 0.0};
int writeReceipts[4] = {0, 0, 0, 0};

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

void ultrasonicMachine(void);

enum UltrasonicMachineStates {
  WAITING_ULTRASONIC,
  WRITING_ULTRASONIC,
};

UltrasonicMachineStates UltrasonicMachineState;


#define ULTRASONIC_PIN A2
volatile int ultrasonicDelay = 0;
float ultrasonicDistance = 0;
const int numUltrasonicSamples = 50;
double ultrasonicSamples[numUltrasonicSamples] = {0};

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
    if(ultrasonicDelay) ultrasonicDelay--;
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
      Serial.write("CAN ISSUE");
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
  motorMachine();
  //serialStream();
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

  if (Serial.available()){

    char outgoingJsonString[512];
    String incomingJsonString = Serial.readStringUntil('\n');

    deserializeJson(jsonPacket, incomingJsonString);

    if (jsonPacket["msgtyp"] == "get"){
      // get messages
      jsonPacket["device"] = "h7";
      jsonPacket["wroteMotor0"] = writeReceipts[0];    
      jsonPacket["wroteMotor1"] = writeReceipts[1];
      jsonPacket["wroteMotor2"] = writeReceipts[2];
      jsonPacket["wroteMotor3"] = writeReceipts[3];    
      jsonPacket["erefsMotor0"] = erefValues[0];
      jsonPacket["erefsMotor1"] = erefValues[1];
      jsonPacket["erefsMotor2"] = erefValues[2];
      jsonPacket["erefsMotor3"] = erefValues[3];
      jsonPacket["ultrasonic"] = ultrasonicDistance;

    }

    if(jsonPacket["msgtyp"] == "set"){
      motorSpeeds[0] = jsonPacket["motorSpeed0"];
      motorSpeeds[1] = jsonPacket["motorSpeed1"];
      motorSpeeds[2] = jsonPacket["motorSpeed2"];
      motorSpeeds[3] = jsonPacket["motorSpeed3"];
    }


    serializeJson(jsonPacket, outgoingJsonString);
    Serial.write(outgoingJsonString);
    Serial.write('\n');
  
  }

}

void serialMachine() {
  switch(serialMachineState) {
    case STREAMING:
      if (!serialMachineDelay) {

        serialStream();
        serialMachineDelay = 1; // 1khz
        serialMachineState = WAITING;

      }
    break;
    case WAITING:
      if (!serialMachineDelay) {
        serialMachineDelay = 1; // 1khz
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
      motorMachineDelay = 10;

      erefs = ms_to_erefs(motorSpeeds[motorIndex], wheelDiameter); // convert to erefs

      erefs_to_hexdata(erefs, EREFS_HEXDATA); // convert to hexdata

      CanMsg MOTOR_SET_EREFS(CanExtendedId(MOTOR_EREFS_IDS[motorIndex]), sizeof(EREFS_HEXDATA), EREFS_HEXDATA); // to can message

      writeReceipts[motorIndex] = CAN.write(MOTOR_SET_EREFS);

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
//
//
//
//
//====================================================
// BEGIN ULTRASONIC MACHINE
//====================================================
void UltrasonicMachine() {
  switch(UltrasonicMachineState){
    case WAITING_ULTRASONIC:
      if (!ultrasonicDelay) {
        UltrasonicMachineState = WRITING_ULTRASONIC;

      }
    break;
    case WRITING_ULTRASONIC:
      if (!ultrasonicDelay) {
        
        // 1. float() to change analogRead() units
        // 2. times 3.1 to scale to max ADC voltage for Portenta
        // 3. divide by 4096 to convert from the ADC precision 
        // 4. times 5.0 to scale to original battery voltage - this comes from the voltage divider on the board
        // leaky integrator over rolling ave, gain of 0.1
        ultrasonicDistance += ((float(analogRead(ULTRASONIC_PIN)) * 3.1 / 4096.0) * (5.0) - ultrasonicDistance) * 0.1;
        ultrasonicDelay = 25; // 0.25 seconds
        UltrasonicMachineState = WAITING_ULTRASONIC;
      }

    break;
    default:
    break;
  }

}

//====================================================
// END ULTRASONIC MACHINE
//====================================================