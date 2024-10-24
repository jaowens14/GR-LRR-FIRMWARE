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
#include <BlueLed.hpp>

BlueLed blueLed;

#include <Motors.hpp>

Motors motors;


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
  if(motors.delay) motors.delay--;

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
    //if (redLedDelay) redLedDelay--;
  }

  // every 10,000/10,000 second - 1hz
  if ((interruptCounter % 10000) == 0) {
    if (blueLed.delay) blueLed.delay--;
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

  blueLed.StateMachine();

  serialMachine();
  motors.motorMachine();

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
      jsonPacket["wroteMotor0"] = motors.writeReceipts[0];    
      jsonPacket["wroteMotor1"] = motors.writeReceipts[1];
      jsonPacket["wroteMotor2"] = motors.writeReceipts[2];
      jsonPacket["wroteMotor3"] = motors.writeReceipts[3];    
      //jsonPacket["erefsMotor0"] = motors.erefValues[0];
      //jsonPacket["erefsMotor1"] = motors.erefValues[1];
      //jsonPacket["erefsMotor2"] = motors.erefValues[2];
      //jsonPacket["erefsMotor3"] = motors.erefValues[3];
      jsonPacket["ultrasonic"] = ultrasonicDistance;

    }

    if(jsonPacket["msgtyp"] == "set"){
      motors.motorSpeeds[0] = jsonPacket["motorSpeed0"];
      motors.motorSpeeds[1] = jsonPacket["motorSpeed1"];
      motors.motorSpeeds[2] = jsonPacket["motorSpeed2"];
      motors.motorSpeeds[3] = jsonPacket["motorSpeed3"];
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