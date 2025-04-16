//====================================================
// TITLE BLOCK
//====================================================
/*  
 *  Project: GR-LRR
 *  Author: Jacob Owens - Vestas Blades America
 *  Date: 07/23/2024
 * 
 *  Author: Giovanni Cordova - CREADIS
 *  Date: 03/24/2025
 */

#include <Arduino.h>
#include <mbed.h>
#include <math.h>


//#include <BlueLed.hpp>
//BlueLed blueLed;

#include <Motors.hpp>
Motors motors;

#include <Ultrasonic.hpp>
Ultrasonic ultrasonic;

#include <Ext_Encoder.hpp>
ExtEncoder encoder;

#include <MySerial.hpp>
MySerial mySerial;

#include <Actuator.hpp>
ActuatorControl actuator;

#include "Portenta_H7_TimerInterrupt.h"
volatile int interruptCounter = 0;
void m7timer() { 
  // every 1/10,000 second - 10,000hz - 0.0001 second
  interruptCounter++;

  //if(mySerial.delay) mySerial.delay--;

  if(mySerial.receiveDelay) mySerial.receiveDelay--;

  // every 10/10,000 second - 1,000hz - 0.001 second
  if ((interruptCounter % 10) == 0) { 
     // this can indicate if something is taking way to long?

      if(motors.thisDelay) motors.thisDelay--;
      if(mySerial.thisDelay) mySerial.thisDelay--;
      if(mySerial.timeout) mySerial.timeout--;
      //if(encoder.thisDelay) encoder.thisDelay--;

  }

  // every 100/10,000 second - 100hz - 0.01 second
  if ((interruptCounter % 100) == 0) { 
    if(ultrasonic.delay) ultrasonic.delay--;
  }

  // every 1,000/10,000 second - 10hz - 0.1 second
  if ((interruptCounter % 1000) == 0) { 
    //if (redLedDelay) redLedDelay--;
  }

  // every 10,000/10,000 second - 1hz
  if ((interruptCounter % 10000) == 0) {
    //if (blueLed.delay) blueLed.delay--;
    interruptCounter = 0;
  }

}
Portenta_H7_Timer M7Timer(TIM7);

// Serial command processor for actuator test
//void processSerialCommands() {
    //if (Serial.available ()) {
        //String cmd = Serial.readStringUntil('\n');
        //cmd.trim();
        //if (cmd.length() == 0) return;
        //Serial.print("Received command: ");
        //Serial.println(cmd);

        //if(cmd.equalsIgnoreCase("help")) {
            //Serial.println("Commands:");
            //Serial.println(" help         - Show this help message");
            //Serial.println(" state        - Print current actuator state");
            //Serial.println(" test         - Run self-test routine");
            //Serial.println(" set <n> <v>   - Set actuator n (0-indexed) to voltage v (0-5V)");
        //}
        //else if (cmd.equalsIgnoreCase("state")) {
           // actuator.debugOutput();
        //}
        //else if (cmd.equalsIgnoreCase("test")) {
        //    actuator.runSelfTest();
        //}
        //else if (cmd.startsWith("set")) {
            // Expected format: "set <actuator> <voltage>"
        //    int firstSpace = cmd.indexOf(' ');
        //    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
        //    if (firstSpace == -1 || secondSpace == -1) {
        //        Serial.println("Invalid set command. Fo rmat: set <actuator> <voltage>");
        //    } else{
        //        String actStr = cmd.substring(firstSpace + 1, secondSpace);
        //        String voltStr = cmd.substring (secondSpace + 1);
        //       int actuatorNum = actStr.toInt();
        //        float voltage = voltStr.toFloat();
        //        if (actuatorNum < 0 || actuatorNum >= NUM_ACTUATORS) {
        //            Serial.println("Invalid actuator number.");
        //        } else if (voltage < 0.0 || voltage > MAX_VOLTAGE) {
        //            Serial.println("Voltage out of range (0-5V).");
        //        } else {
        //            actuator.actuatorPositions[actuatorNum] = voltage;
        //            actuator.writeDAC(actuatorNum, voltage);
        //            Serial.print("Actuator ");
        //            Serial.print(actuatorNum);
        //            Serial.print(" manually set to ");
        //            Serial.print(voltage);
        //            Serial.print(" V.");
        //        }
        //    }
       // }
       // else {
       //     Serial.println("Unknown command. Type 'help' for a list of commands.");
       // }
//
//    }
//}

void setup(void);
void loop(void);


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Serial Starting");
  delay(2000);
  M7Timer.attachInterruptInterval(100, m7timer);
  mySerial.setup();
  motors.setup();
  ultrasonic.setup();
  //actuator.setup();
  encoder.setup();

}

void loop() {
  //processSerialCommands();
  
  //blueLed.stateMachine();
  mySerial.stateMachine();
  motors.stateMachine();
  ultrasonic.stateMachine();
  //actuator.stateMachine();
  encoder.stateMachine();

  // Pinout Test
 // for (int pin = 0; pin < 22; pin++) {
 //   int state = digitalRead(pin);
 //   Serial.print("Pin ");
 //   Serial.print(pin); 
 // }
  //

}
