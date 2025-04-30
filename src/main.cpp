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

//#include <Encoder.hpp>
//Encoder encoder(3, 0x6064);

//#include <Ext_Encoder.hpp>
//ExtEncoder encoder;

#include <Actuator.hpp>
ActuatorControl actuator;

#include <MySerial.hpp>
MySerial mySerial(actuator);

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

/*
 //Serial command processor for actuator test
void processSerialCommands() {
  //Use a static budder to persist data between calls.
    static String commandBuffer = "";

    //Process available characters in the serial input
    while (Serial.available() > 0) {
      char c = Serial.read();
      commandBuffer += c;
    }

    // Process every complete command (delimited by '/n') into the buffer
    int newlineIndex = commandBuffer.indexOf('\n');
    while (newlineIndex != -1) {
      //Extract one command (up to the newline) and trim whitespace.
      String currentCommand = commandBuffer.substring(0,newlineIndex);
      currentCommand.trim();

      //Remove the processed command from the buffer
      commandBuffer = commandBuffer.substring(newlineIndex + 1);

      //Skip empty commands.
      if (currentCommand.length() == 0) {
        newlineIndex = commandBuffer.indexOf('\n');
        continue;
      }

      //Ignore incoming heartbeat JSON messages
      if (currentCommand.startsWith("<{") && currentCommand.endsWith("}>")) {
        if (currentCommand.indexOf("\"hb\"") != -1 ) {
          newlineIndex = commandBuffer.indexOf('\n');
          continue;
        }
      }

        Serial.print("Received command: ");
        Serial.println(currentCommand);

        if(currentCommand.equalsIgnoreCase("help")) {
            Serial.println("Commands:");
            Serial.println(" help            - Show this help message");
            Serial.println(" state           - Print current actuator state");
            Serial.println(" sdo <node> <index>        - Send an SDO read request for the given CANopen node and object index");
            Serial.println("                            e.g., sdo 1 0x6064 (to read 'Position actual value')");
            Serial.println(" test            - Run self-test routine");
            Serial.println(" set <n> <v>     - Set actuator n (0-indexed) to voltage v (0-5V)");
        }
        else if (currentCommand.equalsIgnoreCase("state")) {
            actuator.debugOutput();
        }
        else if (currentCommand.equalsIgnoreCase("test")) {
            actuator.runSelfTest();
        }
        else if (currentCommand.startsWith("set")) {
             //Expected format: "set <actuator> <voltage>"
            int firstSpace = currentCommand.indexOf(' ');
            int secondSpace = currentCommand.indexOf(' ', firstSpace + 1);
            if (firstSpace == -1 || secondSpace == -1) {
                Serial.println("Invalid set command. Format: set <actuator> <voltage>");
            } else{
                String actStr = currentCommand.substring(firstSpace + 1, secondSpace);
                String voltStr = currentCommand.substring (secondSpace + 1);
               int actuatorNum = actStr.toInt();
                float voltage = voltStr.toFloat();

                if (actuatorNum < 0 || actuatorNum >= NUM_ACTUATORS) {
                    Serial.println("Invalid actuator number.");
                }
                else if (voltage < 0.0 || voltage > MAX_VOLTAGE) {
                    Serial.println("Voltage out of range (0-5V).");
                }
                else {
                    actuator.actuatorPositions[actuatorNum] = voltage;
                    actuator.writeDAC(actuatorNum, voltage);
                    Serial.print("Actuator ");
                    Serial.print(actuatorNum);
                    Serial.print(" manually set to ");
                    Serial.print(voltage);
                    Serial.print(" V.\n");
                }
            }
        }
        else if (currentCommand.startsWith("sdo")) {
          int firstSpace = currentCommand.indexOf(' ');
          int secondSpace = currentCommand.indexOf(' ', firstSpace + 1);
          if (firstSpace == -1 || secondSpace == -1) {
            Serial.println("Invalid sdo commannd. Format: sdo <node id> <index>");
          } else {
            String nodeIdStr = currentCommand.substring(firstSpace + 1, secondSpace);
            String indexStr = currentCommand.substring(secondSpace + 1);
            
            uint8_t nodeId = nodeIdStr.toInt();
            uint16_t index = (uint16_t)strtol(indexStr.c_str(), NULL, 0);

            // Send the SDO read request.
            Serial.print("SDO read request sent to node ");
            Serial.print(nodeId, DEC);
            Serial.print(" for index 0x");
            Serial.print(index, HEX);
          }
        }

        else {
            Serial.println("Unknown command. Type 'help' for a list of commands.");
        }

        //Move on to check the next command in the buffer.
        newlineIndex = commandBuffer.indexOf('\n');

    }
}
*/

void setup(void);
void loop(void);


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Serial Starting");

/*
  //  Serial Encoder Testing: Initialize CAN 250kbit
  if (!CAN.begin(CanBitRate::BR_250k)) {
    Serial.println("CAN.begin(...) failed!");
    while(1) {
      delay(1000);
    }
  }
  Serial.println("CAN interface Initialized.");
  Serial.println("Type 'help' for command instructions.");
  //
*/

  delay(2000);
  M7Timer.attachInterruptInterval(100, m7timer);
  mySerial.setup();
  motors.setup();
  ultrasonic.setup();
  //encoder.setup();
  actuator.setup ();

}

void loop() {
  //processSerialCommands();
  //actuator.handleSerialCommand();  
  //blueLed.stateMachine();
  mySerial.stateMachine();
  motors.stateMachine();
  ultrasonic.stateMachine();
  actuator.stateMachine();

  //encoder.stateMachine();  
  //encoder.debugOutput();

  // Encoder Serial Test:
  //while (CAN.available()) {
  //  CanMsg msg = CAN.read();
  //  encoder.dumpCanMsg(msg);
  //  encoder.processCanMessage(msg);
  //}

}
