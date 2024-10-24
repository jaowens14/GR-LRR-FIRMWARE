
#include <ArduinoJson.h>
#include <Data.hpp>

class MySerial {
    public:

    // json packet allows us to use a 'dict' like structure in the form of "jsonPacket['prop'] = value"
    StaticJsonDocument<1024> jsonPacket;
    // json message is a string that contains all the info smashed together

    enum States {
      STREAMING,
      WAITING,
    };
    States state;

    volatile int delay;

    void setup(void){
        // debug setup
        Serial.begin(115200);
    }


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
          //jsonPacket["ultrasonic"] = ultrasonicDistance;

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

    void stateMachine() {
      switch(state) {
        case STREAMING:
          if (!delay) {

            serialStream();
            delay = 1; // 1khz
            state = WAITING;

          }
        break;
        case WAITING:
          if (!delay) {
            delay = 1; // 1khz
            state = STREAMING;
          }
        break;
        default:
        break;
      }
    }
};