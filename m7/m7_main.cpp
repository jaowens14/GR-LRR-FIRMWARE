//====================================================
// title block
//====================================================
/*  
 *  Author: Jacob Owens - Avid Product Development
 *  Customer: Vestas
 *  Date: 08082023
 */
//====================================================
// end title block
//====================================================


//====================================================
// shared data
//====================================================
struct shared_data {
  long stepperSpeed = 0; // buffer
  int stepperCommand = 0; // stepper state : 0 = stopped, 1 = forward, 2 = backward
};
volatile struct shared_data * const xfr_ptr = (struct shared_data *)0x38001000;
//====================================================
// end shared data
//====================================================

//====================================================
// included libraries
//====================================================
#include <Arduino.h>
//====================================================
// end included libraries
//====================================================




//====================================================
// wifi and websockets definitions
//====================================================
#define USE_WIFI_PORTENTA_H7  true
#define USE_WIFI_NINA         false
#define USE_WIFI_CUSTOM       false
#define WEBSOCKETS_USE_PORTENTA_H7_WIFI           true
#define DEBUG_WEBSOCKETS_PORT     Serial
// Debug Level from 0 to 4
#define _WEBSOCKETS_LOGLEVEL_     4
#include <WebSockets2_Generic.h>
#include <WiFi.h>
#define WEBSOCKETS_PORT     8080
#define WEBSOCKETS_HOST     "10.42.0.109"
const char* ssid = "grlrr2024"; //Enter SSID
const char* password = "grlrr2024"; //Enter Password
using namespace websockets2_generic;
#define HEARTBEAT_INTERVAL      300000 // 5 Minutes
uint64_t heartbeatTimestamp     = 0;
uint64_t now                    = 0;
bool wsConnected = false;
bool wsFlag = 0;
volatile int wsReconnectDelay = 0;
volatile int wsDelay = 0;

byte mac[6];

//====================================================
// end wifi and websockets definitions
//====================================================




//====================================================
// stepper definitions
//====================================================
//#include <AccelStepper.h>
//#define STEPPER_STEP_PIN 4
//#define STEPPER_DIRECTION_PIN 5
//AccelStepper stepper1(AccelStepper::FULL2WIRE, STEPPER_STEP_PIN, STEPPER_DIRECTION_PIN);
//int stepper1Position = 0;
int stepperCommand = 0;
int speedMode = 0; // 0 set speed, 1 auto speed
long stepperSpeed = 0;
//====================================================
// end stepper definitions
//====================================================





//====================================================
// ultrasonic definitions
//====================================================
#define ULTRASONIC_PIN A0
long duration = 0; 
long distance = 0;
long ultrasonic_value = 0;
volatile int ultrasonicDelay = 0;
bool ultrasonicFlag = 0;
long current_time = 0;
//====================================================
// end ultrasonic definitions
//====================================================


//====================================================
// led definitions
//====================================================
#define RED_LED     LEDR
#define GREEN_LED   LEDG
#define BLUE_LED    LEDB
bool redLedFlag = false;
bool blueLedFlag = false;
volatile int redLedDelay = 0;
volatile int blueLedDelay = 0;
//====================================================
// end led definitions
//====================================================




//====================================================
// relay definitions
//====================================================
#define RELAY_PIN 1
bool relayFlag = false;
volatile int relayDelay = 0;
bool estop = false;
//====================================================
// end relay definitions
//====================================================




//====================================================
// json definitions
//====================================================
#include <ArduinoJson.h>
// json packet allows us to use a 'dict' like structure in the form of "jsonPacket['prop'] = value"
StaticJsonDocument<256> jsonPacket;
// json message is a string that contains all the info smashed together
String jsonMessage = "";
//====================================================
// end json definitions
//====================================================




//====================================================
// ip address definitions
//====================================================
// Since the esp is also on its own network... it has a local ip that isn't used for anything. 
IPAddress local(10, 42, 0, 135);
// throwing googles dns in for a temp fix
IPAddress testdns(8,8,8,8); 
// Since we are serving as an access point this is the address where the websockets will be posted
IPAddress gateway(10, 42, 0, 1);
// These are set to the same just for ease of use. 
IPAddress nmask(255, 255, 255, 0);
//====================================================
// end ip address definitions
//====================================================




//====================================================
// function prototypes
//====================================================
void setup(void);
void loop(void);
void RedLedMachine(void);
void BlueLedMachine(void);
void UltrasonicMachine(void);
void RelayMachine(void);
void StepperSpeedMachine(void);
void WebSocketMachine(void);
void onMessageCallback(WebsocketsMessage);
void onEventsCallback(WebsocketsEvent, String);
void sendMessage(void);
void checkToSendMessage(void);
void ReceiveJsonMachine(void);
void SendJsonMachine(void);
void CommandToEnumState(void);
long microsecondsToCentimeters(long);
//====================================================
// end function prototypes
//====================================================




//====================================================
// states
//====================================================
enum LedStates {
  LED_OFF,
  LED_ON
};
LedStates RedLedState;
LedStates BlueLedState;

enum wsStates { 
  WS_DISCONNECTED,
  WS_CONNECTED
};
wsStates wsState;

enum SpeedStates { 
  SET_MODE,
  AUTO_MODE
};
SpeedStates speedState;


enum UltrasonicStates {
  UT_WAITING,
  UT_READING
};
UltrasonicStates UltrasonicState;

enum RelayStates {
  RELAY_OFF,
  RELAY_ON
};
RelayStates RelayState;

//====================================================
// end states
//====================================================



//====================================================
// objects
//====================================================
WebsocketsServer wsServer;
WebsocketsMessage wsMessage;
WebsocketsClient wsClient;
//====================================================
// end objects
//====================================================




//====================================================
// timer
//====================================================
#include "Portenta_H7_TimerInterrupt.h"
volatile int interruptCounter;
void TimerHandler() { 
  interruptCounter++;

    // every 1/1000 second
  if ((interruptCounter % 100) == 0) {
    if (ultrasonicDelay) ultrasonicDelay--;
  }

    // every 1/10 second
  if ((interruptCounter % 10000) == 0) {
    if (redLedDelay) redLedDelay--;
    if (wsReconnectDelay) wsReconnectDelay--;
    if (relayDelay) relayDelay--;
    if (wsDelay) wsDelay--;
  }

  // every second
  if ((interruptCounter % 100000) == 0) {
    if (blueLedDelay) blueLedDelay--;
    interruptCounter = 0;
  }

}
Portenta_H7_Timer ITimer(TIM16);
//====================================================
// end timer
//====================================================




//====================================================
// setup
//====================================================
void setup() {
  digitalWrite(RED_LED, LOW);

  // initialize m4 core
  bootM4();

  // timer setup
  ITimer.attachInterruptInterval(10, TimerHandler); // is thje timer messing with timeout of the websockets connection??????????
  // debug setup
  Serial.begin(115200);

  // wifi setup
  while (!Serial && millis() < 2000);

  

  WiFi.begin(ssid, password);

  delay(500);
  WiFi.config(local, testdns, gateway, nmask);
  delay(500);

  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    WiFi.begin(ssid, password);
    delay(1000);
    WiFi.config(local, testdns, gateway, nmask);
    delay(1000);
    Serial.println("trying to connect");
  }
  digitalWrite(RED_LED, HIGH);

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Mac address: ");
  
  WiFi.macAddress(mac);

  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);
  // end wifi setup


  // websocket setup
  wsServer.listen(WEBSOCKETS_PORT);
  Serial.println("websocket server listening...");
  // end websocket setup


  // stepper setup
  //stepper1.setMaxSpeed(10000.0);
  //stepper1.setAcceleration(750000.0);
  // end stepper setup

  // relay setup
  RelayState = RELAY_ON;


  // ultrasonic setup
  analogReadResolution(16);
  pinMode(ULTRASONIC_PIN, INPUT);

}
//====================================================
// end setup
//====================================================




//====================================================
// main loop
//====================================================
void loop() {

  BlueLedMachine();
  //RedLedMachine();
  WebSocketMachine();
  UltrasonicMachine();
  StepperSpeedMachine();
  RelayMachine();
}
//====================================================
// end main loop
//====================================================




//====================================================
// functions
//====================================================
//====================================================
// blue led machine
//====================================================
void BlueLedMachine() {
  switch(BlueLedState) {
    case LED_OFF:
      if (!blueLedDelay && !blueLedFlag) {
        //ledFlag = false;
        blueLedDelay = 1;
        BlueLedState = LED_ON;
        digitalWrite(BLUE_LED, LOW);
        //Serial.println(String(millis()/1000.0/60.0));
      }
    break;
    case LED_ON:
      if (!blueLedDelay && !blueLedFlag) {
        //ledFlag
        blueLedDelay = 1;
        BlueLedState = LED_OFF;
        digitalWrite(BLUE_LED, HIGH);
        //Serial.println(String(millis()/1000.0/60.0));
      }
    break;
    default:
    break;
  }
}
//====================================================
// end blue led machine
//====================================================




//====================================================
// red led machine
//====================================================
void RedLedMachine() {
  switch(RedLedState) {
    case LED_OFF:
      if (!redLedDelay && !redLedFlag) {
        //ledFlag = false;
        redLedDelay = 1;
        RedLedState = LED_ON;
        digitalWrite(RED_LED, LOW);
        //Serial.println("led on");
      }
    break;
    case LED_ON:
      if (!redLedDelay && !redLedFlag) {
        //ledFlag
        redLedDelay = 1;
        RedLedState = LED_OFF;
        digitalWrite(RED_LED, HIGH);
        //Serial.println("led off");
      }
    break;
    default:
    break;
  }
}
//====================================================
// end red led machine
//====================================================




//====================================================
// ultrasonic machine
//====================================================
void UltrasonicMachine() {
  switch(UltrasonicState){
    case UT_WAITING:
      if (!ultrasonicDelay) {
        UltrasonicState = UT_READING;
      }
    break;
    case UT_READING:
      ultrasonic_value = (analogRead(ULTRASONIC_PIN) + ultrasonic_value)/2.0;
      ultrasonicDelay = 40;
      UltrasonicState = UT_WAITING;
    break;
    default:
    break;
  }

}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}
//====================================================
// end ultrasonic machine
//====================================================

//====================================================
// stepper speed machine
//====================================================
void StepperSpeedMachine(void) {
  switch(speedState) {
    case SET_MODE:
      if (speedMode) {
        Serial.println("Going to auto mode");
        speedState = AUTO_MODE;
      }

    break;
    case AUTO_MODE:
      if (!speedMode) {
        Serial.println("Going to set mode");
        speedState = SET_MODE;
        stepperSpeed = 0;
        xfr_ptr->stepperSpeed = stepperSpeed;

      }
      stepperSpeed = map(ultrasonic_value, 0, 20000, 0, 22000);
      xfr_ptr->stepperSpeed = stepperSpeed;

    break;
    default:
    break;
  }

}
//====================================================
// end stepper speed machine
//====================================================


//====================================================
// relay machine
//====================================================
void RelayMachine(void){
  switch(RelayState) {
    case RELAY_OFF: // relay is off waiting for connection
      digitalWrite(RELAY_PIN, LOW);
      if (wsConnected && !estop) {
        RelayState = RELAY_ON;
        relayFlag = 1;
      }
    break;
    case RELAY_ON: // relay is on and we are working
      digitalWrite(RELAY_PIN, HIGH);
      if (!wsConnected || estop) {
        RelayState = RELAY_OFF;
        relayFlag = 0;
      }
    break;
    default:
    break;


  }
}
//====================================================
// end relay machine
//====================================================

//====================================================
// stepper machine
//====================================================
//void StepperMachine(void) {
//  switch(stepperState) {
//    case STEPPER_STOPPED:
//      stepper1.stop();
//    break;
//    case STEPPER_FORWARD:
//      stepper1.move(10);
//    break;
//    case STEPPER_BACKWARD:
//      stepper1.move(-10);
//    break;
//    default:
//    break;
//  }
//  stepper1.run();
//
//}
//====================================================
// end stepper machine
//====================================================




//====================================================
// websocket machine
//====================================================
void WebSocketMachine() {

// the websocket machine here has two states
// it can be connected or not
// if connected it needs to send and receive messages
// if not connected it needs to tell us and try to get connected
  switch(wsState){
    case WS_DISCONNECTED:
      if (!wsReconnectDelay) {
        Serial.println("accepting new");
        delay(1000);
        wsClient.close();
        wsClient = wsServer.accept();
        wsConnected = wsClient.available();

        if (wsConnected){
          // register callback when messages are received
          wsClient.onMessage(onMessageCallback);
          // register callback when events are occuring          
          wsClient.onEvent(onEventsCallback);
          // 

          // change state to connected
          wsState = WS_CONNECTED;

          Serial.println("Connected!");
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(RED_LED, HIGH);

        }
        wsReconnectDelay = 5;
      }
    break;
    case WS_CONNECTED:
      if (!wsClient.available()) {
        wsState = WS_DISCONNECTED;
        Serial.println("Disconnected!");
        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(RED_LED, LOW);
      }

      if (!wsDelay && wsClient.available()) {
        Serial.println("Sent");
        wsClient.ping();    

        wsDelay = 5;
      }
      
      wsClient.poll();

    break;
    default:
    break;
  }


}




void onMessageCallback(WebsocketsMessage message) {
    Serial.print("Got Message: ");
    Serial.println(message.data());
    // save string message to global variable 
    jsonMessage = message.data();
    ReceiveJsonMachine();
  }

void onEventsCallback(WebsocketsEvent event, String data) {
  (void) data;
  
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("Connnection Opened");
  } 

  else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println(String(millis()/1000.0/60.0));
    Serial.println("Connnection Closed");
    Serial.println(wsClient.getCloseReason());
    //delay(2000);
  }

  else if (event == WebsocketsEvent::GotPing) {
    //Serial.println("Got a Ping!");
    //Serial.println(ultrasonic_value);
    SendJsonMachine();
    wsClient.send(jsonMessage);

  }

  else if (event == WebsocketsEvent::GotPong) {
    //Serial.println("Got a Pong!");
    //SendJsonMachine();
    //wsClient.send(jsonMessage);
  }
}
//====================================================
// end websocket machine
//====================================================


//====================================================
// send json machine
//====================================================
void SendJsonMachine(void) {
  jsonPacket.clear();
  jsonMessage = "";
  // send back the global var
  jsonPacket["stepper_speed"] = stepperSpeed;
  // send back the global var
  jsonPacket["stepper_command"] = stepperCommand;
  jsonPacket["ultrasonic_value"] = ultrasonic_value;
  jsonPacket["relay_flag"] = relayFlag;
  serializeJson(jsonPacket, jsonMessage);
}
//====================================================
// end send json machine
//====================================================


//====================================================
// receive json machine
//====================================================
void ReceiveJsonMachine(void) {
    DeserializationError error = deserializeJson(jsonPacket, jsonMessage);
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
  }

  speedMode = bool(jsonPacket["stepper_mode"]);

  // get the value from the tablet packet, global var
  stepperSpeed = long(jsonPacket["stepper_speed"]);
  // set the value in the shared data for the other core
  xfr_ptr->stepperSpeed = stepperSpeed;

  //stepper1.setMaxSpeed(stepperSpeed);

  // get the value from the tablet packet, global var
  stepperCommand = int(jsonPacket["stepper_command"]);
  // set the value in the shared data for the other core
  xfr_ptr->stepperCommand = stepperCommand;

  estop = bool(jsonPacket["relay_flag"]);
  //CommandToEnumState();
  jsonPacket.clear();
  jsonMessage = "";
}
//====================================================
// end receive json machine
//====================================================



//====================================================
// stepper command string to enum state
//====================================================
//void CommandToEnumState(void) {
//  Serial.println("Got command: ");
//  Serial.println(stepperCommand);
//  switch(stepperCommand) {
//      case 0:
//        stepperState = STEPPER_STOPPED;
//        xfr_ptr->stepperCommand = 0;
//      break;
//      case 1:
//        stepperState = STEPPER_FORWARD;
//        xfr_ptr->stepperCommand = 1;
//
//      break;
//      case 2:
//        stepperState = STEPPER_BACKWARD;
//        xfr_ptr->stepperCommand = 2;
//
//      break;
//      default:
//      break;
//    }
//  }
////====================================================
// end stepper command string to enum state
//====================================================


//====================================================
// end functions
//====================================================