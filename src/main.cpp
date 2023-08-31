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
#define WEBSOCKETS_HOST     "192.168.3.1"
const char* ssid = "GR-LRR_POC"; //Enter SSID
const char* password = "GR-LRR_POC"; //Enter Password
using namespace websockets2_generic;
#define HEARTBEAT_INTERVAL      300000 // 5 Minutes
uint64_t heartbeatTimestamp     = 0;
uint64_t now                    = 0;
bool wsConnected = false;
bool wsFlag = 0;
volatile int wsDelay = 0;
//====================================================
// end wifi and websockets definitions
//====================================================




//====================================================
// stepper definitions
//====================================================
#include <AccelStepper.h>
#define STEPPER_STEP_PIN 4
#define STEPPER_DIRECTION_PIN 5
AccelStepper stepper1(AccelStepper::FULL2WIRE, STEPPER_STEP_PIN, STEPPER_DIRECTION_PIN);
int stepper1Position = 0;
int stepperCommand = 0;
long stepperSpeed = 0;
//====================================================
// end stepper definitions
//====================================================





//====================================================
// ultrasonic definitions
//====================================================
#define ULTRASONIC_PIN A6
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
#define RELAY_PIN 2
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
IPAddress local(192, 168, 3, 1);
// throwing googles dns in for a temp fix
IPAddress testdns(8,8,8,8); 
// Since we are serving as an access point this is the address where the websockets will be posted
IPAddress gateway(192, 168, 3, 1);
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
void StepperMachine(void);
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

enum StepperStates { 
  STEPPER_STOPPED,
  STEPPER_FORWARD,
  STEPPER_BACKWARD
};
StepperStates stepperState;


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
    if (wsDelay) wsDelay--;
    if (relayDelay) relayDelay--;
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
  // timer setup
  ITimer.attachInterruptInterval(10, TimerHandler); // is thje timer messing with timeout of the websockets connection??????????
  // debug setup
  Serial.begin(115200);

  // wifi setup
  while (!Serial && millis() < 2000);
  WiFi.config(local, testdns, gateway, nmask);
  delay(500);
  WiFi.beginAP(ssid, password);
  while (WiFi.status() != WL_CONNECTED && millis() < 5000);
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  // end wifi setup


  // websocket setup
  wsServer.listen(WEBSOCKETS_PORT);
  Serial.println("websocket server listening...");
  // end websocket setup


  // stepper setup
  stepper1.setMaxSpeed(10000.0);
  stepper1.setAcceleration(750000.0);
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
  StepperMachine();
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
        Serial.println(String(millis()/1000.0));
      }
    break;
    case LED_ON:
      if (!blueLedDelay && !blueLedFlag) {
        //ledFlag
        blueLedDelay = 1;
        BlueLedState = LED_OFF;
        digitalWrite(BLUE_LED, HIGH);
        Serial.println(String(millis()/1000.0));
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
      ultrasonic_value = analogRead(ULTRASONIC_PIN);
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
void StepperMachine(void) {
  switch(stepperState) {
    case STEPPER_STOPPED:
      stepper1.stop();
    break;
    case STEPPER_FORWARD:
      stepper1.move(10);
    break;
    case STEPPER_BACKWARD:
      stepper1.move(-10);
    break;
    default:
    break;
  }
  stepper1.run();

}
//====================================================
// end stepper machine
//====================================================




//====================================================
// websocket machine
//====================================================
void WebSocketMachine() {

  wsConnected = wsClient.available();
  checkToSendMessage();
  
  // let the websockets client check for incoming messages
  if (wsClient.available()) {

    wsClient.poll();
    now = millis();

    // Send heartbeat in order to avoid disconnections during ISP resetting IPs over night. Thanks @MacSass
    if ((now - heartbeatTimestamp) > HEARTBEAT_INTERVAL) {
      heartbeatTimestamp = now;
      wsClient.send("H");
    }
  }

}

void checkToSendMessage() {
  #define REPEAT_INTERVAL    1000L
  static unsigned long checkstatus_timeout = 1000;
  // Send WebSockets message every REPEAT_INTERVAL (10) seconds.
  if (millis() > checkstatus_timeout) {
    sendMessage();
    checkstatus_timeout = millis() + REPEAT_INTERVAL;
  }
}

void sendMessage() {
  // try to connect to Websockets server
  if (!wsConnected) {
    wsClient = wsServer.accept();
    wsConnected = wsClient.available();
    // run callback when messages are received
    wsClient.onMessage(onMessageCallback);
    // run callback when events are occuring
    wsClient.onEvent(onEventsCallback);
  }
  
  if (wsConnected) {
    Serial.println("Connected!");
    digitalWrite(GREEN_LED, LOW);
  } 

  else {
    digitalWrite(GREEN_LED, HIGH);
    Serial.println("Not Connected!");
    //Serial.println(wsConnected);
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
    Serial.println("Connnection Closed");
    Serial.println(wsClient.getCloseReason());
    delay(2000);
  }

  else if (event == WebsocketsEvent::GotPing) {
    //Serial.println("Got a Ping!");
    Serial.println(ultrasonic_value);
    SendJsonMachine();
    wsClient.send(jsonMessage);

  }

  else if (event == WebsocketsEvent::GotPong) {
    Serial.println("Got a Pong!");
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
  jsonPacket["stepper_speed"] = stepperSpeed;
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
  stepperSpeed = long(jsonPacket["stepper_speed"]);
  stepper1.setMaxSpeed(stepperSpeed);
  stepperCommand = int(jsonPacket["stepper_command"]);
  estop = bool(jsonPacket["relay_flag"]);
  CommandToEnumState();
  jsonPacket.clear();
  jsonMessage = "";
}
//====================================================
// end receive json machine
//====================================================



//====================================================
// stepper command string to enum state
//====================================================
void CommandToEnumState(void) {
  Serial.println("Got command: ");
  Serial.println(stepperCommand);
  switch(stepperCommand) {
      case 0:
        stepperState = STEPPER_STOPPED;
      break;
      case 1:
        stepperState = STEPPER_FORWARD;
      break;
      case 2:
        stepperState = STEPPER_BACKWARD;
      break;
      default:
      break;
    }
  }
//====================================================
// end stepper command string to enum state
//====================================================


//====================================================
// end functions
//====================================================
