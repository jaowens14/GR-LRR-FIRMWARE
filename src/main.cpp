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
#define _WEBSOCKETS_LOGLEVEL_     3
#include <WebSockets2_Generic.h>
#include <WiFi.h>

#define WEBSOCKETS_PORT     8080
#define WEBSOCKETS_HOST     "192.168.3.1"
const char* ssid = "GR-LRR_POC"; //Enter SSID
const char* password = "GR-LRR_POC"; //Enter Password

using namespace websockets2_generic;
//====================================================
// end wifi and websockets definitions
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
void StepperMachine(void);
void WebSocketMachine(void);
void onMessageCallback(WebsocketsMessage);
void onEventsCallback(WebsocketsEvent, String);
void sendMessage(void);
void checkToSendMessage(void);

//====================================================
// end function prototypes
//====================================================


//====================================================
// pin definitions
//====================================================
#define RED_LED     LEDR
#define GREEN_LED   LEDG
#define BLUE_LED    LEDB
//====================================================
// end pin definitions
//====================================================


//====================================================
// global variables
//====================================================
bool redLedFlag = false;
bool blueLedFlag = false;
volatile int redLedDelay = 0;
volatile int blueLedDelay = 0;

bool wsFlag = 0;
volatile int wsDelay = 0;


#define HEARTBEAT_INTERVAL      300000 // 5 Minutes

uint64_t heartbeatTimestamp     = 0;
uint64_t now                    = 0;
bool connected = false;

//====================================================
// end global variables
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
  STEPPER_FORWARD,
  STEPPER_BACKWARD,
  STEPPER_STOPPED
};
StepperStates stepperState;

//====================================================
// end states
//====================================================



//====================================================
// objects
//====================================================
WebsocketsServer wsServer;
WebsocketsMessage wsMessage;
WebsocketsClient wsClient;

//Vector<WebsocketsClient> allClients;

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

    // every 1/10 second
  if ((interruptCounter % 10000) == 0) {
        if (redLedDelay) redLedDelay--;
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
  // timer setup
  ITimer.attachInterruptInterval(10, TimerHandler); // is thje timer messing with timeout of the websockets connection??????????
  // debug setup
  Serial.begin(115200);

  // wifi setup
  while (!Serial && millis() < 3000);
  WiFi.config(local, testdns, gateway, nmask);
  delay(500);
  WiFi.beginAP(ssid, password);
  while (WiFi.status() != WL_CONNECTED && millis() < 10000);

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // websocket setup
  wsServer.listen(WEBSOCKETS_PORT);
  Serial.println("websocket server listening...");

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
// websocket machine
//====================================================

void WebSocketMachine() {

  connected = wsClient.available();
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



void onMessageCallback(WebsocketsMessage message) {
    Serial.print("Got Message: ");
    Serial.println(message.data());
  }

void onEventsCallback(WebsocketsEvent event, String data) {
  (void) data;
  
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("Connnection Opened");
  } 

  else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("Connnection Closed");
  }

  else if (event == WebsocketsEvent::GotPing) {
    Serial.println("Got a Ping!");
  }

  else if (event == WebsocketsEvent::GotPong) {
    Serial.println("Got a Pong!");
  }
}

void sendMessage() {
  // try to connect to Websockets server
  if (!connected) {
    wsClient = wsServer.accept();
    connected = wsClient.available();
    // run callback when messages are received
    wsClient.onMessage(onMessageCallback);
    // run callback when events are occuring
    wsClient.onEvent(onEventsCallback);
  }
  
  if (connected) {
    Serial.println("Connected!");
    String WS_msg = String("Hello to Server from ") + BOARD_NAME;
    wsClient.send(WS_msg);
  } 

  else {
    Serial.println("Not Connected!");
    Serial.println(connected);
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
//====================================================
// websocket machine
//====================================================

//====================================================
// end functions
//====================================================
