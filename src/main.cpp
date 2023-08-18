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

#include <WebSockets2_Generic.h>
#include <WiFi.h>

const char* ssid = "grover1234"; //Enter SSID
const char* password = "grover1234"; //Enter Password

using namespace websockets2_generic;
//====================================================
// end wifi and websockets definitions
//====================================================




//====================================================
// ip address definitions
//====================================================
// Define both wifi server and websocket server
WiFiServer wifiServer(80);
// Since the esp is also on its own network... it has a local ip that isn't used for anything. 
IPAddress local(192, 168, 0, 254);
// throwing googles dns in for a temp fix
IPAddress testdns(8,8,8,8); 
// Since we are serving as an access point this is the address where the websockets will be posted
IPAddress gateway(192, 168, 0, 254);
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
void WebSocketMachine(void);
void onMessageCallback(WebsocketsMessage);
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
int redLedDelay = 0;
int blueLedDelay = 0;

bool wsFlag = 0;
int wsDelay = 0;

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
//====================================================
// end states
//====================================================






//====================================================
// objects
//====================================================
WebsocketsServer wsServer;
WebsocketsClient wsClient;
WebsocketsMessage wsMessage;

//====================================================
// end objects
//====================================================






int status = WL_IDLE_STATUS;

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
  }

  // every second
  if ((interruptCounter % 100000) == 0) {
    if (blueLedDelay) blueLedDelay--;
    if (wsDelay) wsDelay--;
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
  ITimer.attachInterruptInterval(10, TimerHandler);
  // debug setup
  Serial.begin(115200);

  // wifi setup
  while (!Serial && millis() < 3000);
  WiFi.config(local, testdns, gateway, nmask);
  delay(250);
  WiFi.beginAP(ssid, password);
  wifiServer.begin();
  while (WiFi.status() != WL_CONNECTED && millis() < 6000);

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // websocket setup
  wsServer.listen(8080);

  wsClient = wsServer.accept();

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
void BlueLedMachine() {
  switch(BlueLedState) {
    case LED_OFF:
      if (!blueLedDelay && !blueLedFlag) {
        //ledFlag = false;
        blueLedDelay = 1;
        BlueLedState = LED_ON;
        digitalWrite(BLUE_LED, LOW);
        Serial.println("blue led on");
      }
    break;
    case LED_ON:
      if (!blueLedDelay && !blueLedFlag) {
        //ledFlag
        blueLedDelay = 1;
        BlueLedState = LED_OFF;
        digitalWrite(BLUE_LED, HIGH);
        Serial.println("led off");
      }
    break;
    default:
    break;
  }
}

void RedLedMachine() {
  switch(RedLedState) {
    case LED_OFF:
      if (!redLedDelay && !redLedFlag) {
        //ledFlag = false;
        redLedDelay = 1;
        RedLedState = LED_ON;
        digitalWrite(RED_LED, LOW);
        Serial.println("led on");
      }
    break;
    case LED_ON:
      if (!redLedDelay && !redLedFlag) {
        //ledFlag
        redLedDelay = 1;
        RedLedState = LED_OFF;
        digitalWrite(RED_LED, HIGH);
        Serial.println("led off");
      }
    break;
    default:
    break;
  }
}


void WebSocketMachine() {  
  Serial.println("Websocket machine");
  if (wsClient.available()) { 
    Serial.println("if statement");
    WebsocketsMessage msg = wsClient.readNonBlocking();
  
    Serial.print("Got Message: ");
    Serial.println(msg.data());
    //wsClient.close();
  }

  else {
    Serial.println("else statemetn");
  wsClient = wsServer.accept();

  }


}

void onMessageCallback(WebsocketsMessage message)
{
  //Doing something with received String message.data() type
  
  Serial.print("Got Message: ");
  Serial.println(message.data());
}

//====================================================
// end functions
//====================================================