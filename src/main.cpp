/*  Author: Jacob Owens - Avid Product Development
 *  Customer: Vestas
 *  Date: 08082023
 */




//====================================================
// included libraries
//====================================================
#include <Arduino.h>
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

WebsocketsServer websocketServer;
//====================================================


//====================================================
// function prototypes
//====================================================
void setup(void);
void loop(void);
void RedLedMachine(void);
void BlueLedMachine(void);
//====================================================


//====================================================
// pin definitions
//====================================================
#define RED_LED     LEDR
#define GREEN_LED   LEDG
#define BLUE_LED    LEDB
//====================================================



//====================================================
// global variables
//====================================================
bool redLedFlag = false;
bool blueLedFlag = false;
int redLedDelay = 0;
int blueLedDelay = 0;

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
//====================================================


//====================================================
// objects
//====================================================

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
    interruptCounter = 0;
  }

}

Portenta_H7_Timer ITimer(TIM16);
//====================================================

void setup() {

  ITimer.attachInterruptInterval(10, TimerHandler);

  Serial.begin(115200);
  while (!Serial && millis() < 5000);

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true) {
      digitalWrite(BLUE_LED, LED_ON);
      digitalWrite(RED_LED, LED_OFF);
      delay(200);
      digitalWrite(BLUE_LED, LED_OFF);
      digitalWrite(RED_LED, LED_ON);
    }
  }

  WiFi.config(local, testdns, gateway, nmask);
  delay(250);
  WiFi.beginAP(ssid, password);
  wifiServer.begin();

  for(int i = 0; i < 5 && WiFi.status() != WL_CONNECTED; i++) {
      Serial.print(".");
      delay(1000);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());   //You can get IP address assigned to ESP

  websocketServer.listen(8080);
  Serial.print("Is server live? ");
  Serial.println(websocketServer.available());


}

void loop() {

  BlueLedMachine();
  RedLedMachine();
}

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