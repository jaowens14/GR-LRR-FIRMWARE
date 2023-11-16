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

const uint16_t websockets_server_port = WEBSOCKETS_PORT;
const char* websockets_server_host = "10.42.0.109";

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
#include <SPI.h>
#include <HighPowerStepperDriver.h>

const uint8_t CSPin = D13;
HighPowerStepperDriver sd;

int stepperCommand = 0;
int speedMode = 0; // 0 set speed, 1 auto speed
long stepperSpeed = 0;
volatile int stepperDelay = 0;
long milliamps = 1500;

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

double aveUltrasonicValue = 0;
const int numUltrasonicSamples = 20;
long ultrasonicSamples[numUltrasonicSamples] = {0};
int ultrasonicSampleNumber = 0;
//====================================================
// end ultrasonic definitions
//====================================================


//====================================================
// PID definitions
//====================================================
double setpoint = 4000;
double Kp = 0.1;
double Ki = 0;
double Kd = 0;

double last_error = 0;
double current_error = 0;
double changeError = 0;
double totalError = 0;
double pidTerm = 0;
double pidTerm_scaled = 0;

//====================================================
// end PID definitions
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
StaticJsonDocument<512> jsonPacket;
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
void StepperMachine(void);
void ReceiveJsonMachine(void);
void SendJsonMachine(void);
void CommandToEnumState(void);
long microsecondsToCentimeters(long);
void StepperPID(void);
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

enum StepperStates {
  OFF,
  INIT,
  RUN,
  ERR
};
StepperStates StepperState;


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
    if (stepperDelay) stepperDelay--;
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

  // spi setup for steppers:
  SPI.begin();

  sd.setChipSelectPin(CSPin);
  // Give the driver some time to power up.
  delay(10);
  // Reset the driver to its default settings and clear latched status
  // conditions.
  sd.resetSettings();
  sd.clearStatus();
  // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode
  // for most applications, and we find that it usually works well.
  sd.setDecayMode(HPSDDecayMode::Slow);
  // Set the current limit. You should change the number here to an appropriate
  // value for your particular system.
  sd.setCurrentMilliamps36v4(milliamps);

  // Set the number of microsteps that correspond to one full step.
  sd.setStepMode(HPSDStepMode::MicroStep1);
  // Enable the motor outputs.
  //sd.enableDriver();

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
  wsServer.listen(websockets_server_port);
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
      ultrasonic_value = analogRead(ULTRASONIC_PIN);
      ultrasonicSamples[ultrasonicSampleNumber++] = ultrasonic_value;

      if (ultrasonicSampleNumber >= numUltrasonicSamples) {ultrasonicSampleNumber = 0;}

      aveUltrasonicValue = 0;

      for(int i=0; i< numUltrasonicSamples; ++i){aveUltrasonicValue += ultrasonicSamples[i];}
      aveUltrasonicValue /= numUltrasonicSamples;

      ultrasonicDelay = 50; // 50/1000 seconds
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
      StepperPID();
      xfr_ptr->stepperSpeed = stepperSpeed;

    break;
    default:
    break;
  }

}

void StepperPID(void) {
  current_error= setpoint - aveUltrasonicValue;
  
  changeError = current_error - last_error; // derivative term
  totalError += current_error; //accumalate errors to find integral term
  pidTerm = (Kp * current_error) + (Ki * totalError) + (Kd * changeError);//total gain
  pidTerm = constrain(pidTerm, -10000, 10000);//constraining to appropriate value
  pidTerm_scaled = abs(pidTerm);//make sure it's a positive value

  stepperSpeed = pidTerm_scaled;

  last_error = current_error;
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
// websocket machine
//====================================================
void WebSocketMachine() {
wsClient.poll();
// the websocket machine here has two states
// it can be connected or not
// if connected it needs to send and receive messages
// if not connected it needs to tell us and try to get connected
  switch(wsState){
    case WS_DISCONNECTED:
      if (!wsReconnectDelay) {
        Serial.println("accepting new");
        //delay(1000);
        wsClient.close();
        wsClient = wsServer.accept();
        wsConnected = wsClient.available();

        if (wsConnected){
          // register callback when messages are received
          wsClient.onMessage(onMessageCallback);
          // register callback when events are occuring          
          wsClient.onEvent(onEventsCallback);
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
        wsDelay = 2; // 2/10 seconds
        SendJsonMachine();
        wsClient.send(jsonMessage);
        //Serial.println("Just send a message");
      }
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
    Serial.println(String(millis()/1000));
    Serial.println("Connnection Closed");
    Serial.println(wsClient.getCloseReason());
    //delay(2000);
  }

  else if (event == WebsocketsEvent::GotPing) {
    Serial.println("Got a Ping!");
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
  // send back the global var
  jsonPacket["stepper_speed"]    = stepperSpeed;
  // send back the global var
  jsonPacket["stepper_command"]  = stepperCommand;
  jsonPacket["ultrasonic_value"] = aveUltrasonicValue;
  jsonPacket["estop"] = estop;
  jsonPacket["relay_flag"]       = relayFlag;

  jsonPacket["PID_setpoint"]     = setpoint;
  jsonPacket["PID_Kp"]           = Kp;
  jsonPacket["PID_Ki"]           = Ki;
  jsonPacket["PID_Kd"]           = Kd;

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

  // get the pid values from the tablet
  setpoint = double(jsonPacket["PID_setpoint"]);
  Kp       = double(jsonPacket["PID_Kp"]);
  Ki       = double(jsonPacket["PID_Ki"]);
  Kd       = double(jsonPacket["PID_Kd"]);

  estop = bool(jsonPacket["estop"]);    

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
// stepper machine
//====================================================
void StepperMachine() {
  switch(StepperState) {
    case OFF:
      if (wsConnected && !estop) { // if we are connected and the estop is not pressed go to init
        StepperState = INIT;
      }

      if (!stepperDelay && !wsConnected) {
        sd.disableDriver();
        stepperDelay = 2; //seconds 
      }
    break;

    case INIT:
      Serial.println("Initializing Stepper Drivers");
      delay(500);
      sd.setChipSelectPin(CSPin);
      // Give the driver some time to power up.
      delay(10);
      // Reset the driver to its default settings and clear latched status
      // conditions.
      sd.resetSettings();
      sd.clearStatus();
      // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode
      // for most applications, and we find that it usually works well.
      sd.setDecayMode(HPSDDecayMode::Slow);
      // Set the current limit. You should change the number here to an appropriate
      // value for your particular system.
      sd.setCurrentMilliamps36v4(milliamps);
    
      // Set the number of microsteps that correspond to one full step.
      sd.setStepMode(HPSDStepMode::MicroStep1);
      sd.enableDriver();
      delay(500);

      StepperState = RUN;
      stepperDelay = 4; // seconds to let things setttle?
    break;

    case RUN:

      if (!wsConnected || estop) { // if the websocket is disconnected or the estop is pressed go to off
        sd.disableDriver();
        xfr_ptr->stepperCommand = 0; // set stepper command to 0 to stop motion
        stepperCommand = 0;          // set both to 0
        StepperState = OFF;
      }


    break;
    case ERR:
      if (!stepperDelay) {
        Serial.println("Steppers are in error");
        Serial.print("The stepper error is: ");
        Serial.println(sd.readStatus());

        StepperState = INIT;
        stepperDelay = 2;
      }

    default:
    break;
  }
}


//====================================================
// end stepper machine
//====================================================




//====================================================
// end functions
//====================================================