#include <Arduino.h>

//====================================================
// ON BOARD LED DEFINITIONS
//====================================================
#define RED_LED     LEDR
#define GREEN_LED   LEDG
#define BLUE_LED    LEDB
bool redLedFlag = false;
bool blueLedFlag = false;
volatile int redLedDelay = 0;
volatile int blueLedDelay = 0;

enum LedStates {
  LED_OFF,
  LED_ON
};
LedStates RedLedState;
LedStates BlueLedState;

void RedLedMachine(void);
void BlueLedMachine(void);
//====================================================
// END ON BOARD LED DEFINITIONS
//====================================================