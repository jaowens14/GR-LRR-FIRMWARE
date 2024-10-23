
#include <Arduino.h>
class BlueLed {
  public:

    #define RED_LED     LEDR
    #define GREEN_LED   LEDG
    #define BLUE_LED    LEDB
    bool redLedFlag = false;
    bool blueLedFlag = false;
    volatile int redLedDelay = 0;
    volatile int delay = 0;

    enum LedStates {
      OFF,
      ON
    };
    LedStates State;


    void StateMachine(void);

    void StateMachine() {
      switch(State) {
        case OFF:
          if (!delay) {
            delay = 1;
            State = ON;
            digitalWrite(BLUE_LED, LOW);

          }
        break;
        case ON:
          if (!delay) {
            delay = 1;
            State = OFF;
            digitalWrite(BLUE_LED, HIGH);
          }
        break;
        default:
        break;
      }
    }



};

//====================================================
// ON BOARD LED DEFINITIONS
//====================================================

//====================================================
// END ON BOARD LED DEFINITIONS
//====================================================