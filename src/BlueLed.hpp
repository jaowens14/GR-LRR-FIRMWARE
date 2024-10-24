
#include <Arduino.h>

#define GREEN_LED   LEDG
#define BLUE_LED    LEDB

class BlueLed {
  public:

    bool redLedFlag = false;
    bool blueLedFlag = false;
    volatile int redLedDelay = 0;
    volatile int delay = 0;

    enum States {
      OFF,
      ON
    };
    States state;


    void stateMachine(void) {
      switch(state) {
        case OFF:
          if (!delay) {
            delay = 1;
            state = ON;
            digitalWrite(BLUE_LED, LOW);

          }
        break;
        case ON:
          if (!delay) {
            delay = 1;
            state = OFF;
            digitalWrite(BLUE_LED, HIGH);
          }
        break;
        default:
        break;
      }
    }



};