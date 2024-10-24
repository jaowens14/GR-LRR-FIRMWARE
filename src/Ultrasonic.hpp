
#include <Arduino.h>
#define ULTRASONIC_PIN A2

class Ultrasonic{
    public:

        enum States {
          WAITING,
          READING,
        };

        States state;

        volatile int delay = 0;
        float ultrasonicDistance = 0;
        const static int numUltrasonicSamples = 50;
        double ultrasonicSamples[numUltrasonicSamples] = {0};


        void stateMachine() {
            switch(state){
              case WAITING:
                if (!delay) {
                  state = READING;  
                }
              break;
              case READING:
                if (!delay) { 
                  // 1. float() to change analogRead() units
                  // 2. times 3.1 to scale to max ADC voltage for Portenta
                  // 3. divide by 4096 to convert from the ADC precision 
                  // 4. times 5.0 to scale to original battery voltage - this comes from the voltage divider on the board
                  // leaky integrator over rolling ave, gain of 0.1
                  ultrasonicDistance += ((float(analogRead(ULTRASONIC_PIN)) * 3.1 / 4096.0) * (5.0) - ultrasonicDistance) * 0.1;
                  delay = 25; // 0.25 seconds
                  state = WAITING;
                }   
              break;
              default:
              break;
            }

        }
    };