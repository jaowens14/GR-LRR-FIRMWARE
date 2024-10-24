#include <Arduino.h>
#define RED_LED     LEDR

class RedLed{
    public:

        void setup(void){
            digitalWrite(RED_LED, LOW);
            delay(2000);
            digitalWrite(RED_LED, HIGH);
        }
};  