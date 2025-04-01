#ifndef MY_ENCODER_CLASS
#define MY_ENCODER_CLASS

#include <Arduino.h>

class Encoders
{

public:
    volatile long position;
    int encoderPinA;
    int encoderPinB;
    int encoderPinZ;

    Encoders(int pinA, int pinB, int pinZ) {
        encoderPinA = pinA;
        encoderPinB = pinB;
        encoderPinZ = pinZ;
        position = 0;
    }

    void setup(void) {
        pinMode(encoderPinA, INPUT);
        pinMode(encoderPinB, INPUT);
        pinMode(encoderPinZ, INPUT);
        attachInterrupt(digitalPinToInterrupt(encoderPinA), std::bind(&Encoders::updatePosition, this), CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderPinB), std::bind(&Encoders::updatePosition, this), CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderPinZ), std::bind(&Encoders::resetPosition, this), RISING);
    }

    void updatePosition() {
        int stateA = digitalRead(encoderPinA);
        int stateB = digitalRead(encoderPinB);
        if (stateA == stateB) {
            position++;
        }
        else {
            position --;
        }
    }

    void resetPosition() {
        position = 0;
    }

    long getPosition() {
        return position;
    }
};

#endif