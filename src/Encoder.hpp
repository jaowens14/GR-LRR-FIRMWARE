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
    volatile int thisDelay;

    enum States {
        WAITING,
        UPDATING,
    };

    States state;

    Encoders(int pinA, int pinB, int pinZ) {
        encoderPinA = pinA;
        encoderPinB = pinB;
        encoderPinZ = pinZ;
        position = 0;
        thisDelay = 0;
        state = WAITING;
    }

    void setup(void) {
        pinMode(encoderPinA, INPUT);
        pinMode(encoderPinB, INPUT);
        pinMode(encoderPinZ, INPUT);
        attachInterrupt(digitalPinToInterrupt(encoderPinA), updatePosition, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderPinB), updatePosition, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderPinZ), resetPosition, RISING);
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

    void stateMachine(void) {
        switch (state) {
            case WAITING:
                if (!thisDelay) {
                    thisDelay = 10;
                    state = UPDATING;
                }
                break;

            case UPDATING:
                if (!thisDelay) {
                    thisDelay = 10;
                    state = WAITING;
                }
                break;
        default:
            break;
        }
    }

    static void updatePosition() {
        if (encoderInstance != nullptr) {
            encoderInstance->updatePosition();
        }
    }

    static void resetPosition() {
        if (encoderInstance != nullptr) {
            encoderInstance->resetPosition();
        }
    }

    static Encoders* encoderInstance; // Static reference to the encoder instance.
};

#endif