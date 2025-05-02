#ifndef EXTERNAL_ENCODER
#define EXTERNAL_ENCODER

#include <Arduino.h>

#define pinA A5
#define pinB A6

class ExtEncoder {
    public: 

        volatile long position;
        volatile int lastEncoded;
        volatile bool indexDetected;


        ExtEncoder(): position(0), lastEncoded(0), indexDetected(false) { }

        void setup() {

            pinMode(pinA, INPUT_PULLUP);
            pinMode(pinB, INPUT_PULLUP);
            // pinMode(pinZ, INPUT);

            lastEncoded = readEncoderPins();

            _instance = this;

            attachInterrupt(digitalPinToInterrupt(pinA), isrA, CHANGE);
            attachInterrupt(digitalPinToInterrupt(pinB), isrB, CHANGE);
            // attachInterrupt(digitalPinToInterrupt(pinZ), isrIndex, RISING);
        }

        int readEncoderPins() {
            int a = digitalRead(pinA);
            int b = digitalRead(pinB);
            return (a << 1) | b;
        }

        void updateEncoder() {
            int encoded = readEncoderPins();

            int sum = (lastEncoded << 2) | encoded;

            if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
                position++;
            } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
                position--;
            }
            lastEncoded = encoded;
        }

        void updateIdex() {
            position = 0;
            indexDetected = true;
        }

        long getPosition() {
            noInterrupts();
            long pos = position;
            interrupts();
            return pos;
        }

        void debugOutput() {
            Serial.print("Encoder Position: ");
            Serial.print(getPosition());
            //Serial.print(digitalRead(pinA));
            delay(100);
            if(indexDetected) {
                Serial.print(" (Index pulse detected!)");
                indexDetected = false;
            }
            Serial.println();
        }

        void stateMachine() {
            debugOutput();
        }

        void debugRawInputs() {
            Serial.print("Pin A: ");
            Serial.print(digitalRead(pinA));
            Serial.print(" Pin B: ");
            Serial.println(digitalRead(pinB));
            delay(200);
        }   

        static ExtEncoder* _instance;
  // put your main code here, to run repeatedly:

        static void isrA() {
            if (_instance) _instance->updateEncoder();
        }

        static void isrB() {
            if (_instance) _instance->updateEncoder();
        }

        static void isrIndex() {
            if (_instance) _instance->updateIdex();
        }
};

ExtEncoder* ExtEncoder::_instance = nullptr;

#endif