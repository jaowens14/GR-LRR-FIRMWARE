#ifndef EXTERNAL_ENCODER
#define EXTERNAL_ENCODER

#include <Arduino.h>
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"

#define PIN_PHASE_A GPIO_PIN_9
#define PIN_PHASE_B GPIO_PIN_10

#define ENCODER_GPIO_PORT GPIOH

class ExtEncoder {
    public: 

        volatile long position;
        volatile int lastEncoded;
        volatile bool indexDetected;
  
        const uint8_t pinA = PH_9; 
        const uint8_t pinB = PH_10;
        // const uint8_t pinZ = D4;

        ExtEncoder(): position(0), lastEncoded(0), indexDetected(false) { }

        void setup() {
            
            /*
            HAL_Init();

            __HAL_RCC_GPIOH_CLK_ENABLE();

            GPIO_InitTypeDef GPIO_InitStruct = {0};

            GPIO_InitStruct.Pin = GPIO_PIN_9;
            GPIO_InitStruct.Pin = GPIO_PIN_10;
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

            HAL_GPIO_DeInit(GPIOH, GPIO_PIN_9);
            HAL_GPIO_DeInit(GPIOH, GPIO_PIN_10);

            HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
            */

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
            int a = HAL_GPIO_ReadPin(ENCODER_GPIO_PORT, PIN_PHASE_A);
            int b = HAL_GPIO_ReadPin(ENCODER_GPIO_PORT, PIN_PHASE_B);
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
            Serial.print(digitalRead(PH_9));
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
        }   

        static ExtEncoder* _instance;

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