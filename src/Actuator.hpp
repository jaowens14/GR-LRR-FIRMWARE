#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <Adafruit_ADS1X15.h>

#define NUM_ACTUATORS 2
#define MAX_VOLTAGE 5.0
#define MAX_FEEDBACK 3.3

class ActuatorControl {
    public:
        Adafruit_MCP4728 dac;
        Adafruit_ADS1115 adc;
        float actuatorPositions[NUM_ACTUATORS] = {0}; // Desired positions (0-5V)
        float feedbackSignals[NUM_ACTUATORS] = {0}; //Feedback (0-3.3V)
        enum States { SET_POSITION, READ_FEEDBACK};
        States state;

        void setup() {
            Wire.begin(); // Initialize I2C on specified pins
            if (!dac.begin()) {
                Serial.println("Failed to initialize MCP4728");
                while(1); // Stop if DAC initialization fails.
            }
            adc.begin(); // Initialize ADC
            Serial.begin(115200);
        }

        void writeDAC(uint8_t channel, float voltage) {
            // Ensure voltage is within bounds
            voltage = constrain(voltage, 0.0, MAX_VOLTAGE);

            // Scale voltage to 12-bit value
            uint16_t dacValue = (voltage / MAX_VOLTAGE) * 4095;

            // Map uint8_t channel to MCP4728_channel_t enum
            MCP4728_channel_t channelEnum;
            switch (channel) {
                case 0:
                    channelEnum = MCP4728_CHANNEL_A; break;

                case 1:
                    channelEnum = MCP4728_CHANNEL_B; break;

                case 2:
                    channelEnum = MCP4728_CHANNEL_C; break;
                
                case 3:
                    channelEnum = MCP4728_CHANNEL_D; break;

                default:
                    Serial.println("Invalid channel specified for DAC!");
                    return; // Exit if an invalid channel is given
            }

            
            // Write the value to the specified DAC channel
            if (!dac.setChannelValue(channelEnum, dacValue, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_MODE_NORMAL)) {
                Serial.println("Failed to set DAC channel value!");
            }
        }

        float readADC(uint8_t channel) {
            //Read ADC values for given channel
            int16_t rawValue = adc.readADC_SingleEnded(channel);
            return (rawValue / 32767.0) * MAX_FEEDBACK; // Scale raw ADC value to feedback range
        }

        void stateMachine(){
            switch (state) {
                case SET_POSITION:
                    for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
                        uint8_t dacChannel = i % 4; //Channel within MCP4728
                        writeDAC(dacChannel, actuatorPositions[i]);
                    }
                    state = READ_FEEDBACK;
                    break;

                case READ_FEEDBACK:
                    for (uint8_t i = 0; i< NUM_ACTUATORS; i++) {
                        uint8_t adcChannel = i % 4; // Channel within ADS1115
                        feedbackSignals[i] = readADC(adcChannel);
                    }
                    state = SET_POSITION; 
                    break;

                default:
                    break;

            }
        }

        void debugOutput() {
            for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
                Serial.print("Actuator ");
                Serial.print(i);
                Serial.print("Position Set = ");
                Serial.print(actuatorPositions[i]);
                Serial.print(" V, Feedback = ");
                Serial.print(feedbackSignals[i]);
                Serial.print({" V"});
            }
        }
};

#endif