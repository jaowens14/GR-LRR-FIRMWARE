#include <Arduino.h>
#include <Arduino_CAN.h>

volatile long encoder1_count = 0;
volatile long encoder2_count = 0;
volatile long encoder3_count = 0;
volatile long encoder4_count = 0;

volatile long encoder1_velocity = 0;
volatile long encoder2_velocity = 0;
volatile long encoder3_velocity = 0;
volatile long encoder4_velocity = 0;

const uint32_t ENCODER_BASE_ID = 0x580;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ;
    }

    if (!CAN.begin(CAN_BAUD_250K)) {
        Serial.println("CAN initialization failed! Check wiring of CAN configureation");
        while(1) {

        }
    } else {
        Serial.println("CAN initialized successfully!");
    }
}

void loop() {
    if (CAN.available()) {
        CANMsg msg;

        if (CAN.read(msg)) {
            processCANEncoderMsg(msg);
        }
    }
}    

void processCANEncoderMsg(const CANMsg &msg) {
    if (msg.len == 8){
        if (msg.id >= ENCODER_BASE_ID && msg.id < ENCODER_BASE_ID + 4) {
            int motor = msg.id - ENCODER_BASE_ID;

            uint8_t status = msg.data[0];

            long encoder_count = ((long)msg.data[1] << 24) | ((long)msg.data[2] << 16) | ((long)msg.data[3] << 8) | msg.data[4];

            int16_t velocity = ((int16_t)msg.data[5] << 8 | msg.data[6]);

            uint8_t diagnostics = msg.data[7];

            Serial.print("Motor ");
            Serial.print(motor +1);
            Serial.print("| Status: ")
            Serial.print(status, HEX);
            Serial.print("| Encoder count: ");
            Serial.print(encoder_count);
            Serial.print("| Velocity: ");
            Serial.print(velocity);
            Serial.print("| Diagnostics: ")
            Serial.print(diagnostics, HEX):

            switch (motor) {
                case 0;
                    encoder1_count = encoder_count;
                    encoder1_velocity = velocity;
                    break;
                
                case 1;
                    encoder2_count = encoder_count;
                    encoder2_velocity = velocity;
                    break;
                    
                case 2;
                    encoder3_count = encoder_count;
                    encoder3_velocity = velocity;
                    break;
                    
                case 3;
                    encoder4_count = encoder_count;
                    encoder4_velocity = velocity;
                    break;
                default:
                    break;    
            }

        }
    }
}