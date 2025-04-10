#ifndef MY_ENCODER_CLASS
#define MY_ENCODER_CLASS

#include <Arduino.h>
#include <Arduino_CAN.h>

class Encoders
{

public:
    volatile int thisDelay;
    int index = 0;
    float erefs = 0.0;
    const float wheelDiameter = 0.048;

    uint8_t APOS_HEXDATA[4];

    uint32_t const M1_APOS_ID = 0x048020A8;
    uint32_t const M2_APOS_ID = 0x048040A8;
    uint32_t const M3_APOS_ID = 0x048060A8;
    uint32_t const M4_APOS_ID = 0x048080A8;
    uint32_t const MD_APOS_ID = 0x049FE0A8; // default ID

    uint32_t const MOTOR_APOS_IDS[4] = {M1_APOS_ID, M2_APOS_ID, M3_APOS_ID, M4_APOS_ID};
    float speeds[4] = {0.0, 0.0, 0.0, 0.0};
    int receipts[4] = {0, 0, 0, 0};

    enum States
    {
        WAITING,
        READING,
    };

    States state;

    void setup(void)
    {
        // CAN SETUP
        if (!CAN.begin(CanBitRate::BR_250k))
        {
            Serial.println("CAN.begin(...) failed.");
            for (;;)
            {
                Serial.println("CAN ISSUE");

                delay(1000);
            }
        }
    }

    void stateMachine(void)
    {

        switch (state)
        {

        case WAITING:
            if (!thisDelay)
            {
                thisDelay = 10;
                state = READING;
            }
            break;

        case READING:
            if (!thisDelay)
            {

                thisDelay = 10;
                CanMsg MOTOR_GET_APOS(CanExtendedId(MOTOR_APOS_IDS[index]), sizeof(APOS_HEXDATA), APOS_HEXDATA); // to can message
                //Serial.println(MOTOR_SET_APOS);
                receipts[index] = CAN.write(MOTOR_GET_APOS);
                index++; // 0 , 1 , 2 , 3

                if (index == 4)
                {
                    index = 0;
                    thisDelay = 10;
                    state = WAITING;
                }
            }
            break;

        default:
            break;
        }
    }

    void readPositionData() {
        for (int i = 0; i < 4; i++) {
            CanMsg MOTOR_GET_APOS(CanExtendedId(MOTOR_APOS_IDS[i]), sizeof(APOS_HEXDATA), APOS_HEXDATA); // Request APOS data
            receipts[i] = CAN.write(MOTOR_GET_APOS);
            delay(10);
        }
    }
};

#endif