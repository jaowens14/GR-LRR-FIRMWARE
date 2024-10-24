

#include <Arduino_CAN.h>

class Motors{
    public:

        volatile int delay;
        int motorIndex = 0;
        float erefs = 0.0;
        const float wheelDiameter = 0.048;

        uint8_t EREFS_HEXDATA[4];  

        uint32_t const M1_EREFS_ID = 0x048020A8;
        uint32_t const M2_EREFS_ID = 0x048040A8;
        uint32_t const M3_EREFS_ID = 0x048060A8;
        uint32_t const M4_EREFS_ID = 0x048080A8;
        uint32_t const MD_EREFS_ID = 0x049FE0A8; // default ID

        uint32_t const MOTOR_EREFS_IDS [4] = {M1_EREFS_ID, M2_EREFS_ID, M3_EREFS_ID, M4_EREFS_ID};
        float motorSpeeds[4] = {0.0, 0.0, 0.0, 0.0};
        int writeReceipts[4] = {0, 0, 0, 0};


        enum States {
          WAITING,
          WRITING,
        };

        States state;


        void erefs_to_hexdata(float input_float, uint8_t hexdata[4]) {
            uint32_t initialInt = int(round(input_float*16*16*16*16));
            // Split the number into bytes
            hexdata[3] = (initialInt >> 24) & 0xFF;  // Most significant byte
            hexdata[2] = (initialInt >> 16) & 0xFF;
            hexdata[1] = (initialInt >> 8) & 0xFF;
            hexdata[0] = initialInt & 0xFF;          // Least significant byte
        }

        float ms_to_erefs(float ms, float wheelDiameter) {
            // meters per second linear speed to erefs
            // wheel diameter in meters 0.048
            float angularVelocity = ms/wheelDiameter; // 0.1 m/s /m = 1/s
            float rpm = angularVelocity/0.10471975057; // convert from rads/s to rpm
            //std::cout<<"EREFS: " << rpm * 18.7187185 << "\n";
            return rpm * 18.7187185; // this was pulled from the eletrocraft setup file
        }




        void motorMachine(void){
        
          switch(state) {
        
            case WAITING:
            if (!delay) {
              delay = 10;
              state = WRITING;
            }
            break;

            case WRITING:
            if (!delay) {

              delay = 10;
              erefs = ms_to_erefs(motorSpeeds[motorIndex], wheelDiameter); // convert to erefs
              erefs_to_hexdata(erefs, EREFS_HEXDATA); // convert to hexdata
              CanMsg MOTOR_SET_EREFS(CanExtendedId(MOTOR_EREFS_IDS[motorIndex]), sizeof(EREFS_HEXDATA), EREFS_HEXDATA); // to can message
              writeReceipts[motorIndex] = CAN.write(MOTOR_SET_EREFS);
              motorIndex++; // 0 , 1 , 2 , 3 

              if (motorIndex == 4) {
                motorIndex = 0;
                delay = 10;
                state = WAITING;
              }
            }
            break;

            default:
            break;
          }

        }

    };



