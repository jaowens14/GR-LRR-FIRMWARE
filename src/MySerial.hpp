
#ifndef MY_SERIAL_CLASS
#define MY_SERIAL_CLASS

#include <ArduinoJson.h>

#define RED_LED LEDR

class MySerial
{
public:
    StaticJsonDocument<64> jsonPacket;
    boolean serialStarted = false;
    boolean serialEnded = false;
    bool LED_STATE = false;
    static const byte numChars = 64;
    char receivedChars[numChars];
    boolean newData = false;
    volatile int thisDelay = 0;
    volatile int timeout = 0;
    volatile int receiveDelay = 0;

    enum States
    {
        CONNECTED,
        DISCONNECTED,
        WAITING,
    };

    States state;

    void stateMachine(void)
    {
        receiveLinux(); // will tell us if the serial has ended or not

        switch (state)
        {

        case CONNECTED:
            // while we are connected, if the timeout is zero, then we have lost connection
            if (!thisDelay && !timeout)
            {
                thisDelay = 500;

                state = DISCONNECTED;

                motors.STOP();
            }
            if (!thisDelay)
            {
                thisDelay = 500;

                Serial.println("connected");
            }
            break;

        case DISCONNECTED:
            // while we are disconnected, look for messages
            if (!thisDelay && timeout > 0)
            {
                thisDelay = 500;

                state = CONNECTED;
            }
            if (!thisDelay)
            {
                thisDelay = 500;

                Serial.println("disconnected");
            }
            break;

        default:
            break;
        }
    }

    void setup(void)
    {
        state = DISCONNECTED;
        Serial.begin(115200);
        Serial.println("Serial Starting");
    }

    void receiveLinux(void)
    {
        if (!receiveDelay)
        {
            digitalWrite(RED_LED, HIGH);
        }

        recvWithStartEndMarkers();

        processMessage();
    }

    void recvWithStartEndMarkers(void)
    {

        static boolean recvInProcess = false;
        static byte ndx = 0;
        char startMarker = '<';
        char endMarker = '>';
        char rc;

        while (Serial.available() > 0 && newData == false)
        {

            rc = Serial.read();

            if (recvInProcess == true)
            {
                if (rc != endMarker)
                {
                    receivedChars[ndx] = rc;
                    ndx++;
                    if (ndx >= numChars)
                    {
                        ndx = numChars - 1;
                    }
                }
                else
                {
                    receivedChars[ndx] = '\0';
                    recvInProcess = false;
                    ndx = 0;
                    newData = true;
                }
            }
            else if (rc == startMarker)
            {
                recvInProcess = true;
            }
        }
    }

    void processMessage(void) // message has tags removed
    {
        if (newData == true)
        {
            depackage();
            newData = false;
            timeout = 2000;

            digitalWrite(RED_LED, LED_STATE);
            receiveDelay = 5;
        }
    }

    void depackage(void)
    {

        DeserializationError err = deserializeJson(jsonPacket, receivedChars);

        switch (err.code())
        {
        case DeserializationError::Ok:

            updateParameters();
            break;

        case DeserializationError::InvalidInput:
            Serial.print(F("Invalid input!"));
            break;

        case DeserializationError::NoMemory:
            Serial.print(F("Not enough memory"));
            break;

        default:
            Serial.print(F("Deserialization failed"));
            break;
        }
    }

    void updateParameters(void)
    {
        // Serial.println("updated speeds");
        if (jsonPacket.containsKey("speed0"))
        {
            // Serial.print("updated speed 0");
            motors.speeds[0] = jsonPacket["speed0"];
        }

        if (jsonPacket.containsKey("speed1"))
        {
            motors.speeds[1] = jsonPacket["speed1"];
        }

        if (jsonPacket.containsKey("speed2"))
        {
            motors.speeds[2] = jsonPacket["speed2"];
        }

        if (jsonPacket.containsKey("speed3"))
        {
            motors.speeds[3] = jsonPacket["speed3"];
        }

        // if (jsonPacket.containsKey("start_serial"))
        //{
        //     Serial.println("contained start serial");
        //
        //}
    }
};
#endif