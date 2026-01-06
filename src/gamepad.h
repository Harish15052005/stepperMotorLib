#ifndef GAMEPAD_H
#define GAMEPAD_H

#include "Arduino.h"

HardwareSerial gamepadSerial(PB7, PB6);

struct GamepadStruct
{
    int16_t lx, ly, rx, ry;
    uint8_t l2, r2;
    bool connected, printStates;
    union
    {
        uint16_t buttons;
        struct
        {
            uint16_t cross : 1;
            uint16_t circle : 1;
            uint16_t square : 1;
            uint16_t triangle : 1;
            uint16_t up : 1;
            uint16_t down : 1;
            uint16_t left : 1;
            uint16_t right : 1;
            uint16_t l1 : 1;
            uint16_t r1 : 1;
            uint16_t l3 : 1;
            uint16_t r3 : 1;
            uint16_t select : 1;
            uint16_t start : 1;
        };
    };
};
GamepadStruct gamepad;

uint8_t serialState = 0;

#define HEADER_1 0xAA
#define HEADER_2 0x96

uint8_t gamepadSerialBuffer[sizeof(GamepadStruct) + 1]; // data + checksum(1)

void gamepadLoop()
{

    if (gamepadSerial.available())
    {
        switch (serialState)
        {
        case 0:
            if (gamepadSerial.read() == HEADER_1)
                serialState = 1;
            break;

        case 1:
            if (gamepadSerial.read() == HEADER_2)
                serialState = 2;
            else
                serialState = 0;
            break;

        case 2:
            if (gamepadSerial.available() >= sizeof(gamepad) + 1)
            {
                uint8_t checksum = 0;
                for (uint8_t i = 0; i <= sizeof(gamepad); i++)
                {
                    gamepadSerialBuffer[i] = gamepadSerial.read();
                    checksum += gamepadSerialBuffer[i] * (i < sizeof(gamepad));
                }
                if ((checksum % 256) == gamepadSerialBuffer[sizeof(gamepad)])
                {
                    memcpy(&gamepad, gamepadSerialBuffer, sizeof(gamepad));
                }
                serialState = 0;
            }
            break;

        default:
            serialState = 0;
        }
    } else{
        return;
    }
}

#endif // GAMEPAD_H