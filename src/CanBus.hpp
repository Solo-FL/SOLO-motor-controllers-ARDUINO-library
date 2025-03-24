/**
 *******************************************************************************
 * @file    CanBus.hpp
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the functions prototypes for communicate Native CANopen
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2025
 * @version 5.5.0
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 *******************************************************************************
 */

#if defined(ARDUINO_PORTENTA_C33) || defined(ARDUINO_UNOWIFIR4) || defined(ARDUINO_MINIMA) || defined(ARDUINO_PORTENTA_H7_M7)
#ifndef CANBUS_H
#define CANBUS_H

#include "Interface.hpp"
#include <Arduino_CAN.h>
#include "SOLOMotorControllersCanopenNative.h"

class CanBus : public Interface
{
public:
    CanBus(SOLOMotorControllers::CanbusBaudrate _baudrate, arduino::HardwareCAN &_CAN, long _millisecondsTimeout);
    void Init();
    bool CANOpenSdoTransmit(uint8_t _address, bool isSet, uint16_t _object, uint8_t _subIndex, uint8_t *_informatrionToSend, uint8_t *_informationReceived, int &error);

    bool SendPdoSync(int &error);
    bool PDOTransmit(long _address, uint8_t *_informatrionToSend, int &error);
    bool PDOReceive(long _address, uint8_t *_informationReceived, int &error);
    void storeCanMessages();
};
#endif // CANBUS_H
#endif // ARDUINO_PORTENTA_C33 ARDUINO_UNOWIFIR4 ARDUINO_MINIMA ARDUINO_PORTENTA_H7_M7