/**
 *******************************************************************************
 * @file    CanBus.hpp
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the functions prototypes for communicate Native CANopen
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.3.1
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)
 *******************************************************************************
 */

#ifndef CANBUS_H
#ifdef ARDUINO_CAN_NATIVE_SUPPORTED
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
#endif //ARDUINO_CAN_NATIVE_SUPPORTED
#endif //CANBUS_H