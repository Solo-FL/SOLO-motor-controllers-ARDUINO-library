/**
 *******************************************************************************
 * @file    Interface.hpp
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the functions prototypes for CANopen low level implementation
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.4.0
 *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 *******************************************************************************
 */

#ifndef INTERFACE_H
#define INTERFACE_H

#include "stdint.h"

class Interface
{
public:
    virtual void Init();
    virtual bool CANOpenSdoTransmit(uint8_t _address, bool isSet, uint16_t _object, uint8_t _subIndex, uint8_t *_informatrionToSend, uint8_t *_informationReceived, int &error);

    virtual bool SendPdoSync(int &error);
    virtual bool PDOTransmit(long _address, uint8_t *_informatrionToSend, int &error);
    virtual bool PDOReceive(long _address, uint8_t *_informationReceived, int &error);
};

#endif // INTERFACE_H