/**
 *******************************************************************************
 * @file    CanBus.cpp
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
#include "CanBus.hpp"

#define MSG_BUFF_SIZE 50

CanMsg msgBuf[MSG_BUFF_SIZE];
uint8_t msgBufStaus[MSG_BUFF_SIZE] = {0};
uint8_t writeIdx = 0;
uint32_t millisecondsTimeout;

HardwareCAN *myCAN;

CanBus::CanBus(SOLOMotorControllers::CanbusBaudrate _baudrate, arduino::HardwareCAN &_CAN, long _millisecondsTimeout)
{
    CanBitRate bitrate;
    switch (_baudrate)
    {
    case SOLOMotorControllers::CanbusBaudrate::RATE_125:
        bitrate = CanBitRate::BR_125k;
        break;
    case SOLOMotorControllers::CanbusBaudrate::RATE_250:
        bitrate = CanBitRate::BR_250k;
        break;
    case SOLOMotorControllers::CanbusBaudrate::RATE_500:
        bitrate = CanBitRate::BR_500k;
        break;
    case SOLOMotorControllers::CanbusBaudrate::RATE_1000:
        bitrate = CanBitRate::BR_1000k;
        break;
    default:
        bitrate = CanBitRate::BR_1000k;
        break;
    }
    myCAN = &_CAN;
    myCAN->begin(bitrate);
    millisecondsTimeout = _millisecondsTimeout;
}

void CanBus::Init()
{
}
bool CanBus::CANOpenSdoTransmit(uint8_t _address, bool isSet, uint16_t _object, uint8_t _subIndex, uint8_t *_informatrionToSend, uint8_t *_informationReceived, int &error)
{
    int stat = -1;
    uint8_t sendData1;
    uint8_t receiveData1;
    uint16_t ID_Read;
    uint8_t DataRead[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t counter = 0;

    if (isSet)
    {
        sendData1 = 0x22;
        receiveData1 = 0x60;
    }
    else
    {
        sendData1 = 0x40;
        receiveData1 = 0x42;
    }

    uint8_t Data[8] = {
        sendData1,               // Data 0 - SDO Write Request
        (uint8_t)(_object),      // Data 1 - Object Index(LSB)
        (uint8_t)(_object >> 8), // Data 2 - Object Index(MSB)
        _subIndex,               // Data 3 - SubIndex
        _informatrionToSend[3],  // Data 4 - LSB First Data
        _informatrionToSend[2],  // Data 5
        _informatrionToSend[1],  // Data 6
        _informatrionToSend[0]   // Data 7
    };
    
    CanMsg const msg(CanStandardId(0x600 + _address), sizeof(Data), Data);
    stat = myCAN->write(msg);

    /*
    if (stat < 0)
    {
        Serial.println("ERROR WRITE");
        error = SOLOMotorControllers::Error::MCP2515_TRANSMIT_ERROR;
        return false;
    }
    */

    unsigned long startTime = millis();
    unsigned long actualTime;
    do
    {
        storeCanMessages();
        for (int i = 0; i < MSG_BUFF_SIZE; i++)
        {
            if (msgBufStaus[i] != 1)
            {
                continue;
            }

            if ((msgBuf[i].id == (uint16_t)(0x580 + _address))     // Check COB-ID
                && (msgBuf[i].data[0] == receiveData1)             // Check Byte1
                && (msgBuf[i].data[1] == (uint8_t)(_object))       // Check Object Index(LSB)
                && (msgBuf[i].data[2] == (uint8_t)(_object >> 8))) // Check Object Index(MSB)
            {
                _informationReceived[0] = msgBuf[i].data[7];
                _informationReceived[1] = msgBuf[i].data[6];
                _informationReceived[2] = msgBuf[i].data[5];
                _informationReceived[3] = msgBuf[i].data[4];
                msgBufStaus[i] = 0;

                error = SOLOMotorControllers::Error::NO_ERROR_DETECTED;
                return true;
            }
        }
        actualTime = millis();

    } while (actualTime - startTime < millisecondsTimeout && actualTime >= startTime);
    error = SOLOMotorControllers::Error::RECEIVE_TIMEOUT_ERROR;

    return false;
}

bool CanBus::SendPdoSync(int &error)
{
    uint32_t const CAN_ID = 0x80;
    error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

    CanMsg const msg(CanStandardId(CAN_ID), 0, NULL);
    if (myCAN->write(msg) < 0)
    {
        error = SOLOMotorControllers::Error::GENERAL_ERROR;
        return false;
    }
    error = SOLOMotorControllers::Error::NO_ERROR_DETECTED;
    return true;
}

bool CanBus::PDOTransmit(long _address, uint8_t *_informatrionToSend, int &error)
{
    int stat;
    error = SOLOMotorControllers::Error::NO_ERROR_DETECTED;
    uint8_t Data[4] = {
        _informatrionToSend[3], // Data 4 - LSB First Data
        _informatrionToSend[2], // Data 5
        _informatrionToSend[1], // Data 6
        _informatrionToSend[0]  // Data 7
    };

    CanMsg const msg(CanStandardId(_address), sizeof(Data), Data);
    stat = myCAN->write(msg);
    if (stat < 0)
    {
        error = SOLOMotorControllers::Error::GENERAL_ERROR;
        return false;
    }
    return true;
}

bool CanBus::PDOReceive(long _address, uint8_t *_informationReceived, int &error)
{
    uint8_t dlc, flags;
    uint8_t rcvMsg[4] = {0, 0, 0, 0};
    uint32_t counter = 0;
    uint16_t addr = (uint16_t)_address;

    unsigned long startTime = millis();
    unsigned long actualTime;
    do
    {
        storeCanMessages();
        for (int i = 0; i < MSG_BUFF_SIZE; i++)
        {
            if (msgBufStaus[i] != 1)
            {
                continue;
            }

            if (msgBuf[i].id == _address && msgBuf[i].data_length == 4)
            {
                _informationReceived[0] = msgBuf[i].data[3];
                _informationReceived[1] = msgBuf[i].data[2];
                _informationReceived[2] = msgBuf[i].data[1];
                _informationReceived[3] = msgBuf[i].data[0];
                msgBufStaus[i] = 0;
                error = SOLOMotorControllers::Error::NO_ERROR_DETECTED;
                return true;
            }
        }
        actualTime = millis();
    } while (actualTime - startTime < millisecondsTimeout && actualTime >= startTime);

    error = SOLOMotorControllers::Error::RECEIVE_TIMEOUT_ERROR;
    return false;
}

void CanBus::storeCanMessages()
{
    while (myCAN->available())
    {
        int i = 0;
        for (i; i < MSG_BUFF_SIZE; i++)
        {
            if (msgBufStaus[i] == 0)
            {
                msgBufStaus[i] = 1;
                msgBuf[i] = myCAN->read();
                break;
            }
        }
        if (i == MSG_BUFF_SIZE)
        {
            msgBufStaus[writeIdx] = 1;
            msgBuf[writeIdx++] = myCAN->read();
            if (writeIdx == MSG_BUFF_SIZE)
                writeIdx = 0;
        }
    }
}
#endif // ARDUINO_PORTENTA_C33 ARDUINO_UNOWIFIR4 ARDUINO_MINIMA ARDUINO_PORTENTA_H7_M7