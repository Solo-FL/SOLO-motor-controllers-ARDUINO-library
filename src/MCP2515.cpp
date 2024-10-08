/**
 *******************************************************************************
 * @file    MCP2515.cpp
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the functions for communicate with MCP2515 IC
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

#include "MCP2515.hpp"
#include <SPI.h>
//#include "string.h"

extern volatile bool isSpiBusy;
extern volatile unsigned char interruptPin;
extern volatile uint8_t canBuf[CAN_BUFF_SIZE][12];
extern volatile uint8_t canBufStaus[CAN_BUFF_SIZE]; // init at all as true
enum class Status
{
    EMPTY,
    LOCKED,
    FILLED
};

SPISettings spiSettings(8000000, MSBFIRST, SPI_MODE0);

MCP2515::MCP2515(uint8_t _chipSelectPin, SOLOMotorControllers::CanbusBaudrate _baudrate, unsigned char _interruptPin, SOLOMotorControllers::Frequency _frequency, long _millisecondsTimeout)
{
    chipSelectPin = _chipSelectPin;
    baudrate = _baudrate;
    interruptPin = _interruptPin;
    frequency = _frequency;
    millisecondsTimeout = _millisecondsTimeout;
    attachInterrupt(digitalPinToInterrupt(_interruptPin), ISR_Handler, FALLING); // start interrupt
}

void MCP2515::StartSPI()
{
    SPI.beginTransaction(spiSettings);
    digitalWrite(chipSelectPin, LOW);
}

void MCP2515::EndSPI()
{
    digitalWrite(chipSelectPin, HIGH);
    SPI.endTransaction();
}

void MCP2515::MCP2515_Reset()
{
    StartSPI();
    SPI.transfer(RESET_INSTRUCTION);
    EndSPI();
}

void MCP2515::MCP2515_Write_Register(uint8_t Address, uint8_t Value)
{
    StartSPI();
    SPI.transfer(WRITE_INSTRUCTION);
    SPI.transfer(Address);
    SPI.transfer(Value);
    EndSPI();
}

uint8_t MCP2515::MCP2515_Read_Register(uint8_t Address)
{

    StartSPI();
    SPI.transfer(READ_INSTRUCTION);
    SPI.transfer(Address);
    uint8_t Value = SPI.transfer(0xFF);
    EndSPI();

    return (Value);
}

void MCP2515::MCP2515_Bit_Modify_Register(uint8_t _Address, uint8_t _Mask, uint8_t _Value)
{

    StartSPI();
    SPI.transfer(BIT_MODIFY_INSTRUCTION);
    SPI.transfer(_Address);
    SPI.transfer(_Mask); //'1' allow to change
    SPI.transfer(_Value);
    EndSPI();
}

void MCP2515::MCP2515_Set_Mode(Mcp2515Mode _Mode)
{

    uint8_t ModeValue = 0x00;

    switch (_Mode)
    {
    case Mcp2515Mode::CONFIGURATION_MODE:
        ModeValue = MODE_CONFIGURATION;
        break;
    case Mcp2515Mode::NORMAL_MODE:
        ModeValue = MODE_NORMAL;
        break;
    case Mcp2515Mode::SLEEP_MODE:
        ModeValue = MODE_SLEEP;
        break;
    case Mcp2515Mode::LISTEN_ONLY_MODE:
        ModeValue = MODE_LISTEN_ONLY;
        break;
    case Mcp2515Mode::LOOPBACK_MODE:
        ModeValue = MODE_LOOPBACK;
        break;
    default:
        break;
    }

    MCP2515_Bit_Modify_Register(CANCTRL, 0xE0, ModeValue);
}

bool MCP2515::MCP2515_Transmit_Frame(Mcp2515TxBuffer _TXBn, uint16_t _ID, uint8_t _DLC, uint8_t *_Data, int &error)
{
    isSpiBusy = true;
    storeDataFromBuffers();

    uint16_t i = 0;
    uint8_t ID_High, ID_Low;
    uint8_t TX_BUF_Number = 0x00;
    uint8_t RTS_BUF_Number = 0x00; // Request to send buffer n

    MCP2515_Disable_TxBuffer_INT(TX_BUFFER_0);
    MCP2515_Disable_TxBuffer_INT(TX_BUFFER_1);
    MCP2515_Disable_TxBuffer_INT(TX_BUFFER_2);

    if (_DLC == 0 && _ID != 128)
    {
        _ID = _ID | 0x800;
    }
    ID_High = (uint8_t)(_ID >> 3);
    ID_Low = (uint8_t)((_ID << 5) & 0x00E0);
    switch (_TXBn)
    {
    case Mcp2515TxBuffer::TX_BUFFER_0:
        TX_BUF_Number = LOAD_TX_BUFFER_0;
        RTS_BUF_Number = RTS_TX_BUF_0;
        break;
    case Mcp2515TxBuffer::TX_BUFFER_1:
        TX_BUF_Number = LOAD_TX_BUFFER_1;
        RTS_BUF_Number = RTS_TX_BUF_1;
        break;
    case Mcp2515TxBuffer::TX_BUFFER_2:
        TX_BUF_Number = LOAD_TX_BUFFER_2;
        RTS_BUF_Number = RTS_TX_BUF_2;
        break;

    default:
        break;
    }

    StartSPI();
    SPI.transfer(TX_BUF_Number);

    if (_DLC == 0 && _ID != 128)
    {
        SPI.transfer(ID_High); // ID High bits
        SPI.transfer(ID_Low);  // ID Low  bits
        SPI.transfer(0x00);
        SPI.transfer(0x00);
        SPI.transfer(0x40); // EID Registers(unused)
    }
    else
    {
        SPI.transfer(ID_High); // ID High bits
        SPI.transfer(ID_Low);  // ID Low  bits
        SPI.transfer(0xFF);    // EID Registers(unused)
        SPI.transfer(0xFF);
    }
    SPI.transfer(_DLC);
    for (i = 0; i < _DLC; i++)
    { // load data buffer
        SPI.transfer(_Data[i]);
    }

    EndSPI();

    MCP2515_Bit_Modify_Register(TXB0CTRL, 0x08, 0x00); // Clear TXREQ bit of TX_BUF_0
    MCP2515_Bit_Modify_Register(TXB1CTRL, 0x08, 0x00); // Clear TXREQ bit of TX_BUF_1
    MCP2515_Bit_Modify_Register(TXB2CTRL, 0x08, 0x00); // Clear TXREQ bit of TX_BUF_2

    StartSPI();
    SPI.transfer(RTS_BUF_Number);
    EndSPI();

    unsigned long startTime = millis();
    unsigned long actualTime;
    while (MCP2515_Read_Register(TXB0CTRL) & (1 << 3)) // Check TXB0REQ bit
    {
        actualTime = millis();
        if (actualTime - startTime > 20 || actualTime < startTime)
        {
            // Serial.println("MCP2515_Transmit_Frame TIMOUT");
            if (MCP2515_Read_Register(TXB0CTRL) & (1 << 4)) // Transmission Error Detected - TXERR bit
            {
                error = SOLOMotorControllers::Error::MCP2515_TRANSMIT_ERROR;
            }
            else if (MCP2515_Read_Register(TXB0CTRL) & (1 << 5)) // Message lost arbitration - MLOA bit
            {
                error = SOLOMotorControllers::Error::MCP2515_TRANSMIT_ARBITRATION_LOST;
            }
            isSpiBusy = false;
            return false;
        }
    }
    isSpiBusy = false;
    return true;
}

bool MCP2515::CANOpenSdoTransmit(uint8_t _address, bool isSet, uint16_t _object, uint8_t _subIndex, uint8_t *_informatrionToSend, uint8_t *_informationReceived, int &error)
{
    uint8_t sendData1;
    uint8_t receiveData1;
    uint16_t ID_Read;
    uint16_t ID_High, ID_Low;
    uint8_t DLC_Read;
    uint8_t DataRead[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t counter = 0;

    // TO MUCH
    // MCP2515_Write_Register(CANINTF, 0x00); // clear all interrupt flags

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

    if (!MCP2515_Transmit_Frame(TX_BUFFER_0, (0x600 + _address), 0x08, Data, error)) // 0x08 is Data Length(DLC)
    {
        // Serial.println("SEND ERROR");
        return false;
    }

    uint8_t filled = static_cast<uint8_t>(Status::FILLED);

    // check if data received before
    unsigned long startTime = millis();
    unsigned long actualTime;
    do
    {
        isSpiBusy = true;
        storeDataFromBuffers();
        isSpiBusy = false;
        for (int i = 0; i < CAN_BUFF_SIZE; i++)
        {
            // Serial.print(" LOOPED: [" + String(i)+ " | " + String(canBufStaus[i]) + "]");
            if (canBufStaus[i] != filled)
            {
                continue;
            }

            canBufStaus[i] = static_cast<uint8_t>(Status::LOCKED);

            ID_High = canBuf[i][0];
            ID_Low = canBuf[i][1];
            ID_Read = ((ID_High << 3) | ((ID_Low & 0xE0) >> 5)); // Repack ID

            if ((ID_Read == (uint16_t)(0x580 + _address))     // Check COB-ID
                && (canBuf[i][3] == receiveData1)             // Check Byte1
                && (canBuf[i][4] == (uint8_t)(_object))       // Check Object Index(LSB)
                && (canBuf[i][5] == (uint8_t)(_object >> 8))) // Check Object Index(MSB)
            {
                _informationReceived[0] = canBuf[i][10];
                _informationReceived[1] = canBuf[i][9];
                _informationReceived[2] = canBuf[i][8];
                _informationReceived[3] = canBuf[i][7];
                canBufStaus[i] = static_cast<uint8_t>(Status::EMPTY);

                // Serial.print(" READ ");
                // //Serial.print(ID_Read == (uint16_t)(0x580 + _address));
                // char tmp2[16];
                // for (int r=0; r<=10; r++) {
                // sprintf(tmp2, "0x%.2X",canBuf[i][r]);
                // Serial.print(tmp2); Serial.print(" ");
                // }
                // Serial.println("");

                // Serial.print(" READED: [" + String(i)+ " | " + String(canBufStaus[i]) + "]");
                error = SOLOMotorControllers::Error::NO_ERROR_DETECTED;
                return true;
            }
            canBufStaus[i] = static_cast<uint8_t>(Status::FILLED);
        }
        actualTime = millis();
    } while (actualTime - startTime < millisecondsTimeout && actualTime >= startTime);
    // Serial.print("RECEIVE_TIMEOUT_ERROR");
    error = SOLOMotorControllers::Error::RECEIVE_TIMEOUT_ERROR;
    return false;
}

void MCP2515::MCP2515_Set_BaudRate()
{
    //_BuadRate is Kbps
    uint8_t CNF1_Value = 0x00;
    uint8_t CNF2_Value = 0x00;
    uint8_t CNF3_Value = 0x00;

    switch (frequency)
    {
    case SOLOMotorControllers::Frequency::RATE_8:
        switch (baudrate)
        {
        case SOLOMotorControllers::CanbusBaudrate::RATE_1000:
            CNF1_Value = MCP_8MHz_1000kBPS_CFG1;
            CNF2_Value = MCP_8MHz_1000kBPS_CFG2;
            CNF3_Value = MCP_8MHz_1000kBPS_CFG3;
            break;
        case SOLOMotorControllers::CanbusBaudrate::RATE_500:
            CNF1_Value = MCP_8MHz_500kBPS_CFG1;
            CNF2_Value = MCP_8MHz_500kBPS_CFG2;
            CNF3_Value = MCP_8MHz_500kBPS_CFG3;
            break;
        case SOLOMotorControllers::CanbusBaudrate::RATE_250:
            CNF1_Value = MCP_8MHz_250kBPS_CFG1;
            CNF2_Value = MCP_8MHz_250kBPS_CFG2;
            CNF3_Value = MCP_8MHz_250kBPS_CFG3;
            break;
        case SOLOMotorControllers::CanbusBaudrate::RATE_125:
            CNF1_Value = MCP_8MHz_125kBPS_CFG1;
            CNF2_Value = MCP_8MHz_125kBPS_CFG2;
            CNF3_Value = MCP_8MHz_125kBPS_CFG3;
            break;
        case SOLOMotorControllers::CanbusBaudrate::RATE_100:
            CNF1_Value = MCP_8MHz_100kBPS_CFG1;
            CNF2_Value = MCP_8MHz_100kBPS_CFG2;
            CNF3_Value = MCP_8MHz_100kBPS_CFG3;
            break;
        default:
            CNF1_Value = MCP_8MHz_1000kBPS_CFG1;
            CNF2_Value = MCP_8MHz_1000kBPS_CFG2;
            CNF3_Value = MCP_8MHz_1000kBPS_CFG3;
            break;
        }
        break;
    case SOLOMotorControllers::Frequency::RATE_20:
        switch (baudrate)
        {
        case SOLOMotorControllers::CanbusBaudrate::RATE_1000:
            CNF1_Value = MCP_20MHz_1000kBPS_CFG1;
            CNF2_Value = MCP_20MHz_1000kBPS_CFG2;
            CNF3_Value = MCP_20MHz_1000kBPS_CFG3;
            break;
        case SOLOMotorControllers::CanbusBaudrate::RATE_500:
            CNF1_Value = MCP_20MHz_500kBPS_CFG1;
            CNF2_Value = MCP_20MHz_500kBPS_CFG2;
            CNF3_Value = MCP_20MHz_500kBPS_CFG3;
            break;
        case SOLOMotorControllers::CanbusBaudrate::RATE_250:
            CNF1_Value = MCP_20MHz_250kBPS_CFG1;
            CNF2_Value = MCP_20MHz_250kBPS_CFG2;
            CNF3_Value = MCP_20MHz_250kBPS_CFG3;
            break;
        case SOLOMotorControllers::CanbusBaudrate::RATE_125:
            CNF1_Value = MCP_20MHz_125kBPS_CFG1;
            CNF2_Value = MCP_20MHz_125kBPS_CFG2;
            CNF3_Value = MCP_20MHz_125kBPS_CFG3;
            break;
        case SOLOMotorControllers::CanbusBaudrate::RATE_100:
            CNF1_Value = MCP_20MHz_100kBPS_CFG1;
            CNF2_Value = MCP_20MHz_100kBPS_CFG2;
            CNF3_Value = MCP_20MHz_100kBPS_CFG3;
            break;
        default:
            CNF1_Value = MCP_20MHz_1000kBPS_CFG1;
            CNF2_Value = MCP_20MHz_1000kBPS_CFG2;
            CNF3_Value = MCP_20MHz_1000kBPS_CFG3;
            break;
        }
        break;
    case SOLOMotorControllers::Frequency::RATE_16:
    default:
        switch (baudrate)
        {
        case SOLOMotorControllers::CanbusBaudrate::RATE_1000:
            CNF1_Value = MCP_16MHz_1000kBPS_CFG1;
            CNF2_Value = MCP_16MHz_1000kBPS_CFG2;
            CNF3_Value = MCP_16MHz_1000kBPS_CFG3;
            break;
        case SOLOMotorControllers::CanbusBaudrate::RATE_500:
            CNF1_Value = MCP_16MHz_500kBPS_CFG1;
            CNF2_Value = MCP_16MHz_500kBPS_CFG2;
            CNF3_Value = MCP_16MHz_500kBPS_CFG3;
            break;
        case SOLOMotorControllers::CanbusBaudrate::RATE_250:
            CNF1_Value = MCP_16MHz_250kBPS_CFG1;
            CNF2_Value = MCP_16MHz_250kBPS_CFG2;
            CNF3_Value = MCP_16MHz_250kBPS_CFG3;
            break;
        case SOLOMotorControllers::CanbusBaudrate::RATE_125:
            CNF1_Value = MCP_16MHz_125kBPS_CFG1;
            CNF2_Value = MCP_16MHz_125kBPS_CFG2;
            CNF3_Value = MCP_16MHz_125kBPS_CFG3;
            break;
        case SOLOMotorControllers::CanbusBaudrate::RATE_100:
            CNF1_Value = MCP_16MHz_100kBPS_CFG1;
            CNF2_Value = MCP_16MHz_100kBPS_CFG2;
            CNF3_Value = MCP_16MHz_100kBPS_CFG3;
            break;
        default:
            CNF1_Value = MCP_16MHz_1000kBPS_CFG1;
            CNF2_Value = MCP_16MHz_1000kBPS_CFG2;
            CNF3_Value = MCP_16MHz_1000kBPS_CFG3;
            break;
        }
        break;
    }

    MCP2515_Write_Register(CNF1, CNF1_Value);
    MCP2515_Write_Register(CNF2, CNF2_Value);
    MCP2515_Write_Register(CNF3, CNF3_Value);
}

void MCP2515::MCP2515_Set_ReceiveMask_SID(Mcp2515RxBuffer _RXBn, uint16_t _Mask)
{
    switch (_RXBn)
    {
    case Mcp2515RxBuffer::RX_BUFFER_0:
        MCP2515_Write_Register(RXM0SIDH, (uint8_t)(_Mask >> 3));
        MCP2515_Write_Register(RXM0SIDL, (uint8_t)(_Mask << 5));
        break;
    case Mcp2515RxBuffer::RX_BUFFER_1:
        MCP2515_Write_Register(RXM1SIDH, (uint8_t)(_Mask >> 3));
        MCP2515_Write_Register(RXM1SIDL, (uint8_t)(_Mask << 5));
        break;

    default:
        break;
    }
}

void MCP2515::MCP2515_Set_ReceiveMask_EID(Mcp2515RxBuffer _RXBn, uint16_t _Mask)
{

    switch (_RXBn)
    {
    case Mcp2515RxBuffer::RX_BUFFER_0:
        MCP2515_Write_Register(RXM0EID8, (uint8_t)(_Mask >> 8));
        MCP2515_Write_Register(RXM0EID0, (uint8_t)(_Mask));
        break;
    case Mcp2515RxBuffer::RX_BUFFER_1:
        MCP2515_Write_Register(RXM1EID8, (uint8_t)(_Mask >> 8));
        MCP2515_Write_Register(RXM1EID0, (uint8_t)(_Mask));
        break;

    default:
        break;
    }
}

uint8_t MCP2515::MCP2515_Read_RX_Status()
{
    StartSPI();
    SPI.transfer(RX_STATUS_INSTRUCTION);
    uint8_t Value = SPI.transfer(0xFF);
    EndSPI();

    return (Value);
}

void MCP2515::Init()
{
    pinMode(chipSelectPin, OUTPUT);

    // SPI Initialize
    SPI.begin();

    MCP2515_Reset();
    MCP2515_Set_Mode(Mcp2515Mode::CONFIGURATION_MODE);
    MCP2515_Set_BaudRate();

    // enable RXbuf0 filters with 0 to don't receive enything
    MCP2515_Write_Register(RXM0SIDH, 0x00); // Mask0 byte 0
    MCP2515_Write_Register(RXM0SIDL, 0x00); // Mask0 Byte 1
    MCP2515_Write_Register(RXF0SIDH, 0x00);
    MCP2515_Write_Register(RXF0SIDL, 0x00);
    MCP2515_Write_Register(RXF1SIDH, 0x00);
    MCP2515_Write_Register(RXF1SIDH, 0x00);
    // disable RXbuf1 mask
    MCP2515_Write_Register(RXM1SIDH, 0x00); // Mask1 byte 0
    MCP2515_Write_Register(RXM1SIDL, 0x00); // Mask1 Byte 1
    MCP2515_Write_Register(RXF2SIDH, 0x00);
    MCP2515_Write_Register(RXF2SIDL, 0x00);
    MCP2515_Write_Register(RXF3SIDH, 0x00);
    MCP2515_Write_Register(RXF3SIDH, 0x00);
    MCP2515_Write_Register(RXF4SIDH, 0x00);
    MCP2515_Write_Register(RXF4SIDL, 0x00);
    MCP2515_Write_Register(RXF4SIDH, 0x00);
    MCP2515_Write_Register(RXF4SIDH, 0x00);

    MCP2515_Set_Mode(Mcp2515Mode::NORMAL_MODE);
    MCP2515_Enable_Rollover();
    MCP2515_Write_Register(CANINTF, 0x00); // clear all interrupt flags
    MCP2515_Write_Register(CANINTE, 0x03);
    MCP2515_Disable_MaskFilter(RX_BUFFER_0);
    MCP2515_Disable_MaskFilter(RX_BUFFER_1);
}

void MCP2515::MCP2515_Receive_Frame(Mcp2515RxBuffer _RXBn, uint16_t *_ID, uint8_t *_DLC, uint8_t *_Data)
{

    uint8_t i, len;
    uint8_t RX_BUF_Number = 0x00;
    uint16_t ID_High, ID_Low;

    switch (_RXBn)
    {
    case Mcp2515RxBuffer::RX_BUFFER_0:
        RX_BUF_Number = READ_RX_BUFFER_0;
        break;
    case Mcp2515RxBuffer::RX_BUFFER_1:
        RX_BUF_Number = READ_RX_BUFFER_1;
        break;
    default:
        break;
    }

    StartSPI();
    SPI.transfer(RX_BUF_Number);
    ID_High = (uint16_t)SPI.transfer(0xFF);
    ID_Low = (uint16_t)SPI.transfer(0xFF);
    SPI.transfer(0xFF);                  // Extended ID High(unused)
    SPI.transfer(0xFF);                  // Extended ID Low (unused)
    len = (SPI.transfer(0xFF) & (0x0F)); // DLC

    for (i = 0; i < len; i++)
    {
        _Data[i] = SPI.transfer(0xFF); // Data
    }
    EndSPI();

    (*_DLC) = len;
    (*_ID) = ((ID_High << 3) | ((ID_Low & 0xE0) >> 5)); // Repack ID

    if (_RXBn == Mcp2515RxBuffer::RX_BUFFER_0)
        MCP2515_Bit_Modify_Register(CANINTF, 0x01, 0x00); // Clear RX0IF and RX1IF in CANINTF[1:0]
    else
        MCP2515_Bit_Modify_Register(CANINTF, 0x02, 0x00); // Clear RX0IF and RX1IF in CANINTF[1:0]
}

void MCP2515::MCP2515_Receive_Frame_IT(Mcp2515RxBuffer _RXBn, uint8_t *_Data)
{
    uint8_t i, len;
    uint8_t RX_BUF_Number = 0x00;
    uint16_t ID, ID_High, ID_Low;

    switch (_RXBn)
    {
    case Mcp2515RxBuffer::RX_BUFFER_0:
        RX_BUF_Number = READ_RX_BUFFER_0;
        break;
    case Mcp2515RxBuffer::RX_BUFFER_1:
        RX_BUF_Number = READ_RX_BUFFER_1;
        break;
    default:
        break;
    }

    StartSPI();
    SPI.transfer(RX_BUF_Number);
    _Data[0] = (uint16_t)SPI.transfer(0xFF);
    _Data[1] = (uint16_t)SPI.transfer(0xFF);
    SPI.transfer(0xFF);                       // Extended ID High(unused)
    SPI.transfer(0xFF);                       // Extended ID Low (unused)
    _Data[2] = (SPI.transfer(0xFF) & (0x0F)); // DLC

    for (i = 0; i < _Data[2]; i++)
    {
        _Data[i + 3] = SPI.transfer(0xFF); // Data
    }
    EndSPI();
}

void MCP2515::MCP2515_Clear_TxBuffer_Flag(Mcp2515TxBuffer _TXBn)
{
    switch (_TXBn)
    {
    case Mcp2515TxBuffer::TX_BUFFER_0:
        MCP2515_Bit_Modify_Register(CANINTF, 0x04, 0x00); // CANINTF[3]
        break;
    case Mcp2515TxBuffer::TX_BUFFER_1:
        MCP2515_Bit_Modify_Register(CANINTF, 0x08, 0x00); // CANINTF[4]
        break;
    case Mcp2515TxBuffer::TX_BUFFER_2:
        MCP2515_Bit_Modify_Register(CANINTF, 0x10, 0x00); // CANINTF[5]
        break;

    default:
        break;
    }
}

void MCP2515::MCP2515_Clear_RxBuffer_Flag(Mcp2515RxBuffer _RXBn)
{
    switch (_RXBn)
    {
    case Mcp2515RxBuffer::RX_BUFFER_0:
        MCP2515_Bit_Modify_Register(CANINTF, 0x01, 0x00); // CANINTF[0]
        break;
    case Mcp2515RxBuffer::RX_BUFFER_1:
        MCP2515_Bit_Modify_Register(CANINTF, 0x02, 0x00); // CANINTF[1]
        break;

    default:
        break;
    }
}
void MCP2515::MCP2515_Enable_TxBuffer_INT(Mcp2515TxBuffer _TXBn)
{
    switch (_TXBn)
    {
    case Mcp2515TxBuffer::TX_BUFFER_0:
        MCP2515_Bit_Modify_Register(CANINTE, 0x04, 0x04); // CANINTE[2]
        break;
    case Mcp2515TxBuffer::TX_BUFFER_1:
        MCP2515_Bit_Modify_Register(CANINTE, 0x08, 0x08); // CANINTE[3]
        break;
    case Mcp2515TxBuffer::TX_BUFFER_2:
        MCP2515_Bit_Modify_Register(CANINTE, 0x10, 0x10); // CANINTE[4]
        break;

    default:
        break;
    }
}

void MCP2515::MCP2515_Disable_TxBuffer_INT(Mcp2515TxBuffer _TXBn)
{
    switch (_TXBn)
    {
    case Mcp2515TxBuffer::TX_BUFFER_0:
        MCP2515_Bit_Modify_Register(CANINTE, 0x04, 0x00); // CANINTE[2]
        break;
    case Mcp2515TxBuffer::TX_BUFFER_1:
        MCP2515_Bit_Modify_Register(CANINTE, 0x08, 0x00); // CANINTE[3]
        break;
    case Mcp2515TxBuffer::TX_BUFFER_2:
        MCP2515_Bit_Modify_Register(CANINTE, 0x10, 0x00); // CANINTE[4]
        break;

    default:
        break;
    }
}

void MCP2515::MCP2515_Enable_RxBuffer_INT(Mcp2515RxBuffer _RXBn)
{
    switch (_RXBn)
    {
    case Mcp2515RxBuffer::RX_BUFFER_0:
        MCP2515_Bit_Modify_Register(CANINTE, 0x01, 0x01); // CANINTE[0]
        break;
    case Mcp2515RxBuffer::RX_BUFFER_1:
        MCP2515_Bit_Modify_Register(CANINTE, 0x02, 0x02); // CANINTE[1]
        break;

    default:
        break;
    }
}
void MCP2515::MCP2515_Enable_MaskFilter(Mcp2515RxBuffer _RXBn)
{
    switch (_RXBn)
    {
    case Mcp2515RxBuffer::RX_BUFFER_0:
        MCP2515_Bit_Modify_Register(RXB0CTRL, 0x60, 0x00); // RXB0CTRL[6:5]
        break;
    case Mcp2515RxBuffer::RX_BUFFER_1:
        MCP2515_Bit_Modify_Register(RXB1CTRL, 0x60, 0x00); // RXB1CTRL[6:5]
        break;

    default:
        break;
    }
}
void MCP2515::MCP2515_Disable_MaskFilter(Mcp2515RxBuffer _RXBn)
{
    switch (_RXBn)
    {
    case Mcp2515RxBuffer::RX_BUFFER_0:
        MCP2515_Bit_Modify_Register(RXB0CTRL, 0x60, 0x60); // RXB0CTRL[6:5]
        break;
    case Mcp2515RxBuffer::RX_BUFFER_1:
        MCP2515_Bit_Modify_Register(RXB1CTRL, 0x60, 0x60); // RXB1CTRL[6:5]
        break;

    default:
        break;
    }
}

void MCP2515::MCP2515_Enable_Rollover()
{
    MCP2515_Bit_Modify_Register(RXB0CTRL, 0x04, 0x04); // RXB0CTRL[2]
}
void MCP2515::MCP2515_Disable_Rollover()
{
    MCP2515_Bit_Modify_Register(RXB0CTRL, 0x04, 0x00); // RXB0CTRL[2]
}

uint8_t MCP2515::Mcp2515ReadReceiveErrorCounter()
{
    return (MCP2515_Read_Register(REC));
}

uint8_t MCP2515::Mcp2515ReadTransmitErrorCounter()
{
    return (MCP2515_Read_Register(TEC));
}

void MCP2515::Mcp2515ReadErrorMode(int &errorMode)
{
    uint8_t ErrorModeValue = MCP2515_Read_Register(EFLG);
    if (ErrorModeValue & (1 << 1))
    {
        errorMode = MCP2515::Mcp2515ErrorMode::RECEIVE_ERROR_WARNING;
    }
    else if (ErrorModeValue & (1 << 2))
    {
        errorMode = MCP2515::Mcp2515ErrorMode::TRANSMIT_ERROR_WARNING;
    }
    else if (ErrorModeValue & (1 << 3))
    {
        errorMode = MCP2515::Mcp2515ErrorMode::RECEIVE_ERROR_PASSIVE;
    }
    else if (ErrorModeValue & (1 << 4))
    {
        errorMode = MCP2515::Mcp2515ErrorMode::TRANSMIT_ERROR_PASSIVE;
    }
    else if (ErrorModeValue & (1 << 5))
    {
        errorMode = MCP2515::Mcp2515ErrorMode::BUS_OFF;
    }
    else
    {
        errorMode = MCP2515::Mcp2515ErrorMode::ACTIVE_ERROR;
    }
}

bool MCP2515::PDOReceive(long _address, uint8_t *_informationReceived, int &error)
{
    long ID_Read;
    uint8_t DLC_Read;
    uint16_t ID_High, ID_Low;
    uint8_t dlc, flags;
    uint8_t rcvMsg[4] = {0, 0, 0, 0};
    uint32_t counter = 0;
    uint16_t addr = (uint16_t)_address;
    uint8_t filled = static_cast<uint8_t>(Status::FILLED);

    unsigned long startTime = millis();
    unsigned long actualTime;
    do
    {
        isSpiBusy = true;
        storeDataFromBuffers();
        isSpiBusy = false;
        for (int i = 0; i < CAN_BUFF_SIZE; i++)
        {
            if (canBufStaus[i] != filled)
            {
                continue;
            }

            canBufStaus[i] = static_cast<uint8_t>(Status::LOCKED);
            dlc = canBuf[i][2];
            ID_High = canBuf[i][0];
            ID_Low = canBuf[i][1];
            ID_Read = ((ID_High << 3) | ((ID_Low & 0xE0) >> 5)); // Repack ID
            if (ID_Read == _address && dlc == 4)
            {
                _informationReceived[0] = canBuf[i][6];
                _informationReceived[1] = canBuf[i][5];
                _informationReceived[2] = canBuf[i][4];
                _informationReceived[3] = canBuf[i][3];
                canBufStaus[i] = static_cast<uint8_t>(Status::EMPTY);
                error = SOLOMotorControllers::Error::NO_ERROR_DETECTED;
                return true;
            }
            canBufStaus[i] = static_cast<uint8_t>(Status::FILLED);
        }
        actualTime = millis();
    } while (actualTime - startTime < millisecondsTimeout && actualTime >= startTime);

    error = SOLOMotorControllers::Error::RECEIVE_TIMEOUT_ERROR;
    return false;
}
bool MCP2515::PDOTransmit(long _address, uint8_t *_informatrionToSend, int &error)
{
    // std::cout << "PDOTransmit - _address: "<< _address << " \n";
    bool stat;
    error = SOLOMotorControllers::Error::NO_ERROR_DETECTED;
    uint8_t msg[4] = {
        _informatrionToSend[3], // Data 4 - LSB First Data
        _informatrionToSend[2], // Data 5
        _informatrionToSend[1], // Data 6
        _informatrionToSend[0]  // Data 7
    };

    stat = MCP2515_Transmit_Frame(TX_BUFFER_0, _address, 4, msg, error);
    if (stat != true)
    {
        // std::cout << " - not canOK "<<stat<<"\n";
        error = SOLOMotorControllers::Error::GENERAL_ERROR;
        return false;
    }
    // std::cout << "SendPSendPdoSyncdoSync - OK "<<stat<<"\n";
    return true;
}
bool MCP2515::SendPdoSync(int &error)
{
    bool stat;

    stat = MCP2515_Transmit_Frame(TX_BUFFER_0, 128, 0, NULL, error);
    if (stat != true)
    {
        // std::cout << "SendPdoSync - not canOK "<<stat<<"\n";
        error = SOLOMotorControllers::Error::GENERAL_ERROR;
        return false;
    }
    // std::cout << "SendPdoSync - OK "<<stat<<"\n";
    error = SOLOMotorControllers::Error::NO_ERROR_DETECTED;
    return true;
}

bool MCP2515::SendPdoRtr(long _address, int &error)
{
    bool stat;

    stat = MCP2515_Transmit_Frame(TX_BUFFER_0, _address, 0, NULL, error);
    if (stat != true)
    {
        // std::cout << "SendPdoSync - not canOK "<<stat<<"\n";
        error = SOLOMotorControllers::Error::GENERAL_ERROR;
        return false;
    }
    // std::cout << "SendPdoSync - OK "<<stat<<"\n";
    return true;
}