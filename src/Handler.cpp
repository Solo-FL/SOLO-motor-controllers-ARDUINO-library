/**
 *******************************************************************************
 * @file    Handler.cpp
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the functions interrupt based for CANopen
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.0.0
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)
 *******************************************************************************
 */

#include "Handler.h"

#define RXFSIDH_BASE 0x00
#define RXFSIDL_BASE 0x01

volatile uint8_t canIntFlag;
volatile bool isCanBufEmpty[CAN_BUFF_SIZE] = {true, true, true, true, true, true, true, true, true, true, true, true, true, true, true};
volatile uint8_t canBuf[CAN_BUFF_SIZE][12];
volatile uint8_t writeIndex = 0;
MCP2515 *_MCP2515;
void ISR_Handler()
{
    int i;
    // find the first possible empty spot
    for (i = 0; i < CAN_BUFF_SIZE; i++)
    {
        if (isCanBufEmpty[i] == true)
        {
            isCanBufEmpty[i] = false;
            writeIndex = i;
            break;
        }
    }

    // if spot is not present write in last time position +1
    if (i == CAN_BUFF_SIZE)
    {
        writeIndex++;

        // make the writeIndex circular
        if (writeIndex > CAN_BUFF_SIZE)
        {
            writeIndex = 0;
        }
    }

    _MCP2515->MCP2515_Receive_Frame_IT(MCP2515::MCP2515_RX_BUF::RX_BUFFER_1, canBuf[writeIndex]);

    canIntFlag = 1;
}

void enableNodeFilter(uint8_t node_number, uint8_t filter_number)
{
    uint16_t id = 0x580 + node_number;
    enableAddrFilter(id, filter_number);
}

void enableAddrFilter(uint16_t addr, uint8_t filter_number)
{
    if (filter_number > 3)
    {
        filter_number = 3;
    }
    _MCP2515->MCP2515_Set_Mode(MCP2515::MCP2515_MODE::CONFIGURATION_MODE);

    if (filter_number == 0)
    {
        filter_number--;
    }
    _MCP2515->MCP2515_Write_Register(RXM1SIDH, 0xFF);
    _MCP2515->MCP2515_Write_Register(RXM1SIDL, 0xFF);
    _MCP2515->MCP2515_Write_Register(RXFSIDH_BASE + (4 * (filter_number + 3)), (uint8_t)(addr >> 3));
    _MCP2515->MCP2515_Write_Register(RXFSIDL_BASE + (4 * (filter_number + 3)), (uint8_t)(addr << 5));
    _MCP2515->MCP2515_Enable_MaskFilter(MCP2515::MCP2515_RX_BUF::RX_BUFFER_1);

    _MCP2515->MCP2515_Set_Mode(MCP2515::MCP2515_MODE::NORMAL_MODE);
}

int enableRangeFilter(uint16_t start_addr, uint16_t end_addr, uint8_t filter_number)
{
    uint8_t difference = end_addr ^ start_addr;
    uint8_t bit_counter = 0;

    while (difference)
    {
        difference /= 2;
        bit_counter++;
    }
    if (filter_number > 3)
    {
        filter_number = 3;
    }
    _MCP2515->MCP2515_Set_Mode(MCP2515::MCP2515_MODE::CONFIGURATION_MODE);

    if (filter_number == 0)
    {
        filter_number--;
    }
    _MCP2515->MCP2515_Write_Register(RXM1SIDH, (0xFF << (bit_counter > 3 ? (bit_counter - 3) : 0)));
    _MCP2515->MCP2515_Write_Register(RXM1SIDL, (0xE0 >> (bit_counter > 3 ? 3 : bit_counter)));
    _MCP2515->MCP2515_Write_Register(RXFSIDH_BASE + (4 * (filter_number + 3)), (uint8_t)(start_addr >> 3));
    _MCP2515->MCP2515_Write_Register(RXFSIDL_BASE + (4 * (filter_number + 3)), (uint8_t)(start_addr << 5));
    _MCP2515->MCP2515_Enable_MaskFilter(MCP2515::MCP2515_RX_BUF::RX_BUFFER_1);

    _MCP2515->MCP2515_Set_Mode(MCP2515::MCP2515_MODE::NORMAL_MODE);
    return bit_counter;
}

void disableAllFilters()
{
    _MCP2515->MCP2515_Set_Mode(MCP2515::MCP2515_MODE::CONFIGURATION_MODE);
    _MCP2515->MCP2515_Disable_MaskFilter(MCP2515::MCP2515_RX_BUF::RX_BUFFER_1);
    _MCP2515->MCP2515_Set_Mode(MCP2515::MCP2515_MODE::NORMAL_MODE);
}

void enableAllFilters()
{
    _MCP2515->MCP2515_Set_Mode(MCP2515::MCP2515_MODE::CONFIGURATION_MODE);
    _MCP2515->MCP2515_Enable_MaskFilter(MCP2515::MCP2515_RX_BUF::RX_BUFFER_1);
    _MCP2515->MCP2515_Set_Mode(MCP2515::MCP2515_MODE::NORMAL_MODE);
}

void removeAllFilters()
{
    _MCP2515->MCP2515_Set_Mode(MCP2515::MCP2515_MODE::CONFIGURATION_MODE);
    _MCP2515->MCP2515_Enable_MaskFilter(MCP2515::MCP2515_RX_BUF::RX_BUFFER_1);
    for (int i = 0; i < 3; i++)
    {
        enableAddrFilter(0, i);
    }
    _MCP2515->MCP2515_Set_Mode(MCP2515::MCP2515_MODE::NORMAL_MODE);
}

bool getCanIntFlag()
{
    if (canIntFlag == 0)
    {
        return false;
    }
    else if (canIntFlag == 1)
    {
        return true;
    }
}