/**
 *******************************************************************************
 * @file    MCP2515.hpp
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the functions prototypes for communicate with MCP2515 IC
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.1.0
 *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)
 *******************************************************************************
 */

#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>
#include "Arduino.h"
#include "Handler.h"

/** @addtogroup MCP2515_Instructions MCP2515 Instructions
 * @{
 */
#define READ_STATUS_Instruction 0xA0
#define BIT_MODIFY_Instruction 0x05
#define RX_STATUS_Instruction 0xB0
#define RESET_Instruction 0xC0
#define WRITE_Instruction 0x02
#define READ_Instruction 0x03
/**
 * @}
 */

/** @addtogroup RTS_Instructions RTS Instructions
 * @{
 */
#define RTS_TX_BUF_0 0x81
#define RTS_TX_BUF_1 0x82
#define RTS_TX_BUF_2 0x84
/**
 * @}
 */

/** @addtogroup Read_RX_BUFFER_Instructions Read RX BUFFER Instructions
 * @brief SPI Commands point to the first byte in the buffer.
 * @{
 */
#define READ_RX_BUFFER_0 0x90      // Point to RXB0SIDH Register   (Address is 0x61)
#define READ_RX_BUFFER_0_DATA 0x92 // Point to RXB0D0   Register   (Address is 0x66)
#define READ_RX_BUFFER_1 0x94      // Point to RXB1SIDH Register   (Address is 0x71)
#define READ_RX_BUFFER_1_DATA 0x96 // Point to RXB1D0   Register   (Address is 0x76)
/**
 * @}
 */

/** @addtogroup LOAD_TX_BUFFER_Instructions LOAD TX BUFFER Instructions
 * @brief Load Transmitter Buffers with Standard Address and Data Bytes
 * @{
 */
#define LOAD_TX_BUFFER_0 0x40      // Points to TXB0SIDH Register  (Address is 0x31)
#define LOAD_TX_BUFFER_0_DATA 0x41 // Points to TXB0D0   Register  (Address is 0x36)
#define LOAD_TX_BUFFER_1 0x42      // Points to TXB1SIDH Register  (Address is 0x41)
#define LOAD_TX_BUFFER_1_DATA 0x43 // Points to TXB1D0   Register  (Address is 0x46)
#define LOAD_TX_BUFFER_2 0x44      // Points to TXB2SIDH Register  (Address is 0x51)
#define LOAD_TX_BUFFER_2_DATA 0x45 // Points to TXB2D0   Register  (Address is 0x56)
/**
 * @}
 */

//***** END MCP2515 Instructions

//***** Begin Registers

// Receiver Registers

/** @addtogroup Receiver_Buffer_0_Register_Address Receiver Buffer 0 Register Address
 * @{
 */
// Receiver Buffer 0 Register Address                                                                 //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define RXB0CTRL 0x60 // Receiver Buffer 0 Control Register                               ----    RXM1    RXM0    ----    RXRTR   BUKT    BUKT1   FILHIT0
#define RXB0SIDH 0x61 // Receiver Buffer 0 Standard Identifier Register High              SID10   SID9    SID8    SID7    SID6    SID5    SID4    SID3
#define RXB0SIDL 0x62 // Receiver Buffer 0 Standard Identifier Register Low               SID2    SID1    SID0    SRR     IDE     ----    EID17   EID16
#define RXB0EID8 0x63 // Receiver Buffer 0 Extended ID Register High                      EID15   EID14   EID13   EID12   EID11   EID10   EID9    EID8
#define RXB0EID0 0x64 // Receiver Buffer 0 Extended ID Register Low                       EID7    EID6    EID5    EID4    EID3    EID2    EID1    EID0
#define RXB0DLC 0x65  // Receiver Buffer 0 Data Length Code Register                      ----    RTR     RB1     RB0     DLC3    DLC2    DLC1    DLC0
#define RXB0D0 0x66   // Receiver Buffer 0 Data Byte 0 Register                           RB0D7   RB0D6   RB0D5   RB0D4   RB0D3   RB0D2   RB0D1   RB0D0
#define RXB0D1 0x67   // Receiver Buffer 0 Data Byte 1 Register                           RB0D7   RB0D6   RB0D5   RB0D4   RB0D3   RB0D2   RB0D1   RB0D0
#define RXB0D2 0x68   // Receiver Buffer 0 Data Byte 2 Register                           RB0D7   RB0D6   RB0D5   RB0D4   RB0D3   RB0D2   RB0D1   RB0D0
#define RXB0D3 0x69   // Receiver Buffer 0 Data Byte 3 Register                           RB0D7   RB0D6   RB0D5   RB0D4   RB0D3   RB0D2   RB0D1   RB0D0
#define RXB0D4 0x6A   // Receiver Buffer 0 Data Byte 4 Register                           RB0D7   RB0D6   RB0D5   RB0D4   RB0D3   RB0D2   RB0D1   RB0D0
#define RXB0D5 0x6B   // Receiver Buffer 0 Data Byte 5 Register                           RB0D7   RB0D6   RB0D5   RB0D4   RB0D3   RB0D2   RB0D1   RB0D0
#define RXB0D6 0x6C   // Receiver Buffer 0 Data Byte 6 Register                           RB0D7   RB0D6   RB0D5   RB0D4   RB0D3   RB0D2   RB0D1   RB0D0
#define RXB0D7 0x6D   // Receiver Buffer 0 Data Byte 7 Register                           RB0D7   RB0D6   RB0D5   RB0D4   RB0D3   RB0D2   RB0D1   RB0D0
/**
 * @}
 */

/** @addtogroup Receiver_Buffer_1_Register_Address Receiver Buffer 1 Register Address
 * @{
 */
// Receiver Buffer 1 Register Address                                                                 //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define RXB1CTRL 0x70 // Receiver Buffer 1 Control Register                               ----    RXM1    RXM0    ----    RXRTR   FILHIT2 FILHIT1 FILHIT0
#define RXB1SIDH 0x71 // Receiver Buffer 1 Standard ID Register High                      SID10   SID9    SID8    SID7    SID6    SID5    SID4    SID3
#define RXB1SIDL 0x72 // Receiver Buffer 1 Standard ID Register Low                       SID2    SID1    SID0    SRR     IDE     ----    EID17   EID16
#define RXB1EID8 0x73 // Receiver Buffer 1 Extended ID Register High                      EID15   EID14   EID13   EID12   EID11   EID10   EID9    EID8
#define RXB1EID0 0x74 // Receiver Buffer 1 Extended ID Register Low                       EID7    EID6    EID5    EID4    EID3    EID2    EID1    EID0
#define RXB1DLC 0x75  // Receiver Buffer 1 Data Length Code Register                      ----    RTR     RB1     RB0     DLC3    DLC2    DLC1    DLC0
#define RXB1D0 0x76   // Receiver Buffer 1 Data Byte 0 Register                           RB1D7   RB1D6   RB1D5   RB1D4   RB1D3   RB1D2   RB1D1   RB1D0
#define RXB1D1 0x77   // Receiver Buffer 1 Data Byte 1 Register                           RB1D7   RB1D6   RB1D5   RB1D4   RB1D3   RB1D2   RB1D1   RB1D0
#define RXB1D2 0x78   // Receiver Buffer 1 Data Byte 2 Register                           RB1D7   RB1D6   RB1D5   RB1D4   RB1D3   RB1D2   RB1D1   RB1D0
#define RXB1D3 0x79   // Receiver Buffer 1 Data Byte 3 Register                           RB1D7   RB1D6   RB1D5   RB1D4   RB1D3   RB1D2   RB1D1   RB1D0
#define RXB1D4 0x7A   // Receiver Buffer 1 Data Byte 4 Register                           RB1D7   RB1D6   RB1D5   RB1D4   RB1D3   RB1D2   RB1D1   RB1D0
#define RXB1D5 0x7B   // Receiver Buffer 1 Data Byte 5 Register                           RB1D7   RB1D6   RB1D5   RB1D4   RB1D3   RB1D2   RB1D1   RB1D0
#define RXB1D6 0x7C   // Receiver Buffer 1 Data Byte 6 Register                           RB1D7   RB1D6   RB1D5   RB1D4   RB1D3   RB1D2   RB1D1   RB1D0
#define RXB1D7 0x7D   // Receiver Buffer 1 Data Byte 7 Register                           RB1D7   RB1D6   RB1D5   RB1D4   RB1D3   RB1D2   RB1D1   RB1D0
/**
 * @}
 */

// Transmitter Registers

/** @addtogroup Transmitter_Buffer_0_Register_Address Transmitter Buffer 0 Register Address
 * @{
 */
// Transmitter Buffer 0 Register Address                                                              //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define TXB0CTRL 0x30 // Transmitter Buffer 0 Control Register                            ----    ABTF    MLOA    TXERR   TXREQ   ----    TXP1    TXP0
#define TXB0SIDH 0x31 // Transmitter Buffer 0 Standard ID Register High                   SID10   SID9    SID8    SID7    SID6    SID5    SID4    SID3
#define TXB0SIDL 0x32 // Transmitter Buffer 0 Standard ID Register Low                    SID2    SID1    SID0    ----    EXIDE   ----    EID17   EID16
#define TXB0EID8 0x33 // Transmitter Buffer 0 Extended ID Register High                   EID15   EID14   EID13   EID12   EID11   EID10   EID9    EID8
#define TXB0EID0 0x34 // Transmitter Buffer 0 Extended ID Register Low                    EID7    EID6    EID5    EID4    EID3    EID2    EID1    EID0
#define TXB0DLC 0x35  // Transmitter Buffer 0 Data Length Code Register                   ----    RTR     ----    ----    DLC3    DLC2    DLC1    DLC0
#define TXB0D0 0x36   // Transmitter Buffer 0 Data Byte 0 Register                        TB0D7   TB0D6   TB0D5   TB0D4   TB0D3   TB0D2   TB0D1   TB0D0
#define TXB0D1 0x37   // Transmitter Buffer 0 Data Byte 1 Register                        TB0D7   TB0D6   TB0D5   TB0D4   TB0D3   TB0D2   TB0D1   TB0D0
#define TXB0D2 0x38   // Transmitter Buffer 0 Data Byte 2 Register                        TB0D7   TB0D6   TB0D5   TB0D4   TB0D3   TB0D2   TB0D1   TB0D0
#define TXB0D3 0x39   // Transmitter Buffer 0 Data Byte 3 Register                        TB0D7   TB0D6   TB0D5   TB0D4   TB0D3   TB0D2   TB0D1   TB0D0
#define TXB0D4 0x3A   // Transmitter Buffer 0 Data Byte 4 Register                        TB0D7   TB0D6   TB0D5   TB0D4   TB0D3   TB0D2   TB0D1   TB0D0
#define TXB0D5 0x3B   // Transmitter Buffer 0 Data Byte 5 Register                        TB0D7   TB0D6   TB0D5   TB0D4   TB0D3   TB0D2   TB0D1   TB0D0
#define TXB0D6 0x3C   // Transmitter Buffer 0 Data Byte 6 Register                        TB0D7   TB0D6   TB0D5   TB0D4   TB0D3   TB0D2   TB0D1   TB0D0
#define TXB0D7 0x3D   // Transmitter Buffer 0 Data Byte 7 Register                        TB0D7   TB0D6   TB0D5   TB0D4   TB0D3   TB0D2   TB0D1   TB0D0
/**
 * @}
 */

/** @addtogroup Transmitter_Buffer_1_Register_Address Transmitter Buffer 1 Register Address
 * @{
 */
// Transmitter Buffer 1 Register Address                                                              //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define TXB1CTRL 0x40 // Transmitter Buffer 1 Control Register                            ----    ABTF    MLOA    TXERR   TXREQ   ----    TXP1    TXP0
#define TXB1SIDH 0x41 // Transmitter Buffer 1 Standard ID Register High                   SID10   SID9    SID8    SID7    SID6    SID5    SID4    SID3
#define TXB1SIDL 0x42 // Transmitter Buffer 1 Standard ID Register Low                    SID2    SID1    SID0    ----    EXIDE   ----    EID17   EID16
#define TXB1EID8 0x43 // Transmitter Buffer 1 Extended ID Register High                   EID15   EID14   EID13   EID12   EID11   EID10   EID9    EID8
#define TXB1EID0 0x44 // Transmitter Buffer 1 Extended ID Register Low                    EID7    EID6    EID5    EID4    EID3    EID2    EID1    EID0
#define TXB1DLC 0x45  // Transmitter Buffer 1 Data Length Code Register                   ----    RTR     ----    ----    DLC3    DLC2    DLC1    DLC0
#define TXB1D0 0x46   // Transmitter Buffer 1 Data Byte 0 Register                        TB1D7   TB1D6   TB1D5   TB1D4   TB1D3   TB1D2   TB1D1   TB1D0
#define TXB1D1 0x47   // Transmitter Buffer 1 Data Byte 1 Register                        TB1D7   TB1D6   TB1D5   TB1D4   TB1D3   TB1D2   TB1D1   TB1D0
#define TXB1D2 0x48   // Transmitter Buffer 1 Data Byte 2 Register                        TB1D7   TB1D6   TB1D5   TB1D4   TB1D3   TB1D2   TB1D1   TB1D0
#define TXB1D3 0x49   // Transmitter Buffer 1 Data Byte 3 Register                        TB1D7   TB1D6   TB1D5   TB1D4   TB1D3   TB1D2   TB1D1   TB1D0
#define TXB1D4 0x4A   // Transmitter Buffer 1 Data Byte 4 Register                        TB1D7   TB1D6   TB1D5   TB1D4   TB1D3   TB1D2   TB1D1   TB1D0
#define TXB1D5 0x4B   // Transmitter Buffer 1 Data Byte 5 Register                        TB1D7   TB1D6   TB1D5   TB1D4   TB1D3   TB1D2   TB1D1   TB1D0
#define TXB1D6 0x4C   // Transmitter Buffer 1 Data Byte 6 Register                        TB1D7   TB1D6   TB1D5   TB1D4   TB1D3   TB1D2   TB1D1   TB1D0
#define TXB1D7 0x4D   // Transmitter Buffer 1 Data Byte 7 Register                        TB1D7   TB1D6   TB1D5   TB1D4   TB1D3   TB1D2   TB1D1   TB1D0
/**
 * @}
 */

/** @addtogroup Transmitter_Buffer_2_Register_Address Transmitter Buffer 2 Register Address
 * @{
 */
// Transmitter Buffer 2 Register Address                                                              //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define TXB2CTRL 0x50 // Transmitter Buffer 2 Control Register                            ----    ABTF    MLOA    TXERR   TXREQ   ----    TXP1    TXP0
#define TXB2SIDH 0x51 // Transmitter Buffer 2 Standard ID Register High                   SID10   SID9    SID8    SID7    SID6    SID5    SID4    SID3
#define TXB2SIDL 0x52 // Transmitter Buffer 2 Standard ID Register Low                    SID2    SID1    SID0    ----    EXIDE   ----    EID17   EID16
#define TXB2EID8 0x53 // Transmitter Buffer 2 Extended ID Register High                   EID15   EID14   EID13   EID12   EID11   EID10   EID9    EID8
#define TXB2EID0 0x54 // Transmitter Buffer 2 Extended ID Register Low                    EID7    EID6    EID5    EID4    EID3    EID2    EID1    EID0
#define TXB2DLC 0x55  // Transmitter Buffer 2 Data Length Code Register                   ----    RTR     ----    ----    DLC3    DLC2    DLC1    DLC0
#define TXB2D0 0x56   // Transmitter Buffer 2 Data Byte 0 Register                        TB2D7   TB2D6   TB2D5   TB2D4   TB2D3   TB2D2   TB2D1   TB2D0
#define TXB2D1 0x57   // Transmitter Buffer 2 Data Byte 1 Register                        TB2D7   TB2D6   TB2D5   TB2D4   TB2D3   TB2D2   TB2D1   TB2D0
#define TXB2D2 0x58   // Transmitter Buffer 2 Data Byte 2 Register                        TB2D7   TB2D6   TB2D5   TB2D4   TB2D3   TB2D2   TB2D1   TB2D0
#define TXB2D3 0x59   // Transmitter Buffer 2 Data Byte 3 Register                        TB2D7   TB2D6   TB2D5   TB2D4   TB2D3   TB2D2   TB2D1   TB2D0
#define TXB2D4 0x5A   // Transmitter Buffer 2 Data Byte 4 Register                        TB2D7   TB2D6   TB2D5   TB2D4   TB2D3   TB2D2   TB2D1   TB2D0
#define TXB2D5 0x5B   // Transmitter Buffer 2 Data Byte 5 Register                        TB2D7   TB2D6   TB2D5   TB2D4   TB2D3   TB2D2   TB2D1   TB2D0
#define TXB2D6 0x5C   // Transmitter Buffer 2 Data Byte 6 Register                        TB2D7   TB2D6   TB2D5   TB2D4   TB2D3   TB2D2   TB2D1   TB2D0
#define TXB2D7 0x5D   // Transmitter Buffer 2 Data Byte 7 Register                        TB2D7   TB2D6   TB2D5   TB2D4   TB2D3   TB2D2   TB2D1   TB2D0
/**
 * @}
 */

// Control Registers

/** @addtogroup Hardware_Pins_Control_and_Status_Register_Address Hardware Pins Control and Status Register Address
 * @{
 */
// Hardware Pins Control and Status Register Address                                                  //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define BFPCTRL 0x0C   // Receiver    Hardware Pins Control and Status Register            ----    ----    B1BFS   B0BFS   B1BFE   B0BFE   B1BFM   B0BFM
#define TXRTSCTRL 0x0D // Transmitter Hardware Pins Control and Status Register            ----    ----    B2RST   B1RST   B0RST   B2RTSM  B1RTSM  B0RTSM
/**
 * @}
 */

/** @addtogroup Main_Control_Register_Address Main Control Register Address
 * @{
 */
// Main Control Register Address                                                                      //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define CANSTAT 0x0E // CAN Status  Register                                             OPMOD2  OPMOD1  OPMOD0  ----    ICOD2   ICOD1   ICOD0   ----
#define CANCTRL 0x0F // CAN Control Register                                             REQOP2  REQOP1  REQOP0  ABAT    OSM     CLKEN   CLKPRE1 CLKPRE0
/**
 * @}
 */

/** @addtogroup Bus_Error_Counter_Register_Address Bus Error Counter Register Address
 * @{
 */
// Bus Error Counter Register Address                                                                 //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define TEC 0x1C // Transmit Error Counter                                           TEC7    TEC6    TEC5    TEC4    TEC3    TEC2    TEC1    TEC0
#define REC 0x1D // Receive  Error Counter                                           REC7    REC6    REC5    REC4    REC3    REC2    REC1    REC0
/**
 * @}
 */

/** @addtogroup Bit_Timing_Configuration_Register_Address  Bit Timing Configuration Register Address
 * @{
 */
// Bit Timing Configuration Register Address                                                          //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define CNF3 0x28 // Configuration Register 3                                         SOF     WAKFIL  ----    ----    ----    PHSEG22 PHSEG21 PHSEG20
#define CNF2 0x29 // Configuration Register 2                                         BTLMODE SAM     PHSEG12 PHSEG11 PHSEG10 PRSEG2  PRSEG1  PRSEG0
#define CNF1 0x2A // Configuration Register 1                                         SJW1    SJW0    BRP5    BRP4    BRP3    BRP2    BRP1    BRP0
/**
 * @}
 */

/** @addtogroup Interrupt_and_Error_Flag_Register_Address  Interrupt and Error Flag Register Address
 * @{
 */
// Interrupt and Error Flag Register Address                                                          //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define CANINTE 0x2B // CAN Interrupt Enable Register                                    MERRE   WAKIE   ERRIE   TX2IE   TX1IE   TX0IE   RX1IE   RX0IE
#define CANINTF 0x2C // CAN Interrupt Flag   Register                                    MERRF   WAKIF   ERRIF   TX2IF   TX1IF   TX0IF   RX1IF   RX0IF
#define EFLG 0x2D    // Error Flag           Register                                    RX1OVR  RX0OVR  TXBO    TXEP    RXEP    TXWAR   RXWAR   EWARN
/**
 * @}
 */

// Masks and Filters Registers

/** @addtogroup Receiver_Mask_0_Registers_Address  Receiver Mask 0 Registers Address
 * @{
 */
// Receiver Mask 0 Registers Address                                                                  //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define RXM0SIDH 0x20 // Mask 0 Standard ID Register High                                 SID10   SID9    SID8    SID7    SID6    SID5    SID4    SID3
#define RXM0SIDL 0x21 // Mask 0 Standard ID Register Low                                  SID2    SID1    SID0    ----    ----    ----    EID17   EID16
#define RXM0EID8 0x22 // Mask 0 Extended ID Register High                                 EID15   EID14   EID13   EID12   EID11   EID10   EID9    EID8
#define RXM0EID0 0x23 // Mask 0 Extended ID Register Low                                  EID7    EID6    EID5    EID4    EID3    EID2    EID1    EID0
/**
 * @}
 */

/** @addtogroup Receiver_Mask_1_Registers_Address  Receiver Mask 1 Registers Address
 * @{
 */
// Receiver Mask 1 Registers Address                                                                  //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define RXM1SIDH 0x24 // Mask 1 Standard ID Register High                                 SID10   SID9    SID8    SID7    SID6    SID5    SID4    SID3
#define RXM1SIDL 0x25 // Mask 1 Standard ID Register Low                                  SID2    SID1    SID0    ----    ----    ----    EID17   EID16
#define RXM1EID8 0x26 // Mask 1 Extended ID Register High                                 EID15   EID14   EID13   EID12   EID11   EID10   EID9    EID8
#define RXM1EID0 0x27 // Mask 1 Extended ID Register Low                                  EID7    EID6    EID5    EID4    EID3    EID2    EID1    EID0
/**
 * @}
 */

/** @addtogroup Receiver_Filter_0_Registers_Address  Receiver Filter 0 Registers Address
 * @{
 */
// Receiver Filter 0 Registers Address                                                                //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define RXF0SIDH 0x00 // Filter 0 Standard ID Register High                               SID10   SID9    SID8    SID7    SID6    SID5    SID4    SID3
#define RXF0SIDL 0x01 // Filter 0 Standard ID Register Low                                SID2    SID1    SID0    ----    EXIDE   ----    EID17   EID16
#define RXF0EID8 0x02 // Filter 0 Extended ID Register High                               EID15   EID14   EID13   EID12   EID11   EID10   EID9    EID8
#define RXF0EID0 0x03 // Filter 0 Extended ID Register Low                                EID7    EID6    EID5    EID4    EID3    EID2    EID1    EID0
/**
 * @}
 */

/** @addtogroup Receiver_Filter_1_Registers_Address  Receiver Filter 1 Registers Address
 * @{
 */
// Receiver Filter 1 Registers Address                                                                //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define RXF1SIDH 0x04 // Filter 1 Standard ID Register High                               SID10   SID9    SID8    SID7    SID6    SID5    SID4    SID3
#define RXF1SIDL 0x05 // Filter 1 Standard ID Register Low                                SID2    SID1    SID0    ----    EXIDE   ----    EID17   EID16
#define RXF1EID8 0x06 // Filter 1 Extended ID Register High                               EID15   EID14   EID13   EID12   EID11   EID10   EID9    EID8
#define RXF1EID0 0x07 // Filter 1 Extended ID Register Low                                EID7    EID6    EID5    EID4    EID3    EID2    EID1    EID0
/**
 * @}
 */

/** @addtogroup Receiver_Filter_2_Registers_Address  Receiver Filter 2 Registers Address
 * @{
 */
// Receiver Filter 2 Registers Address                                                                //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define RXF2SIDH 0x08 // Filter 2 Standard ID Register High                               SID10   SID9    SID8    SID7    SID6    SID5    SID4    SID3
#define RXF2SIDL 0x09 // Filter 2 Standard ID Register Low                                SID2    SID1    SID0    ----    EXIDE   ----    EID17   EID16
#define RXF2EID8 0x0A // Filter 2 Extended ID Register High                               EID15   EID14   EID13   EID12   EID11   EID10   EID9    EID8
#define RXF2EID0 0x0B // Filter 2 Extended ID Register Low                                EID7    EID6    EID5    EID4    EID3    EID2    EID1    EID0
/**
 * @}
 */

/** @addtogroup Receiver_Filter_3_Registers_Address  Receiver Filter 3 Registers Address
 * @{
 */
// Receiver Filter 3 Registers Address                                                                //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define RXF3SIDH 0x10 // Filter 3 Standard ID Register High                               SID10   SID9    SID8    SID7    SID6    SID5    SID4    SID3
#define RXF3SIDL 0x11 // Filter 3 Standard ID Register Low                                SID2    SID1    SID0    ----    EXIDE   ----    EID17   EID16
#define RXF3EID8 0x12 // Filter 3 Extended ID Register High                               EID15   EID14   EID13   EID12   EID11   EID10   EID9    EID8
#define RXF3EID0 0x13 // Filter 3 Extended ID Register Low                                EID7    EID6    EID5    EID4    EID3    EID2    EID1    EID0
/**
 * @}
 */

/** @addtogroup Receiver_Filter_4_Registers_Address  Receiver Filter 4 Registers Address
 * @{
 */
// Receiver Filter 4 Registers Address                                                                //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define RXF4SIDH 0x14 // Filter 4 Standard ID Register High                               SID10   SID9    SID8    SID7    SID6    SID5    SID4    SID3
#define RXF4SIDL 0x15 // Filter 4 Standard ID Register Low                                SID2    SID1    SID0    ----    EXIDE   ----    EID17   EID16
#define RXF4EID8 0x16 // Filter 4 Extended ID Register High                               EID15   EID14   EID13   EID12   EID11   EID10   EID9    EID8
#define RXF4EID0 0x17 // Filter 4 Extended ID Register Low                                EID7    EID6    EID5    EID4    EID3    EID2    EID1    EID0
/**
 * @}
 */

/** @addtogroup Receiver_Filter_5_Registers_Address  Receiver Filter 5 Registers Address
 * @{
 */
// Receiver Filter 5 Registers Address                                                                //Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
#define RXF5SIDH 0x18 // Filter 5 Standard ID Register High                               SID10   SID9    SID8    SID7    SID6    SID5    SID4    SID3
#define RXF5SIDL 0x19 // Filter 5 Standard ID Register Low                                SID2    SID1    SID0    ----    EXIDE   ----    EID17   EID16
#define RXF5EID8 0x1A // Filter 5 Extended ID Register High                               EID15   EID14   EID13   EID12   EID11   EID10   EID9    EID8
#define RXF5EID0 0x1B // Filter 5 Extended ID Register Low                                EID7    EID6    EID5    EID4    EID3    EID2    EID1    EID0
/**
 * @}
 */

//***** END Registers

//***** BEGIN CONST Values

// BaudRate Values for 16MHz Frequency
#define MCP_16MHz_1000kBPS_CFG1 0x00
#define MCP_16MHz_1000kBPS_CFG2 0xD0
#define MCP_16MHz_1000kBPS_CFG3 0x82

#define MCP_16MHz_500kBPS_CFG1 0x00
#define MCP_16MHz_500kBPS_CFG2 0xF0
#define MCP_16MHz_500kBPS_CFG3 0x86

#define MCP_16MHz_250kBPS_CFG1 0x41
#define MCP_16MHz_250kBPS_CFG2 0xF1
#define MCP_16MHz_250kBPS_CFG3 0x85

#define MCP_16MHz_200kBPS_CFG1 0x01
#define MCP_16MHz_200kBPS_CFG2 0xFA
#define MCP_16MHz_200kBPS_CFG3 0x87

#define MCP_16MHz_125kBPS_CFG1 0x03
#define MCP_16MHz_125kBPS_CFG2 0xF0
#define MCP_16MHz_125kBPS_CFG3 0x86

#define MCP_16MHz_100kBPS_CFG1 0x03
#define MCP_16MHz_100kBPS_CFG2 0xFA
#define MCP_16MHz_100kBPS_CFG3 0x87

#define MCP_16MHz_80kBPS_CFG1 0x03
#define MCP_16MHz_80kBPS_CFG2 0xFF
#define MCP_16MHz_80kBPS_CFG3 0x87

#define MCP_16MHz_50kBPS_CFG1 0x07
#define MCP_16MHz_50kBPS_CFG2 0xFA
#define MCP_16MHz_50kBPS_CFG3 0x87

#define MCP_16MHz_40kBPS_CFG1 0x07
#define MCP_16MHz_40kBPS_CFG2 0xFF
#define MCP_16MHz_40kBPS_CFG3 0x87

#define MCP_16MHz_20kBPS_CFG1 0x0F
#define MCP_16MHz_20kBPS_CFG2 0xFF
#define MCP_16MHz_20kBPS_CFG3 0x87

#define MCP_16MHz_10kBPS_CFG1 0x1F
#define MCP_16MHz_10kBPS_CFG2 0xFF
#define MCP_16MHz_10kBPS_CFG3 0x87

#define MCP_16MHz_5kBPS_CFG1 0x3F
#define MCP_16MHz_5kBPS_CFG2 0xFF
#define MCP_16MHz_5kBPS_CFG3 0x87

// MCP2515 Modes of Operation
#define MODE_CONFIGURATION 0x80 // Configuration Mode
#define MODE_LISTEN_ONLY 0x60   // Listen-Only Mode
#define MODE_LOOPBACK 0x40      // Loopback Mode
#define MODE_SLEEP 0x20         // Sleep Mode
#define MODE_NORMAL 0x00        // Normal Mode

//***** END CONST Values

class MCP2515
{

private:
  uint8_t chipSelectPin;
  long countTimeout;

public:
  // Constructor
  MCP2515(uint8_t _chipSelectPin, long _countTimeout);

  enum MCP2515_MODE
  {
    CONFIGURATION_MODE,
    NORMAL_MODE,
    SLEEP_MODE,
    LISTEN_ONLY_MODE,
    LOOPBACK_MODE
  };

  enum MCP2515_TX_BUF
  {
    TX_BUFFER_0,
    TX_BUFFER_1,
    TX_BUFFER_2
  };

  enum MCP2515_RX_BUF
  {
    RX_BUFFER_0,
    RX_BUFFER_1
  };

  enum MCP2515_Error_Mode
  {
    ACTIVE_ERROR,
    TRANSMIT_ERROR_PASSIVE,
    RECEIVE_ERROR_PASSIVE,
    TRANSMIT_ERROR_WARNING,
    RECEIVE_ERROR_WARNING,
    BUS_OFF
  };

  // Methods
  static void MCP2515_ISR();
  void StartSPI();
  void EndSPI();
  void MCP2515_Reset();
  void MCP2515_Set_Mode(MCP2515_MODE _Mode);
  void MCP2515_Write_Register(uint8_t _Address, uint8_t _Value);
  void MCP2515_Bit_Modify_Register(uint8_t _Address, uint8_t _Mask, uint8_t _Value);
  bool MCP2515_Transmit_Frame(MCP2515_TX_BUF _TXBn, uint16_t _ID, uint8_t _DLC, uint8_t *_Data, int &error);
  void MCP2515_Receive_Frame(MCP2515_RX_BUF _RXBn, uint16_t *_ID, uint8_t *_DLC, uint8_t *_Data);
  void MCP2515_Receive_Frame_IT(MCP2515_RX_BUF _RXBn, uint8_t *_Data);
  void MCP2515_Set_BaudRate(uint16_t _BaudRate);
  void MCP2515_Set_ReceiveMask_SID(MCP2515_RX_BUF _RXBn, uint16_t _Mask);
  void MCP2515_Set_ReceiveMask_EID(MCP2515_RX_BUF _RXBn, uint16_t _Mask);
  void MCP2515_Init(uint16_t baudrate);

  void MCP2515_Clear_TxBuffer_Flag(MCP2515_TX_BUF _TXBn);
  void MCP2515_Clear_RxBuffer_Flag(MCP2515_RX_BUF _RXBn);
  void MCP2515_Enable_TxBuffer_INT(MCP2515_TX_BUF _TXBn);
  void MCP2515_Disable_TxBuffer_INT(MCP2515_TX_BUF _TXBn);
  void MCP2515_Enable_RxBuffer_INT(MCP2515_RX_BUF _RXBn);
  void MCP2515_Enable_Rollover();
  void MCP2515_Disable_Rollover();
  void MCP2515_Enable_MaskFilter(MCP2515_RX_BUF _RXBn);
  void MCP2515_Disable_MaskFilter(MCP2515_RX_BUF _RXBn);
  void Mcp2515ReadErrorMode(int &errorMode);

  uint8_t MCP2515_Read_RX_Status();
  uint8_t MCP2515_Read_Register(uint8_t _Address);
  uint8_t Mcp2515ReadReceiveErrorCounter();
  uint8_t Mcp2515ReadTransmitErrorCounter();

  bool CANOpenTransmit(uint8_t _address, uint16_t _object, uint8_t _subIndex, uint8_t *_informatrionToSend, int &error);
  bool CANOpenReceive(uint8_t _address, uint16_t _object, uint8_t _subIndex, uint8_t *_informatrionToSend, uint8_t *_informationReceived, int &error);

  bool SendPdoSync(int &error);
  bool SendPdoRtr(long _address, int &error);
  bool PDOTransmit(long _address, uint8_t *_informatrionToSend, int &error);
  bool PDOReceive(long _address, uint8_t *_informationReceived, int &error);
};

#endif // MCP2515_H