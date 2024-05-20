// Copyright: (c) 2021-present, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers Arduino Library
*    Author: SOLOMotorControllers
*    Date: 2024
*    Code version: 5.3.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library

This Library is made by SOLOMotorControllers.COM
please visit:  https://www.SOLOMotorControllers.com/

*/

// EXAMPLE of how to WRITE a Generic message on CANBus using SOLO CAN library for MCP2515
// In this Example, the line is read continously and ID, DCL and all the  data bytes recieved are printed
// The user can use this example to work with other CAN MODULES in the Line except SOLO

// Importing SOLO Arduino library
#include "SOLOMotorControllersCanopenMcp2515.h"

// instanciate a SOLO object as Canopen IMPORTANT
SOLOMotorControllersCanopenMcp2515 *SOLO_Obj1;

// Init writing variable
uint16_t ID_Write = 605; // int
uint8_t DLC_Write = 8;
uint8_t DataWrite[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
int ErrorWrite = 0;
String SOLOMotorControllersErrors[10] = {"NO_ERROR_DETECTED", "GENERAL_ERROR", "NO_PROCESSED_COMMAND", "OUT_OF_RANGE_SETTING", "PACKET_FAILURE_TRIAL_ATTEMPTS_OVERFLOW", "RECEIVE_TIMEOUT_ERROR", "ABORT_OBJECT", "ABORT_VALUE", "MCP2515_TRANSMIT_ARBITRATION_LOST", "MCP2515_TRANSMIT_ERROR"};

void setup()
{
  Serial.begin(115200);

  // Initialize the SOLO object
  int SOLOdeviceAddress = 0;
  int chipSelectPin = 9;
  SOLO_Obj1 = new SOLOMotorControllersCanopenMcp2515(SOLOdeviceAddress, chipSelectPin);
}

void loop()
{
  // writing
  Serial.print("NEW WRITE: ");
  SOLO_Obj1->GenericCanbusWrite(ID_Write, &DLC_Write, DataWrite, ErrorWrite);
  Serial.println(SOLOMotorControllersErrors[ErrorWrite]);
  delay(1000);
}