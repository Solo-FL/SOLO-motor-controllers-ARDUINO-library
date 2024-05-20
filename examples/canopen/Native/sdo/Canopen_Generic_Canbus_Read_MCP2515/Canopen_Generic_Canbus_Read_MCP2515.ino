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

// EXAMPLE of how to READ a Generic message on CANBus using SOLO CAN library for MCP2515
// In this Example, the line is read continously and ID, DCL and all the  data bytes recieved are printed
// The user can use this example to work with other CAN MODULES in the Line except SOLO

// Importing SOLO Arduino library
#include "SOLOMotorControllersCanopenNative.h"

// instanciate a SOLO object as Canopen IMPORTANT
SOLOMotorControllersCanopenNative *SOLO_Obj1;

// Init reading variable
uint16_t ID_Read;
uint8_t DLC_Read;
uint8_t DataRead[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void setup()
{
  Serial.begin(115200);

  // Initialize the SOLO object
  int SOLOdeviceAddress = 0;
  SOLO_Obj1 = new SOLOMotorControllersCanopenNative(SOLOdeviceAddress);
}

void loop()
{
  // Reading
  Serial.println("NEW READING");
  SOLO_Obj1->GenericCanbusRead(&ID_Read, &DLC_Read, DataRead);

  if (ID_Read != 0)
  { // if ID_Read == 0 mean no data recived
    // Printing ID
    Serial.print("ID: ");
    Serial.println(ID_Read, HEX);
    // Printing DLC
    Serial.print("DLC: ");
    Serial.println(DLC_Read);
    // Printing Data Read
    for (int i = 0; i < DLC_Read; i++)
    {
      Serial.print((String) "Byte[" + i + "]: ");
      Serial.println(DataRead[i], HEX);
    }
  }

  delay(100);
}
