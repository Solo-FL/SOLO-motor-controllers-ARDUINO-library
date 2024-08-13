// Copyright: (c) 2021-present, SOLO motor controllers project
// MIT License (see LICENSE file for more details)

/*
*    Title: SOLO Motor Controllers Arduino Library
*    Author: SOLOMotorControllers
*    Date: 2024
*    Code version: 5.4.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library

This Library is made by SOLOMotorControllers.COM
please visit:  https://www.SOLOMotorControllers.com/

*/

// EXAMPLE of how to set the Motor number of poles,
// every second we repit the setting and the reading of it

// Importing SOLO Arduino library
#include "SOLOMotorControllersCanopenNative.h"

// instanciate a SOLO object
SOLOMotorControllersCanopenNative *SOLO_Obj1;

// Motor's Number of Poles
long NumberOfPoles_write = 4; // Write
long NumberOfPoles_read = 0;  // Read

void setup()
{
  Serial.begin(115200);

  // Initialize the SOLO object
  int SOLOdeviceAddress = 0;
  SOLO_Obj1 = new SOLOMotorControllersCanopenNative(SOLOdeviceAddress);
}

void loop()
{
  // Setting
  SOLO_Obj1->SetMotorPolesCounts(NumberOfPoles_write);

  // Reading
  NumberOfPoles_read = SOLO_Obj1->GetMotorPolesCounts();

  // Print
  Serial.println("\nRead from SOLO");
  Serial.println(NumberOfPoles_read);

  delay(1000);
}