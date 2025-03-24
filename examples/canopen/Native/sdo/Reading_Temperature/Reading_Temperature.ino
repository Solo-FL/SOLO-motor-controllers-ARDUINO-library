// Copyright: (c) 2021-present, SOLO motor controllers project
// MIT License (see LICENSE file for more details)

/*
*    Title: SOLO Motor Controllers Arduino Library
*    Author: SOLOMotorControllers
*    Date: 2025
*    Code version: 5.5.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library

This Library is made by SOLOMotorControllers.COM
please visit:  https://www.SOLOMotorControllers.com/

*/

// EXAMPLE of how read the SOLO board temperature,
// every second we print the value of the temperature

// Importing SOLO Arduino library
#include "SOLOMotorControllersCanopenNative.h"

// instanciate a SOLO object
SOLOMotorControllersCanopenNative *SOLO_Obj1;

float Temperature = 0;
int errorSolo;

void setup()
{
  Serial.begin(115200);

  // Initialize the SOLO object
  int SOLODeviceAddress = 0;
  SOLO_Obj1 = new SOLOMotorControllersCanopenNative(SOLODeviceAddress);
}

void loop()
{
  // Reading
  Temperature = SOLO_Obj1->GetBoardTemperature(errorSolo);

  // Print
  Serial.println("Read from SOLO");
  Serial.println(Temperature, 2);
  Serial.println("Error");
  Serial.println(errorSolo);

  delay(1000);
}
