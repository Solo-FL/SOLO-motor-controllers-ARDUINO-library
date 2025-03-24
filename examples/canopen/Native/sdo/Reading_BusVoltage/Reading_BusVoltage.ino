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

// EXAMPLE of how read the SOLO Battery or Supply Input Voltage,
// every second we print the value of it.

// Importing SOLO Arduino library
#include "SOLOMotorControllersCanopenNative.h"

// instanciate a SOLO object
SOLOMotorControllersCanopenNative *SOLO_Obj1;

// SOLO board Temperature
float BusVoltage = 0;

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
  BusVoltage = SOLO_Obj1->GetBusVoltage();

  // Print
  Serial.println("\n Read from SOLO");
  Serial.println(BusVoltage, 2);

  delay(1000);
}
