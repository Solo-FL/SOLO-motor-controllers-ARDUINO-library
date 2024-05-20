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

// EXAMPLE of how read the SOLO Battery or Supply Input Voltage,
// every second we print the value of it.

// Importing SOLO Arduino library
#include "SOLOMotorControllersCanopenMcp2515.h"

// instanciate a SOLO object
SOLOMotorControllersCanopenMcp2515 *SOLO_Obj1;

// SOLO board Temperature
float BusVoltage = 0;

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
  // Reading
  BusVoltage = SOLO_Obj1->GetBusVoltage();

  // Print
  Serial.println("\n Read from SOLO");
  Serial.println(BusVoltage, 2);

  delay(1000);
}
