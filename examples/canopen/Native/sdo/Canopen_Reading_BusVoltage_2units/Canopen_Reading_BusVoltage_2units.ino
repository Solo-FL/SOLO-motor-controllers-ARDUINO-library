// Copyright: (c) 2021-present, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers Arduino Library
*    Author: SOLOMotorControllers
*    Date: 2024
*    Code version: 5.3.1
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library

This Library is made by SOLOMotorControllers.COM
please visit:  https://www.SOLOMotorControllers.com/

*/
// EXAMPLE of how read the SOLO Battery or Supply Input Voltage,
// every second we print the value of it.

// Importing SOLO Arduino library
#define ARDUINO_CAN_NATIVE_SUPPORTED
#include "SOLOMotorControllersCanopenNative.h"

// instanciate a SOLO object
SOLOMotorControllersCanopenNative *SOLO_1;
SOLOMotorControllersCanopenNative *SOLO_2;

// SOLO board Temperature
float BusVoltage = 0;

// Boudrate Arduino will use it for the comunication (SOLO need the same value, is possible to set in the Motion Terminal)
SOLOMotorControllers::CanbusBaudrate baudrate = SOLOMotorControllers::CanbusBaudrate::RATE_1000;

// Iteration before interrupt the receive from SOLO
long millisecondsTimeout = 50;

void setup()
{
  Serial.begin(115200);

  // Initialize the SOLO object
  SOLO_1 = new SOLOMotorControllersCanopenNative(0, baudrate, millisecondsTimeout); // Solo with 0 Device Address
  SOLO_2 = new SOLOMotorControllersCanopenNative(6, baudrate, millisecondsTimeout); // Solo with 6 Device Address
}

void loop()
{
  // Reading
  BusVoltage = SOLO_1->GetBusVoltage();

  // Print
  Serial.print("Read from SOLO 1: ");
  Serial.println(BusVoltage, 2);

  // Reading
  BusVoltage = SOLO_2->GetBusVoltage();

  // Print
  Serial.print("Read from SOLO 2: ");
  Serial.println(BusVoltage, 2);

  delay(1000);
}
