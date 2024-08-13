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

// IN THIS EXAMPLE WE WILL SHOW HOW TO WORK WITH DIFFERENT SERIAL (ARDUINO UNO have only 1 serial)

#include "SOLOMotorControllersUart.h"

SOLOMotorControllersUart *SOLO_Obj1;
float Temperature = 0;
int error;

void setup()
{
  // SOLO Initialization:
  //    0 is the address of SOLO device
  //    Serial1 is the Serial port 1. On ARDUINO DUE are PIN 18 19 a reference here: https://www.arduino.cc/reference/en/language/functions/communication/serial/
  //    SOLOMotorControllers::UartBaudrate::RATE_115200 is the baudrate of UART
  SOLO_Obj1 = new SOLOMotorControllersUart(0, Serial1, SOLOMotorControllers::UartBaudrate::RATE_115200); // this line will give error if build selecting ARDUINO UNO
  Serial.begin(115200);
}

void loop()
{
  // Reading
  Temperature = SOLO_Obj1->GetBoardTemperature(error);

  // Print
  Serial.println("\n Read from SOLO");
  Serial.println(Temperature, 2);
  Serial.println("Error");
  Serial.println(error);

  delay(1000);
}
