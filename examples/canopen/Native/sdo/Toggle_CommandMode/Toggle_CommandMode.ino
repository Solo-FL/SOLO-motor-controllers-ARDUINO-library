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

// In this example we want:
//     STEP 1: to print the command mode of SOLO and the errorSolo status of the reading operation
//     STEP 2: if we read the command mode without errorSolo we want to change the command mode of SOLO.
#include "SOLOMotorControllersCanopenNative.h"

SOLOMotorControllersCanopenNative *solo;
int errorSolo;
long commandMode;
bool setIsSuccesfull;

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial Init");

  // Initialize the SOLO object
  int SOLODeviceAddress = 0;
  solo = new SOLOMotorControllersCanopenNative(SOLODeviceAddress);
}

void loop()
{
  delay(500);

  // STEP 1
  // response : is the Command Mode reading from SOLO device
  // errorSolo : after the execution of the fuction will have the errorSolo status of the execution
  commandMode = solo->GetCommandMode(errorSolo);
  
  // errorSolo is not mandatory, we can call the function without it, as other examples:
  // response = solo->GetCommandMode();

  // we print the info:
  Serial.println((String) "COMMAND MODE: " + commandMode + " ERROR: " + errorSolo);

  // STEP 2
  // if we have no errorSolo we want to change the command mode of SOLO
  // we can compare errorSolo with SOLOMotorControllersError enum or int value. Equal code:
  //     errorSolo == SOLOMotorControllers::SOLOMotorControllersError::NO_ERROR_DETECTED
  //     errorSolo == 0
  if (errorSolo == SOLOMotorControllers::Error::NO_ERROR_DETECTED)
  {
    // we check the commandMode readed value.
    // we can compare commandMode with CommandMode enum or int value. Equal code:
    //     commandMode == SOLOMotorControllers::CommandMode::ANALOGUE
    //     commandMode == 0
    if (commandMode == SOLOMotorControllers::CommandMode::ANALOGUE)
    {

      // setIsSuccesfull : set return if the set was succesfull
      // SOLOMotorControllers::CommandMode::DIGITAL : is the command mode i want to set to SOLO.
      // errorSolo : after the execution of the fuction will have the errorSolo status of the execution
      setIsSuccesfull = solo->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL, errorSolo);

      // errorSolo is not mandatory, we can call the function without it, as for the setIsSuccesfull, other examples:
      // setIsSuccesfull = solo->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL);
      // solo->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL, errorSolo);
      // solo->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL);

      // we print the info:
      Serial.println((String) "SET COMMAND SUCCESS: " + setIsSuccesfull + " ERROR: " + errorSolo);
    }
    else
    {
      // in this situation we want to set analogue as command mode in SOLO
      // we choose the alternative code with less herror and status controlling:
      solo->SetCommandMode(SOLOMotorControllers::CommandMode::ANALOGUE);

      // Alternative are:
      // setIsSuccesfull = solo->SetCommandMode(SOLOMotorControllers::CommandMode::ANALOGUE, errorSolo);
      // setIsSuccesfull = solo->SetCommandMode(SOLOMotorControllers::CommandMode::ANALOGUE);
      // solo->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL, errorSolo);
    }
  }
}