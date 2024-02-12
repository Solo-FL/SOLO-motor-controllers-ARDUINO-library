// Copyright: (c) 2021-present, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers Arduino Library
*    Author: SOLOMotorControllers
*    Date: 2022
*    Code version: 4.0.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library

This Library is made by SOLOMotorControllers.COM
please visit:  https://www.SOLOMotorControllers.com/

*/

// In this example we want:
//     STEP 1: to print the command mode of SOLO and the error status of the reading operation
//     STEP 2: if we read the command mode without error we want to change the command mode of SOLO.
#include "SOLOMotorControllersCanopen.h"

SOLOMotorControllers *solo;
int error;
long commandMode;
bool setIsSuccesfull;

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial Init");

  // Initialize the SOLO object
  int SOLOdeviceAddress = 0;
  int chipSelectPin = 9;
  solo = new SOLOMotorControllersCanopen(SOLOdeviceAddress, chipSelectPin);
}

void loop()
{
  delay(500);

  // STEP 1
  // response : is the Command Mode reading from SOLO device
  // error : after the execution of the fuction will have the error status of the execution
  commandMode = solo->GetCommandMode(error);
  
  // error is not mandatory, we can call the function without it, as other examples:
  // response = solo->GetCommandMode();

  // we print the info:
  Serial.println((String) "COMMAND MODE: " + commandMode + " ERROR: " + error);

  // STEP 2
  // if we have no error we want to change the command mode of SOLO
  // we can compare error with SOLOMotorControllersError enum or int value. Equal code:
  //     error == SOLOMotorControllers::SOLOMotorControllersError::NO_ERROR_DETECTED
  //     error == 0
  if (error == SOLOMotorControllers::Error::NO_ERROR_DETECTED)
  {
    // we check the commandMode readed value.
    // we can compare commandMode with CommandMode enum or int value. Equal code:
    //     commandMode == SOLOMotorControllers::CommandMode::ANALOGUE
    //     commandMode == 0
    if (commandMode == SOLOMotorControllers::CommandMode::ANALOGUE)
    {

      // setIsSuccesfull : set return if the set was succesfull
      // SOLOMotorControllers::CommandMode::DIGITAL : is the command mode i want to set to SOLO.
      // error : after the execution of the fuction will have the error status of the execution
      setIsSuccesfull = solo->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL, error);

      // error is not mandatory, we can call the function without it, as for the setIsSuccesfull, other examples:
      // setIsSuccesfull = solo->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL);
      // solo->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL, error);
      // solo->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL);

      // we print the info:
      Serial.println((String) "SET COMMAND SUCCESS: " + setIsSuccesfull + " ERROR: " + error);
    }
    else
    {
      // in this situation we want to set analogue as command mode in SOLO
      // we choose the alternative code with less herror and status controlling:
      solo->SetCommandMode(SOLOMotorControllers::CommandMode::ANALOGUE);

      // Alternative are:
      // setIsSuccesfull = solo->SetCommandMode(SOLOMotorControllers::CommandMode::ANALOGUE, error);
      // setIsSuccesfull = solo->SetCommandMode(SOLOMotorControllers::CommandMode::ANALOGUE);
      // solo->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL, error);
    }
  }
}