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

#include "SOLOMotorControllersUart.h"
SOLOMotorControllers *mySolo;

// Basic example to show how to use ARDUINO SOLO library in Sebertooth / roboclaw manner
// the scope is to provide a basic starting point for the user in order to now more about the conversion
// (we suggest to use the ARDUINO SOLO Library without myCommands as will be hard to read the loop -> see cleanLoop alternative)

void setup()
{
  mySolo = new SOLOMotorControllersUart();
  Serial.begin(115200);

  // set motor configuration
  // is not needed to be at every arduino setup, as data will store in SOLO
  // comment it after 1 time SOLO configuration
  myCommands(0);

  delay(1000);
}

void loop()
{
  // set motor direction clockwise
  myCommands(4);

  // set speed 10000 RPM
  myCommands(2);
  delay(2000);

  // set speed 30000 RPM
  myCommands(3);
  delay(2000);

  // set speed 0 RPM (stop the motor)
  myCommands(1);
  delay(1000);

  // set motor direction COUNTERCLOCKWISE
  myCommands(5);

  // set speed 10000 RPM
  myCommands(2);
  delay(2000);

  // set speed 30000 RPM
  myCommands(3);
  delay(2000);

  // set speed 0 RPM (stop the motor)
  myCommands(1);
  delay(1000);
}

// This method will be the equivalent of Sabertooth .write or roboclaw .ForwardM1, .BackwardM1 , ...
// the commandNumber is customizable, the affect of every action is customizable too.
// ACTUAL COMMAND NUMBER TABLE (what every commandNumber will do to the motor)
// 0 -> motor configuration
// 1 -> speed 0 RPM (stop the motor)
// 2 -> speed 10000 RPM
// 3 -> speed 30000 RPM
// 4 -> direction clockwise / forward
// 5 -> direction COUNTERCLOCKWISE / bacward
void myCommands(int commandNumber)
{
  switch (commandNumber)
  {
  case 0:
    myMotorConfiguration1();
    break;

  case 1:
    mySolo->SetSpeedReference(0);
    break;

  case 2:
    mySolo->SetSpeedReference(10000);
    break;

  case 3:
    mySolo->SetSpeedReference(30000);
    break;

  case 4:
    mySolo->SetMotorDirection(SOLOMotorControllers::Direction::CLOCKWISE);
    break;

  case 5:
    mySolo->SetMotorDirection(SOLOMotorControllers::Direction::COUNTERCLOCKWISE);
    break;

  default:
    Serial.println((String) "ERROR command " + commandNumber + " no supported");
  }
}

// Configuration example for 4150KV, 4x4 SCT 550 model
// Read more about this motor model here:https://www.solomotorcontrollers.com/drive-fast-drone-rc-car-brushless-motors-arduino-solo-sensorless
//  is possible to create a different myMotorConfiguration2 or modify this myMotorConfiguration1 based to the user motor
void myMotorConfiguration1()
{
  // --- Waiting Communication Established succuessfully ---
  while (mySolo->CommunicationIsWorking() == false)
  {
    delay(500);
  }

  // --- Communication Established succuessfully ---

  // --- Initial Configurations ---
  mySolo->SetOutputPwmFrequencyKhz(79);
  mySolo->SetCurrentLimit(32.0);
  mySolo->SetMotorPolesCounts(2);

  // select Digital Mode
  mySolo->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL);

  mySolo->SetMotorType(SOLOMotorControllers::MotorType::BLDC_PMSM_ULTRAFAST);

  // run the motor identification
  // run ID. always after selecting the Motor Type!
  mySolo->MotorParametersIdentification(SOLOMotorControllers::Action::START);

  // --- Identifying the Motor ---

  // wait at least for 2sec till ID. is done
  delay(2000);

  // Operate in Sensor-less Mode
  mySolo->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::SENSORLESS);

  // Control The Speed
  mySolo->SetControlMode(SOLOMotorControllers::ControlMode::SPEED_MODE);

  // Controller Tunings
  mySolo->SetSpeedControllerKp(0.03);
  mySolo->SetSpeedControllerKi(0.001);

  Serial.println("1 Time motor configuration done. (avoid to do it at every lunch of the program)");
}

// EXTRA, this method is not used but show how the code of lopp can be without using the myCommands but only SOLO ARDUINO Library
void cleanLoop()
{
  // set motor direction clockwise
  mySolo->SetMotorDirection(SOLOMotorControllers::Direction::CLOCKWISE);

  // set speed 10000 RPM
  mySolo->SetSpeedReference(10000);
  delay(2000);

  // set speed 30000 RPM
  mySolo->SetSpeedReference(30000);
  delay(2000);

  // set speed 0 RPM (stop the motor)
  mySolo->SetSpeedReference(0);
  delay(1000);

  // set motor direction COUNTERCLOCKWISE
  mySolo->SetMotorDirection(SOLOMotorControllers::Direction::COUNTERCLOCKWISE);

  // set speed 10000 RPM
  mySolo->SetSpeedReference(10000);
  delay(2000);

  // set speed 30000 RPM
  mySolo->SetSpeedReference(30000);
  delay(2000);

  // set speed 0 RPM (stop the motor)
  mySolo->SetSpeedReference(0);
  delay(1000);
}
