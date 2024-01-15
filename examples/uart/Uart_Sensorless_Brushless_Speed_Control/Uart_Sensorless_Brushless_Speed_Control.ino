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

// instanciate a SOLO object:
SOLOMotorControllers *SOLO_Obj1;

// the device address of SOLO:
unsigned char SOLO_address1 = 0;

// Desired Switching or PWM Frequency at Output
long pwmFrequency = 75;

// Motor's Number of Poles
long numberOfPoles = 8;

// Speed controller Kp
float speedControllerKp = 0.04;

// Speed controller Ki
float speedControllerKi = 0.006;

// Current Limit of the Motor
float currentLimit = 16.55;

// Battery of Bus Voltage
float busVoltage = 0;

// Desired Speed Reference
long desiredMotorSpeed = 0;

// Motor speed feedback
long actualMotorSpeed = 0;

void setup()
{
  // In this example, make sure you put SOLO into Closed-Loop by
  //  pressing the Piano Switch NO# 5 DOWN. in SOLO UNO

  // High Speed High Performance Baudrate (Recommended)
  // Use this baudrate to have the best and real performance
  // of SOLO under all conditions.
  // Serial.begin(937500);

  // Low Speed Low Performance Baudrate
  // Use this baudrate only for devices that don't support
  // 937500 or 921600 baudrates.
  Serial.begin(115200);

  // Initialize the SOLO object
  SOLO_Obj1 = new SOLOMotorControllersUart(SOLO_address1);
  delay(1000);

  Serial.println("\n Trying to Connect To SOLO");
  delay(1000);
  // wait here till communication is established
  while (SOLO_Obj1->CommunicationIsWorking() == false)
  {
    delay(500);
  }

  Serial.println("\n Communication Established succuessfully!");

  // Initial Configurations
  SOLO_Obj1->SetOutputPwmFrequencyKhz(pwmFrequency);
  SOLO_Obj1->SetCurrentLimit(currentLimit);

  // select Digital Mode
  SOLO_Obj1->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL);

  SOLO_Obj1->SetMotorType(SOLOMotorControllers::MotorType::BLDC_PMSM);

  // run the motor identification
  // run ID. always after selecting the Motor Type!
  SOLO_Obj1->MotorParametersIdentification(SOLOMotorControllers::Action::START);

  Serial.println("\n Identifying the Motor");

  // wait at least for 2sec till ID. is done
  delay(2000);

  // Operate in Sensor-less Mode
  SOLO_Obj1->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::SENSORLESS);

  // Control The Speed
  SOLO_Obj1->SetControlMode(SOLOMotorControllers::ControlMode::SPEED_MODE);

  // Controller Tunings
  SOLO_Obj1->SetSpeedControllerKp(speedControllerKp);
  SOLO_Obj1->SetSpeedControllerKi(speedControllerKi);
}

void loop()
{

  // set the Direction on C.W.
  SOLO_Obj1->SetMotorDirection(SOLOMotorControllers::Direction::CLOCKWISE);

  // set a new reference for speed
  desiredMotorSpeed = 5000;
  SOLO_Obj1->SetSpeedReference(desiredMotorSpeed);

  // wait till motor reaches to the reference
  delay(2000);

  actualMotorSpeed = SOLO_Obj1->GetSpeedFeedback();
  Serial.println("\n Motor Speed: ");
  Serial.println(actualMotorSpeed);

  // set the Direction on C.C.W.
  SOLO_Obj1->SetMotorDirection(SOLOMotorControllers::Direction::CLOCKWISE);

  // set a new reference for speed
  desiredMotorSpeed = 1500;
  SOLO_Obj1->SetSpeedReference(desiredMotorSpeed);

  // wait till motor reaches to the reference
  delay(2000);

  actualMotorSpeed = SOLO_Obj1->GetSpeedFeedback();
  Serial.println("\n Motor Speed: ");
  Serial.println(actualMotorSpeed);

  // stop the motor
  desiredMotorSpeed = 0;
  SOLO_Obj1->SetSpeedReference(desiredMotorSpeed);
  delay(2000);
}