// Copyright: (c) 2021-present, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
 *    Title: Position Control of a BLDC-PMSM Motor with Arduino and SOLO
 *    Author: SOLOMOTORCONTROLLERS
 *    Date: 2022
 *    Code version: 4.0.0
 *    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *    Please make sure you are applying the right wiring between SOLO and your ARDUINO
 *    The Code below has been tested on Arduino UNO
 *    The Motor used for Testings: teknic m-2310P-LN-04K
 *    Read more about this code here:https://www.solomotorcontrollers.com/position-control-brushless-arduino-and-solo/
 */

#include "SOLOMotorControllersCanopen.h"

// For this Test, make sure you have calibrated your Encoder before
// to know more please read: https://www.solomotorcontrollers.com/how-to-connect-calibrate-incremental-encoder-with-solo/

// instanciate a SOLO object:
SOLOMotorControllers *SOLO_Obj1;

// the device address of SOLO:
unsigned char SOLO_address1 = 0;

// Desired Switching or PWM Frequency at Output
long pwmFrequency = 20;

// Motor's Number of Poles
long numberOfPoles = 8;

// Motor's Number of Encoder Lines (PPR pre-quad)
long numberOfEncoderLines = 1000;

// Speed controller Kp
float speedControllerKp = 0.1;

// Speed controller Ki
float speedControllerKi = 0.008;

// Position controller Kp
float positionControllerKp = 0.12;

// Position controller Ki
float positionControllerKi = 0.02;

// Current Limit of the Motor
float currentLimit = 7.5;

// Battery or Bus Voltage
float busVoltage = 0;

// Desired Speed Limit[RPM]
long desiredSpeedLimit = 1000;

// Desired Position Reference
long desiredPositionReference = 0;

// Motor speed feedback
long actualMotorSpeed = 0;

// Motor position feedback
long actualMotorPosition = 0;

void setup()
{
  // In this example, make sure you put SOLO into Closed-Loop by
  //  pressing the Piano Switch NO# 5 DOWN. in SOLO UNO

  // used for Monitoring in Arduino Only
  Serial.begin(115200);

  // Initialize the SOLO object
  int SOLOdeviceAddress = 0;
  int chipSelectPin = 9; // SPI CS pin for CANshield
  SOLO_Obj1 = new SOLOMotorControllersCanopen(SOLOdeviceAddress, chipSelectPin);

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
  SOLO_Obj1->SetMotorPolesCounts(numberOfPoles);
  SOLO_Obj1->SetIncrementalEncoderLines(numberOfEncoderLines);
  SOLO_Obj1->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL);
  SOLO_Obj1->SetMotorType(SOLOMotorControllers::MotorType::BLDC_PMSM);

  // run the motor identification to Auto-tune the current controller gains Kp and Ki needed for Torque Loop
  // run ID. always after selecting the Motor Type!
  // ID. doesn't need to be called everytime, only one time after wiring up the Motor will be enough
  // the ID. values will be remembered by SOLO after power recycling
  SOLO_Obj1->MotorParametersIdentification(SOLOMotorControllers::Action::START);
  Serial.println("\n Identifying the Motor");
  // wait at least for 2sec till ID. is done
  delay(2000);

  // Operate while using Quadrature Incremental Encoders
  SOLO_Obj1->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::ENCODERS);
  SOLO_Obj1->SetControlMode(SOLOMotorControllers::ControlMode::POSITION_MODE);
  // Speed Controller Tunings
  SOLO_Obj1->SetSpeedControllerKp(speedControllerKp);
  SOLO_Obj1->SetSpeedControllerKi(speedControllerKi);
  // Position Controller Tunings
  SOLO_Obj1->SetPositionControllerKp(positionControllerKp);
  SOLO_Obj1->SetPositionControllerKi(positionControllerKi);
}

void loop()
{

  // set a desired Speed Limit for trajectory in RPM
  desiredSpeedLimit = 1200;
  SOLO_Obj1->SetSpeedLimit(desiredSpeedLimit);
  // set a positive desired Position Reference in terms of pulses
  SOLO_Obj1->SetPositionReference(+10500);
  delay(1000);
  actualMotorSpeed = SOLO_Obj1->GetSpeedFeedback();
  Serial.println("\n Measured Speed[RPM]: ");
  Serial.println(actualMotorSpeed);
  // wait till motor reaches to the reference
  delay(8000);
  actualMotorPosition = SOLO_Obj1->GetPositionCountsFeedback();
  Serial.println("\n Number of Pulses passed: ");
  Serial.println(actualMotorPosition);

  // set a new desired Speed Limit for trajectory in RPM
  desiredSpeedLimit = 2500;
  SOLO_Obj1->SetSpeedLimit(desiredSpeedLimit);
  // set a negative desired Position Reference in terms of pulses
  SOLO_Obj1->SetPositionReference(-320000);
  delay(1000);
  actualMotorSpeed = SOLO_Obj1->GetSpeedFeedback();
  Serial.println("\n Measured Speed[RPM]: ");
  Serial.println(actualMotorSpeed);
  // wait till motor reaches to the reference
  delay(8000);
  actualMotorPosition = SOLO_Obj1->GetPositionCountsFeedback();
  Serial.println("\n Number of Pulses passed: ");
  Serial.println(actualMotorPosition);
}
