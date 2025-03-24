// Copyright: (c) 2021-present, SOLO motor controllers project
// MIT License (see LICENSE file for more details)

/*
     Title: Position Control of a BLDC-PMSM Motor with Arduino and SOLO - Position Profile
     Author: SOLOMOTORCONTROLLERS
     Date: 2025
     Code version: 5.5.0
     Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
     Totorial article at: https://www.solomotorcontrollers.com/blog/canopen-brushless-motor-position-profile-controlling-arduino/

     Please make sure you are applying the right wiring between SOLO and your ARDUINO
     The Code below has been tested on Arduino UNO
     The Motor used for Testings: teknic m-2310P-LN-04K
     Read more about this code here:https://www.solomotorcontrollers.com/position-control-brushless-arduino-and-solo/
*/

#include "SOLOMotorControllersCanopenMcp2515.h"
#define PI 3.1415926535897932384626433832795
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

// For this Test, make sure you have calibrated your Encoder before
// to know more please read: https://www.solomotorcontrollers.com/how-to-connect-calibrate-incremental-encoder-with-solo/

// instanciate a SOLO object:
SOLOMotorControllersCanopenMcp2515 *SOLO_Obj1;

// the device address of SOLO:
unsigned char SOLO_address1 = 0;

// Desired Switching or PWM Frequency at Output
long pwmFrequency = 20;

// Motor's Number of Poles
long numberOfPoles = 8;

// Motor's Number of Encoder Lines (PPR pre-quad)
long numberOfEncoderLines = 1000;

// Speed controller Kp
float speedControllerKp = 0.08;

// Speed controller Ki
float speedControllerKi = 0.008;

// Position controller Kp
float positionControllerKp = 0.25;

// Position controller Ki
float positionControllerKi = 0.02;

// Current Limit of the Motor
float currentLimit = 10.0;

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
  int SOLODeviceAddress = 0;
  int chipSelectPin = 9; // SPI CS pin for CANshield
  SOLO_Obj1 = new SOLOMotorControllersCanopenMcp2515(SOLODeviceAddress, chipSelectPin);

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
  // Homming
  // Speed Controller Tunings
  SOLO_Obj1->SetSpeedControllerKp(speedControllerKp);
  SOLO_Obj1->SetSpeedControllerKi(speedControllerKi);
  desiredSpeedLimit = 2000;
  SOLO_Obj1->SetSpeedLimit(desiredSpeedLimit);
  SOLO_Obj1->SetPositionReference(0);
  actualMotorPosition = SOLO_Obj1->GetPositionCountsFeedback();
  Serial.println("\n Number of Pulses passed: ");
  Serial.println(actualMotorPosition);
  delay(2000);

  // Sine profile
  // set a desired Speed Limit for trajectory in RPM
  desiredSpeedLimit = 6000;
  SOLO_Obj1->SetSpeedLimit(desiredSpeedLimit);
  for (int j = 0; j <= 3; j += 1)
  {
    for (float i = 0; i <= 2 * PI; i += 0.01)
    {
      SOLO_Obj1->SetPositionReference(+80000 * sin(i));
      delay(1);
      actualMotorPosition = SOLO_Obj1->GetPositionCountsFeedback();
      Serial.println("\n Number of Pulses passed: ");
      Serial.println(actualMotorPosition);
    }
  }
  delay(1000);

  desiredPositionReference = -1;
  // set a desired Speed Limit for trajectory in RPM
  desiredSpeedLimit = 2000;
  SOLO_Obj1->SetSpeedLimit(desiredSpeedLimit);
  // Speed Controller Tunings
  SOLO_Obj1->SetSpeedControllerKp(0.03);
  SOLO_Obj1->SetSpeedControllerKi(0.003);

  // Step profile
  for (int j = 0; j < 2; j += 1)
  {
    desiredPositionReference = -1 * 80000 * sgn(desiredPositionReference);
    SOLO_Obj1->SetPositionReference(desiredPositionReference);
    delay(3000);
    actualMotorPosition = SOLO_Obj1->GetPositionCountsFeedback();
    Serial.println("\n Number of Pulses passed: ");
    Serial.println(actualMotorPosition);
  }
  delay(1000);
}
