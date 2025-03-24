// Copyright: (c) 2021-present, SOLO motor controllers project
// MIT License (see LICENSE file for more details)

/*
 *    Title: Torque Control of PMSM equipped with Incremental Encoders using Arduino and SOLO
 *    Author: SOLOMOTORCONTROLLERS
 *    Date: 2025
 *    Code version: 5.5.0
 *    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *    Please make sure you are applying the right wiring between SOLO and your ARDUINO
 *    The Code below has been tested on Arduino MEGA
 *    The Motor used for Testings: teknic m-2310P-LN-04K
 */

#include "SOLOMotorControllersUart.h"

// For this Test, make sure you have calibrated your Motor and Hall sensors before
// to know more please read: https://www.solomotorcontrollers.com/how-to-connect-calibrate-incremental-encoder-with-solo/

// instanciate a SOLO object:
SOLOMotorControllersUart *SOLO_Obj1;

// Desired Switching or PWM Frequency at Output
long pwmFrequency = 20;

// Motor's Number of Poles
long numberOfPoles = 8;

// Current Limit of the Motor
float currentLimit = 7.0;

// Motor's Number of Encoder Lines (PPR pre-quad)
long numberOfEncoderLines = 1000;

// Battery or Bus Voltage
float busVoltage = 0;

// Motor Torque feedback
float actualMotorTorque = 0;

// Motor speed feedback
long actualMotorSpeed = 0;

// Motor position feedback
long actualMotorPosition = 0;

void setup()
{
  // In this example, make sure you put SOLO into Closed-Loop Mode

  // High Speed High Performance Baudrate (Recommended)
  // Use this baudrate to have the best and real performance
  // of SOLO under all conditions.
  // Serial.begin(937500);

  // Low Speed Low Performance Baudrate
  // Use this baudrate only for devices that don't support
  // 937500 or 921600 baudrates.
  Serial.begin(115200);

  // Initialize the SOLO object
  int SOLODeviceAddress = 0;
  int chipSelectPin = 9;
  SOLO_Obj1 = new SOLOMotorControllersUart(SOLODeviceAddress, Serial1, SOLOMotorControllers::UartBaudrate::RATE_115200); // this line will exert error if built with ARDUINO UNO

  Serial.println("\n Trying to Connect To SOLO");
  delay(1000);
  // wait here till communication is established
  while (SOLO_Obj1->CommunicationIsWorking() == false)
  {
    delay(500);
  }

  Serial.println("\n Communication Established succuessfully!");

  // Initial Configuration of the device and the Motor
  SOLO_Obj1->SetOutputPwmFrequencyKhz(pwmFrequency);
  SOLO_Obj1->SetCurrentLimit(currentLimit);
  SOLO_Obj1->SetMotorPolesCounts(numberOfPoles);
  SOLO_Obj1->SetIncrementalEncoderLines(numberOfEncoderLines);
  SOLO_Obj1->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL);
  SOLO_Obj1->SetMotorType(SOLOMotorControllers::MotorType::BLDC_PMSM);
  SOLO_Obj1->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::ENCODERS);
  SOLO_Obj1->SetControlMode(SOLOMotorControllers::ControlMode::TORQUE_MODE);

  // run the motor identification to Auto-tune the current controller gains Kp and Ki needed for Torque Controlling
  // run ID. always after selecting the Motor Type!
  // ID. doesn't need to be called everytime, only one time after connection of a new motor it will be  enough
  // the ID. values will be remembered by SOLO after power recycling
  SOLO_Obj1->MotorParametersIdentification(SOLOMotorControllers::Action::START);
  Serial.println("\n Identifying the Motor");
  // wait at least for 2sec till ID. is done
  delay(2000);
}

void loop()
{

  // set the Direction on C.C.W.
  SOLO_Obj1->SetMotorDirection(SOLOMotorControllers::Direction::COUNTERCLOCKWISE);
  // set an arbitrary Positive torque reference
  SOLO_Obj1->SetTorqueReferenceIq(1.3);
  // wait till motor reaches to the reference
  delay(100);
  actualMotorTorque = SOLO_Obj1->GetQuadratureCurrentIqFeedback();
  Serial.println("\n Measured Iq/Torque[A]: ");
  Serial.println(actualMotorTorque);
  // wait for the motor to speed up naturally
  delay(1000);
  actualMotorSpeed = SOLO_Obj1->GetSpeedFeedback();
  Serial.println("\n Measured Speed[RPM]: ");
  Serial.println(actualMotorSpeed);
  // wait for the motor to speed up naturally
  delay(3000);

  // set the Direction on C.W.
  SOLO_Obj1->SetMotorDirection(SOLOMotorControllers::Direction::CLOCKWISE);
  // set an arbitrary Positive torque reference
  SOLO_Obj1->SetTorqueReferenceIq(0.85);
  // wait till motor reaches to the reference
  delay(100);
  actualMotorTorque = SOLO_Obj1->GetQuadratureCurrentIqFeedback();
  Serial.println("\n Measured Iq/Torque[A]: ");
  Serial.println(actualMotorTorque);
  // wait for the motor to speed up naturally
  delay(1000);
  actualMotorSpeed = SOLO_Obj1->GetSpeedFeedback();
  Serial.println("\n Measured Speed[RPM]: ");
  Serial.println(actualMotorSpeed);
  // wait for the motor to speed up naturally
  delay(3000);
}
