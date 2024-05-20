// Copyright: (c) 2021-present, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
 *    Title: How to Drive Fast Drone or RC car Brushless Motors using ARDUINO and SOLO in Sensoless Mode
 *    Author: SOLOMOTORCONTROLLERS
 *    Date: 2024
 *    Code version: 5.3.0
 *    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *    Please make sure you are applying the right wiring between SOLO and your ARDUINO
 *    The Code below has been tested on Arduino UNO, The Motor used for Testings: 4150KV, 4x4 SCT 550
 *    Read more about this code here:https://www.solomotorcontrollers.com/drive-fast-drone-rc-car-brushless-motors-arduino-solo-sensorless
 */

#include "SOLOMotorControllersCanopenNative.h"

// instanciate a SOLO object:
SOLOMotorControllersCanopenNative *SOLO_Obj1;

// Desired Switching or PWM Frequency at Output
long pwmFrequency = 79;

// Motor's Number of Poles
long numberOfPoles = 2;

// Speed controller Kp
float speedControllerKp = 0.03;

// Speed controller Ki
float speedControllerKi = 0.001;

// Current Limit of the Motor
float currentLimit = 32.0;

// Battery of Bus Voltage
float busVoltage = 0;

// Desired Speed Reference
long desiredMotorSpeed = 0;

// Motor speed feedback
long actualMotorSpeed = 0;

void setup()
{

  // In this example, make sure you put SOLO into Closed-Loop Mode

  // used for Monitoring in Arduino Only
  Serial.begin(115200);

  // Initialize the SOLO object
  int SOLOdeviceAddress = 0;
  SOLO_Obj1 = new SOLOMotorControllersCanopenNative(SOLOdeviceAddress);

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
  SOLO_Obj1->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL);
  SOLO_Obj1->SetMotorType(SOLOMotorControllers::MotorType::BLDC_PMSM_ULTRAFAST);
  SOLO_Obj1->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::SENSORLESS);
  SOLO_Obj1->SetSpeedControllerKp(speedControllerKp);
  SOLO_Obj1->SetSpeedControllerKi(speedControllerKi);
  SOLO_Obj1->SetControlMode(SOLOMotorControllers::ControlMode::SPEED_MODE);

  // run the motor identification to Auto-tune the current controller gains Kp and Ki needed for Torque Loop
  // run ID. always after selecting the Motor Type!
  // ID. doesn't need to be called everytime, only one time after wiring up the Motor will be enough
  // the ID. values will be remembered by SOLO after power recycling
  SOLO_Obj1->MotorParametersIdentification(SOLOMotorControllers::Action::START);
  Serial.println("\n Identifying the Motor");
  // wait at least for 2sec till ID. is done
  delay(2000);
}

void loop()
{

  // set the Direction on C.W.
  SOLO_Obj1->SetMotorDirection(SOLOMotorControllers::Direction::CLOCKWISE);

  // set a new reference for speed [RPM]
  desiredMotorSpeed = 10000;
  SOLO_Obj1->SetSpeedReference(desiredMotorSpeed);

  // wait till motor reaches to the reference
  delay(5000);

  actualMotorSpeed = SOLO_Obj1->GetSpeedFeedback();
  Serial.println("\n Motor Speed: ");
  Serial.println(actualMotorSpeed);

  // set the Direction on C.C.W.
  SOLO_Obj1->SetMotorDirection(SOLOMotorControllers::Direction::COUNTERCLOCKWISE);

  // set a new reference for speed [RPM]
  desiredMotorSpeed = 30000;
  SOLO_Obj1->SetSpeedReference(desiredMotorSpeed);

  // wait till motor reaches to the reference
  delay(5000);

  actualMotorSpeed = SOLO_Obj1->GetSpeedFeedback();
  Serial.println("\n Motor Speed: ");
  Serial.println(actualMotorSpeed);

  // stop the motor
  desiredMotorSpeed = 0;
  SOLO_Obj1->SetSpeedReference(desiredMotorSpeed);
  delay(2000);
}
