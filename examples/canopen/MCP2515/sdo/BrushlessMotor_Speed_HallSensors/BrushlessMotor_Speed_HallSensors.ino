// Copyright: (c) 2021-present, SOLO motor controllers project
// MIT License (see LICENSE file for more details)

/*
 *    Title: Speed Control of a Brushless Motor with Arduino and SOLO using HALL sensors
 *    Author: SOLOMOTORCONTROLLERS
 *    Date: 2024
 *    Code version: 5.4.0
 *    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *    Please make sure you are applying the right wiring between SOLO and your ARDUINO
 *    The Code below has been tested on Arduino UNO
 *    The Motor used for Testings: DB56C036030-A
 *    Read more about this code here:https://www.solomotorcontrollers.com/speed-torque-control-brushless-motor-hall-sensors-arduino-code/
 */

#include "SOLOMotorControllersCanopenMcp2515.h"

// For this Test, make sure you have calibrated your HALL sensors before
// to know more please read: https://www.solomotorcontrollers.com/hall-sensors-to-solo-for-controlling-speed-torque-brushless-motor/

// In this example, make sure you put SOLO into Closed-Loop by
// pressing the Piano Switch NO# 5 DOWN. in SOLO UNO
// The Speed controller Kp and Ki gains, in this Example are set manually using
// two potentiometers on SOLO_UNO board as we are controlling SOLO in Analogue mode

/* The Piano Switch Setup on SOLO UNO are as below since SOLO will be commandaed in Analogue Mode with PWM:
 *  PIN 5 Down: closed-loop
 *  PIN 4 Down : Speed Mode (Analogue mode)
 *  PIN 1 Down and PIN 2 UP:  BLDC_PMSM motor type (Analogue mode)
 *  PIN 3 UP ( Not in DFU mode )
 */

// instanciate a SOLO object:
SOLOMotorControllersCanopenMcp2515 *SOLO_Obj1;

// the device address of SOLO:
unsigned char SOLO_address1 = 0;

// Desired Switching or PWM Frequency at Output
long pwmFrequency = 16;

// Motor's Number of Poles
long numberOfPoles = 8;

// Current Limit of the Motor
float currentLimit = 12.5;

// Define Desired Torque referrrence
float desiredSpeed_RPM = 0.0;

// Converted value to PWM duty cycle for Iq
int desiredDutyCycle_Speed = 0;

// Converted value to PWM duty cycle for currentLimit
int desiredDutyCycle_CurrentLimit = 0;

// Define the Max Measurable current in SOLO UNO for 100% duty Cycle
float MaxMeasurableCurrent_SOLO_UNO = 32.0;

// Define the Max SPEED in RPM for SOLO UNO for 100% duty Cycle for BLDC-PMSM Normal type
int MaxSpeed_BLDC_PMSM_Normal_SOLO_UNO = 8000;

// Battery or Bus Voltage
float busVoltage = 0;

// Motor speed feedback
long actualMotorSpeed_RPM = 0;

// Motor Iq (torque) feedback
float actualMotorTorque_Iq = 0;

void setup()
{

  // High Speed High Performance Baudrate (Recommended)
  // Use this baudrate to have the best and real performance
  // of SOLO under all conditions.
  // Serial.begin(937500);

  // Low Speed Low Performance Baudrate
  // Use this baudrate only for devices that don't support
  // 937500 or 921600 baudrates.
  Serial.begin(115200);

  // Initialize the SOLO object
  int SOLOdeviceAddress = 0;
  int chipSelectPin = 9;
  SOLO_Obj1 = new SOLOMotorControllersCanopenMcp2515(SOLOdeviceAddress, chipSelectPin);

  Serial.println("\n Trying to Connect To SOLO");
  delay(1000);
  // wait here till communication is established
  while (SOLO_Obj1->CommunicationIsWorking() == false)
  {
    delay(500);
  }

  Serial.println("\n Communication Established succuessfully!");

  // dummy read after Serial.println to open the UART line
  SOLO_Obj1->GetBusVoltage();
  // Initial Configurations
  SOLO_Obj1->SetOutputPwmFrequencyKhz(pwmFrequency);
  SOLO_Obj1->SetCurrentLimit(currentLimit);

  // select Digital Mode
  SOLO_Obj1->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL);
  SOLO_Obj1->SetMotorType(SOLOMotorControllers::MotorType::BLDC_PMSM);

  // Operate while using Hall sensors
  SOLO_Obj1->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::HALL_SENSORS);

  // run the motor identification
  // run ID. always after selecting the Motor Type!
  SOLO_Obj1->MotorParametersIdentification(SOLOMotorControllers::Action::START);

  Serial.println("\n Identifying the Motor");
  // wait at least for 2sec till ID. is done
  delay(2000);

  // dummy read after Serial.println to open the UART line
  SOLO_Obj1->GetBusVoltage();

  // Go back to Analogue Mode to accept PWM as Reference for Torque and Speed
  SOLO_Obj1->SetCommandMode(SOLOMotorControllers::CommandMode::ANALOGUE);
  // Enable PWM pins of Arduiono with Fixed frequency greater than 5kHz

  pinMode(3, OUTPUT); // speed adjuster connected to "S/T" on SOLO UNO
  pinMode(9, OUTPUT); // current Limit adjuster connected to "P/F" on SOLO UNO

  pinMode(2, OUTPUT); // Direction Control Pin
}

void loop()
{

  // Converted value to PWM duty cycle for desired current Limit on 8 bit PWM of Arduino
  desiredDutyCycle_CurrentLimit = (int)(255 - (255 * (currentLimit / MaxMeasurableCurrent_SOLO_UNO)));
  // Set the right Duty cycle on Pin ~9 for current Limit
  analogWrite(9, desiredDutyCycle_CurrentLimit);

  // Define the desired Speed in RPM
  desiredSpeed_RPM = 1000.0;

  // Define the Direction of Rotation
  digitalWrite(2, LOW);

  // Converted value to PWM duty cycle for desired speed on 8 bit PWM of Arduino
  desiredDutyCycle_Speed = (int)(255 * (desiredSpeed_RPM / MaxSpeed_BLDC_PMSM_Normal_SOLO_UNO));

  // Set the right Duty cycle on Pin ~3 for Speed
  analogWrite(3, desiredDutyCycle_Speed);

  // wait till motor reaches to the reference
  delay(500);

  actualMotorSpeed_RPM = SOLO_Obj1->GetSpeedFeedback();
  Serial.println("\n Speed [RPM]: ");
  Serial.print(actualMotorSpeed_RPM);

  // wait for user to read
  delay(3000);

  // Define the desired Speed in RPM
  desiredSpeed_RPM = 3000.0;

  // Define the Direction of Rotation
  digitalWrite(2, HIGH);

  // Converted value to PWM duty cycle for desired speed on 8 bit PWM of Arduino
  desiredDutyCycle_Speed = (int)(255 * (desiredSpeed_RPM / MaxSpeed_BLDC_PMSM_Normal_SOLO_UNO));

  // Set the right Duty cycle on Pin ~3 for Speed
  analogWrite(3, desiredDutyCycle_Speed);

  // wait till motor reaches to the reference
  delay(500);

  actualMotorSpeed_RPM = SOLO_Obj1->GetSpeedFeedback();
  Serial.println("\n Speed [RPM]: ");
  Serial.print(actualMotorSpeed_RPM);

  // wait for user to read
  delay(3000);
}
