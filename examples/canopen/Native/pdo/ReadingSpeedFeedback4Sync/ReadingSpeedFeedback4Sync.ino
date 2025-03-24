// Copyright: (c) 2025-present, SOLO motor controllers project
// MIT License (see LICENSE file for more details)

/*
 *    Title: Torque Control of PMSM equipped with Incremental Encoders using Arduino and SOLO
 *    Author: SOLOMOTORCONTROLLERS
 *    Date: 2025
 *    Code version: 5.5.0
 *    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *    Please make sure you are applying the right wiring between SOLO and your ARDUINO
 *    The Code below has been tested on Arduino UNO
 *    The Motor used for Testings: teknic m-2310P-LN-04K
 */

#include <SOLOMotorControllersCanopenNative.h>

SOLOMotorControllersCanopenNative *solo;

int errorSolo;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Initialize the SOLO object
  int SOLODeviceAddress = 0;
  solo = new SOLOMotorControllersCanopenNative(SOLODeviceAddress);

  // 1 time needed CONFIGURATION:
  Serial.println("PdoParameterConfig:");
  SOLOMotorControllersCanopenNative::PdoParameterConfig config;
  config.parameterName = SOLOMotorControllersCanopenNative::PdoParameterName::SPEED_FEEDBACK;
  config.parameterCobId = 0x280;
  config.isPdoParameterEnable = true;
  config.isRrtParameterEnable = true;
  config.syncParameterCount = 4;
  // send the configuration to SOLO
  solo->SetPdoParameterConfig(config, errorSolo);
  delay(100);

  // if CONFIGURATION already done you can avoid and use the next commad:
  // solo->pdoParameterCobIdByPdoParameterName[PdoParameterName::SPEED_FEEDBACK ] = 0x280;
}

void loop()
{
  // ACTIVE section
  // send the sync message
  solo->SendPdoSync(errorSolo);
  solo->SendPdoSync(errorSolo);
  solo->SendPdoSync(errorSolo);
  solo->SendPdoSync(errorSolo);
  delay(50);

  // read the older value in the PDO buffer
  long getValue = solo->GetPdoSpeedFeedback(errorSolo);
  Serial.print("READ VALUE: ");
  Serial.print(getValue);
  Serial.print(" ERROR: ");
  Serial.println(errorSolo);

  // send the sync message
  solo->SendPdoSync(errorSolo);
  solo->SendPdoSync(errorSolo);
  solo->SendPdoSync(errorSolo);
  solo->SendPdoSync(errorSolo);
  delay(50);

  // read the older value in the PDO buffer
  getValue = solo->GetPdoSpeedFeedback(errorSolo);
  Serial.print("READ VALUE: ");
  Serial.print(getValue);
  Serial.print(" ERROR: ");
  Serial.println(errorSolo);

  delay(1000);
}