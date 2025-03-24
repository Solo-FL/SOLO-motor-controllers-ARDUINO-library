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
  config.parameterName = SOLOMotorControllersCanopenNative::PdoParameterName::POSITION_REFERENCE;
  config.parameterCobId = 0x201;
  config.isPdoParameterEnable = true;
  config.isRrtParameterEnable = true;
  config.syncParameterCount = 0;
  // send the configuration to SOLO
  solo->SetPdoParameterConfig(config, errorSolo);
  delay(100);

  // if CONFIGURATION already done you can avoid and use the next commad:
  // solo->pdoParameterCobIdByPdoParameterName[PdoParameterName::POSITION_REFERENCE] = 0x201;
}

void loop()
{
  // ACTIVE section
  solo->SetPdoPositionReference(1710592, errorSolo);
  Serial.print("Set value: 1710592 ERROR: ");
  Serial.println(errorSolo);

  long getValue = solo->GetPositionReference(errorSolo);
  Serial.print("Get value: ");
  Serial.print(getValue);
  Serial.print(" ERROR: ");
  Serial.println(errorSolo);

  delay(1000);
}