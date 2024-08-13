// Copyright: (c) 2024-present, SOLO motor controllers project
// MIT License (see LICENSE file for more details)

/*
 *    Title: Torque Control of PMSM equipped with Incremental Encoders using Arduino and SOLO
 *    Author: SOLOMOTORCONTROLLERS
 *    Date: 2024
 *    Code version: 5.4.0
 *    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *    Please make sure you are applying the right wiring between SOLO and your ARDUINO
 *    The Code below has been tested on Arduino UNO
 *    The Motor used for Testings: teknic m-2310P-LN-04K
 */

#include <SOLOMotorControllersCanopenMcp2515.h>

SOLOMotorControllersCanopenMcp2515 *solo;

int error;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Initialize the SOLO object
  int SOLOdeviceAddress = 0;
  int chipSelectPin = 9;
  solo = new SOLOMotorControllersCanopenMcp2515(SOLOdeviceAddress, chipSelectPin);

  // 1 time needed CONFIGURATION:
  Serial.println("PdoParameterConfig:");
  SOLOMotorControllersCanopenMcp2515::PdoParameterConfig config;
  config.parameterName = SOLOMotorControllersCanopenMcp2515::PdoParameterName::TORQUE_REFERENCE_IQ;
  config.parameterCobId = 0x201;
  config.isPdoParameterEnable = true;
  config.isRrtParameterEnable = true;
  config.syncParameterCount = 0;
  // send the configuration to SOLO
  solo->SetPdoParameterConfig(config, error);
  delay(100);

  // if CONFIGURATION already done you can avoid and use the next commad:
  // solo->pdoParameterCobIdByPdoParameterName[PdoParameterName::TORQUE_REFERENCE_IQ] = 0x201;
}

void loop()
{
  // ACTIVE section
  solo->SetPdoTorqueReferenceIq(2, error);
  Serial.print("Set value: 2 ERROR: ");
  Serial.println(error);

  long getValue = solo->GetTorqueReferenceIq(error);
  Serial.print("Get value: ");
  Serial.print(getValue);
  Serial.print(" ERROR: ");
  Serial.println(error);

  delay(1000);
}