// Copyright: (c) 2024-present, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
 *    Title: Torque Control of PMSM equipped with Incremental Encoders using Arduino and SOLO
 *    Author: SOLOMOTORCONTROLLERS
 *    Date: 2024
 *    Code version: 5.3.0
 *    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *    Please make sure you are applying the right wiring between SOLO and your ARDUINO
 *    The Code below has been tested on Arduino UNO
 *    The Motor used for Testings: teknic m-2310P-LN-04K
 */

#include <SOLOMotorControllersCanopenNative.h>

SOLOMotorControllersCanopenNative *solo;

int error;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Initialize the SOLO object
  int SOLOdeviceAddress = 0;
  solo = new SOLOMotorControllersCanopenNative(SOLOdeviceAddress);

  // 1 time needed CONFIGURATION:
  Serial.println("PdoParameterConfig:");
  SOLOMotorControllersCanopenNative::PdoParameterConfig config;
  config.parameterName = SOLOMotorControllersCanopenNative::PdoParameterName::POSITION_REFERENCE;
  config.parameterCobId = 0x201;
  config.isPdoParameterEnable = true;
  config.isRrtParameterEnable = true;
  config.syncParameterCount = 0;
  // send the configuration to SOLO
  solo->SetPdoParameterConfig(config, error);
  delay(100);

  // if CONFIGURATION already done you can avoid and use the next commad:
  // solo->pdoParameterCobIdByPdoParameterName[PdoParameterName::POSITION_REFERENCE] = 0x201;
}

void loop()
{
  // ACTIVE section
  solo->SetPdoPositionReference(1710592, error);
  Serial.print("Set value: 1710592 ERROR: ");
  Serial.println(error);

  long getValue = solo->GetPositionReference(error);
  Serial.print("Get value: ");
  Serial.print(getValue);
  Serial.print(" ERROR: ");
  Serial.println(error);

  delay(1000);
}