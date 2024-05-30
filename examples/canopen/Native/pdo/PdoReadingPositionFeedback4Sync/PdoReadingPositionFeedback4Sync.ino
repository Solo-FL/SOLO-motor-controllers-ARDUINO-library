// Copyright: (c) 2024-present, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
 *    Title: Torque Control of PMSM equipped with Incremental Encoders using Arduino and SOLO
 *    Author: SOLOMOTORCONTROLLERS
 *    Date: 2024
 *    Code version: 5.3.1
 *    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *    Please make sure you are applying the right wiring between SOLO and your ARDUINO
 *    The Code below has been tested on Arduino UNO
 *    The Motor used for Testings: teknic m-2310P-LN-04K
 */

#define ARDUINO_CAN_NATIVE_SUPPORTED
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
  config.parameterName = SOLOMotorControllersCanopenNative::PdoParameterName::POSITION_COUNTS_FEEDBACK;
  config.parameterCobId = 0x280;
  config.isPdoParameterEnable = true;
  config.isRrtParameterEnable = true;
  config.syncParameterCount = 4;
  // send the configuration to SOLO
  solo->SetPdoParameterConfig(config, error);
  delay(100);

  // if CONFIGURATION already done you can avoid and use the next commad:
  // solo->pdoParameterCobIdByPdoParameterName[PdoParameterName::POSITION_COUNTS_FEEDBACK] = 0x280;
}

void loop()
{
  // ACTIVE section
  // send the sync message
  solo->SendPdoSync(error);
  solo->SendPdoSync(error);
  solo->SendPdoSync(error);
  solo->SendPdoSync(error);
  delay(50);

  // read the older value in the PDO buffer
  long getValue = solo->GetPdoPositionCountsFeedback(error);
  Serial.print("READ VALUE: ");
  Serial.print(getValue);
  Serial.print(" ERROR: ");
  Serial.println(error);

  // send the sync message
  solo->SendPdoSync(error);
  solo->SendPdoSync(error);
  solo->SendPdoSync(error);
  solo->SendPdoSync(error);
  delay(50);

  // read the older value in the PDO buffer
  getValue = solo->GetPdoPositionCountsFeedback(error);
  Serial.print("READ VALUE: ");
  Serial.print(getValue);
  Serial.print(" ERROR: ");
  Serial.println(error);

  delay(1000);
}