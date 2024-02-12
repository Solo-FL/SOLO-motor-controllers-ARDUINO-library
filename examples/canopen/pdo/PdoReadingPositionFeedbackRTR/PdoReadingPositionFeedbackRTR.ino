// Copyright: (c) 2024-present, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
 *    Title: Torque Control of PMSM equipped with Incremental Encoders using Arduino and SOLO
 *    Author: SOLOMOTORCONTROLLERS
 *    Date: 2024
 *    Code version: 1.0.0
 *    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *    Please make sure you are applying the right wiring between SOLO and your ARDUINO
 *    The Code below has been tested on Arduino UNO
 *    The Motor used for Testings: teknic m-2310P-LN-04K
 */

#include <SOLOMotorControllersCanopen.h>

SOLOMotorControllersCanopen *solo;

int error;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Initialize the SOLO object
  int SOLOdeviceAddress = 0;
  int chipSelectPin = 9;
  solo = new SOLOMotorControllersCanopen(SOLOdeviceAddress, chipSelectPin);
  
  // 1 time needed CONFIGURATION:
  Serial.println("PdoParameterConfig:");
  PdoParameterConfig config;
  config.parameterName = PdoParameterName::POSITION_COUNTS_FEEDBACK;
  config.parameterCobId = 0x280;
  config.isPdoParameterEnable = true;
  config.isRrtParameterEnable = true;
  config.syncParameterCount = 0;
  // send the configuration to SOLO
  solo->SetPdoParameterConfig(config, error);
  delay(100);

  // if CONFIGURATION already done you can avoid and use the next commad:
  // solo->pdoParameterCobIdByPdoParameterName[PdoParameterName::POSITION_COUNTS_FEEDBACK] = 0x280;
}

void loop()
{
  // ACTIVE section
  // send the RTR message
  solo->SendPdoRtr(PdoParameterName::POSITION_COUNTS_FEEDBACK);
  delay(50);

  // read the older value in the PDO buffer
  long getValue = solo->GetPdoPositionCountsFeedback(error);
  Serial.print("READ VALUE: ");
  Serial.print(getValue);
  Serial.print(" ERROR: ");
  Serial.println(error);

  delay(1000);
}