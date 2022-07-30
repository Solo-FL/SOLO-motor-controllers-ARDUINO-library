// Copyright: (c) 2021, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers Arduino Library
*    Author: SOLOMotorControllers
*    Date: 2022
*    Code version: 4.0.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library

This Library is made by SOLOMotorControllers.COM
please visit:  https://www.SOLOMotorControllers.com/

*/

// EXAMPLE of how to use CANOpen Protocol,
// the main steps are 3


//STEP 1: Importing CANOpen - SOLO library
#include "SOLOMotorControllersCanopen.h" 

//STEP 2: Declaration of SOLO object as SOLOMotorControllers
SOLOMotorControllers *SOLO_Obj1; 

void setup() {
  Serial.begin(115200);

//STEP 3: Initialization of the object as SOLOMotorControllersCanopen
  SOLO_Obj1 = new SOLOMotorControllersCanopen(); 
}
  
void loop() {
  Serial.println((String) "Read from SOLO: " + SOLO_Obj1->GetBoardTemperature());
  delay(1000);
}
