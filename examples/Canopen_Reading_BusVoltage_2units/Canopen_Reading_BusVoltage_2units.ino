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
// EXAMPLE of how read the SOLO Battery or Supply Input Voltage, 
// every second we print the value of it.

//Importing SOLO Arduino library
#include "SOLOMotorControllersCanopen.h" 

// instanciate a SOLO object
SOLOMotorControllers *SOLO_1; 
SOLOMotorControllers *SOLO_2; 

// SOLO board Temperature
float BusVoltage=0; 

// Boudrate Arduino will use it for the comunication (SOLO need the same value, is possible to set in the Motion Terminal) 
SOLOMotorControllers::CanbusBaudrate baudrate = SOLOMotorControllers::CanbusBaudrate::rate1000;  

//Iteration before interrupt the receive from SOLO
long countTimeout = 6000 ;

void setup() {
  Serial.begin(115200);

  //Initialize the SOLO object
  int chipSelectPin = 9; 
  SOLO_1 = new SOLOMotorControllersCanopen(0, chipSelectPin, baudrate, countTimeout); //Solo with 0 Device Address
  SOLO_2 = new SOLOMotorControllersCanopen(6, chipSelectPin, baudrate, countTimeout); //Solo with 6 Device Address
}
  
void loop() {
  //Reading
  BusVoltage = SOLO_1->GetBusVoltage();

  //Print
  Serial.print("Read from SOLO 1: ");
  Serial.println(BusVoltage,7);

  //Reading
  BusVoltage = SOLO_2->GetBusVoltage();

  //Print
  Serial.print("Read from SOLO 2: ");
  Serial.println(BusVoltage,7);

  delay(1000);
}
