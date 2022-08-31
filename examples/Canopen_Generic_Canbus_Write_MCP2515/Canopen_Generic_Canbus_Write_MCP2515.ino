// Copyright: (c) 2021, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers Arduino Library
*    Author: SOLOMotorControllers
*    Date: 2022
*    Code version: 4.0.3
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library

This Library is made by SOLOMotorControllers.COM
please visit:  https://www.SOLOMotorControllers.com/

*/

// EXAMPLE of how to WRITE a Generic message on CANBus using SOLO CAN library for MCP2515
// In this Example, the line is read continously and ID, DCL and all the  data bytes recieved are printed
// The user can use this example to work with other CAN MODULES in the Line except SOLO


//Importing SOLO Arduino library
#include "SOLOMotorControllersCanopen.h" 

// instanciate a SOLO object as Canopen IMPORTANT
SOLOMotorControllersCanopen *SOLO_Obj1; 

//Init writing variable
uint16_t ID_Write = 605; //int 
uint8_t  DLC_Write = 8;
uint8_t  DataWrite[8]  = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
int ErrorWrite = 0; 
String SOLOMotorControllersErrors[10] = {"noErrorDetected","generalError","noProcessedCommand","outOfRengeSetting","packetFailureTrialAttemptsOverflow","recieveTimeOutError","Abort_Object","Abort_Value","MCP2515_Transmit_ArbitrationLost","MCP2515_Transmit_Error"};

void setup() {
  Serial.begin(115200);

  //Initialize the SOLO object
  int SOLOdeviceAddress = 0; 
  int chipSelectPin = 9; 
  SOLO_Obj1 = new SOLOMotorControllersCanopen(SOLOdeviceAddress, chipSelectPin); 
}
  
void loop() {
  //writing
  Serial.print("NEW WRITE: ");
  SOLO_Obj1->Generic_Canbus_Write_MCP2515(ID_Write , &DLC_Write , DataWrite, ErrorWrite);
  Serial.println(SOLOMotorControllersErrors[ErrorWrite]);
  delay(1000);
}