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

#include "SOLOMotorControllersCanopen.h" 

SOLOMotorControllers *SOLO_Obj1; // instanciate a SOLO object

long PWMFrequency_write=20; //Desired Switching Frequency at Output
long NumberOfPoles_write = 4; // Set the Motor's Number of Poles
long EncoderLines_write = 2000; // Set PPR for the Encoder 

float BusVoltage=0; //Battery or Supply Input Voltage
float Temperature=0; // SOLO board Temperature
float VoltageA =0; // Phase A voltage reading (3phase)
float Inductance = 0; //Motor Phase Inductance
long PWMFrequency_read=0; //Read Switching Frequency of SOLO
long NumberOfPoles_read = 0; // Read the Motor's Number of Poles
long EncoderLines_read = 0; // Read the PPR set for the Encoder 


void setup() {
  //Serial.begin(937500); // default baudrate of SOLO
  Serial.begin(115200); // selectable baudrate of SOLO 
  
  //Initialize the SOLO object
  int SOLOdeviceAddress = 0; 
  int chipSelectPin = 9; 
  SOLO_Obj1 = new SOLOMotorControllersCanopen(SOLOdeviceAddress, chipSelectPin);  
}
  
void loop() {
  //Setting Some Parameters  
  SOLO_Obj1->SetOutputPwmFrequencyKhz(PWMFrequency_write);
  SOLO_Obj1->SetMotorPolesCounts(NumberOfPoles_write);
  SOLO_Obj1->SetIncrementalEncoderLines(EncoderLines_write);
  
  //Reading Some Parameters
  BusVoltage = SOLO_Obj1->GetBusVoltage();
  Temperature = SOLO_Obj1->GetBoardTemperature();
  VoltageA = SOLO_Obj1->GetPhaseAVoltage();
  Inductance = SOLO_Obj1->GetMotorInductance();
  PWMFrequency_read = SOLO_Obj1->GetOutputPwmFrequencyKhz();
  NumberOfPoles_read = SOLO_Obj1->GetMotorPolesCounts();
  EncoderLines_read = SOLO_Obj1->GetIncrementalEncoderLines();
  
  Serial.println("\n List Of some parameters read from SOLO");
  
  Serial.println(BusVoltage,7);
  Serial.println(Temperature,7);
  Serial.println(VoltageA,7);
  Serial.println(Inductance,7);
  Serial.println(PWMFrequency_read);
  Serial.println(NumberOfPoles_read);
  Serial.println(EncoderLines_read);

  delay(1000);
}
