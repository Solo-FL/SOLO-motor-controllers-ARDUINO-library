#include <SOLOMotorController.h>

SOLOMotorController *SOLO_Obj1; // instanciate a SOLO object
unsigned char SOLO_address1=0; // the device address of SOLO

/**********************************************/
//You can instanciate up to 254 SOLOs in a network
//Each SOLO in the network should have a unique address

//  SOLOMotorController *SOLO_Obj2; // instanciate a SOLO object
//  unsigned char SOLO_address1=1; // the device address of SOLO
  
//  SOLOMotorController *SOLO_Obj3; // instanciate a SOLO object
//  unsigned char SOLO_address1=2; // the device address of SOLO
/*********************************************/

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
  
  SOLO_Obj1 = new SOLOMotorController(SOLO_address1); //Initialize the SOLO object
}
  
void loop() {

  //Setting Some Parameters  
  SOLO_Obj1->SetPWMFrequency(PWMFrequency_write);
  SOLO_Obj1->SetNumberOfPoles(NumberOfPoles_write);
  SOLO_Obj1->SetEncoderLines(EncoderLines_write);
  
  //Reading Some Parameters
  BusVoltage = SOLO_Obj1->GetBusVoltage();
  Temperature = SOLO_Obj1->GetTemperature();
  VoltageA = SOLO_Obj1->GetVoltageA();
  Inductance = SOLO_Obj1->GetInductance();
  PWMFrequency_read = SOLO_Obj1->GetPWMFrequency();
  NumberOfPoles_read = SOLO_Obj1->GetNumberOfPoles();
  EncoderLines_read = SOLO_Obj1->GetEncoderLines();
  
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
