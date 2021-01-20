// EXAMPLE of how read the SOLO Battery or Supply Input Voltage, 
// every second we print the value of it.


//Importing SOLO Arduino library
#include <SOLOMotorController.h>

// instanciate a SOLO object
SOLOMotorController *SOLO_Obj1; 

// SOLO board Temperature
float BusVoltage=0; 


void setup() {
  Serial.begin(937500); 

  //Initialize the SOLO object using the device address of SOLO at 0
  SOLO_Obj1 = new SOLOMotorController(0); 
}
  
void loop() {
  //Reading
  BusVoltage = SOLO_Obj1->GetBusVoltage();

  //Print
  Serial.println("\n Read from SOLO");
  Serial.println(BusVoltage,7);

  delay(1000);
}
