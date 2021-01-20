// EXAMPLE of how read the SOLO board temperature, 
// every second we print the value of the temperature


//Importing SOLO Arduino library
#include <SOLOMotorController.h>

// instanciate a SOLO object
SOLOMotorController *SOLO_Obj1; 

// SOLO board Temperature
float Temperature=0; 


void setup() {
  Serial.begin(937500); 

  //Initialize the SOLO object using the device address of SOLO at 0
  SOLO_Obj1 = new SOLOMotorController(0); 
}
  
void loop() {
  //Reading
  Temperature = SOLO_Obj1->GetTemperature();

  //Print
  Serial.println("\n Read from SOLO");
  Serial.println(Temperature,7);

  delay(1000);
}
