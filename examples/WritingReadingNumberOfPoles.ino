// EXAMPLE of how to set the Motor number of poles, 
// every second we repit the setting and the reading of it


//Importing SOLO Arduino library
#include <SOLOMotorController.h>

// instanciate a SOLO object
SOLOMotorController *SOLO_Obj1; 

// Motor's Number of Poles
long NumberOfPoles_write = 4; // Write 
long NumberOfPoles_read = 0; // Read

void setup() {
  Serial.begin(937500); 

  //Initialize the SOLO object using the device address of SOLO at 0
  SOLO_Obj1 = new SOLOMotorController(0); 
}
  
void loop() {
  //Setting
   SOLO_Obj1->SetNumberOfPoles(NumberOfPoles_write);

  //Reading
  NumberOfPoles_read = SOLO_Obj1->GetNumberOfPoles();

  //Print
  Serial.println("\n Read from SOLO");
  Serial.println(NumberOfPoles_read);

  delay(1000);
}
