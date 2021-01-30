/*
*    Title: Controlling the speed of a Brushless Motor using Arduino in Sensorless Mode
*    Author: SOLOMOTORCONTROLLERS.COM
*    Date: 2021
*    Code version: 1.0.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
*    Please make sure you are applying the right wiring between SOLO and your ARDUINO
*    The Code below has been tested on Arduino UNO, The Motor used for Testings: teknic m-2310P-LN-04K
*    Read more about this code here: https://www.solomotorcontrollers.com/sensorless-control-brushless-motor-arduino-solo-digital-mode-uart/
*/

#include <SOLOMotorController.h>

//instanciate a SOLO object:
SOLOMotorController *SOLO_Obj1; 

//the device address of SOLO:
unsigned char SOLO_address1=0; 

//Desired Switching or PWM Frequency at Output
long pwmFrequency=75; 

//Motor's Number of Poles
long numberOfPoles = 8; 

//Select the Normal BLDC_PMSM motor type
long motorType= 1; 

//Speed controller Kp
float speedControllerKp = 0.04; 

//Speed controller Ki
float speedControllerKi = 0.006; 

// Current Limit of the Motor
float currentLimit= 16.55; 

// Battery of Bus Voltage
float busVoltage = 0; 

// Desired Speed Reference 
long desiredMotorSpeed =0; 

// Motor speed feedback
long actualMotorSpeed = 0; 


void setup() {
  //In this example, make sure you put SOLO into Closed-Loop by
  // pressing the Piano Switch NO# 5 DOWN. in SOLO UNO
  
  //High Speed High Performance Baudrate (Recommended)
  //Use this baudrate to have the best and real performance
  //of SOLO under all conditions.
  //Serial.begin(937500);   

  //Low Speed Low Performance Baudrate
  //Use this baudrate only for devices that don't support
  //937500 or 921600 baudrates.
  Serial.begin(115200); 

  //Initialize the SOLO object
  SOLO_Obj1 = new SOLOMotorController(SOLO_address1); 
  delay(2000);
  
  busVoltage = SOLO_Obj1->GetBusVoltage();
  while(busVoltage <=0){
      busVoltage = SOLO_Obj1->GetBusVoltage();
      //wait here till communication is established
      Serial.println("\n Trying to Connect To SOLO");
      delay(1000);
      }
      
  Serial.println("\n Communication Established succuessfully!");
  
  //dummy read after Serial.println to open the UART line
  SOLO_Obj1->GetBusVoltage();
  // Initial Configurations
  SOLO_Obj1->SetPWMFrequency(pwmFrequency);
  SOLO_Obj1->SetCurrentLimit(currentLimit);

  //select Digital Mode
  SOLO_Obj1->SetCommandMode(1);

  SOLO_Obj1->SetMotorType(motorType);

  //run the motor identification
  //run ID. always after selecting the Motor Type!
  SOLO_Obj1->SetIdentification(1);

  Serial.println("\n Identifying the Motor");

  //wait at least for 2sec till ID. is done
  delay(2000); 
  
  //dummy read after Serial.println to open the UART line
  SOLO_Obj1->GetBusVoltage();
  
  //Operate in Sensor-less Mode
  SOLO_Obj1->SetSpeedControlMode(0);

  //Control The Speed
  SOLO_Obj1->SetControlMode(0);

  //Controller Tunings
  SOLO_Obj1->SetSpeedControllerKp(speedControllerKp);
  SOLO_Obj1->SetSpeedControllerKi(speedControllerKi);
}


void loop() {

  //set the Direction on C.W. 
  SOLO_Obj1->SetDirection(0); 

  //set a new reference for speed
  desiredMotorSpeed = 5000;
  SOLO_Obj1->SetSpeedReference(desiredMotorSpeed);

  // wait till motor reaches to the reference 
  delay(2000); 

  actualMotorSpeed = SOLO_Obj1->GetSpeed();
  Serial.println("\n Motor Speed: ");
  Serial.println(actualMotorSpeed);

  //dummy read after Serial.println to open the UART line
  SOLO_Obj1->GetBusVoltage();
  
  //set the Direction on C.C.W. 
  SOLO_Obj1->SetDirection(1);

  //set a new reference for speed
  desiredMotorSpeed = 1500;
  SOLO_Obj1->SetSpeedReference(desiredMotorSpeed);

  // wait till motor reaches to the reference 
  delay(2000);

  actualMotorSpeed = SOLO_Obj1->GetSpeed();
  Serial.println("\n Motor Speed: ");
  Serial.println(actualMotorSpeed);
  
  //dummy read after Serial.println to open the UART line
  SOLO_Obj1->GetBusVoltage();
  
  //stop the motor
  desiredMotorSpeed = 0;
  SOLO_Obj1->SetSpeedReference(desiredMotorSpeed);
  delay(2000);

}
