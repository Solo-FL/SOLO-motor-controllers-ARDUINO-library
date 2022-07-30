// Copyright: (c) 2021, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: Torque Control of a Brushless Motor with Arduino and SOLO using HALL sensors
*    Author: SOLOMOTORCONTROLLERS
*    Date: 2022
*    Code version: 4.0.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
*    Please make sure you are applying the right wiring between SOLO and your ARDUINO
*    The Code below has been tested on Arduino UNO
*    The Motor used for Testings: DB56C036030-A
*    Read more about this code here:https://www.solomotorcontrollers.com/drive-fast-drone-rc-car-brushless-motors-arduino-solo-sensorless
*/

#include "SOLOMotorControllersCanopen.h"  

//For this Test, make sure you have calibrated your HALL sensors before
//to know more please read: https://www.solomotorcontrollers.com/hall-sensors-to-solo-for-controlling-speed-torque-brushless-motor/

//In this example, make sure you put SOLO into Closed-Loop by
// pressing the Piano Switch NO# 5 DOWN. in SOLO UNO

  /* The Piano Switch Setup on SOLO UNO are as below since SOLO will be commandaed in Analogue Mode with PWM:
   *  PIN 5 Down: closed-loop
   *  PIN 4 UP : Torque Mode (Analogue mode)
   *  PIN 1 Down and PIN 2 UP:  BLDC_PMSM motor type (Analogue mode)
   *  PIN 3 UP ( Not in DFU mode )
 */
 
//instanciate a SOLO object:
SOLOMotorControllers *SOLO_Obj1; 

//the device address of SOLO:
unsigned char SOLO_address1 = 0; 

//Desired Switching or PWM Frequency at Output
long pwmFrequency = 16; 

//Motor's Number of Poles
long numberOfPoles = 8; 

// Current Limit of the Motor
float currentLimit = 12.5;

// Define Desired Torque referrrence
float desiredTorque_Iq = 1.5;

// Converted value to PWM duty cycle for Iq
int desiredDutyCycle_Iq = 0;

// Converted value to PWM duty cycle for currentLimit 
int desiredDutyCycle_CurrentLimit = 0;

// Define the Max Measurable current in SOLO UNO for 100% duty Cycle
float MaxMeasurableCurrent_SOLO_UNO = 32.0;

// Battery or Bus Voltage
float busVoltage = 0; 

// Desired Speed Limit[RPM]
long desiredSpeedLimit =3000; 

// Motor speed feedback
long actualMotorSpeed = 0; 

// Motor Iq (torque) feedback
float actualMotorTorque_Iq = 0; 

void setup() {
  
  //High Speed High Performance Baudrate (Recommended)
  //Use this baudrate to have the best and real performance
  //of SOLO under all conditions.
  //Serial.begin(937500);   

  //Low Speed Low Performance Baudrate
  //Use this baudrate only for devices that don't support
  //937500 or 921600 baudrates.
  Serial.begin(115200); 

  //Initialize the SOLO object
  int SOLOdeviceAddress = 0; 
  int chipSelectPin = 9; 
  SOLO_Obj1 = new SOLOMotorControllersCanopen(SOLOdeviceAddress, chipSelectPin); 
  delay(1000);

  Serial.println("\n Trying to Connect To SOLO");
  delay(1000);
  //wait here till communication is established
  while(SOLO_Obj1->CommunicationIsWorking() == false)
  {
    delay(500);
  }
      
  Serial.println("\n Communication Established succuessfully!");

  // Initial Configurations
  SOLO_Obj1->SetOutputPwmFrequencyKhz(pwmFrequency);
  SOLO_Obj1->SetCurrentLimit(currentLimit);
  
  //select Digital Mode
  SOLO_Obj1->SetCommandMode(SOLOMotorControllers::CommandMode::digital);;
  SOLO_Obj1->SetMotorType(SOLOMotorControllers::MotorType::bldcPmsm);

  //Operate while using Hall sensors
  SOLO_Obj1->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::hallSensors);

  //run the motor identification
  //run ID. always after selecting the Motor Type!
  SOLO_Obj1->MotorParametersIdentification(SOLOMotorControllers::Action::start);

  Serial.println("\n Identifying the Motor");
  //wait at least for 2sec till ID. is done
  delay(2000); 

  //Go back to Analogue Mode to accept PWM as Reference for Torque and Speed
  SOLO_Obj1->SetCommandMode(SOLOMotorControllers::CommandMode::analogue);
  //Enable PWM pins of Arduiono with Fixed frequency greater than 5kHz
  
  pinMode(3, OUTPUT); //torque adjuster connected to "S/T" on SOLO UNO
  pinMode(9, OUTPUT);//current Limit adjuster connected to "P/F" on SOLO UNO
  
  pinMode(2,OUTPUT); //Direction Control Pin
    
  // declare pin 3 to be a PWM enabled output with 31kHz of fixed Frequency
  TCCR2B = TCCR2B & 0b11111000 | 0x01;
  // declare pin 9 to be a PWM enabled output 31kHz of fixed Frequency
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
}

void loop() {
  
  // Converted value to PWM duty cycle for desired current Limit on 8 bit PWM of Arduino
  desiredDutyCycle_CurrentLimit = (int)(255 - (255*( currentLimit/ MaxMeasurableCurrent_SOLO_UNO)));
  //Set the right Duty cycle on Pin ~9 for current Limit
  analogWrite(9, desiredDutyCycle_CurrentLimit);

  //Define the desired torque in Amps
  desiredTorque_Iq = 1.5;

  //Define the Direction of Rotation
  digitalWrite(2,LOW);
  
  // Converted value to PWM duty cycle for desired Iq on 8 bit PWM of Arduino
  desiredDutyCycle_Iq = (int)(255*(desiredTorque_Iq / MaxMeasurableCurrent_SOLO_UNO));

  //Set the right Duty cycle on Pin ~3 for Torque
  analogWrite(3, desiredDutyCycle_Iq);

  // wait till motor reaches to the reference 
  delay(500); 

  actualMotorTorque_Iq= SOLO_Obj1->GetQuadratureCurrentIqFeedback();
  Serial.println("\n Torque (Iq): ");
  Serial.print(actualMotorTorque_Iq , 7);
  
 // wait for user to read
  delay(3000);
  
  //Define the desired torque in Amps
  desiredTorque_Iq = 2.5;

  //Define the Direction of Rotation
  digitalWrite(2,HIGH);
  
  // Converted value to PWM duty cycle for desired Iq on 8 bit PWM of Arduino
  desiredDutyCycle_Iq = (int)(255*(desiredTorque_Iq / MaxMeasurableCurrent_SOLO_UNO));

  //Set the right Duty cycle on Pin ~3 for Torque
  analogWrite(3, desiredDutyCycle_Iq);

  // wait till motor reaches to the reference 
  delay(500); 

  actualMotorTorque_Iq= SOLO_Obj1->GetQuadratureCurrentIqFeedback();
  Serial.println("\n Torque (Iq): ");
  Serial.print(actualMotorTorque_Iq , 7);
  
  // wait for user to read
  delay(3000);
  
}
