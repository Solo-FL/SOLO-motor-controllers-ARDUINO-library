// Copyright: (c) 2021, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: Speed Control of PMSM equipped with Incremental Encoders using Arduino and SOLO
*    Author: SOLOMOTORCONTROLLERS
*    Date: 2022
*    Code version: 4.0.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
*    Please make sure you are applying the right wiring between SOLO and your ARDUINO
*    The Code below has been tested on Arduino UNO
*    The Motor used for Testings: teknic m-2310P-LN-04K
*/

#include "SOLOMotorControllersCanopen.h" 

//For this Test, make sure you have calibrated your Motor and Incremental Encoders before
//to know more please read: https://www.solomotorcontrollers.com/how-to-connect-calibrate-incremental-encoder-with-solo/

//instanciate a SOLO object:
SOLOMotorControllers *SOLO_Obj1; 

//the device address of SOLO:
unsigned char SOLO_address1 = 0; 

//Desired Switching or PWM Frequency at Output
long pwmFrequency = 20; 

//Motor's Number of Poles
long numberOfPoles = 8; 

// Current Limit of the Motor
float currentLimit = 7.0; 

//Motor's Number of Encoder Lines (PPR pre-quad)
long numberOfEncoderLines = 1000; 

//Speed controller Kp
float speedControllerKp = 0.15; 

//Speed controller Ki
float speedControllerKi = 0.005; 

// Battery or Bus Voltage
float busVoltage = 0; 

// Motor Torque feedback
float actualMotorTorque = 0; 

// Motor speed feedback
long actualMotorSpeed = 0; 

// Motor position feedback
long actualMotorPosition = 0; 

void setup() {
  //In this example, make sure you put SOLO into Closed-Loop Mode
  
  //used for Monitoring in Arduino Only
  Serial.begin(115200); 

  //Initialize the SOLO object
  int SOLOdeviceAddress = 0; 
  int chipSelectPin = 9; //SPI CS pin for CANshield
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

  // Initial Configuration of the device and the Motor
  SOLO_Obj1->SetOutputPwmFrequencyKhz(pwmFrequency);
  SOLO_Obj1->SetCurrentLimit(currentLimit);
  SOLO_Obj1->SetMotorPolesCounts(numberOfPoles);
  SOLO_Obj1->SetIncrementalEncoderLines(numberOfEncoderLines);
  SOLO_Obj1->SetCommandMode(SOLOMotorControllers::CommandMode::digital);
  SOLO_Obj1->SetMotorType(SOLOMotorControllers::MotorType::bldcPmsm);
  SOLO_Obj1->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::encoders);
  SOLO_Obj1->SetSpeedControllerKp(speedControllerKp);
  SOLO_Obj1->SetSpeedControllerKi(speedControllerKi);
  SOLO_Obj1->SetControlMode(SOLOMotorControllers::ControlMode::speedMode);
 
  //run the motor identification to Auto-tune the current controller gains Kp and Ki needed for Torque Loop
  //run ID. always after selecting the Motor Type!
  //ID. doesn't need to be called everytime, only one time after wiring up the Motor will be enough
  //the ID. values will be remembered by SOLO after power recycling
  SOLO_Obj1->MotorParametersIdentification(SOLOMotorControllers::Action::start);
  Serial.println("\n Identifying the Motor");
  //wait at least for 2sec till ID. is done
  delay(2000); 
}

void loop() {

   //set the Direction on C.C.W. 
   SOLO_Obj1->SetMotorDirection(SOLOMotorControllers::Direction::counterclockwise);
   //set an arbitrary Positive speed reference[RPM]
   SOLO_Obj1->SetSpeedReference(1500);
   // wait till motor reaches to the reference 
   delay(300);
   actualMotorSpeed = SOLO_Obj1->GetSpeedFeedback();
   Serial.println("\n Measured Speed[RPM]: ");
   Serial.println(actualMotorSpeed);
   actualMotorTorque = SOLO_Obj1->GetQuadratureCurrentIqFeedback();
   Serial.println("\n Measured Iq/Torque[A]: ");
   Serial.println(actualMotorTorque);
   delay(3000);

   //set the Direction on C.W. 
   SOLO_Obj1->SetMotorDirection(SOLOMotorControllers::Direction::clockwise);
   //set an arbitrary Positive speed reference[RPM]
   SOLO_Obj1->SetSpeedReference(3000);
   // wait till motor reaches to the reference 
   delay(300);
   actualMotorSpeed = SOLO_Obj1->GetSpeedFeedback();
   Serial.println("\n Measured Speed[RPM]: ");
   Serial.println(actualMotorSpeed);
   actualMotorTorque = SOLO_Obj1->GetQuadratureCurrentIqFeedback();
   Serial.println("\n Measured Iq/Torque[A]: ");
   Serial.println(actualMotorTorque);
   delay(3000);
}
