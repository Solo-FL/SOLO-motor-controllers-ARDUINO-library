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

//In this example we like to show how the error are pass and they can be handled  


//EXAMPLE 1: get function with no error 
// if error happen the value recived is -1
long commandMode = SOLO_Obj1->GetCommandMode();
Serial.println((String)"COMMAND MODE: " + commandMode);


//EXAMPLE 2: get function with error 
int error; // if is 0 mean no Error Detected.
long commandMode = SOLO_Obj1->GetCommandMode();
Serial.println((String)"COMMAND MODE: " + commandMode + " ERROR: "+ error );


//NOTE error is generated following this enum SOLOMotorControllers::Error 
enum SOLOMotorControllers::Error
    {
        noErrorDetected = 0,          //operation whent well
        generalError = 1,             //error happen but no specific definition 
        noProcessedCommand = 2,       //error that happen if the method will be interrupt 
        outOfRengeSetting = 3,        //if the set of the function is with a parameter out of the range 
        packetFailureTrialAttemptsOverflow = 4, //if the comunication had multiple error consecutive
        recieveTimeOutError = 5,      //SOLO don't respond in time
        Abort_Object = 6,             //CAN OPEN specif error over the type of object handled
        Abort_Value  = 7,             //CAN OPEN specif error over the type of object handled
        MCP2515_Transmit_ArbitrationLost = 8, // CAN OPEN MCP2515 chip error
        MCP2515_Transmit_Error = 9    // CAN OPEN MCP2515 chip error
    };


//EXAMPLE 3: is possible to compare the error with SOLOMotorControllers specific error
if (error != SOLOMotorControllers::Error::noErrorDetected)
{
  Serial.println(" Error happen, is possible to react according to this sitatuon!");
}


//EXAMPLE 4: set funtion with no error
// if error happen the value recived is -1
SOLO_Obj1->SetCommandMode(SOLOMotorControllers::CommandMode::analogue);


//EXAMPLE 5: set function with error
int error; // if is 0 mean no Error Detected.
SOLO_Obj1->SetCommandMode(SOLOMotorControllers::CommandMode::analogue, error);
Serial.println((String)" ERROR: "+ error );


//EXAMPLE 6: set function with error and success boolean
bool isSetSuccess; // set function return True if the operation go well
int error; // if is 0 mean no Error Detected.
isSetSuccess = SOLO_Obj1->SetCommandMode(SOLOMotorControllers::CommandMode::analogue, error);
Serial.println((String)"SUCCESS : "+ isSetSuccess + " ERROR: "+ error );