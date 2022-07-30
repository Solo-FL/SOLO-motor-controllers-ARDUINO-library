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

// EXAMPLES of all to setup possible with UART Protocol
#include "SOLOMotorControllersUart.h" 
SOLOMotorControllers *SOLO_Obj1; 

//EXAMPLE 1: All setup are as default: 
SOLO_Obj1 = new SOLOMotorControllersUart(); 


//EXAMPLE 2: All parameters are as default but explicit:
//This option give no difference to the example 1 
int SOLOdeviceAddress = 0;  //Device adress of SOLO (default vale in SOLO units is 0)
HardwareSerial &serialSelected = Serial; // Serial port that Arduino will use (Arduino 1 have only Serial, but other version have more like Serial1)
SOLOMotorControllers::UartBaudrate baudrate = SOLOMotorControllers::UartBaudrate::rate115200;  // Boudrate Arduino will use for the comunication (SOLO neet the same value, is possible to set in the Motion Terminal) 
long millisecondsTimeout = 200; //Timout in milliseconds before interrupt the wait
int packetFailureTrialAttempts = 5; //How many time will happen a try on the same comunication if an error heppen 
SOLO_Obj1 = new SOLOMotorControllersUart(SOLOdeviceAddress, serialSelected, baudrate, millisecondsTimeout, packetFailureTrialAttempts);


//EXAMPLE 3: Partial usage of paramets
//the order of the parametrs is mandatory, but the value can change
int SOLOdeviceAddress = 3;  //Device adress of SOLO (default vale in SOLO units is 0)
SOLO_Obj1 = new SOLOMotorControllersUart(SOLOdeviceAddress);


//EXAMPLE 4: Partial usage of paramets without variable
//is possible to pass directly in the Initialization the value
//in this case, we have no difference with the example 3
SOLO_Obj1 = new SOLOMotorControllersUart(3);