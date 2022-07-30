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
#include "SOLOMotorControllersCanopen.h" 
SOLOMotorControllers *SOLO_Obj1; 

//EXAMPLE 1: All setup are as default: 
SOLO_Obj1 = new SOLOMotorControllersCanopen(); 


//EXAMPLE 2: All parameters are as default but explicit:
//This option give no difference to the example 1 
int SOLOdeviceAddress = 0;  //Device adress of SOLO (default vale in SOLO units is 0)
int chipSelectPin = 9; //The chip select dipends on what board you want to work with, the default value is matching with shild board for arduino uno CAN-BUS Shield v2. 
SOLOMotorControllers::CanbusBaudrate baudrate = SOLOMotorControllers::UartBaudrate::rate1000;  // Boudrate Arduino will use it for the comunication [kbits/s] (SOLO need the same value, is possible to set in the Motion Terminal) 
long countTimeout = 6000 //Iteration before interrupt the receive from SOLO
SOLO_Obj1 = new SOLOMotorControllersCanopen(SOLOdeviceAddress, chipSelectPin, baudrate, countTimeout);


//EXAMPLE 3: Partial usage of paramets
//the order of the parametrs is mandatory, but the value can change
int SOLOdeviceAddress = 3;  //Device adress of SOLO (default vale in SOLO units is 0)
SOLO_Obj1 = new SOLOMotorControllersCanopen(SOLOdeviceAddress);


//EXAMPLE 4: Partial usage of paramets without variable
//is possible to pass directly in the Initialization the value
//in this case, we have no difference with the example 3
SOLO_Obj1 = new SOLOMotorControllersCanopen(3);