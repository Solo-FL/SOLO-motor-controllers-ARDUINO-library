// Copyright: (c) 2021, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controller Arduino Library
*    Author: SOLOMOTORCONTROLLERS
*    Date: 2021
*    Code version: 1.0.1
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library

This Library is made by SOLOMOTORCONTROLLERS.COM
It can be used with UART line of Arduino or any similar controller to control, command
or read all the parameters that are stored or existing in command set of SOLO. to learn more
please visit:  https://www.solomotorcontrollers.com/

*/

#include <stdint.h>
#include "Arduino.h"

#define ReadData                            0x00 // 0x00000000
#define INITIATOR                           0xFF //0xFFFF
#define BroadcastAddress                    0xFF
#define ENDING                              0xFE
#define ERROR                               0xEE //0xEEEEEEEE
#define CRC                                 0x00
#define WriteAddress                        0x01
#define WriteCommandMode                    0x02
#define WriteCurrentLimit                   0x03
#define WriteTorqueReference                0x04
#define WriteSpeedReference                 0x05
#define WritePowerReference                 0x06
#define WriteIdentification                 0x07
#define WriteStopSystem                     0x08
#define WritePWMFrequency                   0x09
#define WriteSpeedControllerKp              0x0A
#define WriteSpeedControllerKi              0x0B
#define WriteDirection                      0x0C
#define WriteResistance                     0x0D
#define WriteInductance                     0x0E
#define WriteNumberOfPoles                  0x0F
#define WriteEncoderLines                   0x10
#define WriteSpeedLimit                     0x11
#define WriteResetAddress                   0x12
#define WriteSpeedControlMode               0x13
#define WriteResetToFactory                 0x14
#define WriteMotorType                      0x15
#define WriteControlMode                    0x16
#define WriteCurrentControllerKp            0x17
#define WriteCurrentControllerKi            0x18
#define WriteMonitoringMode                 0x19
#define WriteMagnetizingCurrentReference    0x1A
#define WriteDesiredPosition                0x1B
#define WritePositionControllerKp           0x1C
#define WritePositionControllerKi           0x1D
#define WriteResetPositionToZero            0x1F //Home
#define WriteOverwriteTheErrors             0x20
#define ReadAddress                         0x81
#define ReadVoltageA                        0x82
#define ReadVoltageB                        0x83
#define ReadCurrentA                        0x84
#define ReadCurrentB                        0x85
#define ReadBusVoltage                      0x86
#define ReadIM                              0x87
#define ReadVM                              0x88
#define ReadSpeedControllerKp               0x89
#define ReadSpeedControllerKi               0x8A
#define ReadPWMFrequency                    0x8B
#define ReadCurrentLimit                    0x8C
#define ReadQuadratureCurrent               0x8D
#define ReadDirectCurrent                   0x8E //Magnetizing
#define ReadNumberOfPoles                   0x8F
#define ReadEncoderLine                     0x90
#define ReadCurrentControllerKp             0x91
#define ReadCurrentControllerKi             0x92
#define ReadTemperature                     0x93
#define ReadResistance                      0x94
#define ReadInductance                      0x95
#define ReadSpeed                           0x96
#define ReadMotorType                       0x97
#define ReadSpeedControlMode                0x99
#define ReadCommandMode                     0x9A
#define ReadControlMode                     0x9B
#define ReadSpeedLimit                      0x9C
#define ReadPositionControllerKp            0x9D
#define ReadPositionControllerKi            0x9E
#define ReadEncoderPosition                 0xA0
#define ReadErrorRegister                   0xA1
#define ReadFirmwareVersion                 0xA2
 
 /*struct SoloResult
 {
     bool Success;
     string Error;
     byte Data;
 }*/
class SOLOMotorController {   

private:
    unsigned char addr;
    
public:
    //static HardwareSerial Serial21; 
    SOLOMotorController(unsigned char _addr = 0);
public:
bool Test();
bool ExeCMD(unsigned char cmd[]);
float ConvertToFloat(unsigned char data[]);
void  ConvertToData(float f, unsigned char data[]);
long ConvertToLong(unsigned char data[]);
void ConvertToData(long l, unsigned char data[]);
void SplitData(unsigned char data[], unsigned char cmd[]);
bool SetAddress(unsigned char addr);
bool SetCommandMode(bool mode);
bool SetCurrentLimit(float A);
bool SetTorqueReference(float A);
bool SetSpeedReference(long rpm);
bool SetPowerReference(float P);
bool SetIdentification(bool S);
bool StopSystem();
bool SetPWMFrequency(long pwm);
bool SetSpeedControllerKp(float Kp);
bool SetSpeedControllerKi(float Ki);
bool SetDirection(bool dir); 
bool SetResistance(float res); 
bool SetInductance(float ind);
bool SetNumberOfPoles(long poles);
bool SetEncoderLines(long enc);
bool SetSpeedLimit(long speed);
bool ResetAddress();
bool SetSpeedControlMode(bool mode); 
bool ResetToFactory();
bool SetMotorType(long type); 
bool SetControlMode(long mode); 
bool SetCurrentControllerKp(float Kp);
bool SetCurrentControllerKi(float Ki);
bool SetMonitoringMode(bool mode);
bool SetMagnetizingCurrentReference(float A);
bool SetDesiredPosition(long pos);
bool SetPositionControllerKp(float Kp);
bool SetPositionControllerKi(float Ki);
bool ResetPositionToZero(); //Home
bool OverwriteTheErrors();
//----------Read----------
long GetAddress(long _addr);
float GetVoltageA();
float GetVoltageB();
float GetCurrentA();
float GetCurrentB();
float GetBusVoltage(); //Battery Voltage
float GetMotorCurrent();
float GetMotorVoltage();
float GetSpeedControllerKp();
float GetSpeedControllerKi();
long  GetPWMFrequency();
float GetCurrentLimit();
float GetQuadratureCurrent();
float GetDirectCurrent(); //Magnetizing
long  GetNumberOfPoles();
long  GetEncoderLines();
float GetCurrentControllerKp();
float GetCurrentControllerKi();
float GetTemperature();
float GetResistance();
float GetInductance();
long  GetSpeed();
long  GetMotorType();
long  GetSpeedControlMode();
long  GetCommandMode();
long  GetControlMode();
long  GetSpeedLimit();
float GetPositionControllerKp();
float GetPositionControllerKi();
long  GetEncoderPosition();
long  GetErrorRegister();//TODO
long  GetFirmwareVersion();

};