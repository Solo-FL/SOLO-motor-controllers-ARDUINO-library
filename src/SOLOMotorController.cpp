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
#include <SOLOMotorController.h>
// -------------------- constructor & destructor --------------------
SOLOMotorController::SOLOMotorController(unsigned char _addr){
    addr = _addr;
}

bool SOLOMotorController::Test()
{
    
    char cmd[] = {0xFF,0xFF,65,66,67,68,69,70,0,0xEF};
    //cmd = 22;
    //Serial.write(cmd,10);
    //Serial21.begin(9600);
    //Serial21.write(cmd,10);
    int i=0;
    //String str;
    /*while(Serial.available()<=0);
    if(Serial.available()>0)
    {
       str = Serial.readString();
    }*/
    if(WriteAddress == 0x01)
        return true;
    else return false;
}

bool SOLOMotorController::ExeCMD(unsigned char cmd[])
{
    unsigned char _cmd[] = {INITIATOR,INITIATOR,cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5],CRC,ENDING};
    unsigned char _readPacket[10];
    
    Serial.write(_cmd,10);
    
    while(Serial.available()<=0); // wait for respons
    Serial.readBytes(_readPacket, 10);  //read received data 
    //Serial.write(_readPacket,10);return;
    if(_readPacket[0] == _cmd[0] && _readPacket[1] == _cmd[1] 
        && _readPacket[2] == _cmd[2] && _readPacket[3] == _cmd[3]
        && _readPacket[8] == _cmd[8] && _readPacket[9] == _cmd[9])
        {
            cmd[0] = _readPacket[2];
            cmd[1] = _readPacket[3];
            cmd[2] = _readPacket[4];
            cmd[3] = _readPacket[5];
            cmd[4] = _readPacket[6];
            cmd[5] = _readPacket[7];
        }
        else
        {
            cmd[0] = ERROR;
            cmd[1] = ERROR;
            cmd[2] = ERROR;
            cmd[3] = ERROR;
            cmd[4] = ERROR;
            cmd[5] = ERROR;
        }

    if(cmd[2] == ERROR && cmd[3] == ERROR && cmd[4] == ERROR && cmd[5] == ERROR)
            return false;
        else
            return true;
}
float SOLOMotorController::ConvertToFloat(unsigned char data[])
{
    long dec = 0;
    dec = (data[0]/16)*268435456 + (data[0]%16)*16777216 +
          (data[1]/16)*1048576 + (data[1]%16)*65536 +
          (data[2]/16)*4096 + (data[2]%16)*256 +
          (data[3]/16)*16 + (data[3]%16)*1;
    if(dec <= 0x7FFE0000)
    {
        return (float)dec/131072.0;
    }
    else
    {
        dec = 0xFFFFFFFF - dec + 1;
        return ((float)dec/131072.0) * -1;
    }
}
void SOLOMotorController::ConvertToData(float f, unsigned char data[])
{
    long dec = (long)(f * 131072);
    if(dec<0) 
    {
        dec*=-1;
        dec = 0xFFFFFFFF - dec;
    }
    data[0] = (dec/16777216);
    dec = dec%16777216;
    data[1] = (dec/65536);
    dec = dec%65536;
    data[2] = (dec/256);
    dec = dec%256;
    data[3] = (dec);
}
long SOLOMotorController::ConvertToLong(unsigned char data[])
{
    long dec = 0;
    dec = (data[0]/16)*268435456 + (data[0]%16)*16777216 +
          (data[1]/16)*1048576 + (data[1]%16)*65536 +
          (data[2]/16)*4096 + (data[2]%16)*256 +
          (data[3]/16)*16 + (data[3]%16)*1;
    if(dec <= 2147483647/*0x7FFFFFFF*/)
    {
        return dec;
    }
    else
    {
        dec = /*0xFFFFFFFF*/4294967295 - dec + 1;
        return dec * -1;
    }
}
void SOLOMotorController::ConvertToData(long l, unsigned char data[])
{
    long dec = l;
    if(dec<0) 
    {
        dec*=-1;
        dec = 0xFFFFFFFF - dec + 1;
    }
    //char data[4] = {0x00,0x00,0x00,0x00};
    data[0] = (dec/16777216);
    dec = dec%16777216;
    data[1] = (dec/65536);
    dec = dec%65536;
    data[2] = (dec/256);
    dec = dec%256;
    data[3] = (dec);
}
void SOLOMotorController::SplitData(unsigned char data[], unsigned char cmd[])
{
    data[0] = cmd[2];
    data[1] = cmd[3];
    data[2] = cmd[4];
    data[3] = cmd[5];
}
bool SOLOMotorController::SetAddress(unsigned char newAddr)
{
    unsigned char cmd[] = {addr,WriteAddress,0x00,0x00,0x00,newAddr};
    
    if(newAddr < 0 || newAddr > 254)
    {
        return false;
    }
    
    return SOLOMotorController::ExeCMD(cmd);       
}
bool SOLOMotorController::SetCommandMode(bool mode)
{
    unsigned char cmd[] = {addr,WriteCommandMode,0x00,0x00,0x00,mode};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetCurrentLimit(float A)
{
    if (A < 0.2 || A > 32)
    {
        return false;
    }
        
    unsigned char data[4];
    ConvertToData(A, data);
    unsigned char cmd[] = {addr,WriteCurrentLimit,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetTorqueReference(float A)
{
    if (A < 0.2 || A > 32)
    {
        return false;
    }
    
    unsigned char data[4];
    ConvertToData(A, data);
    unsigned char cmd[] = {addr,WriteTorqueReference,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetSpeedReference(long rmp)
{
    if (rmp < 0 && rmp > 30000)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(rmp, data);
    unsigned char cmd[] = {addr,WriteSpeedReference,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetPowerReference(float P)
{
    if (P < 0 || P > 100)
    {
        return false;
    }
        
    unsigned char data[4];
    ConvertToData(P, data);
    unsigned char cmd[] = {addr,WritePowerReference,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetIdentification(bool S)
{
    unsigned char cmd[] = {addr,WriteIdentification,0x00,0x00,0x00,S};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::StopSystem()
{
    unsigned char cmd[] = {addr,WriteStopSystem,0x00,0x00,0x00,0x00};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetPWMFrequency(long pwm)
{
    if (pwm < 8 || pwm > 80)
    {
        return false;
    }
        
    unsigned char data[4];
    ConvertToData(pwm, data);
    unsigned char cmd[] = {addr,WritePWMFrequency,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
    
}
bool SOLOMotorController::SetSpeedControllerKp(float Kp)
{
    if (Kp < 0 || Kp > 300)
    {
        return false;
    }
    
    unsigned char data[4];
    ConvertToData(Kp, data);
    unsigned char cmd[] = {addr,WriteSpeedControllerKp,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetSpeedControllerKi(float Ki)
{
    if (Ki < 0 || Ki > 300)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(Ki, data);
    unsigned char cmd[] = {addr,WriteSpeedControllerKi,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetDirection(bool dir)
{
    unsigned char cmd[] = {addr,WriteDirection,0x00,0x00,0x00,dir};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetResistance(float res)
{
    if (res < 0.001 || res > 50)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(res, data);
    unsigned char cmd[] = {addr,WriteResistance,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetInductance(float ind)
{
    if (ind < 0.00001 || ind > 0.2)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(ind, data);
    unsigned char cmd[] = {addr,WriteInductance,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetNumberOfPoles(long poles)
{
    if (poles < 1 || poles > 80)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(poles, data);
    unsigned char cmd[] = {addr,WriteNumberOfPoles,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetEncoderLines(long enc)
{
    if (enc < 1 || enc > 40000)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(enc, data);
    unsigned char cmd[] = {addr,WriteEncoderLines,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetSpeedLimit(long speed)
{
    if (speed < 1 || speed > 30000)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(speed, data);
    unsigned char cmd[] = {addr,WriteSpeedLimit,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::ResetAddress()
{
    unsigned char cmd[] = {0xFF,WriteResetAddress,0x00,0x00,0x00,0xFF};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetSpeedControlMode(bool mode)
{
    unsigned char cmd[] = {addr,WriteSpeedControlMode,0x00,0x00,0x00,mode};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::ResetToFactory()
{
    unsigned char cmd[] = {addr,WriteResetToFactory,0x00,0x00,0x00,0x01};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetMotorType(long type)
{
    if (type < 0 || type > 3)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(type, data);
    unsigned char cmd[] = {addr,WriteSpeedLimit,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetControlMode(long mode)
{
    if (mode < 0 || mode > 2)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(mode, data);
    unsigned char cmd[] = {addr,WriteControlMode,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetCurrentControllerKp(float Kp)
{
    if (Kp < 0 || Kp > 16000)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(Kp, data);
    unsigned char cmd[] = {addr,WriteCurrentControllerKp,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetCurrentControllerKi(float Ki)
{
    if (Ki < 0 || Ki > 16000)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(Ki, data);
    unsigned char cmd[] = {addr,WriteCurrentControllerKi,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetMonitoringMode(bool mode)
{
    unsigned char cmd[] = {addr,WriteMonitoringMode,0x00,0x00,0x00,mode};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetMagnetizingCurrentReference(float A)
{
    if (A < 0 || A > 32)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(A, data);
    unsigned char cmd[] = {addr,WriteMagnetizingCurrentReference,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetDesiredPosition(long pos)
{
    if (pos < -2147483647 || pos > 2147483647)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(pos, data);
    unsigned char cmd[] = {addr,WriteDesiredPosition,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetPositionControllerKp(float Kp)
{
    if (Kp < 0 || Kp > 16000)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(Kp, data);
    unsigned char cmd[] = {addr,WritePositionControllerKp,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetPositionControllerKi(float Ki)
{
    if (Ki < 0 || Ki > 16000)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(Ki, data);
    unsigned char cmd[] = {addr,WritePositionControllerKi,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::ResetPositionToZero()
{
    unsigned char cmd[] = {addr,WriteResetPositionToZero,0x00,0x00,0x00,0x01};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::OverwriteTheErrors()
{
    unsigned char cmd[] = {addr,WriteOverwriteTheErrors,0x00,0x00,0x00,0x00};
    
    return SOLOMotorController::ExeCMD(cmd);
}
// SOG => Sensorless Observer Gain 
bool SOLOMotorController::SetSOGNormalBrushlessMotor(float G)
{
    if (G < 0.01 || G > 1000)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(G, data);
    unsigned char cmd[] = {addr,WriteGainNormalBrushless,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetSOGUltraFastBrushlessMotor(float G)
{
    if (G < 0.01 || G > 1000)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(G, data);
    unsigned char cmd[] = {addr,WriteGainUltraFastBrushless,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetSOGDCMotor(float G)
{
    if (G < 0.01 || G > 1000)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(G, data);
    unsigned char cmd[] = {addr,WriteGainDC,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
// SOFG => Sensorless Observer Filter Gain
bool SOLOMotorController::SetSOFGNormalBrushlessMotor(float G)
{
    if (G < 0.01 || G > 16000)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(G, data);
    unsigned char cmd[] = {addr,WriteFilterGainNormalBrushless,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetSOFGUltraFastBrushlessMotor(float G)
{
    if (G < 0.01 || G > 16000)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(G, data);
    unsigned char cmd[] = {addr,WriteFilterGainUltraFastBrushless,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
bool SOLOMotorController::SetUARTBaudrate(long baudrate)
{
    if (baudrate != 0 || baudrate != 1)
    {
        return false;
    }

    unsigned char data[4];
    ConvertToData(baudrate, data);
    unsigned char cmd[] = {addr,WriteUartBaudRate,data[0],data[1],data[2],data[3]};
    
    return SOLOMotorController::ExeCMD(cmd);
}
//----------Read----------
long SOLOMotorController::GetAddress(long _addr)
{
    unsigned char cmd[] = {_addr,ReadAddress,0x00,0x00,0x00,0x00};
    //return ReadAddress;
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
float SOLOMotorController::GetVoltageA()
{
    unsigned char cmd[] = {addr,ReadVoltageA,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetVoltageB()
{
    unsigned char cmd[] = {addr,ReadVoltageB,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetCurrentA()
{
    unsigned char cmd[] = {addr,ReadCurrentA,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetCurrentB()
{
    unsigned char cmd[] = {addr,ReadCurrentB,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
//Battery Voltage
float SOLOMotorController::GetBusVoltage()
{
    unsigned char cmd[] = {addr,ReadBusVoltage,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetMotorCurrent()
{
    unsigned char cmd[] = {addr,ReadIM,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetMotorVoltage()
{
    unsigned char cmd[] = {addr,ReadVM,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetSpeedControllerKp()
{
    unsigned char cmd[] = {addr,ReadSpeedControllerKp,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetSpeedControllerKi()
{
    unsigned char cmd[] = {addr,ReadSpeedControllerKi,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
long SOLOMotorController::GetPWMFrequency()
{
    unsigned char cmd[] = {addr,ReadPWMFrequency,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
float SOLOMotorController::GetCurrentLimit()
{
    unsigned char cmd[] = {addr,ReadCurrentLimit,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetQuadratureCurrent()
{
    unsigned char cmd[] = {addr,ReadQuadratureCurrent,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetDirectCurrent()
{
    unsigned char cmd[] = {addr,ReadDirectCurrent,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
long SOLOMotorController::GetNumberOfPoles()
{
    unsigned char cmd[] = {addr,ReadNumberOfPoles,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
long SOLOMotorController::GetEncoderLine()
{
    unsigned char cmd[] = {addr,ReadEncoderLine,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
float SOLOMotorController::GetCurrentControllerKp()
{
    unsigned char cmd[] = {addr,ReadCurrentControllerKp,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetCurrentControllerKi()
{
    unsigned char cmd[] = {addr,ReadCurrentControllerKi,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetTemperature()
{
    unsigned char cmd[] = {addr,ReadTemperature,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetResistance()
{
    unsigned char cmd[] = {addr,ReadResistance,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetInductance()
{
    unsigned char cmd[] = {addr,ReadInductance,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
long SOLOMotorController::GetSpeed()
{
    unsigned char cmd[] = {addr,ReadSpeed,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
long SOLOMotorController::GetMotorType()
{
    unsigned char cmd[] = {addr,ReadMotorType,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
long SOLOMotorController::GetSpeedControlMode()
{
    unsigned char cmd[] = {addr,ReadSpeedControlMode,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
long SOLOMotorController::GetCommandMode()
{
    unsigned char cmd[] = {addr,ReadCommandMode,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
long SOLOMotorController::GetControlMode()
{
    unsigned char cmd[] = {addr,ReadControlMode,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
long SOLOMotorController::GetSpeedLimit()
{
    unsigned char cmd[] = {addr,ReadSpeedLimit,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
float SOLOMotorController::GetPositionControllerKp()
{
    unsigned char cmd[] = {addr,ReadPositionControllerKp,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetPositionControllerKi()
{
    unsigned char cmd[] = {addr,ReadPositionControllerKi,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
long SOLOMotorController::GetEncoderPosition()
{
    unsigned char cmd[] = {addr,ReadEncoderPosition,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
//TODO
long SOLOMotorController::GetErrorRegister()
{
    unsigned char cmd[] = {addr,ReadErrorRegister,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
long SOLOMotorController::GetFirmwareVersion()
{
    unsigned char cmd[] = {addr,ReadFirmwareVersion,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
long SOLOMotorController::GetHardwareVersion()
{
    unsigned char cmd[] = {addr,ReadHardwareVersion,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
float SOLOMotorController::GetTorqueReference()
{
    unsigned char cmd[] = {addr,ReadTorque,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
long SOLOMotorController::GetSpeedReference()
{
    unsigned char cmd[] = {addr,ReadSpeedReference,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
float SOLOMotorController::GetMagnetizingCurrent()
{
    unsigned char cmd[] = {addr,ReadMagnetizingCurrent,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
long SOLOMotorController::GetPositionReference()
{
    unsigned char cmd[] = {addr,ReadPositionReference,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
float SOLOMotorController::GetPowerReference()
{
    unsigned char cmd[] = {addr,ReadPowerReference,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
long SOLOMotorController::GetDirectionRotation()
{
    unsigned char cmd[] = {addr,ReadDirectionRotation,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
float SOLOMotorController::GetSOGNormalBrushlessMotor()
{
    unsigned char cmd[] = {addr,ReadGainNormalBrushless,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetSOGUltraFastBrushlessMotor()
{
    unsigned char cmd[] = {addr,ReadGainUltraFastBrushless,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetSOGDCMotor()
{
    unsigned char cmd[] = {addr,ReadGainDC,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetSOFGNormalBrushlessMotor()
{
    unsigned char cmd[] = {addr,ReadFilterGainNormalBrushless,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
float SOLOMotorController::GetSOFGUltraFastBrushlessMotor()
{
    unsigned char cmd[] = {addr,ReadFilterGainUltraFastBrushless,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToFloat(data);
    }
    else return -1;
}
long SOLOMotorController::GetUARTBaudrate()
{
    unsigned char cmd[] = {addr,ReadUartBaudRate,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorController::ExeCMD(cmd))
    {
        unsigned char data[4];
        SOLOMotorController::SplitData(data,cmd);
        return SOLOMotorController::ConvertToLong(data);
    }
    else return -1;
}
