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
#include "SOLOMotorControllersUart.h"


// -------------------- constructor & destructor --------------------
SOLOMotorControllersUart::SOLOMotorControllersUart(unsigned char _deviceAddress, HardwareSerial &_serial, SOLOMotorControllers::UartBaudrate _baudrate, long _millisecondsTimeout, int _packetFailureTrialAttempts)
    :addr( _deviceAddress )
    ,serialToUse(&_serial)
    ,millisecondsTimeout(_millisecondsTimeout)
    ,packetFailureTrialAttempts(_packetFailureTrialAttempts)
{
    switch (_baudrate)
    {
    case rate937500:
        baudrate = 937500;
        break;
    case rate115200:
        baudrate = 115200;
        break;
    default:
        baudrate = 115200;
        break;
    }
    serialToUse->begin(baudrate);
    serialToUse->setTimeout(millisecondsTimeout);    
    soloUtils = new SOLOMotorControllersUtils(); 
}

bool SOLOMotorControllersUart::ExeCMD(unsigned char cmd[], int &error)
{
    unsigned char _cmd[] = {INITIATOR,INITIATOR,cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5],CRC,ENDING};
    unsigned char _readPacket[10] = {0};

    /*
    //DEBUG SENT
    Serial.println("packet SENT");
    char tmp[16];
    for (int i=0; i<10; i++) { 
        sprintf(tmp, "0x%.2X",_cmd[i]); 
        Serial.print(tmp); Serial.print(" ");
        }
    Serial.print("\n");
    */

    bool isPacketFailureTrialAttemptsOverflow = true;
    //FailureTrialAttempts block
    for (int attempts = 0; attempts < packetFailureTrialAttempts; attempts++)
    {
        _readPacket[0] = 0;
        _readPacket[1] = 0;
        _readPacket[2] = 0;
        _readPacket[3] = 0;
        _readPacket[4] = 0;
        _readPacket[5] = 0;
        _readPacket[6] = 0;
        _readPacket[7] = 0;
        _readPacket[8] = 0;
        _readPacket[9] = 0;

        //DELAY for default serial of Arduino 
        //(Arduino serial is shared with usb line and can be over noise line)
        if(serialToUse == &Serial)
        {
            //Serial.println("DELAY");
            delay(50);
        }
        serialToUse->write(_cmd,10);
        size_t returned = serialToUse->readBytes(_readPacket, 10);  //read received data 

        /*
        //DEBUG 
        Serial.print("packet READ");
        char tmp2[16];
        for (int r=0; r<10; r++) { 
            sprintf(tmp2, "0x%.2X",_readPacket[r]); 
            Serial.print(tmp2); Serial.print(" ");
            }
        */
        
        if( (_readPacket[3]!=0 && _readPacket[0] == INITIATOR && _readPacket[1] == INITIATOR && _readPacket[8] == CRC && _readPacket[9] == ENDING ) || //packet response for NON noise comunication
        (_readPacket[0] == 0 && _readPacket[1] == 0 && _readPacket[9] == 0) ) //packet response equal to no response 
        {
            isPacketFailureTrialAttemptsOverflow = false;
            break; 
        } 

        //packet not well formatted
        if(_readPacket[0] != INITIATOR || _readPacket[1] != INITIATOR || _readPacket[8] != CRC || _readPacket[9] != ENDING)
        {
            serialToUse->end();
            delay(50);
            serialToUse->begin(baudrate);
            serialToUse->setTimeout(millisecondsTimeout);
        }
    }

    if(isPacketFailureTrialAttemptsOverflow)
    {
        cmd[0] = ERROR;
        cmd[1] = ERROR;
        cmd[2] = ERROR;
        cmd[3] = ERROR;
        cmd[4] = ERROR;
        cmd[5] = ERROR;
        error = SOLOMotorControllers::Error::packetFailureTrialAttemptsOverflow;
        return false;
    }


	if  (_readPacket[0] == _cmd[0] 
    && _readPacket[1] == _cmd[1] 
    && ( _readPacket[2] == _cmd[2] || _cmd[2] == 0xFF) 
    && _readPacket[3] == _cmd[3]
    && _readPacket[8] == _cmd[8] 
    && _readPacket[9] == _cmd[9])
    {
        cmd[0] = _readPacket[2];
        cmd[1] = _readPacket[3];
        cmd[2] = _readPacket[4];
        cmd[3] = _readPacket[5];
        cmd[4] = _readPacket[6];
        cmd[5] = _readPacket[7];
        error = SOLOMotorControllers::Error::noErrorDetected;
        return true;
    }

    cmd[0] = ERROR;
    cmd[1] = ERROR;
    cmd[2] = ERROR;
    cmd[3] = ERROR;
    cmd[4] = ERROR;
    cmd[5] = ERROR;
    error = SOLOMotorControllers::Error::generalError;
    return false; 
}

bool SOLOMotorControllersUart::SetDeviceAddress(unsigned char deviceAddress, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetDeviceAddressInputValidation(deviceAddress, error))
    {
        return false;
    }

    unsigned char cmd[] = {addr,WriteDeviceAddres,0x00,0x00,0x00,deviceAddress};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);       
}
bool SOLOMotorControllersUart::SetDeviceAddress(unsigned char deviceAddress)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetDeviceAddress(deviceAddress,error);
}
bool SOLOMotorControllersUart::SetCommandMode(CommandMode mode, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

    unsigned char cmd[] = {addr,WriteCommandMode,0x00,0x00,0x00,mode};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetCommandMode(CommandMode mode)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetCommandMode(mode, error);
}
bool SOLOMotorControllersUart::SetCurrentLimit(float currentLimit, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetCurrentLimitInputValidation(currentLimit,error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(currentLimit, data);

    unsigned char cmd[] = {addr,WriteCurrentLimit,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetCurrentLimit(float currentLimit)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetCurrentLimit(currentLimit, error);
}
bool SOLOMotorControllersUart::SetTorqueReferenceIq(float torqueReferenceIq, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetTorqueReferenceIqInputValidation(torqueReferenceIq, error))
    {
        return false;
    }
    
    unsigned char data[4];
    soloUtils->ConvertToData(torqueReferenceIq, data);

    unsigned char cmd[] = {addr,WriteTorqueReferenceIq,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetTorqueReferenceIq(float torqueReferenceIq)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetTorqueReferenceIq(torqueReferenceIq, error);
}
bool SOLOMotorControllersUart::SetSpeedReference(long speedReference, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetSpeedReferenceInputValidation(speedReference, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(speedReference, data);

    unsigned char cmd[] = {addr,WriteSpeedReference,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetSpeedReference(long speedReference)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetSpeedReference(speedReference, error);
}
bool SOLOMotorControllersUart::SetPowerReference(float powerReference, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetPowerReferenceInputValidation(powerReference, error))
    {
        return false;
    }
        
    unsigned char data[4];
    soloUtils->ConvertToData(powerReference, data);

    unsigned char cmd[] = {addr,WritePowerReference,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetPowerReference(float powerReference)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetPowerReference(powerReference, error);
}
bool SOLOMotorControllersUart::MotorParametersIdentification(Action identification, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

    unsigned char cmd[] = {addr,WriteMotorParametersIdentification,0x00,0x00,0x00,identification};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::MotorParametersIdentification(Action identification)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::MotorParametersIdentification(identification, error);
}
bool SOLOMotorControllersUart::EmergencyStop(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

    unsigned char cmd[] = {addr,WriteEmergencyStop,0x00,0x00,0x00,0x00};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::EmergencyStop()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::EmergencyStop(error);
}
bool SOLOMotorControllersUart::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetOutputPwmFrequencyKhzInputValidation(outputPwmFrequencyKhz, error))
    {
        return false;
    }
        
    unsigned char data[4];
    soloUtils->ConvertToData(outputPwmFrequencyKhz, data);

    unsigned char cmd[] = {addr,WriteOutputPwmFrequencyKhz,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetOutputPwmFrequencyKhz(outputPwmFrequencyKhz, error);
}
bool SOLOMotorControllersUart::SetSpeedControllerKp(float speedControllerKp, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetSpeedControllerKpInputValidation(speedControllerKp, error))
    {
        return false;
    }
    
    unsigned char data[4];
    soloUtils->ConvertToData(speedControllerKp, data);

    unsigned char cmd[] = {addr,WriteSpeedControllerKp,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetSpeedControllerKp(float speedControllerKp)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetSpeedControllerKp(speedControllerKp, error);
}
bool SOLOMotorControllersUart::SetSpeedControllerKi(float speedControllerKi, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetSpeedControllerKiInputValidation(speedControllerKi, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(speedControllerKi, data);

    unsigned char cmd[] = {addr,WriteSpeedControllerKi,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetSpeedControllerKi(float speedControllerKi)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetSpeedControllerKi(speedControllerKi, error);
}
bool SOLOMotorControllersUart::SetMotorDirection(Direction motorDirection, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

    unsigned char cmd[] = {addr,WriteMotorDirection,0x00,0x00,0x00,motorDirection};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetMotorDirection(Direction motorDirection)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetMotorDirection(motorDirection, error);
}
bool SOLOMotorControllersUart::SetMotorResistance(float motorResistance, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetMotorResistanceInputValidation(motorResistance, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(motorResistance, data);

    unsigned char cmd[] = {addr,WriteMotorResistance,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetMotorResistance(float motorResistance)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetMotorResistance(motorResistance, error);
}
bool SOLOMotorControllersUart::SetMotorInductance(float motorInductance, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetMotorInductanceInputValidation(motorInductance, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(motorInductance, data);

    unsigned char cmd[] = {addr,WriteMotorInductance,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetMotorInductance(float motorInductance)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetMotorInductance(motorInductance, error);
}
bool SOLOMotorControllersUart::SetMotorPolesCounts(long motorPolesCounts, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetMotorPolesCountsInputValidation(motorPolesCounts, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(motorPolesCounts, data);

    unsigned char cmd[] = {addr,WriteMotorPolesCounts,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetMotorPolesCounts(long motorPolesCounts)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetMotorPolesCounts(motorPolesCounts, error);
}
bool SOLOMotorControllersUart::SetIncrementalEncoderLines(long incrementalEncoderLines, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetIncrementalEncoderLinesInputValidation(incrementalEncoderLines, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(incrementalEncoderLines, data);

    unsigned char cmd[] = {addr,WriteIncrementalEncoderLines,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetIncrementalEncoderLines(long incrementalEncoderLines)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetIncrementalEncoderLines(incrementalEncoderLines, error);
}
bool SOLOMotorControllersUart::SetSpeedLimit(long speedLimit, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetSpeedLimitInputValidation(speedLimit, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(speedLimit, data);

    unsigned char cmd[] = {addr,WriteSpeedLimit,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetSpeedLimit(long speedLimit)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetSpeedLimit(speedLimit, error);
}
bool SOLOMotorControllersUart::ResetDeviceAddress(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

    unsigned char cmd[] = {0xFF,WriteDeviceAddres,0x00,0x00,0x00,0xFF};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::ResetDeviceAddress()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::ResetDeviceAddress(error);
}
bool SOLOMotorControllersUart::SetFeedbackControlMode(FeedbackControlMode mode, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

	unsigned char data[4];
    soloUtils->ConvertToData((long) mode, data);

    unsigned char cmd[] = {addr,WriteFeedbackControlMode,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetFeedbackControlMode(FeedbackControlMode mode)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetFeedbackControlMode(mode, error);
}
bool SOLOMotorControllersUart::ResetFactory(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

    unsigned char cmd[] = {addr,WriteResetFactory,0x00,0x00,0x00,0x01};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::ResetFactory()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::ResetFactory(error);
}
bool SOLOMotorControllersUart::SetMotorType(MotorType motorType, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

    unsigned char data[4];
    soloUtils->ConvertToData((long) motorType, data);

    unsigned char cmd[] = {addr,WriteMotorType,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetMotorType(MotorType motorType)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetMotorType(motorType, error);
}
bool SOLOMotorControllersUart::SetControlMode(ControlMode controlMode, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

    unsigned char data[4];
    soloUtils->ConvertToData((long) controlMode, data);

    unsigned char cmd[] = {addr,WriteControlMode,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetControlMode(ControlMode controlMode)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetControlMode(controlMode, error);
}
bool SOLOMotorControllersUart::SetCurrentControllerKp(float currentControllerKp, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetCurrentControllerKpInputValidation(currentControllerKp, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(currentControllerKp, data);

    unsigned char cmd[] = {addr,WriteCurrentControllerKp,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetCurrentControllerKp(float currentControllerKp)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetCurrentControllerKp(currentControllerKp, error);
}
bool SOLOMotorControllersUart::SetCurrentControllerKi(float currentControllerKi, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetCurrentControllerKiInputValidation(currentControllerKi, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(currentControllerKi, data);

    unsigned char cmd[] = {addr,WriteCurrentControllerKi,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetCurrentControllerKi(float currentControllerKi)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetCurrentControllerKi(currentControllerKi, error);    
}
bool SOLOMotorControllersUart::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetMagnetizingCurrentIdReferenceInputValidation(magnetizingCurrentIdReference, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(magnetizingCurrentIdReference, data);

    unsigned char cmd[] = {addr,WriteMagnetizingCurrentIdReference,data[0],data[1],data[2],data[3]}; 
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetMagnetizingCurrentIdReference(magnetizingCurrentIdReference, error);
}
bool SOLOMotorControllersUart::SetPositionReference(long positionReference, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetPositionReferenceInputValidation(positionReference, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(positionReference, data);

    unsigned char cmd[] = {addr,WritePositionReference,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetPositionReference(long positionReference)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetPositionReference(positionReference, error);
}
bool SOLOMotorControllersUart::SetPositionControllerKp(float positionControllerKp, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetPositionControllerKpInputValidation(positionControllerKp, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(positionControllerKp, data);

    unsigned char cmd[] = {addr,WritePositionControllerKp,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetPositionControllerKp(float positionControllerKp)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetPositionControllerKp(positionControllerKp, error);
}
bool SOLOMotorControllersUart::SetPositionControllerKi(float positionControllerKi, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetPositionControllerKiInputValidation(positionControllerKi, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(positionControllerKi, data);

    unsigned char cmd[] = {addr,WritePositionControllerKi,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetPositionControllerKi(float positionControllerKi)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetPositionControllerKi(positionControllerKi, error);
}
bool SOLOMotorControllersUart::ResetPositionToZero(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

    unsigned char cmd[] = {addr,WriteResetPositionToZero,0x00,0x00,0x00,0x01};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::ResetPositionToZero()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::ResetPositionToZero(error);
}
bool SOLOMotorControllersUart::OverwriteErrorRegister(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

    unsigned char cmd[] = {addr,WriteOverwriteErrorRegister,0x00,0x00,0x00,0x00};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::OverwriteErrorRegister()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::OverwriteErrorRegister(error);
}
// SOG => Sensorless Observer Gain 
bool SOLOMotorControllersUart::SetObserverGainBldcPmsm(float observerGain, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetObserverGainBldcPmsmInputValidation(observerGain, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(observerGain, data);

    unsigned char cmd[] = {addr,WriteObserverGainBldcPmsm,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetObserverGainBldcPmsm(float observerGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetObserverGainBldcPmsm(observerGain, error);
}
bool SOLOMotorControllersUart::SetObserverGainBldcPmsmUltrafast(float observerGain, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetObserverGainBldcPmsmUltrafastInputValidation(observerGain, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(observerGain, data);

    unsigned char cmd[] = {addr,WriteObserverGainBldcPmsmUltrafast,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetObserverGainBldcPmsmUltrafast(float observerGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetObserverGainBldcPmsmUltrafast(observerGain, error);
}
bool SOLOMotorControllersUart::SetObserverGainDc(float observerGain, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetObserverGainDcInputValidation(observerGain, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(observerGain, data);

    unsigned char cmd[] = {addr,WriteObserverGainDc,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetObserverGainDc(float observerGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetObserverGainDc(observerGain, error);
}
// SOFG => Sensorless Observer Filter Gain
bool SOLOMotorControllersUart::SetFilterGainBldcPmsm(float filterGain, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetFilterGainBldcPmsmInputValidation(filterGain, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(filterGain, data);

    unsigned char cmd[] = {addr,WriteFilterGainBldcPmsm,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetFilterGainBldcPmsm(float filterGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetFilterGainBldcPmsm(filterGain, error);
}
bool SOLOMotorControllersUart::SetFilterGainBldcPmsmUltrafast(float filterGain, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetFilterGainBldcPmsmUltrafastInputValidation(filterGain, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(filterGain, data);

    unsigned char cmd[] = {addr,WriteFilterGainBldcPmsmUltrafast,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetFilterGainBldcPmsmUltrafast(float filterGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetFilterGainBldcPmsmUltrafast(filterGain, error);
}
bool SOLOMotorControllersUart::SetUartBaudrate(UartBaudrate baudrate, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

    unsigned char data[4];
    soloUtils->ConvertToData((long) baudrate, data);

    unsigned char cmd[] = {addr,WriteUartBaudrate,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetUartBaudrate(UartBaudrate baudrate)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetUartBaudrate(baudrate, error);
}
bool SOLOMotorControllersUart::SensorCalibration(PositionSensorCalibrationAction calibrationAction, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

    unsigned char data[4];
    soloUtils->ConvertToData((long) calibrationAction, data);

    unsigned char cmd[] = {addr, WriteSensorCalibration, data[0], data[1], data[2], data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SensorCalibration(PositionSensorCalibrationAction calibrationAction)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SensorCalibration(calibrationAction, error);
}
bool SOLOMotorControllersUart::SetEncoderHallCcwOffset(float encoderHallOffset, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetEncoderHallCcwOffsetInputValidation(encoderHallOffset, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(encoderHallOffset, data);

    unsigned char cmd[] = {addr, WriteEncoderHallCcwOffset, data[0], data[1], data[2], data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetEncoderHallCcwOffset(float encoderHallOffset)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetEncoderHallCcwOffset(encoderHallOffset, error);
}
bool SOLOMotorControllersUart::SetEncoderHallCwOffset(float encoderHallOffset, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetEncoderHallCwOffsetInputValidation(encoderHallOffset, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(encoderHallOffset, data);

    unsigned char cmd[] = {addr, WriteEncoderHallCwOffset, data[0], data[1], data[2], data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetEncoderHallCwOffset(float encoderHallOffset)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetEncoderHallCwOffset(encoderHallOffset, error);
}
bool SOLOMotorControllersUart::SetSpeedAccelerationValue(float speedAccelerationValue, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetSpeedAccelerationValueInputValidation(speedAccelerationValue, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(speedAccelerationValue, data);

    unsigned char cmd[] = {addr, WriteSpeedAccelerationValue, data[0], data[1], data[2], data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetSpeedAccelerationValue(float speedAccelerationValue)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetSpeedAccelerationValue(speedAccelerationValue, error);
}
bool SOLOMotorControllersUart::SetSpeedDecelerationValue(float speedDecelerationValue, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetSpeedDecelerationValueInputValidation(speedDecelerationValue, error))
    {
        return false;
    }

    unsigned char data[4];
    soloUtils->ConvertToData(speedDecelerationValue, data);

    unsigned char cmd[] = {addr, WriteSpeedDecelerationValue, data[0], data[1], data[2], data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetSpeedDecelerationValue(float speedDecelerationValue)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetSpeedDecelerationValue(speedDecelerationValue, error);
}
bool SOLOMotorControllersUart::SetCanbusBaudrate(CanbusBaudrate canbusBaudrate, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

    unsigned char data[4];
    soloUtils->ConvertToData((long) canbusBaudrate, data);

    unsigned char cmd[] = {addr,WriteUartBaudrate,data[0],data[1],data[2],data[3]};
    return SOLOMotorControllersUart::ExeCMD(cmd,error);
}
bool SOLOMotorControllersUart::SetCanbusBaudrate(CanbusBaudrate canbusBaudrate)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::SetCanbusBaudrate(canbusBaudrate, error);
}
//----------Read----------
long SOLOMotorControllersUart::GetDeviceAddress(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {0xFF,ReadDeviceAddress,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetDeviceAddress()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetDeviceAddress(error);
}
float SOLOMotorControllersUart::GetPhaseAVoltage(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadPhaseAVoltage,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetPhaseAVoltage()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetPhaseAVoltage(error);
}
float SOLOMotorControllersUart::GetPhaseBVoltage(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadPhaseBVoltage,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetPhaseBVoltage()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetPhaseBVoltage(error);
}
float SOLOMotorControllersUart::GetPhaseACurrent(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadPhaseACurrent,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetPhaseACurrent()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetPhaseACurrent(error);
}
float SOLOMotorControllersUart::GetPhaseBCurrent(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadPhaseBCurrent,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetPhaseBCurrent()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetPhaseBCurrent(error);    
}
//Battery Voltage
float SOLOMotorControllersUart::GetBusVoltage(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadBusVoltage,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd ,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetBusVoltage()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetBusVoltage(error);
}
float SOLOMotorControllersUart::GetDcMotorCurrentIm(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadDcMotorCurrentIm,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetDcMotorCurrentIm()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetDcMotorCurrentIm(error);
}
float SOLOMotorControllersUart::GetDcMotorVoltageVm(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadDcMotorVoltageVm,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetDcMotorVoltageVm()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetDcMotorVoltageVm(error);
}
float SOLOMotorControllersUart::GetSpeedControllerKp(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadSpeedControllerKp,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetSpeedControllerKp()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetSpeedControllerKp(error);
}
float SOLOMotorControllersUart::GetSpeedControllerKi(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadSpeedControllerKi,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetSpeedControllerKi()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetSpeedControllerKi(error);
}
long SOLOMotorControllersUart::GetOutputPwmFrequencyKhz(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadOutputPwmFrequencyHz,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return (soloUtils->ConvertToLong(data)/1000L); //PWM reading is in Hz
    }
    return -1;
}
long SOLOMotorControllersUart::GetOutputPwmFrequencyKhz()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetOutputPwmFrequencyKhz(error);
}
float SOLOMotorControllersUart::GetCurrentLimit(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadCurrentLimit,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetCurrentLimit()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetCurrentLimit(error);
}
float SOLOMotorControllersUart::GetQuadratureCurrentIqFeedback(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadQuadratureCurrentIqFeedback,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetQuadratureCurrentIqFeedback()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetQuadratureCurrentIqFeedback(error);
}
float SOLOMotorControllersUart::GetMagnetizingCurrentIdFeedback(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadMagnetizingCurrentIdFeedback,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetMagnetizingCurrentIdFeedback()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetMagnetizingCurrentIdFeedback(error);
}
long SOLOMotorControllersUart::GetMotorPolesCounts(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadMotorPolesCounts,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetMotorPolesCounts()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetMotorPolesCounts(error);
}
long SOLOMotorControllersUart::GetIncrementalEncoderLines(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadIncrementalEncoderLines,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetIncrementalEncoderLines()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetIncrementalEncoderLines(error);
}
float SOLOMotorControllersUart::GetCurrentControllerKp(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadCurrentControllerKp,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetCurrentControllerKp()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetCurrentControllerKp(error);
}
float SOLOMotorControllersUart::GetCurrentControllerKi(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadCurrentControllerKi,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd, error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data) * 0.00005;
    }
    return -1;
}
float SOLOMotorControllersUart::GetCurrentControllerKi()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetCurrentControllerKi(error);
}
float SOLOMotorControllersUart::GetBoardTemperature(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadBoardTemperature,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetBoardTemperature()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetBoardTemperature(error);
}
float SOLOMotorControllersUart::GetMotorResistance(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadMotorResistance,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetMotorResistance()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetMotorResistance(error);
}
float SOLOMotorControllersUart::GetMotorInductance(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadMotorInductance,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetMotorInductance()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetMotorInductance(error);
}
long SOLOMotorControllersUart::GetSpeedFeedback(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadSpeedFeedback,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetSpeedFeedback()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetSpeedFeedback(error);
}
long SOLOMotorControllersUart::GetMotorType(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadMotorType,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetMotorType()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetMotorType(error);
}
long SOLOMotorControllersUart::GetFeedbackControlMode(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadFeedbackControlMode,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetFeedbackControlMode()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetFeedbackControlMode(error);
}
long SOLOMotorControllersUart::GetCommandMode(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadCommandMode,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return  soloUtils->ConvertToLong(data);
        
    }
    return -1;
}
long SOLOMotorControllersUart::GetCommandMode()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetCommandMode(error);
}
long SOLOMotorControllersUart::GetControlMode(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadControlMode,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetControlMode()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetControlMode(error);
}
long SOLOMotorControllersUart::GetSpeedLimit(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadSpeedLimit,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetSpeedLimit()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetSpeedLimit(error);    
}
float SOLOMotorControllersUart::GetPositionControllerKp(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadPositionControllerKp,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetPositionControllerKp()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetPositionControllerKp(error);
}
float SOLOMotorControllersUart::GetPositionControllerKi(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadPositionControllerKi,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetPositionControllerKi()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetPositionControllerKi(error);
}
long SOLOMotorControllersUart::GetPositionCountsFeedback(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadPositionCountsFeedback,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetPositionCountsFeedback()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetPositionCountsFeedback(error);
}
long SOLOMotorControllersUart::GetErrorRegister(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadErrorRegister,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetErrorRegister()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetErrorRegister(error);
}
long SOLOMotorControllersUart::GetDeviceFirmwareVersion(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadDeviceFirmwareVersion,0x00,0x00,0x00,0x00};
    
    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetDeviceFirmwareVersion()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetDeviceFirmwareVersion(error);
}
long SOLOMotorControllersUart::GetDeviceHardwareVersion(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadDeviceHardwareVersion,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetDeviceHardwareVersion()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetDeviceHardwareVersion(error);
}
float SOLOMotorControllersUart::GetTorqueReferenceIq(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadTorqueReferenceIq,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetTorqueReferenceIq()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetTorqueReferenceIq(error);
}
long SOLOMotorControllersUart::GetSpeedReference(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadSpeedReference,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetSpeedReference()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetSpeedReference(error);
}
float SOLOMotorControllersUart::GetMagnetizingCurrentIdReference(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadMagnetizingCurrentIdReference,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetMagnetizingCurrentIdReference()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetMagnetizingCurrentIdReference(error);
}
long SOLOMotorControllersUart::GetPositionReference(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadPositionReference,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetPositionReference()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetPositionReference(error);
}
float SOLOMotorControllersUart::GetPowerReference(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadPowerReference,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetPowerReference()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetPowerReference(error);
}
long SOLOMotorControllersUart::GetMotorDirection(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadMotorDirection,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetMotorDirection()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetMotorDirection(error);
}
float SOLOMotorControllersUart::GetObserverGainBldcPmsm(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadObserverGainBldcPmsm,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetObserverGainBldcPmsm()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetObserverGainBldcPmsm(error);
}
float SOLOMotorControllersUart::GetObserverGainBldcPmsmUltrafast(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadObserverGainBldcPmsmUltrafast,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetObserverGainBldcPmsmUltrafast()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetObserverGainBldcPmsmUltrafast(error);
}
float SOLOMotorControllersUart::GetObserverGainDc(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadObserverGainDc,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetObserverGainDc()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetObserverGainDc(error);
}
float SOLOMotorControllersUart::GetFilterGainBldcPmsm(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadFilterGainBldcPmsm,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetFilterGainBldcPmsm()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetFilterGainBldcPmsm(error);
}
float SOLOMotorControllersUart::GetFilterGainBldcPmsmUltrafast(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadFilterGainBldcPmsmUltrafast,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetFilterGainBldcPmsmUltrafast()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetFilterGainBldcPmsmUltrafast(error);
}
float SOLOMotorControllersUart::Get3PhaseMotorAngle(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr, Read3PhaseMotorAngle, 0x00, 0x00, 0x00, 0x00};

    if (SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data, cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::Get3PhaseMotorAngle()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::Get3PhaseMotorAngle(error);
}
float SOLOMotorControllersUart::GetEncoderHallCcwOffset(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr, ReadEncoderHallCcwOffset, 0x00, 0x00, 0x00, 0x00};

    if (SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data, cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetEncoderHallCcwOffset()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetEncoderHallCcwOffset(error);
}
float SOLOMotorControllersUart::GetEncoderHallCwOffset(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr, ReadEncoderHallCwOffset, 0x00, 0x00, 0x00, 0x00};

    if (SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data, cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetEncoderHallCwOffset()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetEncoderHallCwOffset(error);
}
long SOLOMotorControllersUart::GetUartBaudrate(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadUartBaudrate,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetUartBaudrate()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetUartBaudrate(error);
}
float SOLOMotorControllersUart::GetSpeedAccelerationValue(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr, ReadSpeedAccelerationValue, 0x00, 0x00, 0x00, 0x00};

    if (SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data, cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetSpeedAccelerationValue()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetSpeedAccelerationValue(error);
}
float SOLOMotorControllersUart::GetSpeedDecelerationValue(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr, ReadSpeedDecelerationValue, 0x00, 0x00, 0x00, 0x00};

    if (SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data, cmd);
        return soloUtils->ConvertToFloat(data);
    }
    return -1;
}
float SOLOMotorControllersUart::GetSpeedDecelerationValue()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetSpeedDecelerationValue(error);
}
long SOLOMotorControllersUart::GetEncoderIndexCounts(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr,ReadEncoderIndexCounts,0x00,0x00,0x00,0x00};

    if(SOLOMotorControllersUart::ExeCMD(cmd,error))
    {
        unsigned char data[4];
        soloUtils->SplitData(data,cmd);
        return soloUtils->ConvertToLong(data);
    }
    return -1;
}
long SOLOMotorControllersUart::GetEncoderIndexCounts()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::GetEncoderIndexCounts(error);
}
bool SOLOMotorControllersUart::CommunicationIsWorking(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    float temperature = GetBoardTemperature(error);
    if (error == SOLOMotorControllers::Error::noErrorDetected){
        return true;
    } 
    return false;
}
bool SOLOMotorControllersUart::CommunicationIsWorking()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersUart::CommunicationIsWorking(error);
}