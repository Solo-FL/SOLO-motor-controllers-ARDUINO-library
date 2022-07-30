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
#include <Arduino.h>
#include "SOLOMotorControllersCanopen.h"

SOLOMotorControllersCanopen::SOLOMotorControllersCanopen(unsigned char _deviceAddress , unsigned char _chipSelectPin, SOLOMotorControllers::CanbusBaudrate _baudrate, long _countTimeout)
{
    uint16_t baudrate = 1000;
    countTimeout = _countTimeout;

    if(_deviceAddress == 0 ) //Address 0 is reserved for the host 
    {
        _deviceAddress = 1;
    }
    Address = _deviceAddress;

    switch (_baudrate)
    {
    case rate1000:
        baudrate = 1000;
        break;
    case rate500:
        baudrate = 500;
        break;
    case rate250:
        baudrate = 250;
        break;
    case rate125:
        baudrate = 125;
        break;
    case rate100:
        baudrate = 100;
        break;
    default:
        baudrate = 1000;
        break;
    }

    _MCP2515 = new MCP2515(_chipSelectPin, countTimeout) ;
    _MCP2515->MCP2515_Init(baudrate) ;
    soloUtils = new SOLOMotorControllersUtils();
}
bool SOLOMotorControllersCanopen::SetGuardTime(long guardtime, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetGuardTimeInputValidation(guardtime,error))
    {
        return false;
    }
    soloUtils->ConvertToData(guardtime, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_GuardTime,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetGuardTime(long guardtime)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetGuardTime(guardtime,error);
}
bool SOLOMotorControllersCanopen::SetLifeTimeFactor(long lifeTimeFactor, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetLifeTimeFactorInputValidation(lifeTimeFactor,error))
    {
        return false;
    }
    soloUtils->ConvertToData(lifeTimeFactor, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_LifeTimeFactor,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetLifeTimeFactor(long lifeTimeFactor)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetLifeTimeFactor(lifeTimeFactor,error);
}
bool SOLOMotorControllersCanopen::SetProducerHeartbeatTime(long producerHeartbeatTime, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetProducerHeartbeatTimeInputValidation(producerHeartbeatTime,error))
    {
        return false;
    }
    soloUtils->ConvertToData(producerHeartbeatTime, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_ProducerHeartbeatTime,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetProducerHeartbeatTime(long producerHeartbeatTime)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetProducerHeartbeatTime(producerHeartbeatTime,error);
}
bool SOLOMotorControllersCanopen::SetDeviceAddress(unsigned char deviceAddress, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetDeviceAddressInputValidation(deviceAddress, error))
    {
        return false;
    }
    soloUtils->ConvertToData((long) deviceAddress, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_SetDeviceAddress,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetDeviceAddress(unsigned char deviceAddress)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetDeviceAddress(deviceAddress,error);
}
bool SOLOMotorControllersCanopen::SetCommandMode(SOLOMotorControllers::CommandMode mode, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) mode, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_CommandMode,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetCommandMode(SOLOMotorControllers::CommandMode mode)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetCommandMode(mode, error);
}
bool SOLOMotorControllersCanopen::SetCurrentLimit(float currentLimit, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetCurrentLimitInputValidation(currentLimit,error))
    {
        return false;
    }
    soloUtils->ConvertToData(currentLimit, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_CurrentLimit,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetCurrentLimit(float currentLimit)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetCurrentLimit(currentLimit, error);
}
bool SOLOMotorControllersCanopen::SetTorqueReferenceIq(float torqueReferenceIq, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetTorqueReferenceIqInputValidation(torqueReferenceIq, error))
    {
        return false;
    }
    soloUtils->ConvertToData(torqueReferenceIq, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_TorqueReferenceIq,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetTorqueReferenceIq(float torqueReferenceIq)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetTorqueReferenceIq(torqueReferenceIq, error);
}
bool SOLOMotorControllersCanopen::SetSpeedReference(long speedReference, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetSpeedReferenceInputValidation(speedReference, error))
    {
        return false;
    }
    soloUtils->ConvertToData(speedReference, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_SpeedReference,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetSpeedReference(long speedReference)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetSpeedReference(speedReference, error);
}
bool SOLOMotorControllersCanopen::SetPowerReference(float powerReference, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetPowerReferenceInputValidation(powerReference, error))
    {
        return false;
    }
    soloUtils->ConvertToData(powerReference, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_PowerReference,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetPowerReference(float powerReference)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetPowerReference(powerReference, error);
}
bool SOLOMotorControllersCanopen::MotorParametersIdentification(SOLOMotorControllers::Action identification, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) identification, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_MotorParametersIdentification,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::MotorParametersIdentification(SOLOMotorControllers::Action identification)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::MotorParametersIdentification(identification, error);
}
bool SOLOMotorControllersCanopen::EmergencyStop(int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    return _MCP2515->CANOpenTransmit(Address,Object_EmergencyStop,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::EmergencyStop()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::EmergencyStop(error);
}
bool SOLOMotorControllersCanopen::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int &error) 
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetOutputPwmFrequencyKhzInputValidation(outputPwmFrequencyKhz, error))
    {
        return false;
    }
    soloUtils->ConvertToData(outputPwmFrequencyKhz , informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_OutputPwmFrequencyKhz,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz) 
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetOutputPwmFrequencyKhz(outputPwmFrequencyKhz, error);
}
bool SOLOMotorControllersCanopen::SetSpeedControllerKp(float speedControllerKp, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetSpeedControllerKpInputValidation(speedControllerKp, error))
    {
        return false;
    }
    soloUtils->ConvertToData(speedControllerKp, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_SpeedControllerKp,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetSpeedControllerKp(float speedControllerKp)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetSpeedControllerKp(speedControllerKp, error);
}
bool SOLOMotorControllersCanopen::SetSpeedControllerKi(float speedControllerKi, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetSpeedControllerKiInputValidation(speedControllerKi, error))
    {
        return false;
    }
    soloUtils->ConvertToData(speedControllerKi, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_SpeedControllerKi, informatrionToSend, error);
}
bool SOLOMotorControllersCanopen::SetSpeedControllerKi(float speedControllerKi)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetSpeedControllerKi(speedControllerKi, error);
}
bool SOLOMotorControllersCanopen::SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) motorDirection, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_MotorDirection, informatrionToSend, error);
}
bool SOLOMotorControllersCanopen::SetMotorDirection(SOLOMotorControllers::Direction motorDirection)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotorDirection(motorDirection, error);
}
bool SOLOMotorControllersCanopen::SetMotorResistance(float motorResistance, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetMotorResistanceInputValidation(motorResistance, error))
    {
        return false;
    }
    soloUtils->ConvertToData(motorResistance, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_MotorResistance, informatrionToSend, error);
}
bool SOLOMotorControllersCanopen::SetMotorResistance(float motorResistance)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotorResistance(motorResistance, error);
}
bool SOLOMotorControllersCanopen::SetMotorInductance(float motorInductance, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetMotorInductanceInputValidation(motorInductance, error))
    {
        return false;
    }
    soloUtils->ConvertToData(motorInductance, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_MotorInductance, informatrionToSend, error);
}
bool SOLOMotorControllersCanopen::SetMotorInductance(float motorInductance)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotorInductance(motorInductance, error);
}
bool SOLOMotorControllersCanopen::SetMotorPolesCounts(long motorPolesCounts, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetMotorPolesCountsInputValidation(motorPolesCounts, error))
    {
        return false;
    }

    soloUtils->ConvertToData(motorPolesCounts, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_MotorPolesCounts, informatrionToSend, error);
}
bool SOLOMotorControllersCanopen::SetMotorPolesCounts(long motorPolesCounts)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotorPolesCounts(motorPolesCounts, error);
}
bool SOLOMotorControllersCanopen::SetIncrementalEncoderLines(long incrementalEncoderLines, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetIncrementalEncoderLinesInputValidation(incrementalEncoderLines, error))
    {
        return false;
    }
    soloUtils->ConvertToData(incrementalEncoderLines, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_IncrementalEncoderLines, informatrionToSend, error);
}
bool SOLOMotorControllersCanopen::SetIncrementalEncoderLines(long incrementalEncoderLines)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetIncrementalEncoderLines(incrementalEncoderLines, error);
}
bool SOLOMotorControllersCanopen::SetSpeedLimit(long speedLimit, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetSpeedLimitInputValidation(speedLimit, error))
    {
        return false;
    }
    soloUtils->ConvertToData(speedLimit, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_SpeedLimit, informatrionToSend, error);
}
bool SOLOMotorControllersCanopen::SetSpeedLimit(long speedLimit)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetSpeedLimit(speedLimit, error);
}
bool SOLOMotorControllersCanopen::SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) mode, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address, Object_FeedbackControlMode , informatrionToSend, error);
}
bool SOLOMotorControllersCanopen::SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetFeedbackControlMode(mode, error);	
}
bool SOLOMotorControllersCanopen::ResetFactory(int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x01};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    return _MCP2515->CANOpenTransmit( Address, Object_ResetFactory , informatrionToSend, error);
}
bool SOLOMotorControllersCanopen::ResetFactory()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::ResetFactory(error);
}
bool SOLOMotorControllersCanopen::SetMotorType(SOLOMotorControllers::MotorType motorType, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
	error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) motorType, informatrionToSend);
    return _MCP2515->CANOpenTransmit( Address, Object_MotorType , informatrionToSend, error);
}
bool SOLOMotorControllersCanopen::SetMotorType(SOLOMotorControllers::MotorType motorType)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotorType(motorType, error);
}
bool SOLOMotorControllersCanopen::SetControlMode(SOLOMotorControllers::ControlMode controlMode, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
	error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) controlMode, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_ControlMode,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetControlMode(SOLOMotorControllers::ControlMode controlMode)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetControlMode(controlMode, error);
}
bool SOLOMotorControllersCanopen::SetCurrentControllerKp(float currentControllerKp, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetCurrentControllerKpInputValidation(currentControllerKp, error))
    {
        return false;
    }
    soloUtils->ConvertToData(currentControllerKp, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_CurrentControllerKp,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetCurrentControllerKp(float currentControllerKp)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetCurrentControllerKp(currentControllerKp, error);
}
bool SOLOMotorControllersCanopen::SetCurrentControllerKi(float currentControllerKi, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetCurrentControllerKiInputValidation(currentControllerKi, error))
    {
        return false;
    }
    soloUtils->ConvertToData(currentControllerKi, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_CurrentControllerKi,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetCurrentControllerKi(float currentControllerKi)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetCurrentControllerKi(currentControllerKi, error);
}
bool SOLOMotorControllersCanopen::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetMagnetizingCurrentIdReferenceInputValidation(magnetizingCurrentIdReference, error))
    {
        return false;
    }
    soloUtils->ConvertToData(magnetizingCurrentIdReference, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_MagnetizingCurrentIdReference,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMagnetizingCurrentIdReference(magnetizingCurrentIdReference, error);
}
bool SOLOMotorControllersCanopen::SetPositionReference(long positionReference, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetPositionReferenceInputValidation(positionReference, error))
    {
        return false;
    }
    soloUtils->ConvertToData(positionReference, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_PositionReference,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetPositionReference(long positionReference)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetPositionReference(positionReference, error);
}
bool SOLOMotorControllersCanopen::SetPositionControllerKp(float positionControllerKp, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetPositionControllerKpInputValidation(positionControllerKp, error))
    {
        return false;
    }
    soloUtils->ConvertToData(positionControllerKp, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_PositionControllerKp,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetPositionControllerKp(float positionControllerKp)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetPositionControllerKp(positionControllerKp, error);
}
bool SOLOMotorControllersCanopen::SetPositionControllerKi(float positionControllerKi, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetPositionControllerKiInputValidation(positionControllerKi, error))
    {
        return false;
    }
    soloUtils->ConvertToData(positionControllerKi, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_PositionControllerKi,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetPositionControllerKi(float positionControllerKi)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetPositionControllerKi(positionControllerKi, error);
}
bool SOLOMotorControllersCanopen::ResetPositionToZero(int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x01};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    return _MCP2515->CANOpenTransmit(Address,Object_ResetPositionToZero,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::ResetPositionToZero()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::ResetPositionToZero(error);
}
bool SOLOMotorControllersCanopen::OverwriteErrorRegister(int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    return _MCP2515->CANOpenTransmit(Address,Object_OverwriteErrorRegister,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::OverwriteErrorRegister()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::OverwriteErrorRegister(error);
}
bool SOLOMotorControllersCanopen::SetObserverGainBldcPmsm(float observerGain, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetObserverGainBldcPmsmInputValidation(observerGain, error))
    {
        return false;
    }
    soloUtils->ConvertToData(observerGain, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_ObserverGainBldcPmsm,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetObserverGainBldcPmsm(float observerGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetObserverGainBldcPmsm(observerGain, error);
}
bool SOLOMotorControllersCanopen::SetObserverGainBldcPmsmUltrafast(float observerGain, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetObserverGainBldcPmsmUltrafastInputValidation(observerGain, error))
    {
        return false;
    }
    soloUtils->ConvertToData(observerGain, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_ObserverGainBldcPmsmUltrafast,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetObserverGainBldcPmsmUltrafast(float observerGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetObserverGainBldcPmsmUltrafast(observerGain, error);
}
bool SOLOMotorControllersCanopen::SetObserverGainDc(float observerGain, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetObserverGainDcInputValidation(observerGain, error))
    {
        return false;
    }
    soloUtils->ConvertToData(observerGain, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_ObserverGainDc,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetObserverGainDc(float observerGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetObserverGainDc(observerGain, error);
}
bool SOLOMotorControllersCanopen::SetFilterGainBldcPmsm(float filterGain, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetFilterGainBldcPmsmInputValidation(filterGain, error))
    {
        return false;
    }
    soloUtils->ConvertToData(filterGain, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_FilterGainBldcPmsm,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetFilterGainBldcPmsm(float filterGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetFilterGainBldcPmsm(filterGain, error);
}
bool SOLOMotorControllersCanopen::SetFilterGainBldcPmsmUltrafast(float filterGain, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetFilterGainBldcPmsmUltrafastInputValidation(filterGain, error))
    {
        return false;
    }
    soloUtils->ConvertToData(filterGain, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_FilterGainBldcPmsmUltrafast,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetFilterGainBldcPmsmUltrafast(float filterGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetFilterGainBldcPmsmUltrafast(filterGain, error);
}
bool SOLOMotorControllersCanopen::SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) baudrate, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_UartBaudrate,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetUartBaudrate(baudrate, error);
}
bool SOLOMotorControllersCanopen::SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long)calibrationAction, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_SensorCalibration,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SensorCalibration(calibrationAction, error);
}
bool SOLOMotorControllersCanopen::SetEncoderHallCcwOffset(float encoderHallOffset, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetEncoderHallCcwOffsetInputValidation(encoderHallOffset, error))
    {
        return false;
    }
    soloUtils->ConvertToData(encoderHallOffset, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_EncoderHallCcwOffset,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetEncoderHallCcwOffset(float encoderHallOffset)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetEncoderHallCcwOffset(encoderHallOffset, error);
}
bool SOLOMotorControllersCanopen::SetEncoderHallCwOffset(float encoderHallOffset, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetEncoderHallCwOffsetInputValidation(encoderHallOffset, error))
    {
        return false;
    }
    soloUtils->ConvertToData(encoderHallOffset, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_EncoderHallCwOffset,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetEncoderHallCwOffset(float encoderHallOffset)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetEncoderHallCwOffset(encoderHallOffset, error);
}
bool SOLOMotorControllersCanopen::SetSpeedAccelerationValue(float speedAccelerationValue, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetSpeedAccelerationValueInputValidation(speedAccelerationValue, error))
    {
        return false;
    }
    soloUtils->ConvertToData(speedAccelerationValue , informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_SpeedAccelerationValue,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetSpeedAccelerationValue(float speedAccelerationValue)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetSpeedAccelerationValue(speedAccelerationValue, error);
}
bool SOLOMotorControllersCanopen::SetSpeedDecelerationValue(float speedDecelerationValue, int &error) 
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetSpeedDecelerationValueInputValidation(speedDecelerationValue, error))
    {
        return false;
    }
    soloUtils->ConvertToData(speedDecelerationValue , informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_SpeedDecelerationValue,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetSpeedDecelerationValue(float speedDecelerationValue) 
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetSpeedDecelerationValue(speedDecelerationValue, error);
}
bool SOLOMotorControllersCanopen::SetCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate canbusBaudrate, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) canbusBaudrate, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_CanbusBaudrate,informatrionToSend,error);
}
bool SOLOMotorControllersCanopen::SetCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate canbusBaudrate)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetCanbusBaudrate(canbusBaudrate, error);
}
//---------------------Read---------------------
long SOLOMotorControllersCanopen::GetReadErrorRegister(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_ReadErrorRegister , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1;
}
long SOLOMotorControllersCanopen::GetReadErrorRegister()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetReadErrorRegister(error);    
}
long SOLOMotorControllersCanopen::GetGuardTime(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_GuardTime , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1;
}
long SOLOMotorControllersCanopen::GetGuardTime()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetGuardTime(error);    
}
long SOLOMotorControllersCanopen::GetLifeTimeFactor(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_LifeTimeFactor , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1;
}
long SOLOMotorControllersCanopen::GetLifeTimeFactor()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetLifeTimeFactor(error);    
}
long SOLOMotorControllersCanopen::GetProducerHeartbeatTime(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_ProducerHeartbeatTime , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1;
}
long SOLOMotorControllersCanopen::GetProducerHeartbeatTime()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetProducerHeartbeatTime(error);    
}
long SOLOMotorControllersCanopen::GetDeviceAddress(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_SetDeviceAddress , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1;
}
long SOLOMotorControllersCanopen::GetDeviceAddress()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetDeviceAddress(error);
}
float SOLOMotorControllersCanopen::GetPhaseAVoltage(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_PhaseAVoltage , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0;
}
float SOLOMotorControllersCanopen::GetPhaseAVoltage()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPhaseAVoltage(error);
}
float SOLOMotorControllersCanopen::GetPhaseBVoltage(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_PhaseBVoltage , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0;
}
float SOLOMotorControllersCanopen::GetPhaseBVoltage()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPhaseBVoltage(error);
}
float SOLOMotorControllersCanopen::GetPhaseACurrent(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_PhaseACurrent , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0;
}
float SOLOMotorControllersCanopen::GetPhaseACurrent()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPhaseACurrent(error);
}
float SOLOMotorControllersCanopen::GetPhaseBCurrent(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_PhaseBCurrent , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0;
}
float SOLOMotorControllersCanopen::GetPhaseBCurrent()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPhaseBCurrent(error);
}
float SOLOMotorControllersCanopen::GetBusVoltage(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_BusVoltage , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0;
}
float SOLOMotorControllersCanopen::GetBusVoltage()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetBusVoltage(error);
}
float SOLOMotorControllersCanopen::GetDcMotorCurrentIm(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_DcMotorCurrentIm , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetDcMotorCurrentIm()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetDcMotorCurrentIm(error);
}
float SOLOMotorControllersCanopen::GetDcMotorVoltageVm(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_DcMotorVoltageVm , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetDcMotorVoltageVm()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetDcMotorVoltageVm(error); 
}
float SOLOMotorControllersCanopen::GetSpeedControllerKp(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_SpeedControllerKp , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetSpeedControllerKp()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetSpeedControllerKp(error);
}
float SOLOMotorControllersCanopen::GetSpeedControllerKi(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_SpeedControllerKi , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetSpeedControllerKi()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetSpeedControllerKi(error);
}
long SOLOMotorControllersCanopen::GetOutputPwmFrequencyKhz(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_OutputPwmFrequencyKhz , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1 ;
}
long SOLOMotorControllersCanopen::GetOutputPwmFrequencyKhz()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetOutputPwmFrequencyKhz(error);
}
float SOLOMotorControllersCanopen::GetCurrentLimit(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_CurrentLimit , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetCurrentLimit()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetCurrentLimit(error);
}
float SOLOMotorControllersCanopen::GetQuadratureCurrentIqFeedback(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_QuadratureCurrentIqFeedback , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetQuadratureCurrentIqFeedback()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetQuadratureCurrentIqFeedback(error);
}
float SOLOMotorControllersCanopen::GetMagnetizingCurrentIdFeedback(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_MagnetizingCurrentIdFeedback , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetMagnetizingCurrentIdFeedback()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetMagnetizingCurrentIdFeedback(error);
}
long SOLOMotorControllersCanopen::GetMotorPolesCounts(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_MotorPolesCounts , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1;
}
long SOLOMotorControllersCanopen::GetMotorPolesCounts()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetMotorPolesCounts(error);
}
long SOLOMotorControllersCanopen::GetIncrementalEncoderLines(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_IncrementalEncoderLines , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1;
}
long SOLOMotorControllersCanopen::GetIncrementalEncoderLines()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetIncrementalEncoderLines(error);
}
float SOLOMotorControllersCanopen::GetCurrentControllerKp(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_CurrentControllerKp , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0;
}
float SOLOMotorControllersCanopen::GetCurrentControllerKp()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetCurrentControllerKp(error);
}
float SOLOMotorControllersCanopen::GetCurrentControllerKi(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_CurrentControllerKi , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0;
}
float SOLOMotorControllersCanopen::GetCurrentControllerKi()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetCurrentControllerKi(error);
}
float SOLOMotorControllersCanopen::GetBoardTemperature(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_BoardTemperature , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0;
}
float SOLOMotorControllersCanopen::GetBoardTemperature()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetBoardTemperature(error);
}
float SOLOMotorControllersCanopen::GetMotorResistance(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_MotorResistance , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0;
}
float SOLOMotorControllersCanopen::GetMotorResistance()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetMotorResistance(error);
}
float SOLOMotorControllersCanopen::GetMotorInductance(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_MotorInductance , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0;
}
float SOLOMotorControllersCanopen::GetMotorInductance()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetMotorInductance(error);
}
long SOLOMotorControllersCanopen::GetSpeedFeedback(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_SpeedFeedback , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1;
}
long SOLOMotorControllersCanopen::GetSpeedFeedback()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetSpeedFeedback(error);
}
long SOLOMotorControllersCanopen::GetMotorType(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_MotorType , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1;
}
long SOLOMotorControllersCanopen::GetMotorType()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetMotorType(error);
}
long SOLOMotorControllersCanopen::GetFeedbackControlMode(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_FeedbackControlMode , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1;
}
long SOLOMotorControllersCanopen::GetFeedbackControlMode()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetFeedbackControlMode(error);
}
long SOLOMotorControllersCanopen::GetCommandMode(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_CommandMode , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1;
}
long SOLOMotorControllersCanopen::GetCommandMode()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetCommandMode(error);
}
long SOLOMotorControllersCanopen::GetControlMode(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_ControlMode , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1;
}
long SOLOMotorControllersCanopen::GetControlMode()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetControlMode(error);
}
long SOLOMotorControllersCanopen::GetSpeedLimit(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_SpeedLimit , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1;
}
long SOLOMotorControllersCanopen::GetSpeedLimit()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetSpeedLimit(error);
}
float SOLOMotorControllersCanopen::GetPositionControllerKp(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_PositionControllerKp , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0;
}
float SOLOMotorControllersCanopen::GetPositionControllerKp()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPositionControllerKp(error); 
}
float SOLOMotorControllersCanopen::GetPositionControllerKi(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_PositionControllerKi , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0;
}
float SOLOMotorControllersCanopen::GetPositionControllerKi()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPositionControllerKi(error);
}
long SOLOMotorControllersCanopen::GetPositionCountsFeedback(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_PositionCountsFeedback , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1 ;
}
long SOLOMotorControllersCanopen::GetPositionCountsFeedback()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPositionCountsFeedback(error);
}
long SOLOMotorControllersCanopen::GetErrorRegister(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_OverwriteErrorRegister , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1 ;
}
long SOLOMotorControllersCanopen::GetErrorRegister()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetErrorRegister(error);
}
long SOLOMotorControllersCanopen::GetDeviceFirmwareVersion(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_DeviceFirmwareVersion , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1 ;
}
long SOLOMotorControllersCanopen::GetDeviceFirmwareVersion()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetDeviceFirmwareVersion(error);
}
long SOLOMotorControllersCanopen::GetDeviceHardwareVersion(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_DeviceHardwareVersion , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1 ;
}
long SOLOMotorControllersCanopen::GetDeviceHardwareVersion()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetDeviceHardwareVersion(error);
}
float SOLOMotorControllersCanopen::GetTorqueReferenceIq(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_TorqueReferenceIq , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetTorqueReferenceIq()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetTorqueReferenceIq(error);
}
long SOLOMotorControllersCanopen::GetSpeedReference(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_SpeedReference , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1 ;
}
long SOLOMotorControllersCanopen::GetSpeedReference()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetSpeedReference(error);
}
float SOLOMotorControllersCanopen::GetMagnetizingCurrentIdReference(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_MagnetizingCurrentIdFeedback , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetMagnetizingCurrentIdReference()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetMagnetizingCurrentIdReference(error);
}
long SOLOMotorControllersCanopen::GetPositionReference(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_PositionReference , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1 ;
}
long SOLOMotorControllersCanopen::GetPositionReference()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPositionReference(error);
}
float SOLOMotorControllersCanopen::GetPowerReference(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_PowerReference , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetPowerReference()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPowerReference(error);
}
long SOLOMotorControllersCanopen::GetMotorDirection(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_MotorDirection , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1 ;
}
long SOLOMotorControllersCanopen::GetMotorDirection()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetMotorDirection(error);
}
float SOLOMotorControllersCanopen::GetObserverGainBldcPmsm(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_ObserverGainBldcPmsm , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetObserverGainBldcPmsm()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetObserverGainBldcPmsm(error);
}
float SOLOMotorControllersCanopen::GetObserverGainBldcPmsmUltrafast(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_ObserverGainBldcPmsmUltrafast , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetObserverGainBldcPmsmUltrafast()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetObserverGainBldcPmsmUltrafast(error);
}
float SOLOMotorControllersCanopen::GetObserverGainDc(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_ObserverGainDc , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetObserverGainDc()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetObserverGainDc(error);
}
float SOLOMotorControllersCanopen::GetFilterGainBldcPmsm(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_FilterGainBldcPmsm , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetFilterGainBldcPmsm()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetFilterGainBldcPmsm(error);
}
float SOLOMotorControllersCanopen::GetFilterGainBldcPmsmUltrafast(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_FilterGainBldcPmsmUltrafast , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetFilterGainBldcPmsmUltrafast()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetFilterGainBldcPmsmUltrafast(error);
}
float SOLOMotorControllersCanopen::Get3PhaseMotorAngle(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_3PhaseMotorAngle , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::Get3PhaseMotorAngle()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::Get3PhaseMotorAngle(error);
}
float SOLOMotorControllersCanopen::GetEncoderHallCcwOffset(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_EncoderHallCcwOffset , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetEncoderHallCcwOffset()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetEncoderHallCcwOffset(error);
}
float SOLOMotorControllersCanopen::GetEncoderHallCwOffset(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_EncoderHallCwOffset , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetEncoderHallCwOffset()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetEncoderHallCwOffset(error);
}
long SOLOMotorControllersCanopen::GetUartBaudrate(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_UartBaudrate , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1 ;
}
long SOLOMotorControllersCanopen::GetUartBaudrate()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetUartBaudrate(error);
}
float SOLOMotorControllersCanopen::GetSpeedAccelerationValue(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_SpeedAccelerationValue , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetSpeedAccelerationValue()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetSpeedAccelerationValue(error);
}
float SOLOMotorControllersCanopen::GetSpeedDecelerationValue(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_SpeedDecelerationValue , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1.0 ;
}
float SOLOMotorControllersCanopen::GetSpeedDecelerationValue()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetSpeedDecelerationValue(error);
}
bool SOLOMotorControllersCanopen::CommunicationIsWorking(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    float temperature = SOLOMotorControllersCanopen::GetBoardTemperature(error);
    if (error == SOLOMotorControllers::Error::noErrorDetected){  
        return true;
    } 
    return false;
}
bool SOLOMotorControllersCanopen::CommunicationIsWorking()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::CommunicationIsWorking(error);
}
long SOLOMotorControllersCanopen::GetEncoderIndexCounts(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_EncoderIndexCounts , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1 ;
    
}
long SOLOMotorControllersCanopen::GetEncoderIndexCounts()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetEncoderIndexCounts(error);
}
void SOLOMotorControllersCanopen::MCP2515_ReadErrorMode(int &errorMode)
{
    _MCP2515->MCP2515_ReadErrorMode(errorMode);
}
uint8_t SOLOMotorControllersCanopen::MCP2515_Read_ReceiveErrorCounter()
{
    return _MCP2515->MCP2515_Read_ReceiveErrorCounter();
}
uint8_t SOLOMotorControllersCanopen::MCP2515_Read_TransmitErrorCounter()
{
    return _MCP2515->MCP2515_Read_TransmitErrorCounter();
}