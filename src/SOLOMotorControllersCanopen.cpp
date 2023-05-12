/**
 *******************************************************************************
 * @file    SOLOMotorControllersCanopen.cpp
 * @authors SOLO Motor Controllers 
 * @brief   This file contains all the functions for the Solo Drivers
 *          canopen communications. 
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 * 
 * @date    Date: 2023
 * @version 4.3.0
 * *******************************************************************************    
 * @attention
 * Copyright: (c) 2021-2023, SOLO motor controllers project
 * GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)
 ******************************************************************************* 
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

/**
  * @brief  This command sets the desired device address for a SOLO unit
  *				.The method refers to the Object Dictionary: 0x3001
  * @param  deviceAddress  address want to set for board
  * @param  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the desired device address for a SOLO unit
  *				.The method refers to the Object Dictionary: 0x3001
  * @param  deviceAddress  address want to set for board      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetDeviceAddress(unsigned char deviceAddress)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetDeviceAddress(deviceAddress,error);
}

/**
  * @brief  This command sets the mode of the operation of SOLO
  *         in terms of operating in Analogue mode or Digital
  *				.The method refers to the Object Dictionary: 0x3002
  * @param  mode  enum that specify mode of the operation of SOLO  
  * @param  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetCommandMode(SOLOMotorControllers::CommandMode mode, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) mode, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_CommandMode,informatrionToSend,error);
}

/**
  * @brief  This command sets the mode of the operation of SOLO
  *         in terms of operating in Analogue mode or Digital
  *				.The method refers to the Object Dictionary: 0x3002
  * @param  mode  enum that specify mode of the operation of SOLO      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetCommandMode(SOLOMotorControllers::CommandMode mode)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetCommandMode(mode, error);
}

/**
  * @brief  This command defines the maximum allowed current into the motor in terms of Amps
  * @param  currentLimit  a float value between 0 to 32
  *				.The method refers to the Object Dictionary: 0x3003
  * @param  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command defines the maximum allowed current into the motor in terms of Amps
  *				.The method refers to the Object Dictionary: 0x3003
  * @param  currentLimit  a float value between 0 to 32      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetCurrentLimit(float currentLimit)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetCurrentLimit(currentLimit, error);
}

/**
  * @brief  This command sets the amount of desired current that acts in torque generation
  *				.The method refers to the Object Dictionary: 0x3004
  * @param  torqueReferenceIq  a float value between 0 to 32
  * @param  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the amount of desired current that acts in torque generation
  *				.The method refers to the Object Dictionary: 0x3004
  * @param  torqueReferenceIq  a float value between 0 to 32      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetTorqueReferenceIq(float torqueReferenceIq)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetTorqueReferenceIq(torqueReferenceIq, error);
}

/**
  * @brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
  *				.The method refers to the Object Dictionary: 0x3005
  * @param  speedReference  a long value between 0 to 30000
  * @param  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
  *				.The method refers to the Object Dictionary: 0x3005
  * @param  speedReference  a long value between 0 to 30000      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetSpeedReference(long speedReference)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetSpeedReference(speedReference, error);
}

/**
  * @brief  This command defines the amount of power percentage during only
  *         Open-loop mode for 3-phase motors
  *				.The method refers to the Object Dictionary: 0x3006
  * @param  powerReference  a float value between 0 to 100
  * @param  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command defines the amount of power percentage during only
  *         Open-loop mode for 3-phase motors
  *				.The method refers to the Object Dictionary: 0x3006
  * @param  powerReference  a float value between 0 to 100      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetPowerReference(float powerReference)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetPowerReference(powerReference, error);
}

/**
  * @brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
  *          identifying the electrical parameters of the Motor connected
  *				.The method refers to the Object Dictionary: 0x3007
  * @param  powerReference  enum that specify Start or Stop of something in SOLO 
  * @param  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::MotorParametersIdentification(SOLOMotorControllers::Action identification, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) identification, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_MotorParametersIdentification,informatrionToSend,error);
}

/**
  * @brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
  *          identifying the electrical parameters of the Motor connected
  *				.The method refers to the Object Dictionary: 0x3007
  * @param  powerReference  enum that specify Start or Stop of something in SOLO   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::MotorParametersIdentification(SOLOMotorControllers::Action identification)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::MotorParametersIdentification(identification, error);
}

/**
  * @brief  This command if the DATA is set at zero will stop the whole power and switching system
            connected to the motor and it will cut the current floating into the Motor from SOLO
				.The method refers to the Object Dictionary: 0x3008
  * @param  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::EmergencyStop(int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    return _MCP2515->CANOpenTransmit(Address,Object_EmergencyStop,informatrionToSend,error);
}

/**
  * @brief  This command if the DATA is set at zero will stop the whole power and switching system
            connected to the motor and it will cut the current floating into the Motor from SOLO    
				.The method refers to the Object Dictionary: 0x3008
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::EmergencyStop()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::EmergencyStop(error);
}

/**
  * @brief  This command sets the output switching frequency of the whole power unit on the Motor
				.The method refers to the Object Dictionary: 0x3009
  * @param  outputPwmFrequencyKhz  switching frequencies in kHz. a long value between 8 to 80 
  * @param  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the output switching frequency of the whole power unit on the Motor
				.The method refers to the Object Dictionary: 0x3009
  * @param  outputPwmFrequencyKhz  switching frequencies in kHz. a long value between 8 to 80      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz) 
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetOutputPwmFrequencyKhz(outputPwmFrequencyKhz, error);
}

/**
  * @brief  This command sets the Speed controller Kp Gain, and it will
  *         be functional only in Digital Closed-loop mode
				.The method refers to the Object Dictionary: 0x300A
  * @param  speedControllerKp  a float value between 0 to 300 
  * @param  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the Speed controller Kp Gain, and it will
  *         be functional only in Digital Closed-loop mode
				.The method refers to the Object Dictionary: 0x300A
  * @param  speedControllerKp  a float value between 0 to 300     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetSpeedControllerKp(float speedControllerKp)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetSpeedControllerKp(speedControllerKp, error);
}

/**
  * @brief  This command sets the Speed controller Ki gain, and it will
  *         be functional only in Digital Closed-loop mode
				.The method refers to the Object Dictionary: 0x300B
  * @param  speedControllerKi  a float value between 0 to 300 
  * @param  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the Speed controller Ki gain, and it will
  *         be functional only in Digital Closed-loop mode
				.The method refers to the Object Dictionary: 0x300B
  * @param  speedControllerKi  a float value between 0 to 300     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetSpeedControllerKi(float speedControllerKi)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetSpeedControllerKi(speedControllerKi, error);
}

/**
  * @brief  This commands sets the direction of the rotation of the motor
  *         either to ClockWise rotation or to Counter Clockwise Rotation
				.The method refers to the Object Dictionary: 0x300C
  * @param  motorDirection  enum that specify the direction of the rotation of the motor 
  * @param  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) motorDirection, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_MotorDirection, informatrionToSend, error);
}

/**
  * @brief  This commands sets the direction of the rotation of the motor
  *         either to ClockWise rotation or to Counter Clockwise Rotation
				.The method refers to the Object Dictionary: 0x300C
  * @param  motorDirection  enum that specify the direction of the rotation of the motor    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotorDirection(SOLOMotorControllers::Direction motorDirection)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotorDirection(motorDirection, error);
}

/**
  * @brief  This command sets the amount of the Phase or Armature resistance
  *         for 3-phase or DC Brushed motors respectively
				.The method refers to the Object Dictionary: 0x300D
  * @param  motorResistance  a float value between 0.001 to 50
  * @param  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the amount of the Phase or Armature resistance
  *         for 3-phase or DC Brushed motors respectively
				.The method refers to the Object Dictionary: 0x300D
  * @param  motorResistance  a float value between 0.001 to 50    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotorResistance(float motorResistance)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotorResistance(motorResistance, error);
}

/**
  * @brief  This command sets the amount of the Phase or Armature Inductance
  *         for 3-phase or DC Brushed motors respectively
				.The method refers to the Object Dictionary: 0x300E
  * @param  motorInductance  a float value between 0.00005 to 0.2
  * @param  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the amount of the Phase or Armature Inductance
  *         for 3-phase or DC Brushed motors respectively
				.The method refers to the Object Dictionary: 0x300E
  * @param  motorInductance  a float value between 0.00005 to 0.2    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotorInductance(float motorInductance)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotorInductance(motorInductance, error);
}

/**
  * @brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
				.The method refers to the Object Dictionary: 0x300F
  * @param  motorPolesCounts  a long value between 1 to 254   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
				.The method refers to the Object Dictionary: 0x300F
  * @param  motorPolesCounts  a long value between 1 to 254    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotorPolesCounts(long motorPolesCounts)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotorPolesCounts(motorPolesCounts, error);
}

/**
  * @brief  This command sets the pre-quad number of physical lines of an 
  *         incremental encoder engraved on its disk
				.The method refers to the Object Dictionary: 0x3010
  * @param  incrementalEncoderLines  a long value between 1 to 200000   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the pre-quad number of physical lines of an 
  *         incremental encoder engraved on its disk
				.The method refers to the Object Dictionary: 0x3010
  * @param  incrementalEncoderLines  a long value between 1 to 200000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetIncrementalEncoderLines(long incrementalEncoderLines)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetIncrementalEncoderLines(incrementalEncoderLines, error);
}

/**
  * @brief  This command sets the allowed speed during trajectory following
  *         in closed-loop position controlling mode
				.The method refers to the Object Dictionary: 0x3011
  * @param  speedLimit  a long value between 0 to 30000   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the allowed speed during trajectory following
  *         in closed-loop position controlling mode
				.The method refers to the Object Dictionary: 0x3011
  * @param  speedLimit  a long value between 0 to 30000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetSpeedLimit(long speedLimit)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetSpeedLimit(speedLimit, error);
}

/**
  * @brief  This command sets the type of the feedback control SOLO has to operate
				.The method refers to the Object Dictionary: 0x3013
  * @param  mode  enum that specify the type of the feedback control SOLO 
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) mode, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address, Object_FeedbackControlMode , informatrionToSend, error);
}

/**
  * @brief  This command sets the type of the feedback control SOLO has to operate
				.The method refers to the Object Dictionary: 0x3013
  * @param  mode  enum that specify the type of the feedback control SOLO  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetFeedbackControlMode(mode, error);	
}

/**
  * @brief  This command resets SOLO to its factory setting to all the default parameters  
				.The method refers to the Object Dictionary: 0x3014
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::ResetFactory(int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x01};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    return _MCP2515->CANOpenTransmit( Address, Object_ResetFactory , informatrionToSend, error);
}

/**
  * @brief  This command resets SOLO to its factory setting to all the default parameters   
				.The method refers to the Object Dictionary: 0x3014
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::ResetFactory()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::ResetFactory(error);
}

/**
  * @brief  This command sets the Motor type that is connected to SOLO in Digital Mode
				.The method refers to the Object Dictionary: 0x3015
  * @param  motorType  enum that specify the Motor type that is connected to SOLO in Digital Mode
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotorType(SOLOMotorControllers::MotorType motorType, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
	error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) motorType, informatrionToSend);
    return _MCP2515->CANOpenTransmit( Address, Object_MotorType , informatrionToSend, error);
}

/**
  * @brief  This command sets the Motor type that is connected to SOLO in Digital Mode
				.The method refers to the Object Dictionary: 0x3015
  * @param  motorType  enum that specify the Motor type that is connected to SOLO in Digital Mode
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotorType(SOLOMotorControllers::MotorType motorType)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotorType(motorType, error);
}

/**
  * @brief  This command sets the Control Mode in terms of Torque,
  *         Speed or Position only in Digital Mode
				.The method refers to the Object Dictionary: 0x3016
  * @param  controlMode  enum that specify the Control Mode in terms of Torque,
  *                       Speed or Position only in Digital Mode
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetControlMode(SOLOMotorControllers::ControlMode controlMode, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
	error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) controlMode, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_ControlMode,informatrionToSend,error);
}

/**
  * @brief  This command sets the Control Mode in terms of Torque,
  *         Speed or Position only in Digital Mode
				.The method refers to the Object Dictionary: 0x3016
  * @param  controlMode  enum that specify the Control Mode in terms of Torque,
  *                       Speed or Position only in Digital Mode
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetControlMode(SOLOMotorControllers::ControlMode controlMode)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetControlMode(controlMode, error);
}

/**
  * @brief  This command sets the value for Current Controller Kp or proportional gain
				.The method refers to the Object Dictionary: 0x3017
  * @param  currentControllerKp  a float value between 0 to 16000  
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the value for Current Controller Kp or proportional gain
				.The method refers to the Object Dictionary: 0x3017
  * @param  currentControllerKp  a float value between 0 to 16000   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetCurrentControllerKp(float currentControllerKp)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetCurrentControllerKp(currentControllerKp, error);
}

/**
  * @brief  This command sets the value for Current Controller Ki or integral gain
				.The method refers to the Object Dictionary: 0x3018
  * @param  motorInductance  a float value between 0 to 16000  
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the value for Current Controller Ki or integral gain
				.The method refers to the Object Dictionary: 0x3001
					.The method refers to the Object Dictionary: 0x3018
  * @param  motorInductance  a float value between 0 to 16000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetCurrentControllerKi(float currentControllerKi)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetCurrentControllerKi(currentControllerKi, error);
}

/**
  * @brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
  *         Weakening current reference to help the motor reaching speeds higher than
  *         nominal values and in case of AC Induction Motors Sets the desired magnetizing
  *         current (Id) required for controlling ACIM motors in FOC in Amps 
				.The method refers to the Object Dictionary: 0x301A
  * @param  magnetizingCurrentIdReference  a float value between 0 to 32   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
  *         Weakening current reference to help the motor reaching speeds higher than
  *         nominal values and in case of AC Induction Motors Sets the desired magnetizing
  *         current (Id) required for controlling ACIM motors in FOC in Amps 
				.The method refers to the Object Dictionary: 0x301A
  * @param  magnetizingCurrentIdReference  a float value between 0 to 32    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMagnetizingCurrentIdReference(magnetizingCurrentIdReference, error);
}

/**
  * @brief  This command sets the desired Position reference in terms of quadrature
  *         pulses while SOLO operates with the Incremental Encoders or in terms of
  *         pulses while while SOLO operates with Hall sensors
				.The method refers to the Object Dictionary: 0x301B
  * @param  positionReference  a long value between -2,147,483,647 to 2,147,483,647   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the desired Position reference in terms of quadrature
  *         pulses while SOLO operates with the Incremental Encoders or in terms of
  *         pulses while while SOLO operates with Hall sensors
				.The method refers to the Object Dictionary: 0x301B
  * @param  positionReference  a long value between -2,147,483,647 to 2,147,483,647    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetPositionReference(long positionReference)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetPositionReference(positionReference, error);
}

/**
  * @brief  This command sets the value for Position Controller Kp or proportional gain
				.The method refers to the Object Dictionary: 0x301C
  * @param  positionControllerKp  a float value between 0 to 16000  
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the value for Position Controller Kp or proportional gain
				.The method refers to the Object Dictionary: 0x301C
  * @param  positionControllerKp  a float value between 0 to 16000   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetPositionControllerKp(float positionControllerKp)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetPositionControllerKp(positionControllerKp, error);
}

/**
  * @brief  This command sets the value for Position Controller Ki or integrator gain
				.The method refers to the Object Dictionary: 0x301D
  * @param  positionControllerKi  a float value between 0 to 16000   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the value for Position Controller Ki or integrator gain
				.The method refers to the Object Dictionary: 0x301D
  * @param  positionControllerKi  a float value between 0 to 16000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetPositionControllerKi(float positionControllerKi)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetPositionControllerKi(positionControllerKi, error);
}

/**
  * @brief  This command resets the position counter back to zero    
				.The method refers to the Object Dictionary: 0x301F
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::ResetPositionToZero(int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x01};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    return _MCP2515->CANOpenTransmit(Address,Object_ResetPositionToZero,informatrionToSend,error);
}

/**
  * @brief  This command resets the position counter back to zero      
				.The method refers to the Object Dictionary: 0x3020
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::ResetPositionToZero()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::ResetPositionToZero(error);
}

/**
  * @brief  This command overwrites the reported errors in Error Register
  *         reported with command code of "0xA1"  
				.The method refers to the Object Dictionary: 0x3020
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::OverwriteErrorRegister(int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    return _MCP2515->CANOpenTransmit(Address,Object_OverwriteErrorRegister,informatrionToSend,error);
}

/**
  * @brief  This command overwrites the reported errors in Error Register
  *         reported with command code of "0xA1"   
				.The method refers to the Object Dictionary: 0x3020
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::OverwriteErrorRegister()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::OverwriteErrorRegister(error);
}

/**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed and angle of a BLDC or PMSM once the 
  *         motor type is selected as normal BLDC-PMSM
				.The method refers to the Object Dictionary: 0x3021
  * @param  observerGain  a float value between 0.01 to 1000  
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed and angle of a BLDC or PMSM once the 
  *         motor type is selected as normal BLDC-PMSM
				.The method refers to the Object Dictionary: 0x3021
  * @param  observerGain  a float value between 0.01 to 1000   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetObserverGainBldcPmsm(float observerGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetObserverGainBldcPmsm(observerGain, error);
}

/**
  * @brief  This command sets the observer gain for the Non-linear observer that
  *         estimates the speed and angle of a BLDC or PMSM once the motor type
  *         is selected as ultra-fast BLDC-PMSM
				.The method refers to the Object Dictionary: 0x3022
  * @param  observerGain  a float value between 0.01 to 1000   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the observer gain for the Non-linear observer that
  *         estimates the speed and angle of a BLDC or PMSM once the motor type
  *         is selected as ultra-fast BLDC-PMSM
				.The method refers to the Object Dictionary: 0x3022
  * @param  observerGain  a float value between 0.01 to 1000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetObserverGainBldcPmsmUltrafast(float observerGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetObserverGainBldcPmsmUltrafast(observerGain, error);
}

/**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed of a DC brushed once the motor type 
  *         is selected as DC brushed
				.The method refers to the Object Dictionary: 0x3023
  * @param  observerGain  a float value between 0.01 to 1000   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed of a DC brushed once the motor type 
  *         is selected as DC brushed
				.The method refers to the Object Dictionary: 0x3023
  * @param  observerGain  a float value between 0.01 to 1000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetObserverGainDc(float observerGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetObserverGainDc(observerGain, error);
}

/**
  * @brief  This command sets how fast the observer should operate once
  *         SOLO is in sensorless mode with normal BLDC-PMSM selected as the Motor type
				.The method refers to the Object Dictionary: 0x3024
  * @param  filterGain  a float value between 0.01 to 16000 
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets how fast the observer should operate once
  *         SOLO is in sensorless mode with normal BLDC-PMSM selected as the Motor type
				.The method refers to the Object Dictionary: 0x3024
  * @param  filterGain  a float value between 0.01 to 16000   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetFilterGainBldcPmsm(float filterGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetFilterGainBldcPmsm(filterGain, error);
}

/**
  * @brief  This command sets how fast the observer should operate once SOLO
  *         is in sensorless mode with ultra-fast BLDC-PMSM selected as the Motor type
				.The method refers to the Object Dictionary: 0x3025
  * @param  filterGain  a float value between 0.01 to 16000  
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets how fast the observer should operate once SOLO
  *         is in sensorless mode with ultra-fast BLDC-PMSM selected as the Motor type
				.The method refers to the Object Dictionary: 0x3025
  * @param  filterGain  a float value between 0.01 to 16000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetFilterGainBldcPmsmUltrafast(float filterGain)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetFilterGainBldcPmsmUltrafast(filterGain, error);
}

/**
  * @brief  This command sets the baud-rate of the UART line
				.The method refers to the Object Dictionary: 0x3026
  * @param  baudrate  enum that specify the baud-rate of the UART line 
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) baudrate, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_UartBaudrate,informatrionToSend,error);
}

/**
  * @brief  This command sets the baud-rate of the UART line
				.The method refers to the Object Dictionary: 0x3026
  * @param  baudrate  enum that specify the baud-rate of the UART line    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetUartBaudrate(baudrate, error);
}

/**
  * @brief  This command starts or stops the process of sensor calibration
				.The method refers to the Object Dictionary: 0x3027
  * @param  calibrationAction  enum that specify the process of sensor calibration 
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long)calibrationAction, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_SensorCalibration,informatrionToSend,error);
}

/**
  * @brief  This command starts or stops the process of sensor calibration
				.The method refers to the Object Dictionary: 0x3027
  * @param  calibrationAction  enum that specify the process of sensor calibration     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SensorCalibration(calibrationAction, error);
}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.C.W direction
				.The method refers to the Object Dictionary: 0x3028
  * @param  encoderHallOffset  a float value between 0.0 to 1.0  
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.C.W direction
				.The method refers to the Object Dictionary: 0x3028
  * @param  encoderHallOffset  a float value between 0.0 to 1.0    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetEncoderHallCcwOffset(float encoderHallOffset)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetEncoderHallCcwOffset(encoderHallOffset, error);
}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.W direction
				.The method refers to the Object Dictionary: 0x3029
  * @param  encoderHallOffset  a float value between 0.0 to 1.0   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.W direction
				.The method refers to the Object Dictionary: 0x3029
  * @param  encoderHallOffset  a float value between 0.0 to 1.0     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetEncoderHallCwOffset(float encoderHallOffset)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetEncoderHallCwOffset(encoderHallOffset, error);
}

/**
  * @brief  This command defines the acceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
				.The method refers to the Object Dictionary: 0x302A
  * @param  speedAccelerationValue  a float value between 0 to 1600  
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command defines the acceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
				.The method refers to the Object Dictionary: 0x302A
  * @param  speedAccelerationValue  a float value between 0 to 1600   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetSpeedAccelerationValue(float speedAccelerationValue)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetSpeedAccelerationValue(speedAccelerationValue, error);
}

/**
  * @brief  This command defines the deceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
				.The method refers to the Object Dictionary: 0x302B
  * @param  speedDecelerationValue  a float value between 0 to 1600   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
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

/**
  * @brief  This command defines the deceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
				.The method refers to the Object Dictionary: 0x302B
  * @param  speedDecelerationValue  a float value between 0 to 1600    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetSpeedDecelerationValue(float speedDecelerationValue) 
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetSpeedDecelerationValue(speedDecelerationValue, error);
}

/**
  * @brief  This command sets the baud rate of CAN bus in CANOpen network
				.The method refers to the Object Dictionary: 0x302C
  * @param  canbusBaudrate  enum that specify the baud rate of CAN bus in CANOpen network 
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate canbusBaudrate, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) canbusBaudrate, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_CanbusBaudrate,informatrionToSend,error);
}

/**
  * @brief  This command sets the baud rate of CAN bus in CANOpen network
				.The method refers to the Object Dictionary: 0x302C
  * @param  canbusBaudrate  enum that specify the baud rate of CAN bus in CANOpen network    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate canbusBaudrate)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetCanbusBaudrate(canbusBaudrate, error);
}

/**
  * @brief  This command defines the resolution of the speed at S/T input
  *           while SOLO operates in Analogue mode
  *           .The method refers to the Object Dictionary: 0x303E
  * @param  divisionCoefficient  a long value    
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	  if(!soloUtils->SetAnalogueSpeedResolutionDivisionCoefficientInputValidation(divisionCoefficient, error))
    {
        return false;
    }
    soloUtils->ConvertToData((long) divisionCoefficient, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_ASRDC,informatrionToSend,error);
}

/**
  * @brief  This command defines the resolution of the speed at S/T input
  *           while SOLO operates in Analogue mode
  *           .The method refers to the Object Dictionary: 0x303E
  * @param  divisionCoefficient  a long value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetAnalogueSpeedResolutionDivisionCoefficient(divisionCoefficient, error);
}

/**
  * @brief  This command defines the type of the Motion Profile that is 
  *           being used in Speed or Position Modes
  *           .The method refers to the Object Dictionary: 0x3040
  * @param  motionProfileMode enum that specify the type of the Motion Profile   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileMode( MotionProfileMode motionProfileMode, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) motionProfileMode, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_MotionProfileMode,informatrionToSend,error);
}

/**
  * @brief  This command defines the type of the Motion Profile that is 
  *           being used in Speed or Position Modes
  *           .The method refers to the Object Dictionary: 0x3040
  * @param  motionProfileMode enum that specify the type of the Motion Profile    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileMode( MotionProfileMode motionProfileMode)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotionProfileMode(motionProfileMode, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3041
  * @param  MotionProfileVariable1 a float value   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable1(float MotionProfileVariable1, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetMotionProfileVariable1InputValidation(MotionProfileVariable1, error))
    {
        return false;
    }
    soloUtils->ConvertToData((float) MotionProfileVariable1, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_MotionProfileVariable1,informatrionToSend,error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
				.The method refers to the Object Dictionary: 0x3041
  * @param  MotionProfileVariable1 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable1(float MotionProfileVariable1)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotionProfileVariable1(MotionProfileVariable1, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles  
  * @param  MotionProfileVariable2 a float value   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable2(float MotionProfileVariable2, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetMotionProfileVariable2InputValidation(MotionProfileVariable2, error))
    {
        return false;
    }
    soloUtils->ConvertToData((float) MotionProfileVariable2, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_MotionProfileVariable2,informatrionToSend,error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3022
  * @param  MotionProfileVariable2 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable2(float MotionProfileVariable2)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotionProfileVariable2(MotionProfileVariable2, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3043
  * @param  MotionProfileVariable3 a float value   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable3(float MotionProfileVariable3, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetMotionProfileVariable3InputValidation(MotionProfileVariable3, error))
    {
        return false;
    }
    soloUtils->ConvertToData((float) MotionProfileVariable3, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_MotionProfileVariable3,informatrionToSend,error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
				.The method refers to the Object Dictionary: 0x3043
  * @param  MotionProfileVariable3 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable3(float MotionProfileVariable3)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotionProfileVariable3(MotionProfileVariable3, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3044
  * @param  MotionProfileVariable4 a float value   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable4(float MotionProfileVariable4, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetMotionProfileVariable4InputValidation(MotionProfileVariable4, error))
    {
        return false;
    }
    soloUtils->ConvertToData((float) MotionProfileVariable4, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_MotionProfileVariable4,informatrionToSend,error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3044
  * @param  MotionProfileVariable4 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable4(float MotionProfileVariable4)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotionProfileVariable4(MotionProfileVariable4, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3045
  * @param  MotionProfileVariable5 a float value   
  * @param  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable5(float MotionProfileVariable5, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetMotionProfileVariable5InputValidation(MotionProfileVariable5, error))
    {
        return false;
    }
    soloUtils->ConvertToData((float) MotionProfileVariable5, informatrionToSend);
    return _MCP2515->CANOpenTransmit(Address,Object_MotionProfileVariable5,informatrionToSend,error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
				.The method refers to the Object Dictionary: 0x3045
  * @param  MotionProfileVariable5 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable5(float MotionProfileVariable5)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::SetMotionProfileVariable5(MotionProfileVariable5, error);
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

/**
  * @brief  This command reads the device address connected on the line  
				.The method refers to the Object Dictionary: 0x3001
  * @param  error   pointer to an integer that specify result of function  
  * @retval long device address connected on the line
  */
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

/**
  * @brief  This command reads the device address connected on the line  
				.The method refers to the Object Dictionary: 0x3001
  * @retval long device address connected on the line
  */
long SOLOMotorControllersCanopen::GetDeviceAddress()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetDeviceAddress(error);
}

/**
  * @brief  This command reads the phase-A voltage of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors 
				.The method refers to the Object Dictionary: 0x302D
  * @param  error   pointer to an integer that specify result of function  
  * @retval float phase-A voltage of the motor between -60 to 60
  */
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

/**
  * @brief  This command reads the phase-A voltage of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors  
				.The method refers to the Object Dictionary: 0x302D
  * @retval float phase-A voltage of the motor between -60 to 60
  */
float SOLOMotorControllersCanopen::GetPhaseAVoltage()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPhaseAVoltage(error);
}

/**
  * @brief  This command reads the phase-B voltage of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors 
				.The method refers to the Object Dictionary: 0x302E
  * @param  error   pointer to an integer that specify result of function  
  * @retval float 0 phase-A voltage of the motor between -60 to 60
  */
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

/**
  * @brief  This command reads the phase-B voltage of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors   
				.The method refers to the Object Dictionary: 0x302E
  * @retval float 0 phase-A voltage of the motor between -60 to 60
  */
float SOLOMotorControllersCanopen::GetPhaseBVoltage()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPhaseBVoltage(error);
}

/**
  * @brief  This command reads the phase-A current of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors
				.The method refers to the Object Dictionary: 0x302F
  * @param  error   pointer to an integer that specify result of function  
  * @retval phase-A current of the motor etween -32 to 32 Amps
  */
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

/**
  * @brief  This command reads the phase-A current of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors  
				.The method refers to the Object Dictionary: 0x302F
  * @retval phase-A current of the motor etween -32 to 32 Amps
  */
float SOLOMotorControllersCanopen::GetPhaseACurrent()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPhaseACurrent(error);
}

/**
  * @brief  This command reads the phase-B current of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors 
				.The method refers to the Object Dictionary: 0x3030
  * @param  error   pointer to an integer that specify result of function  
  * @retval float phase-B current of the motor etween -32 to 32 Amps
  */
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

/**
  * @brief  This command reads the phase-B current of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors  
				.The method refers to the Object Dictionary: 0x3030
  * @retval float phase-B current of the motor etween -32 to 32 Amps
  */
float SOLOMotorControllersCanopen::GetPhaseBCurrent()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPhaseBCurrent(error);
}

/**
  * @brief  This command reads the input BUS voltage  
				.The method refers to the Object Dictionary: 0x3031
  * @param  error   pointer to an integer that specify result of function  
  * @retval float  BUS voltage between 0 to 60
  */
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

/**
  * @brief  This command reads the input BUS voltage   
				.The method refers to the Object Dictionary: 0x3031
  * @retval float  BUS voltage between 0 to 60
  */
float SOLOMotorControllersCanopen::GetBusVoltage()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetBusVoltage(error);
}

/**
  * @brief  This command reads the current inside the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO 
				.The method refers to the Object Dictionary: 0x3032
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between -32 to 32
  */
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

/**
  * @brief  This command reads the current inside the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO  
				.The method refers to the Object Dictionary: 0x3032
  * @retval float between -32 to 32
  */
float SOLOMotorControllersCanopen::GetDcMotorCurrentIm()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetDcMotorCurrentIm(error);
}

/**
  * @brief  This command reads the voltage of the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO 
				.The method refers to the Object Dictionary: 0x3033
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between -60 to 60
  */
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

/**
  * @brief  This command reads the voltage of the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO 
				.The method refers to the Object Dictionary: 0x3033
  * @retval float between -60 to 60
  */
float SOLOMotorControllersCanopen::GetDcMotorVoltageVm()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetDcMotorVoltageVm(error); 
}

/**
  * @brief  This command reads the value of the Speed controller Kp gain, 
  *         set for Digital mode operations  
				.The method refers to the Object Dictionary: 0x300A
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
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

/**
  * @brief  This command reads the value of the Speed controller Kp gain, 
  *         set for Digital mode operations  
				.The method refers to the Object Dictionary: 0x300A
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopen::GetSpeedControllerKp()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetSpeedControllerKp(error);
}

/**
  * @brief  This command reads the value of the Speed controller Ki gain,
  *         set for Digital mode operations  
				.The method refers to the Object Dictionary: 0x300B
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
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

/**
  * @brief  This command reads the value of the Speed controller Ki gain,
  *         set for Digital mode operations   
				.The method refers to the Object Dictionary: 0x300B
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopen::GetSpeedControllerKi()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetSpeedControllerKi(error);
}

/**
  * @brief  This command reads the output switching frequency of SOLO in Hertz 
				.The method refers to the Object Dictionary: 0x3009
  * @param  error   pointer to an integer that specify result of function  
  * @retval long between 8000 to 80000 Hz
  */
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

/**
  * @brief  This command reads the output switching frequency of SOLO in Hertz 
				.The method refers to the Object Dictionary: 0x3009
  * @retval long between 8000 to 80000 Hz
  */
long SOLOMotorControllersCanopen::GetOutputPwmFrequencyKhz()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetOutputPwmFrequencyKhz(error);
}

/**
  * @brief  This command reads the value of the current limit set for SOLO in
  *         closed-loop digital operation mode 
				.The method refers to the Object Dictionary: 0x3003
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 32
  */
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

/**
  * @brief  This command reads the value of the current limit set for SOLO in
  *         closed-loop digital operation mode  
				.The method refers to the Object Dictionary: 0x3003
  * @retval float between 0 to 32
  */
float SOLOMotorControllersCanopen::GetCurrentLimit()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetCurrentLimit(error);
}

/**
  * @brief  This command reads the actual monetary value of “Iq” that is
  *         the current acts in torque generation in FOC mode for 3-phase motors
				.The method refers to the Object Dictionary: 0x3034
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between -64 to 64
  */
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

/**
  * @brief  This command reads the actual monetary value of “Iq” that is
  *         the current acts in torque generation in FOC mode for 3-phase motors 
				.The method refers to the Object Dictionary: 0x3034
  * @retval float between -64 to 64
  */
float SOLOMotorControllersCanopen::GetQuadratureCurrentIqFeedback()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetQuadratureCurrentIqFeedback(error);
}

/**
  * @brief  This command reads the actual monetary value of Id that is the
  *         direct current acting in FOC 
				.The method refers to the Object Dictionary: 0x3035
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between -64 to 64
  */
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

/**
  * @brief  This command reads the actual monetary value of Id that is the
  *         direct current acting in FOC  
				.The method refers to the Object Dictionary: 0x3035
  * @retval float between -64 to 64
  */
float SOLOMotorControllersCanopen::GetMagnetizingCurrentIdFeedback()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetMagnetizingCurrentIdFeedback(error);
}

/**
  * @brief  This command reads the number of Poles set for 3-phase motors  
				.The method refers to the Object Dictionary: 0x300F
  * @param  error   pointer to an integer that specify result of function  
  * @retval long between 1 to 254
  */
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

/**
  * @brief  This command reads the number of Poles set for 3-phase motors  
				.The method refers to the Object Dictionary: 0x300F
  * @retval long between 1 to 254
  */
long SOLOMotorControllersCanopen::GetMotorPolesCounts()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetMotorPolesCounts(error);
}

/**
  * @brief  This command reads the number of physical Incremental encoder lines set on SOLO 
				.The method refers to the Object Dictionary: 0x3010
  * @param  error   pointer to an integer that specify result of function  
  * @retval long between 1 to 200000
  */
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

/**
  * @brief  This command reads the number of physical Incremental encoder lines set on SOLO  
				.The method refers to the Object Dictionary: 0x3010
  * @retval long between 1 to 200000
  */
long SOLOMotorControllersCanopen::GetIncrementalEncoderLines()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetIncrementalEncoderLines(error);
}

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Kp or proportional gain 
				.The method refers to the Object Dictionary: 0x3017
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
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

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Kp or proportional gain  
				.The method refers to the Object Dictionary: 0x3017
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopen::GetCurrentControllerKp()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetCurrentControllerKp(error);
}

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Ki or integrator gain  
				.The method refers to the Object Dictionary: 0x3018
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
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

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Ki or integrator gain   
				.The method refers to the Object Dictionary: 0x3018
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopen::GetCurrentControllerKi()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetCurrentControllerKi(error);
}

/**
  * @brief  This command reads the momentary temperature of the board in centigrade 
				.The method refers to the Object Dictionary: 0x3039
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between -30 to 150 Celsius
  */
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

/**
  * @brief  This command reads the momentary temperature of the board in centigrade   
				.The method refers to the Object Dictionary: 0x3039
  * @retval float between -30 to 150 Celsius
  */
float SOLOMotorControllersCanopen::GetBoardTemperature()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetBoardTemperature(error);
}

/**
  * @brief  This command reads the Phase or Armature resistance of
  *         the 3-phase or DC brushed motor connected to SOLO respectively 
				.The method refers to the Object Dictionary: 0x300D
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 100
  */
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

/**
  * @brief  This command reads the Phase or Armature resistance of
  *         the 3-phase or DC brushed motor connected to SOLO respectively   
				.The method refers to the Object Dictionary: 0x300D
  * @retval float between 0 to 100
  */
float SOLOMotorControllersCanopen::GetMotorResistance()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetMotorResistance(error);
}

/**
  * @brief  This command reads the Phase or Armature Inductance of 
  *         the 3-phase or DC brushed motor connected to SOLO respectively  
				.The method refers to the Object Dictionary: 0x300E
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 0.2
  */
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

/**
  * @brief  This command reads the Phase or Armature Inductance of 
  *         the 3-phase or DC brushed motor connected to SOLO respectively  
				.The method refers to the Object Dictionary: 0x300E
  * @retval float between 0 to 0.2
  */
float SOLOMotorControllersCanopen::GetMotorInductance()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetMotorInductance(error);
}

/**
  * @brief  his command reads the actual speed of the motor measured or estimated by SOLO in
            sensorless or sensor-based modes respectively  
				.The method refers to the Object Dictionary: 0x3036
  * @param  error   pointer to an integer that specify result of function  
  * @retval long between -30000 to 30000 RPM
  */
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

/**
  * @brief  his command reads the actual speed of the motor measured or estimated by SOLO in
            sensorless or sensor-based modes respectively    
				.The method refers to the Object Dictionary: 0x3036
  * @retval long between -30000 to 30000 RPM
  */
long SOLOMotorControllersCanopen::GetSpeedFeedback()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetSpeedFeedback(error);
}

/**
  * @brief  This command reads the Motor type selected for Digital or Analogue mode operations 
				.The method refers to the Object Dictionary: 0x3015
  * @param  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 3
  */
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

/**
  * @brief  This command reads the Motor type selected for Digital or Analogue mode operations 
				.The method refers to the Object Dictionary: 0x3015
  * @retval long between 0 to 3
  */
long SOLOMotorControllersCanopen::GetMotorType()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetMotorType(error);
}

/**
  * @brief  This command reads the feedback control mode selected on SOLO both
  *         for Analogue and Digital operations  
				.The method refers to the Object Dictionary: 0x3013
  * @param  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 2
  */
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

/**
  * @brief  This command reads the feedback control mode selected on SOLO both
  *         for Analogue and Digital operations   
				.The method refers to the Object Dictionary: 0x3013
  * @retval long between 0 to 2
  */
long SOLOMotorControllersCanopen::GetFeedbackControlMode()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetFeedbackControlMode(error);
}

/**
  * @brief  This command reads the actual commanding mode that SOLO is operating 
				.The method refers to the Object Dictionary: 0x3002
  * @param  error   pointer to an integer that specify result of function  
  * @retval long between 0 or 1
  */
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

/**
  * @brief  This command reads the actual commanding mode that SOLO is operating  
				.The method refers to the Object Dictionary: 0x3002
  * @retval long between 0 or 1
  */
long SOLOMotorControllersCanopen::GetCommandMode()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetCommandMode(error);
}

/**
  * @brief  This command reads the Control Mode type in terms of Torque,
  *         Speed or Position in both Digital and Analogue modes 
				.The method refers to the Object Dictionary: 0x3013
  * @param  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 2
  */
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

/**
  * @brief  This command reads the Control Mode type in terms of Torque,
  *         Speed or Position in both Digital and Analogue modes  
				.The method refers to the Object Dictionary: 0x3013
  * @retval long between 0 to 2
  */
long SOLOMotorControllersCanopen::GetControlMode()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetControlMode(error);
}

/**
  * @brief  This command reads the value of the speed limit set on SOLO 
				.The method refers to the Object Dictionary: 0x3011
  * @param  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 30000
  */
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

/**
  * @brief  This command reads the value of the speed limit set on SOLO 
				.The method refers to the Object Dictionary: 0x3011
  * @retval long between 0 to 30000
  */
long SOLOMotorControllersCanopen::GetSpeedLimit()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetSpeedLimit(error);
}

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Kp or proportional gain  
				.The method refers to the Object Dictionary: 0x301C
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
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

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Kp or proportional gain  
				.The method refers to the Object Dictionary: 0x301C
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopen::GetPositionControllerKp()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPositionControllerKp(error); 
}

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Ki or integrator gain  
				.The method refers to the Object Dictionary: 0x301D
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
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

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Ki or integrator gain 
				.The method refers to the Object Dictionary: 0x301D
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopen::GetPositionControllerKi()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPositionControllerKi(error);
}

/**
  * @brief  This command reads the number of counted pulses from the
  *         Incremental Encoder or Hall sensors 
				.The method refers to the Object Dictionary: 0x3037
  * @param  error   pointer to an integer that specify result of function  
  * @retval long between -2,147,483,647 to 2,147,483,647
  */
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

/**
  * @brief  This command reads the number of counted pulses from the
  *         Incremental Encoder or Hall sensors  
				.The method refers to the Object Dictionary: 0x3037
  * @retval long between -2,147,483,647 to 2,147,483,647
  */
long SOLOMotorControllersCanopen::GetPositionCountsFeedback()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPositionCountsFeedback(error);
}

/**
  * @brief  This command reads the error register which is a 32 bit register with
  *         each bit corresponding to specific errors  
				.The method refers to the Object Dictionary: 0x3020
  * @param  error   pointer to an integer that specify result of function  
  * @retval long 
  */
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

/**
  * @brief  This command reads the error register which is a 32 bit register with
  *         each bit corresponding to specific errors  
				.The method refers to the Object Dictionary: 0x3020
  * @retval long 
  */
long SOLOMotorControllersCanopen::GetErrorRegister()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetErrorRegister(error);
}

/**
  * @brief  This command reads the Firmware version existing currently on the SOLO unit
				.The method refers to the Object Dictionary: 0x303A
  * @param  error   pointer to an integer that specify result of function  
  * @retval long
  */
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

/**
  * @brief  This command reads the Firmware version existing currently on the SOLO unit   
				.The method refers to the Object Dictionary: 0x303A
  * @retval long
  */
long SOLOMotorControllersCanopen::GetDeviceFirmwareVersion()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetDeviceFirmwareVersion(error);
}

/**
  * @brief  This command reads the Hardware version of the SOLO unit connected   
				.The method refers to the Object Dictionary: 0x303B
  * @param  error   pointer to an integer that specify result of function  
  * @retval long
  */
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

/**
  * @brief  This command reads the Hardware version of the SOLO unit connected 
				.The method refers to the Object Dictionary: 0x303B
  * @retval long
  */
long SOLOMotorControllersCanopen::GetDeviceHardwareVersion()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetDeviceHardwareVersion(error);
}

/**
  * @brief  This command reads the amount of desired Torque reference (Iq or IM)
  *         already set for the Motor to follow in Digital Closed-loop Torque control mode 
				.The method refers to the Object Dictionary: 0x3004
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 32
  */
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

/**
  * @brief  This command reads the amount of desired Torque reference (Iq or IM)
  *         already set for the Motor to follow in Digital Closed-loop Torque control mode   
				.The method refers to the Object Dictionary: 0x3004
  * @retval float between 0 to 32
  */
float SOLOMotorControllersCanopen::GetTorqueReferenceIq()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetTorqueReferenceIq(error);
}

/**
  * @brief  This command reads the amount of desired Speed reference already set for
  *         the Motor to follow in Digital Closed-loop Speed control mode  
				.The method refers to the Object Dictionary: 0x3005
  * @param  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 30000
  */
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

/**
  * @brief  This command reads the amount of desired Speed reference already set for
  *         the Motor to follow in Digital Closed-loop Speed control mode  
				.The method refers to the Object Dictionary: 0x3005
  * @retval long between 0 to 30000
  */
long SOLOMotorControllersCanopen::GetSpeedReference()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetSpeedReference(error);
}

/**
  * @brief  This command reads the amount of desired Id (direct current) or
  *         Magnetizing current reference already set for the Motor to follow
  *         in Digital Closed-loop Speed control mode for ACIM motors 
				.The method refers to the Object Dictionary: 0x301A
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 32
  */
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

/**
  * @brief  This command reads the amount of desired Id (direct current) or
  *         Magnetizing current reference already set for the Motor to follow
  *         in Digital Closed-loop Speed control mode for ACIM motors 
				.The method refers to the Object Dictionary: 0x301A
  * @retval float between 0 to 32
  */
float SOLOMotorControllersCanopen::GetMagnetizingCurrentIdReference()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetMagnetizingCurrentIdReference(error);
}

/**
  * @brief  This command reads the desired position reference set for the Motor
  *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses 
				.The method refers to the Object Dictionary: 0x301B
  * @param  error   pointer to an integer that specify result of function  
  * @retval long between -2,147,483,647 to 2,147,483,647 Quad-Pulses
  */
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

/**
  * @brief  This command reads the desired position reference set for the Motor
  *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses  
				.The method refers to the Object Dictionary: 0x301B
  * @retval long between -2,147,483,647 to 2,147,483,647 Quad-Pulses
  */
long SOLOMotorControllersCanopen::GetPositionReference()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPositionReference(error);
}

/**
  * @brief  This command reads the desired Power reference for SOLO to apply in 
  *         Digital Open-loop speed control mode for 3-phase motors in terms of percentage
				.The method refers to the Object Dictionary: 0x3006
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 100 %
  */
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

/**
  * @brief  This command reads the desired Power reference for SOLO to apply in 
  *         Digital Open-loop speed control mode for 3-phase motors in terms of percentage 
				.The method refers to the Object Dictionary: 0x3006
  * @retval float between 0 to 100 %
  */
float SOLOMotorControllersCanopen::GetPowerReference()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetPowerReference(error);
}

/**
  * @brief  This commands reads the desired direction of rotation set for the Motor
				.The method refers to the Object Dictionary: 0x300C
  * @param  error   pointer to an integer that specify result of function  
  * @retval long 0 Counter ClockWise / 1 ClockWise
  */
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

/**
  * @brief  This commands reads the desired direction of rotation set for the Motor 
				.The method refers to the Object Dictionary: 0x300C
  * @retval long 0 Counter ClockWise / 1 ClockWise
  */
long SOLOMotorControllersCanopen::GetMotorDirection()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetMotorDirection(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors  
				.The method refers to the Object Dictionary: 0x3021
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 1000
  */
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

/**
  * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors 
				.The method refers to the Object Dictionary: 0x3021
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersCanopen::GetObserverGainBldcPmsm()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetObserverGainBldcPmsm(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors 
				.The method refers to the Object Dictionary: 0x3022
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 1000
  */
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

/**
  * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors 
				.The method refers to the Object Dictionary: 0x3022
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersCanopen::GetObserverGainBldcPmsmUltrafast()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetObserverGainBldcPmsmUltrafast(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for DC Motor 
				.The method refers to the Object Dictionary: 0x3023
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 1000
  */
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

/**
  * @brief  This command reads the value of Sensorless Observer Gain for DC Motor 	
				.The method refers to the Object Dictionary: 0x3023
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersCanopen::GetObserverGainDc()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetObserverGainDc(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer
  *         Filter Gain for Normal BLDC-PMSM Motors  
				.The method refers to the Object Dictionary: 0x3024
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 16000
  */
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

/**
  * @brief  This command reads the value of Sensorless Observer
  *         Filter Gain for Normal BLDC-PMSM Motors   	
				.The method refers to the Object Dictionary: 0x3024
  * @retval float between 0.01 to 16000
  */
float SOLOMotorControllersCanopen::GetFilterGainBldcPmsm()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetFilterGainBldcPmsm(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer
  *         Filter Gain for Ultra Fast BLDC-PMSM Motors 
				.The method refers to the Object Dictionary: 0x3025
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 16000
  */
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

/**
  * @brief  This command reads the value of Sensorless Observer
  *         Filter Gain for Ultra Fast BLDC-PMSM Motors   
				.The method refers to the Object Dictionary: 0x3025
  * @retval float between 0.01 to 16000
  */
float SOLOMotorControllersCanopen::GetFilterGainBldcPmsmUltrafast()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetFilterGainBldcPmsmUltrafast(error);
}

/**
  * @brief  This command reads the measured or estimated per-unit angle of the 3-phase motors  
				.The method refers to the Object Dictionary: 0x3038
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between -2 to 2
  */
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

/**
  * @brief  This command reads the measured or estimated per-unit angle of the 3-phase motors  
				.The method refers to the Object Dictionary: 0x3038
  * @retval float between -2 to 2
  */
float SOLOMotorControllersCanopen::Get3PhaseMotorAngle()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::Get3PhaseMotorAngle(error);
}

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction
				.The method refers to the Object Dictionary: 0x3028
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 1.0
  */
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

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction 
				.The method refers to the Object Dictionary: 0x3028
  * @retval float between 0 to 1.0
  */
float SOLOMotorControllersCanopen::GetEncoderHallCcwOffset()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetEncoderHallCcwOffset(error);
}

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction
				.The method refers to the Object Dictionary: 0x3029
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 1.0
  */
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

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction   
				.The method refers to the Object Dictionary: 0x3029
  * @retval float between 0 to 1.0
  */
float SOLOMotorControllersCanopen::GetEncoderHallCwOffset()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetEncoderHallCwOffset(error);
}

/**
  * @brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line 
				.The method refers to the Object Dictionary: 0x3026
  * @param  error   pointer to an integer that specify result of function  
  * @retval long between 0 or 1
  */
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

/**
  * @brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line   
				.The method refers to the Object Dictionary: 0x3026
  * @retval long between 0 or 1
  */
long SOLOMotorControllersCanopen::GetUartBaudrate()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetUartBaudrate(error);
}

/**
  * @brief  This command reads the acceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds  
				.The method refers to the Object Dictionary: 0x302A
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 1600 Rev/S^2
  */
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

/**
  * @brief  This command reads the acceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds   
				.The method refers to the Object Dictionary: 0x302A
  * @retval float between 0 to 1600 Rev/S^2
  */
float SOLOMotorControllersCanopen::GetSpeedAccelerationValue()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetSpeedAccelerationValue(error);
}

/**
  * @brief  This command reads the deceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds 
				.The method refers to the Object Dictionary: 0x302B
  * @param  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 1600 Rev/S^2
  */
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

/**
  * @brief  This command reads the deceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds  
				.The method refers to the Object Dictionary: 0x302B
  * @retval float between 0 to 1600 Rev/S^2
  */
float SOLOMotorControllersCanopen::GetSpeedDecelerationValue()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetSpeedDecelerationValue(error);
}

/**
  * @brief  This command Reads the baud rate of CAN bus in CANOpen network
  *           .The method refers to the Object Dictionary: 0x302C
  * @param  error   pointer to an integer that specify result of function  
  * @retval long [kbits/s]
  */
long  SOLOMotorControllersCanopen::GetCanbusBaudrate(int &error)
{
	uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_CanbusBaudrate , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1.0 ;
}

/**
  * @brief  This command Reads the baud rate of CAN bus in CANOpen network
  *           .The method refers to the Object Dictionary: 0x302C
  * @retval long [kbits/s]
  */
long SOLOMotorControllersCanopen::GetCanbusBaudrate()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::GetCanbusBaudrate(error);
}

/**
  * @brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
				.The method refers to the Object Dictionary: 0x303E
  * @param  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersCanopen::GetAnalogueSpeedResolutionDivisionCoefficient(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_ASRDC , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToLong(informationReceived) ) ;
    }
    return -1 ;
}

/**
  * @brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
				.The method refers to the Object Dictionary: 0x303E
  * @retval long
  */
long SOLOMotorControllersCanopen::GetAnalogueSpeedResolutionDivisionCoefficient()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::GetAnalogueSpeedResolutionDivisionCoefficient(error);
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

/**
  * @brief  This Command reads the number of counted index pulses 
  *         seen on the Incremental Encoder’s output  
				.The method refers to the Object Dictionary: 0x303D
  * @param  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 2,147,483,647
  */
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

/**
  * @brief  This Command reads the number of counted index pulses 
  *         seen on the Incremental Encoder’s output 
				.The method refers to the Object Dictionary: 0x303D
  * @retval long between 0 to 2,147,483,647
  */
long SOLOMotorControllersCanopen::GetEncoderIndexCounts()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return  SOLOMotorControllersCanopen::GetEncoderIndexCounts(error);
}

/**
  * @brief  This command reads the type of the Embedded Motion profile active in the controller 
				.The method refers to the Object Dictionary: 0x303F
  * @param  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersCanopen::GetMotionProfileMode(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_MotionProfileMode , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1 ;
}

/**
  * @brief  This command reads the type of the Embedded Motion profile active in the controller 
				.The method refers to the Object Dictionary: 0x303F
  * @retval long
  */
long SOLOMotorControllersCanopen::GetMotionProfileMode()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::GetMotionProfileMode(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable1 set inside the controller 
				.The method refers to the Object Dictionary: 0x3040
  * @param  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable1(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_MotionProfileVariable1 , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1 ;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable1 set inside the controller  
				.The method refers to the Object Dictionary: 0x3040
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable1()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::GetMotionProfileVariable1(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable2 set inside the controller 
				.The method refers to the Object Dictionary: 0x3041
  * @param  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable2(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_MotionProfileVariable2 , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1 ;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable2 set inside the controller 
				.The method refers to the Object Dictionary: 0x3041
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable2()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::GetMotionProfileVariable2(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable3 set inside the controller 
				.The method refers to the Object Dictionary: 0x3042
  * @param  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable3(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_MotionProfileVariable3 , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1 ;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable3 set inside the controller 
				.The method refers to the Object Dictionary: 0x3042
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable3()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::GetMotionProfileVariable3(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable4 set inside the controller
				.The method refers to the Object Dictionary: 0x3043
  * @param  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable4(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_MotionProfileVariable4 , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1 ;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable4 set inside the controller 
				.The method refers to the Object Dictionary: 0x3043
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable4()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::GetMotionProfileVariable4(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable5 set inside the controller 
				.The method refers to the Object Dictionary: 0x3044
  * @param  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable5(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(_MCP2515->CANOpenReceive(Address, Object_MotionProfileVariable5 , informationToSend, informationReceived, error)){
        return ( soloUtils->ConvertToFloat(informationReceived) ) ;
    }
    return -1 ; 
}

/**
  * @brief  This command reads the value of the Motion Profile Variable5 set inside the controller
				.The method refers to the Object Dictionary: 0x3044
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable5()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersCanopen::GetMotionProfileVariable5(error);
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
void SOLOMotorControllersCanopen::Generic_Canbus_Read_MCP2515(uint16_t *_ID , uint8_t *_DLC, uint8_t *_Data)
{
    *_ID = 0;
    *_DLC = 0;
    if((_MCP2515->MCP2515_Read_RX_Status() & (1<<6))){
        _MCP2515->MCP2515_Receive_Frame(_MCP2515->MCP2515_RX_BUF::RX_BUFFER_0,_ID,_DLC,_Data);
    }
}
void SOLOMotorControllersCanopen::Generic_Canbus_Write_MCP2515(uint16_t _ID, uint8_t *_DLC, uint8_t *_Data, int &error)
{
    _MCP2515->MCP2515_Transmit_Frame(_MCP2515->MCP2515_TX_BUF::TX_BUFFER_0, _ID, _DLC, _Data, error);
}