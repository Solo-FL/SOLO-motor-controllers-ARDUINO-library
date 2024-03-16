/**
 *******************************************************************************
 * @file    SOLOMotorControllersCanopen.cpp
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the functions for the Solo Drivers
 *          CANopen communications.
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.2.0
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)
 *******************************************************************************
 */

#include "SOLOMotorControllersCanopen.h"
#include <stdint.h>

extern MCP2515 *_MCP2515;
int SOLOMotorControllersCanopen::lastError=0;

SOLOMotorControllersCanopen::SOLOMotorControllersCanopen(unsigned char _deviceAddress, unsigned char _chipSelectPin, SOLOMotorControllers::CanbusBaudrate _baudrate,unsigned char _interruptPin, SOLOMotorControllers::Frequency _frequency , long _millisecondsTimeout)
{
  millisecondsTimeout = _millisecondsTimeout;

  if (_deviceAddress == 0) // Address 0 is reserved for the host
  {
    _deviceAddress = 1;
  }
  Address = _deviceAddress;


  _MCP2515 = new MCP2515(_chipSelectPin, _baudrate, _interruptPin, _frequency, _millisecondsTimeout);
  _MCP2515->MCP2515_Init();
  InitPdoConfig();
  soloUtils = new SOLOMotorControllersUtils();
  delay(1000);
}

bool SOLOMotorControllersCanopen::SetGuardTime(long guardtime, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetGuardTimeInputValidation(guardtime, error))
  {
    return false;
  }
  soloUtils->ConvertToData(guardtime, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_GuardTime, 0x00, informationToSend, informatrionToRead, error);
}

bool SOLOMotorControllersCanopen::SetLifeTimeFactor(long lifeTimeFactor, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetLifeTimeFactorInputValidation(lifeTimeFactor, error))
  {
    return false;
  }
  soloUtils->ConvertToData(lifeTimeFactor, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_LifeTimeFactor, 0x00, informationToSend, informatrionToRead, error);
}

bool SOLOMotorControllersCanopen::SetProducerHeartbeatTime(long producerHeartbeatTime, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetProducerHeartbeatTimeInputValidation(producerHeartbeatTime, error))
  {
    return false;
  }
  soloUtils->ConvertToData(producerHeartbeatTime, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_ProducerHeartbeatTime, 0x00, informationToSend, informatrionToRead, error);
}

/**
 * @brief  This command determine the validity of count of SYNC message
 * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to set CobId value
 * @param[in]  parameterCobbId	CobId value
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetPdoParameterCobbIdInputValidation(PdoParameterName parameterName, int parameterCobbId, int &error)
{
  if (parameterName < PdoParameterName::POSITION_COUNTS_FEEDBACK)
  {
    if (parameterCobbId >= RPDO_MIN_COBIB && parameterCobbId <= RPDO_MAX_COBIB)
    {
      return true;
    }
  }
  else
  {
    if (parameterCobbId >= TPDO_MIN_COBIB && parameterCobbId <= TPDO_MAX_COBIB)
    {
      return true;
    }
  }
  error = SOLOMotorControllers::Error::PDO_PARAMETER_ID_OUT_OF_RANGE;
  return false;
}

/**
 * @brief  This command determine the validity of count of SYNC message
 * @param[in]  parameterCount	count of SYNC message
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetSyncParameterCountInputValidation(uint8_t parameterCount, int &error)
{
  if ((parameterCount >= 0 && parameterCount < 12) || parameterCount == 0xFF)
    return true;
  error = SOLOMotorControllers::Error::PDO_SYNC_OUT_OF_RANGE;
  return 0;
}

/**
 * @brief  This command set PDO configs for the intended PDO object
 * @param[in]  config	enum that specifies PDO parameter configs for the intended PDO object
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetPdoParameterConfig(PdoParameterConfig config, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!SetPdoParameterCobbIdInputValidation(config.parameterName, config.parameterCobId, error))
  {
    error = SOLOMotorControllers::Error::PDO_PARAMETER_ID_OUT_OF_RANGE;
    return false;
  }
  if (!SetSyncParameterCountInputValidation(config.syncParameterCount, error))
  {
    error = SOLOMotorControllers::Error::PDO_SYNC_OUT_OF_RANGE;
    return false;
  }

  informationToSend[0] = (config.isPdoParameterEnable << 7) | (config.isRrtParameterEnable << 6);
  informationToSend[1] = 0;
  informationToSend[2] = config.parameterCobId >> 8;
  informationToSend[3] = config.parameterCobId % 256;
  bool isSuccess = _MCP2515->CANOpenSdoTransmit(Address, true, pdoParameterObjectByPdoParameterName[config.parameterName], 0x01, informationToSend, informatrionToRead, error);
  if (isSuccess)
  {
    pdoParameterCobIdByPdoParameterName[config.parameterName] = config.parameterCobId;

    informationToSend[0] = 0;
    informationToSend[1] = 0;
    informationToSend[2] = 0;
    informationToSend[3] = config.syncParameterCount;
    if (config.syncParameterCount == 0 && pdoParameterObjectByPdoParameterName[config.parameterName] < pdoParameterObjectByPdoParameterName[PdoParameterName::POSITION_COUNTS_FEEDBACK])
    {
      informationToSend[3] = 0xFF;
    }

    isSuccess = _MCP2515->CANOpenSdoTransmit(Address, true, pdoParameterObjectByPdoParameterName[config.parameterName], 0x02, informationToSend, informatrionToRead, error);

    return isSuccess;
  }
  return false;
}

/**
 * @brief  This command gets PDO configs for the intended PDO object
 * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to get its config
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval PdoParameterConfig enum @ref PdoParameterConfig
 */
SOLOMotorControllersCanopen::PdoParameterConfig SOLOMotorControllersCanopen::GetPdoParameterConfig(SOLOMotorControllersCanopen::PdoParameterName parameterName, int &error)
{
  PdoParameterConfig config;
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  if (_MCP2515->CANOpenSdoTransmit(Address, true, parameterName, 0x1, informationToSend, informationReceived, error))
  {
    config.parameterName = parameterName;
    config.parameterCobId = (informationReceived[2] << 8) + informationReceived[3];
    config.isPdoParameterEnable = informationReceived[0] & 0x80;
    config.isRrtParameterEnable = informationReceived[0] & 0x40;

    if (_MCP2515->CANOpenSdoTransmit(Address, true, parameterName, 0x2, informationToSend, informationReceived, error))
    {
      config.syncParameterCount = informationReceived[3];
      return config;
    }
    else
    {
      config.parameterCobId = 0;
      config.isPdoParameterEnable = 0;
      config.isRrtParameterEnable = 0;
      config.syncParameterCount = 0;
      error = SOLOMotorControllers::Error::GENERAL_ERROR;
      return config;
    }
  }
  error = SOLOMotorControllers::Error::GENERAL_ERROR;
  return config;
}

/**
 * @brief  This command send a SYNC message on bus
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SendPdoSync(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return _MCP2515->SendPdoSync(error);
}

/**
 * @brief  This command send a RTR for the intended PDO object
 * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to send RTR
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SendPdoRtr(PdoParameterName parameterName, int &error)
{
  SOLOMotorControllersCanopen::PdoRtrValidParameter(parameterName, error);
  if (error != SOLOMotorControllers::Error::NO_ERROR_DETECTED)
  {
    return false;
  }

  int adr = GetPdoParameterCobId(parameterName, error);
  if (error != SOLOMotorControllers::Error::NO_ERROR_DETECTED)
  {
    return false;
  }

  return _MCP2515->SendPdoRtr(adr, error);
}

/**
 * @brief  This command checks the validity of allowing RTR  for the intended PDO parameter
 * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to check RTR validity
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::PdoRtrValidParameter(PdoParameterName parameterName, int &error)
{
  if (parameterName >= PdoParameterName::POSITION_COUNTS_FEEDBACK)
  {
    error = SOLOMotorControllers::Error::NO_ERROR_DETECTED;
    return true;
  }
  error = SOLOMotorControllers::Error::PDO_RTR_COMMAND_NOT_ALLOWED;
  return false;
}

/**
 * @brief  This command returns the CobId value for the intended PDO parameter name
 * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to get parameter CobId
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval long
 */
long SOLOMotorControllersCanopen::GetPdoParameterCobId(PdoParameterName parameterName, int &error)
{
  int pdoParameterCobId = pdoParameterCobIdByPdoParameterName[parameterName];
  if (pdoParameterCobId == 0)
  {
    error = SOLOMotorControllers::Error::PDO_MISSING_COB_ID;
  }
  return pdoParameterCobId;
}

/**
 * @brief  This command initializes all PDO parameter names addresses in @ref pdoParameterObjectByPdoParameterName array
 * @retval void
 */
void SOLOMotorControllersCanopen::InitPdoConfig()
{
  for (int i = 0; i < PDO_PARAMETER_NAME_COUNT; i++)
  {
    pdoParameterCobIdByPdoParameterName[i] = 0;
  }

  pdoParameterObjectByPdoParameterName[PdoParameterName::POSITION_REFERENCE] = 0x1414;
  pdoParameterObjectByPdoParameterName[PdoParameterName::SPEED_REFERENCE] = 0x1415;
  pdoParameterObjectByPdoParameterName[PdoParameterName::TORQUE_REFERENCE_IQ] = 0x1416;
  pdoParameterObjectByPdoParameterName[PdoParameterName::MAGNETIZING_CURRENT_ID_REFERENCE] = 0x1417;
  pdoParameterObjectByPdoParameterName[PdoParameterName::CONTROL_MODE] = 0x1418;
  pdoParameterObjectByPdoParameterName[PdoParameterName::MOTOR_DIRECTION] = 0x1419;
  pdoParameterObjectByPdoParameterName[PdoParameterName::POSITION_COUNTS_FEEDBACK] = 0x1814;
  pdoParameterObjectByPdoParameterName[PdoParameterName::SPEED_FEEDBACK] = 0x1815;
  pdoParameterObjectByPdoParameterName[PdoParameterName::QUADRATURE_CURRENT_IQ_FEEDBACK] = 0x1816;
  pdoParameterObjectByPdoParameterName[PdoParameterName::MAGNETIZING_CURRENT_ID_FEEDBACK] = 0x1817;
  pdoParameterObjectByPdoParameterName[PdoParameterName::ERROR_REGISTER] = 0x1818;
  pdoParameterObjectByPdoParameterName[PdoParameterName::BOARD_TEMPERATURE] = 0x1819;
}

/**
 * @brief  This command update all CobId values for PDO parameters
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::UpdatePdoParameterCobIdByPdoParameterName()
{
  int error;
  for (int i = 0; i < PDO_PARAMETER_NAME_COUNT; i++)
  {
    pdoParameterCobIdByPdoParameterName[i] = GetPdoParameterConfig((PdoParameterName)pdoParameterObjectByPdoParameterName[i], error).parameterCobId;
  }
  return true;
}

/**
 * @brief  This command set the intended long value for a PDO command
 * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to write its value
 * @param[in]  value	the value that wants to be set for the PDO parameter
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetPdoParameterValue(PdoParameterName parameterName, long value,
                                                       int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  soloUtils->ConvertToData(value, informationToSend);

  int adr = GetPdoParameterCobId(parameterName, error);
  if (error == SOLOMotorControllers::Error::PDO_MISSING_COB_ID)
  {
    return false;
  }

  return _MCP2515->PDOTransmit(adr, informationToSend, error);
}

/**
 * @brief  This command set the intended float value for a PDO command
 * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to write its value
 * @param[in]  value	float value that wants to be set for the PDO parameter
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetPdoParameterValue(PdoParameterName parameterName, float value,
                                                       int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  soloUtils->ConvertToData(value, informationToSend);

  int adr = GetPdoParameterCobId(parameterName, error);
  if (error == SOLOMotorControllers::Error::PDO_MISSING_COB_ID)
  {
    return false;
  }

  return _MCP2515->PDOTransmit(adr, informationToSend, error);
}

/**
 * @brief  This command returns the long value of a PDO command
 * @param[in]  parameterName	enum that specifies the name of the parameter that wants to read its value
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval long
 */
long SOLOMotorControllersCanopen::GetPdoParameterValueLong(PdoParameterName parameterName,
                                                           int &error)
{
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_ERROR_DETECTED;

  int adr = GetPdoParameterCobId(parameterName, error);
  if (error != SOLOMotorControllers::Error::NO_ERROR_DETECTED)
  {
    return -1;
  }

  if (_MCP2515->PDOReceive(adr, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
 * @brief  This command returns the float value of a PDO command
 * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to read its value
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval float
 */
float SOLOMotorControllersCanopen::GetPdoParameterValueFloat(PdoParameterName parameterName,
                                                             int &error)
{
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_ERROR_DETECTED;

  int adr = GetPdoParameterCobId(parameterName, error);
  if (error != SOLOMotorControllers::Error::NO_ERROR_DETECTED)
  {
    return -1;
  }

  if (_MCP2515->PDOReceive(adr, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1;
}

/**
 * @brief  This command sets the desired device address for a SOLO unit
 *				.The method refers to the Object Dictionary: 0x3001
 * @param[in] deviceAddress  address want to set for board
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetDeviceAddress(unsigned char deviceAddress, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetDeviceAddressInputValidation(deviceAddress, error))
  {
    return false;
  }
  soloUtils->ConvertToData((long)deviceAddress, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_SetDeviceAddress, 0x00, informationToSend, informatrionToRead, error);
}

/**
 * @brief  This command sets the mode of the operation of SOLO
 *         in terms of operating in Analogue mode or Digital
 *				.The method refers to the Object Dictionary: 0x3002
 * @param[in] mode  enum that specify mode of the operation of SOLO
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetCommandMode(SOLOMotorControllers::CommandMode mode, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)mode, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_CommandMode, 0x00, informationToSend, informatrionToRead, error);
}

/**
 * @brief  This command defines the maximum allowed current into the motor in terms of Amps
 *				.The method refers to the Object Dictionary: 0x3003
 * @param[in] currentLimit  a float value between 0 to 32
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetCurrentLimit(float currentLimit, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetCurrentLimitInputValidation(currentLimit, error))
  {
    return false;
  }
  soloUtils->ConvertToData(currentLimit, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_CurrentLimit, 0x00, informationToSend, informatrionToRead, error);
}

/**
 * @brief  This command sets the amount of desired current that acts in torque generation
 *				.The method refers to the Object Dictionary: 0x3004
 * @param[in] torqueReferenceIq  a float value between 0 to 32
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetTorqueReferenceIq(float torqueReferenceIq, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetTorqueReferenceIqInputValidation(torqueReferenceIq, error))
  {
    return false;
  }
  soloUtils->ConvertToData(torqueReferenceIq, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_TorqueReferenceIq, 0x00, informationToSend, informatrionToRead, error);
}

/**
 * @brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
 *				.The method refers to the Object Dictionary: 0x3005
 * @param[in] speedReference  a long value between 0 to 30000
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetSpeedReference(long speedReference, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedReferenceInputValidation(speedReference, error))
  {
    return false;
  }
  soloUtils->ConvertToData(speedReference, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_SpeedReference, 0x00, informationToSend, informatrionToRead, error);
}


/**
 * @brief  This command defines the amount of power percentage during only
 *         Open-loop mode for 3-phase motors
 *				.The method refers to the Object Dictionary: 0x3006
 * @param[in] powerReference  a float value between 0 to 100
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetPowerReference(float powerReference, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPowerReferenceInputValidation(powerReference, error))
  {
    return false;
  }
  soloUtils->ConvertToData(powerReference, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_PowerReference, 0x00, informationToSend, informatrionToRead, error);
}

/**
 * @brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
 *          identifying the electrical parameters of the Motor connected
 *				.The method refers to the Object Dictionary: 0x3007
 * @param[in] powerReference  enum that specify Start or Stop of something in SOLO
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::MotorParametersIdentification(SOLOMotorControllers::Action identification, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)identification, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_MotorParametersIdentification, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command if the DATA is set at zero will stop the whole power and switching system
            connected to the motor and it will cut the current floating into the Motor from SOLO
        .The method refers to the Object Dictionary: 0x3008
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::EmergencyStop(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_EmergencyStop, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the output switching frequency of the whole power unit on the Motor
        .The method refers to the Object Dictionary: 0x3009
  * @param[in] outputPwmFrequencyKhz  switching frequencies in kHz. a long value between 8 to 80
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetOutputPwmFrequencyKhzInputValidation(outputPwmFrequencyKhz, error))
  {
    return false;
  }
  soloUtils->ConvertToData(outputPwmFrequencyKhz, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_OutputPwmFrequencyKhz, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the Speed controller Kp Gain, and it will
  *         be functional only in Digital Closed-loop mode
        .The method refers to the Object Dictionary: 0x300A
  * @param[in] speedControllerKp  a float value between 0 to 300
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetSpeedControllerKp(float speedControllerKp, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedControllerKpInputValidation(speedControllerKp, error))
  {
    return false;
  }
  soloUtils->ConvertToData(speedControllerKp, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_SpeedControllerKp, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the Speed controller Ki gain, and it will
  *         be functional only in Digital Closed-loop mode
        .The method refers to the Object Dictionary: 0x300B
  * @param[in] speedControllerKi  a float value between 0 to 300
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetSpeedControllerKi(float speedControllerKi, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedControllerKiInputValidation(speedControllerKi, error))
  {
    return false;
  }
  soloUtils->ConvertToData(speedControllerKi, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_SpeedControllerKi, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This commands sets the direction of the rotation of the motor
  *         either to ClockWise rotation or to Counter Clockwise Rotation
        .The method refers to the Object Dictionary: 0x300C
  * @param[in] motorDirection  enum that specify the direction of the rotation of the motor
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)motorDirection, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_MotorDirection, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the amount of the Phase or Armature resistance
  *         for 3-phase or DC Brushed motors respectively
        .The method refers to the Object Dictionary: 0x300D
  * @param[in] motorResistance  a float value between 0.001 to 50
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotorResistance(float motorResistance, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotorResistanceInputValidation(motorResistance, error))
  {
    return false;
  }
  soloUtils->ConvertToData(motorResistance, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_MotorResistance, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the amount of the Phase or Armature Inductance
  *         for 3-phase or DC Brushed motors respectively
        .The method refers to the Object Dictionary: 0x300E
  * @param[in] motorInductance  a float value between 0.00005 to 0.2
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotorInductance(float motorInductance, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotorInductanceInputValidation(motorInductance, error))
  {
    return false;
  }
  soloUtils->ConvertToData(motorInductance, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_MotorInductance, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
        .The method refers to the Object Dictionary: 0x300F
  * @param[in] motorPolesCounts  a long value between 1 to 254
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotorPolesCounts(long motorPolesCounts, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotorPolesCountsInputValidation(motorPolesCounts, error))
  {
    return false;
  }

  soloUtils->ConvertToData(motorPolesCounts, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_MotorPolesCounts, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the pre-quad number of physical lines of an
  *         incremental encoder engraved on its disk
        .The method refers to the Object Dictionary: 0x3010
  * @param[in] incrementalEncoderLines  a long value between 1 to 200000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetIncrementalEncoderLines(long incrementalEncoderLines, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetIncrementalEncoderLinesInputValidation(incrementalEncoderLines, error))
  {
    return false;
  }
  soloUtils->ConvertToData(incrementalEncoderLines, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_IncrementalEncoderLines, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the allowed speed during trajectory following
  *         in closed-loop position controlling mode
        .The method refers to the Object Dictionary: 0x3011
  * @param[in] speedLimit  a long value between 0 to 30000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetSpeedLimit(long speedLimit, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedLimitInputValidation(speedLimit, error))
  {
    return false;
  }
  soloUtils->ConvertToData(speedLimit, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_SpeedLimit, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the type of the feedback control SOLO has to operate
        .The method refers to the Object Dictionary: 0x3013
  * @param[in] mode  enum that specify the type of the feedback control SOLO
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)mode, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_FeedbackControlMode, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command resets SOLO to its factory setting to all the default parameters
        .The method refers to the Object Dictionary: 0x3014
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::ResetFactory(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x01};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_ResetFactory, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the Motor type that is connected to SOLO in Digital Mode
        .The method refers to the Object Dictionary: 0x3015
  * @param[in] motorType  enum that specify the Motor type that is connected to SOLO in Digital Mode
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotorType(SOLOMotorControllers::MotorType motorType, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)motorType, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_MotorType, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the Control Mode in terms of Torque,
  *         Speed or Position only in Digital Mode
        .The method refers to the Object Dictionary: 0x3016
  * @param[in] controlMode  enum that specify the Control Mode in terms of Torque,
  *                       Speed or Position only in Digital Mode
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetControlMode(SOLOMotorControllers::ControlMode controlMode, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)controlMode, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_ControlMode, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the value for Current Controller Kp or proportional gain
        .The method refers to the Object Dictionary: 0x3017
  * @param[in] currentControllerKp  a float value between 0 to 16000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetCurrentControllerKp(float currentControllerKp, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetCurrentControllerKpInputValidation(currentControllerKp, error))
  {
    return false;
  }
  soloUtils->ConvertToData(currentControllerKp, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_CurrentControllerKp, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the value for Current Controller Ki or integral gain
        .The method refers to the Object Dictionary: 0x3018
  * @param[in] motorInductance  a float value between 0 to 16000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetCurrentControllerKi(float currentControllerKi, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetCurrentControllerKiInputValidation(currentControllerKi, error))
  {
    return false;
  }
  soloUtils->ConvertToData(currentControllerKi, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_CurrentControllerKi, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
  *         Weakening current reference to help the motor reaching speeds higher than
  *         nominal values and in case of AC Induction Motors Sets the desired magnetizing
  *         current (Id) required for controlling ACIM motors in FOC in Amps
        .The method refers to the Object Dictionary: 0x301A
  * @param[in] magnetizingCurrentIdReference  a float value between 0 to 32
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMagnetizingCurrentIdReferenceInputValidation(magnetizingCurrentIdReference, error))
  {
    return false;
  }
  soloUtils->ConvertToData(magnetizingCurrentIdReference, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_MagnetizingCurrentIdReference, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the desired Position reference in terms of quadrature
  *         pulses while SOLO operates with the Incremental Encoders or in terms of
  *         pulses while while SOLO operates with Hall sensors
        .The method refers to the Object Dictionary: 0x301B
  * @param[in] positionReference  a long value between -2,147,483,647 to 2,147,483,647
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetPositionReference(long positionReference, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPositionReferenceInputValidation(positionReference, error))
  {
    return false;
  }
  soloUtils->ConvertToData(positionReference, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_PositionReference, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the value for Position Controller Kp or proportional gain
        .The method refers to the Object Dictionary: 0x301C
  * @param[in] positionControllerKp  a float value between 0 to 16000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetPositionControllerKp(float positionControllerKp, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPositionControllerKpInputValidation(positionControllerKp, error))
  {
    return false;
  }
  soloUtils->ConvertToData(positionControllerKp, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_PositionControllerKp, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the value for Position Controller Ki or integrator gain
        .The method refers to the Object Dictionary: 0x301D
  * @param[in] positionControllerKi  a float value between 0 to 16000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetPositionControllerKi(float positionControllerKi, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPositionControllerKiInputValidation(positionControllerKi, error))
  {
    return false;
  }
  soloUtils->ConvertToData(positionControllerKi, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_PositionControllerKi, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command overwrites the reported errors in Error Register
  *         reported with command code of "0xA1"
        .The method refers to the Object Dictionary: 0x3020
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::OverwriteErrorRegister(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_OverwriteErrorRegister, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed and angle of a BLDC or PMSM once the
  *         motor type is selected as normal BLDC-PMSM
        .The method refers to the Object Dictionary: 0x3021
  * @param[in] observerGain  a float value between 0.01 to 1000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetObserverGainBldcPmsm(float observerGain, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetObserverGainBldcPmsmInputValidation(observerGain, error))
  {
    return false;
  }
  soloUtils->ConvertToData(observerGain, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_ObserverGainBldcPmsm, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the observer gain for the Non-linear observer that
  *         estimates the speed and angle of a BLDC or PMSM once the motor type
  *         is selected as ultra-fast BLDC-PMSM
        .The method refers to the Object Dictionary: 0x3022
  * @param[in] observerGain  a float value between 0.01 to 1000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetObserverGainBldcPmsmUltrafast(float observerGain, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetObserverGainBldcPmsmUltrafastInputValidation(observerGain, error))
  {
    return false;
  }
  soloUtils->ConvertToData(observerGain, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_ObserverGainBldcPmsmUltrafast, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed of a DC brushed once the motor type
  *         is selected as DC brushed
        .The method refers to the Object Dictionary: 0x3023
  * @param[in] observerGain  a float value between 0.01 to 1000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetObserverGainDc(float observerGain, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetObserverGainDcInputValidation(observerGain, error))
  {
    return false;
  }
  soloUtils->ConvertToData(observerGain, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_ObserverGainDc, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets how fast the observer should operate once
  *         SOLO is in sensorless mode with normal BLDC-PMSM selected as the Motor type
        .The method refers to the Object Dictionary: 0x3024
  * @param[in] filterGain  a float value between 0.01 to 16000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetFilterGainBldcPmsm(float filterGain, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetFilterGainBldcPmsmInputValidation(filterGain, error))
  {
    return false;
  }
  soloUtils->ConvertToData(filterGain, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_FilterGainBldcPmsm, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets how fast the observer should operate once SOLO
  *         is in sensorless mode with ultra-fast BLDC-PMSM selected as the Motor type
        .The method refers to the Object Dictionary: 0x3025
  * @param[in] filterGain  a float value between 0.01 to 16000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetFilterGainBldcPmsmUltrafast(float filterGain, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetFilterGainBldcPmsmUltrafastInputValidation(filterGain, error))
  {
    return false;
  }
  soloUtils->ConvertToData(filterGain, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_FilterGainBldcPmsmUltrafast, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the baud-rate of the UART line
        .The method refers to the Object Dictionary: 0x3026
  * @param[in] baudrate  enum that specify the baud-rate of the UART line
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)baudrate, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_UartBaudrate, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command starts or stops the process of sensor calibration
        .The method refers to the Object Dictionary: 0x3027
  * @param[in] calibrationAction  enum that specify the process of sensor calibration
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)calibrationAction, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_SensorCalibration, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.C.W direction
        .The method refers to the Object Dictionary: 0x3028
  * @param[in] encoderHallOffset  a float value between 0.0 to 1.0
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetEncoderHallCcwOffset(float encoderHallOffset, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetEncoderHallCcwOffsetInputValidation(encoderHallOffset, error))
  {
    return false;
  }
  soloUtils->ConvertToData(encoderHallOffset, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_EncoderHallCcwOffset, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.W direction
        .The method refers to the Object Dictionary: 0x3029
  * @param[in] encoderHallOffset  a float value between 0.0 to 1.0
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetEncoderHallCwOffset(float encoderHallOffset, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetEncoderHallCwOffsetInputValidation(encoderHallOffset, error))
  {
    return false;
  }
  soloUtils->ConvertToData(encoderHallOffset, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_EncoderHallCwOffset, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command defines the acceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
        .The method refers to the Object Dictionary: 0x302A
  * @param[in] speedAccelerationValue  a float value between 0 to 1600
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetSpeedAccelerationValue(float speedAccelerationValue, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedAccelerationValueInputValidation(speedAccelerationValue, error))
  {
    return false;
  }
  soloUtils->ConvertToData(speedAccelerationValue, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_SpeedAccelerationValue, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command defines the deceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
        .The method refers to the Object Dictionary: 0x302B
  * @param[in] speedDecelerationValue  a float value between 0 to 1600
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetSpeedDecelerationValue(float speedDecelerationValue, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedDecelerationValueInputValidation(speedDecelerationValue, error))
  {
    return false;
  }
  soloUtils->ConvertToData(speedDecelerationValue, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_SpeedDecelerationValue, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This command sets the baud rate of CAN bus in CANOpen network
        .The method refers to the Object Dictionary: 0x302C
  * @param[in] canbusBaudrate  enum that specify the baud rate of CAN bus in CANOpen network
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate canbusBaudrate, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)canbusBaudrate, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_CanbusBaudrate, 0x00, informationToSend, informatrionToRead, error);
}

/**
 * @brief  This command defines the resolution of the speed at S/T input
 *           while SOLO operates in Analogue mode
 *           .The method refers to the Object Dictionary: 0x303E
 * @param[in] divisionCoefficient  a long value
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetAnalogueSpeedResolutionDivisionCoefficientInputValidation(divisionCoefficient, error))
  {
    return false;
  }
  soloUtils->ConvertToData((long)divisionCoefficient, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_ASRDC, 0x00, informationToSend, informatrionToRead, error);
}

/**
 * @brief  This command defines the type of the Motion Profile that is
 *           being used in Speed or Position Modes
 *           .The method refers to the Object Dictionary: 0x3040
 * @param[in] motionProfileMode enum that specify the type of the Motion Profile
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetMotionProfileMode(SOLOMotorControllers::MotionProfileMode motionProfileMode, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)motionProfileMode, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_MotionProfileMode, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
        .The method refers to the Object Dictionary: 0x3041
  * @param[in] MotionProfileVariable1 a float value
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable1(float MotionProfileVariable1, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable1InputValidation(MotionProfileVariable1, error))
  {
    return false;
  }
  soloUtils->ConvertToData((float)MotionProfileVariable1, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_MotionProfileVariable1, 0x00, informationToSend, informatrionToRead, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 * @param[in] MotionProfileVariable2 a float value
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable2(float MotionProfileVariable2, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable2InputValidation(MotionProfileVariable2, error))
  {
    return false;
  }
  soloUtils->ConvertToData((float)MotionProfileVariable2, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_MotionProfileVariable2, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
        .The method refers to the Object Dictionary: 0x3043
  * @param[in] MotionProfileVariable3 a float value
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable3(float MotionProfileVariable3, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable3InputValidation(MotionProfileVariable3, error))
  {
    return false;
  }
  soloUtils->ConvertToData((float)MotionProfileVariable3, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_MotionProfileVariable3, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
        .The method refers to the Object Dictionary: 0x3044
  * @param[in] MotionProfileVariable4 a float value
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable4(float MotionProfileVariable4, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable4InputValidation(MotionProfileVariable4, error))
  {
    return false;
  }
  soloUtils->ConvertToData((float)MotionProfileVariable4, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_MotionProfileVariable4, 0x00, informationToSend, informatrionToRead, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
        .The method refers to the Object Dictionary: 0x3045
  * @param[in] MotionProfileVariable5 a float value
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopen::SetMotionProfileVariable5(float MotionProfileVariable5, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informatrionToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable5InputValidation(MotionProfileVariable5, error))
  {
    return false;
  }
  soloUtils->ConvertToData((float)MotionProfileVariable5, informationToSend);
  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_MotionProfileVariable5, 0x00, informationToSend, informatrionToRead, error);
}

/**
 * @brief  This PDO command sets the desired Position reference in terms of quadrature
 *         pulses while SOLO operates with the Incremental Encoders or in terms of
 *         pulses while while SOLO operates with Hall sensors
 *				.The method refers to the Object Dictionary: 0x1414
 * @param[in] positionReference  a long value between -2,147,483,647 to 2,147,483,647
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetPdoPositionReference(long positionReference, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPositionReferenceInputValidation(positionReference, error))
  {
    return false;
  }
  return SOLOMotorControllersCanopen::SetPdoParameterValue(PdoParameterName::POSITION_REFERENCE, positionReference, error);
}

/**
 * @brief  This PDO command defines the speed reference for SOLO once it’s in Digital Speed Mode
 *				.The method refers to the Object Dictionary: 0x1415
 * @param[in] speedReference  a long value defining the speed (only positive)
 * @param[out]  error   optional pointer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetPdoSpeedReference(long speedReference, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedReferenceInputValidation(speedReference, error))
  {
    return false;
  }

  return SOLOMotorControllersCanopen::SetPdoParameterValue(PdoParameterName::SPEED_REFERENCE, speedReference, error);
}

/**
 * @brief  This PDO command sets the amount of desired current that acts in torque generation
 *				.The method refers to the Object Dictionary: 0x1416
 * @param[in] torqueReferenceIq  a float value between 0 to 32
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetPdoTorqueReferenceIq(float torqueReferenceIq, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetTorqueReferenceIqInputValidation(torqueReferenceIq, error))
  {
    return false;
  }
  return SOLOMotorControllersCanopen::SetPdoParameterValue(PdoParameterName::TORQUE_REFERENCE_IQ, torqueReferenceIq, error);
}

/**
 * @brief  this PDO command depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
 *         Weakening current reference to help the motor reaching speeds higher than
 *         nominal values and in case of AC Induction Motors Sets the desired magnetizing
 *         current (Id) required for controlling ACIM motors in FOC in Amps
 *				.The method refers to the Object Dictionary: 0x1417
 * @param[in] magnetizingCurrentIdReference  a float value between 0 to 32
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetPdoMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMagnetizingCurrentIdReferenceInputValidation(magnetizingCurrentIdReference, error))
  {
    return false;
  }
  return SOLOMotorControllersCanopen::SetPdoParameterValue(PdoParameterName::MAGNETIZING_CURRENT_ID_REFERENCE, magnetizingCurrentIdReference, error);
}

/**
 * @brief  This PDO command sets the Control Mode in terms of Torque,
 *         Speed or Position only in Digital Mode
 *				.The method refers to the Object Dictionary: 0x1418
 * @param[in] controlMode  enum that specify the Control Mode in terms of Torque,
 *                       Speed or Position only in Digital Mode
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetPdoControlMode(SOLOMotorControllers::ControlMode controlMode, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersCanopen::SetPdoParameterValue(PdoParameterName::CONTROL_MODE, (long)controlMode, error);
}

/**
 * @brief  This PDO command sets the direction of the rotation of the motor
 *         either to ClockWise rotation or to Counter Clockwise Rotation
 *				.The method refers to the Object Dictionary: 0x1419
 * @param[in] motorDirection  enum that specify the direction of the rotation of the motor
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetPdoMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersCanopen::SetPdoParameterValue(PdoParameterName::MOTOR_DIRECTION, (long)motorDirection, error);
}

/**
 * @brief  This command Set the Digiatal Ouput pin Status. The method refers to the Object Dictionary: 0x3048
 * @param[out] pinNumber   specify the pin you want to controll. (Ensure your SOLO model support this functions)
 * @param[out] digitalStatus   specify the DigitalStatus you want to set. 
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopen::SetDigitalOutput(int pinNumber, SOLOMotorControllers::DigitalStatus digitalStatus, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->DigitalInputValidation(pinNumber, error))
  {
    return false;
  }

  informationToSend[3] = GetDigitalOutputs(error);
  if(error != SOLOMotorControllers::Error::NO_ERROR_DETECTED){
    return false;
  }

  uint32_t mask = 1 << pinNumber;
  if(digitalStatus == SOLOMotorControllers::DigitalStatus::LOW_STATUS)
  {
    informationToSend[3] &= ~mask; 
  }else{
    informationToSend[3] |= mask; 
  }

  return _MCP2515->CANOpenSdoTransmit(Address, true, Object_DigitalOutput, 0x00, informationToSend, informationReceived, error);
}
//---------------------Read---------------------
long SOLOMotorControllersCanopen::GetReadErrorRegister(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_ReadErrorRegister, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}
long SOLOMotorControllersCanopen::GetGuardTime(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_GuardTime, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

long SOLOMotorControllersCanopen::GetLifeTimeFactor(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_LifeTimeFactor, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

long SOLOMotorControllersCanopen::GetProducerHeartbeatTime(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_ProducerHeartbeatTime, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the device address connected on the line
        .The method refers to the Object Dictionary: 0x3001
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long device address connected on the line
  */
long SOLOMotorControllersCanopen::GetDeviceAddress(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_SetDeviceAddress, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the phase-A voltage of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors
        .The method refers to the Object Dictionary: 0x302D
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float phase-A voltage of the motor between -60 to 60
  */
float SOLOMotorControllersCanopen::GetPhaseAVoltage(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_PhaseAVoltage, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}


/**
  * @brief  This command reads the phase-B voltage of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors
        .The method refers to the Object Dictionary: 0x302E
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float 0 phase-A voltage of the motor between -60 to 60
  */
float SOLOMotorControllersCanopen::GetPhaseBVoltage(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_PhaseBVoltage, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the phase-A current of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors
        .The method refers to the Object Dictionary: 0x302F
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval phase-A current of the motor etween -32 to 32 Amps
  */
float SOLOMotorControllersCanopen::GetPhaseACurrent(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_PhaseACurrent, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}


/**
  * @brief  This command reads the phase-B current of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors
        .The method refers to the Object Dictionary: 0x3030
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float phase-B current of the motor etween -32 to 32 Amps
  */
float SOLOMotorControllersCanopen::GetPhaseBCurrent(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_PhaseBCurrent, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the input BUS voltage
        .The method refers to the Object Dictionary: 0x3031
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float  BUS voltage between 0 to 60
  */
float SOLOMotorControllersCanopen::GetBusVoltage(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_BusVoltage, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the current inside the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO
        .The method refers to the Object Dictionary: 0x3032
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between -32 to 32
  */
float SOLOMotorControllersCanopen::GetDcMotorCurrentIm(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_DcMotorCurrentIm, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the voltage of the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO
        .The method refers to the Object Dictionary: 0x3033
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between -60 to 60
  */
float SOLOMotorControllersCanopen::GetDcMotorVoltageVm(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_DcMotorVoltageVm, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the value of the Speed controller Kp gain,
  *         set for Digital mode operations
        .The method refers to the Object Dictionary: 0x300A
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopen::GetSpeedControllerKp(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_SpeedControllerKp, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the value of the Speed controller Ki gain,
  *         set for Digital mode operations
        .The method refers to the Object Dictionary: 0x300B
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopen::GetSpeedControllerKi(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_SpeedControllerKi, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the output switching frequency of SOLO in Hertz
        .The method refers to the Object Dictionary: 0x3009
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 8000 to 80000 Hz
  */
long SOLOMotorControllersCanopen::GetOutputPwmFrequencyKhz(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_OutputPwmFrequencyKhz, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived) / 1000L);
  }
  return -1;
}

/**
  * @brief  This command reads the value of the current limit set for SOLO in
  *         closed-loop digital operation mode
        .The method refers to the Object Dictionary: 0x3003
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 32
  */
float SOLOMotorControllersCanopen::GetCurrentLimit(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_CurrentLimit, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the actual monetary value of “Iq” that is
  *         the current acts in torque generation in FOC mode for 3-phase motors
        .The method refers to the Object Dictionary: 0x3034
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between -64 to 64
  */
float SOLOMotorControllersCanopen::GetQuadratureCurrentIqFeedback(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_QuadratureCurrentIqFeedback, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the actual monetary value of Id that is the
  *         direct current acting in FOC
        .The method refers to the Object Dictionary: 0x3035
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between -64 to 64
  */
float SOLOMotorControllersCanopen::GetMagnetizingCurrentIdFeedback(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_MagnetizingCurrentIdFeedback, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the number of Poles set for 3-phase motors
        .The method refers to the Object Dictionary: 0x300F
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 1 to 254
  */
long SOLOMotorControllersCanopen::GetMotorPolesCounts(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_MotorPolesCounts, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the number of physical Incremental encoder lines set on SOLO
        .The method refers to the Object Dictionary: 0x3010
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 1 to 200000
  */
long SOLOMotorControllersCanopen::GetIncrementalEncoderLines(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_IncrementalEncoderLines, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Kp or proportional gain
        .The method refers to the Object Dictionary: 0x3017
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopen::GetCurrentControllerKp(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_CurrentControllerKp, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Ki or integrator gain
        .The method refers to the Object Dictionary: 0x3018
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopen::GetCurrentControllerKi(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_CurrentControllerKi, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived) * 0.00005);
  }
  return -1.0;
}

/**
  * @brief  This command reads the momentary temperature of the board in centigrade
        .The method refers to the Object Dictionary: 0x3039
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between -30 to 150 Celsius
  */
float SOLOMotorControllersCanopen::GetBoardTemperature(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_BoardTemperature, 0x00, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the Phase or Armature resistance of
  *         the 3-phase or DC brushed motor connected to SOLO respectively
        .The method refers to the Object Dictionary: 0x300D
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 100
  */
float SOLOMotorControllersCanopen::GetMotorResistance(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_MotorResistance, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the Phase or Armature Inductance of
  *         the 3-phase or DC brushed motor connected to SOLO respectively
        .The method refers to the Object Dictionary: 0x300E
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 0.2
  */
float SOLOMotorControllersCanopen::GetMotorInductance(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_MotorInductance, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  his command reads the actual speed of the motor measured or estimated by SOLO in
            sensorless or sensor-based modes respectively
        .The method refers to the Object Dictionary: 0x3036
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between -30000 to 30000 RPM
  */
long SOLOMotorControllersCanopen::GetSpeedFeedback(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_SpeedFeedback, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the Motor type selected for Digital or Analogue mode operations
        .The method refers to the Object Dictionary: 0x3015
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 0 to 3
  */
long SOLOMotorControllersCanopen::GetMotorType(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_MotorType, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the feedback control mode selected on SOLO both
  *         for Analogue and Digital operations
        .The method refers to the Object Dictionary: 0x3013
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 0 to 2
  */
long SOLOMotorControllersCanopen::GetFeedbackControlMode(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_FeedbackControlMode, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the actual commanding mode that SOLO is operating
        .The method refers to the Object Dictionary: 0x3002
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 0 or 1
  */
long SOLOMotorControllersCanopen::GetCommandMode(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_CommandMode, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the Control Mode type in terms of Torque,
  *         Speed or Position in both Digital and Analogue modes
        .The method refers to the Object Dictionary: 0x3013
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 0 to 2
  */
long SOLOMotorControllersCanopen::GetControlMode(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_ControlMode, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the value of the speed limit set on SOLO
        .The method refers to the Object Dictionary: 0x3011
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 0 to 30000
  */
long SOLOMotorControllersCanopen::GetSpeedLimit(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_SpeedLimit, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Kp or proportional gain
        .The method refers to the Object Dictionary: 0x301C
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopen::GetPositionControllerKp(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_PositionControllerKp, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Ki or integrator gain
        .The method refers to the Object Dictionary: 0x301D
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopen::GetPositionControllerKi(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_PositionControllerKi, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the number of counted pulses from the
  *         Incremental Encoder or Hall sensors
        .The method refers to the Object Dictionary: 0x3037
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between -2,147,483,647 to 2,147,483,647
  */
long SOLOMotorControllersCanopen::GetPositionCountsFeedback(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_PositionCountsFeedback, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the error register which is a 32 bit register with
  *         each bit corresponding to specific errors
        .The method refers to the Object Dictionary: 0x3020
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long
  */
long SOLOMotorControllersCanopen::GetErrorRegister(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_OverwriteErrorRegister, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the Firmware version existing currently on the SOLO unit
        .The method refers to the Object Dictionary: 0x303A
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long
  */
long SOLOMotorControllersCanopen::GetDeviceFirmwareVersion(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_DeviceFirmwareVersion, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the Hardware version of the SOLO unit connected
        .The method refers to the Object Dictionary: 0x303B
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long
  */
long SOLOMotorControllersCanopen::GetDeviceHardwareVersion(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_DeviceHardwareVersion, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the amount of desired Torque reference (Iq or IM)
  *         already set for the Motor to follow in Digital Closed-loop Torque control mode
        .The method refers to the Object Dictionary: 0x3004
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 32
  */
float SOLOMotorControllersCanopen::GetTorqueReferenceIq(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_TorqueReferenceIq, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the amount of desired Speed reference already set for
  *         the Motor to follow in Digital Closed-loop Speed control mode
        .The method refers to the Object Dictionary: 0x3005
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 0 to 30000
  */
long SOLOMotorControllersCanopen::GetSpeedReference(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_SpeedReference, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the amount of desired Id (direct current) or
  *         Magnetizing current reference already set for the Motor to follow
  *         in Digital Closed-loop Speed control mode for ACIM motors
        .The method refers to the Object Dictionary: 0x301A
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 32
  */
float SOLOMotorControllersCanopen::GetMagnetizingCurrentIdReference(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_MagnetizingCurrentIdReference, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the desired position reference set for the Motor
  *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses
        .The method refers to the Object Dictionary: 0x301B
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between -2,147,483,647 to 2,147,483,647 Quad-Pulses
  */
long SOLOMotorControllersCanopen::GetPositionReference(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_PositionReference, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the desired Power reference for SOLO to apply in
  *         Digital Open-loop speed control mode for 3-phase motors in terms of percentage
        .The method refers to the Object Dictionary: 0x3006
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 100 %
  */
float SOLOMotorControllersCanopen::GetPowerReference(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_PowerReference, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This commands reads the desired direction of rotation set for the Motor
        .The method refers to the Object Dictionary: 0x300C
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long 0 Counter ClockWise / 1 ClockWise
  */
long SOLOMotorControllersCanopen::GetMotorDirection(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_MotorDirection, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors
        .The method refers to the Object Dictionary: 0x3021
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersCanopen::GetObserverGainBldcPmsm(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_ObserverGainBldcPmsm, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors
        .The method refers to the Object Dictionary: 0x3022
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersCanopen::GetObserverGainBldcPmsmUltrafast(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_ObserverGainBldcPmsmUltrafast, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for DC Motor
        .The method refers to the Object Dictionary: 0x3023
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersCanopen::GetObserverGainDc(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_ObserverGainDc, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Observer
  *         Filter Gain for Normal BLDC-PMSM Motors
        .The method refers to the Object Dictionary: 0x3024
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0.01 to 16000
  */
float SOLOMotorControllersCanopen::GetFilterGainBldcPmsm(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_FilterGainBldcPmsm, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Observer
  *         Filter Gain for Ultra Fast BLDC-PMSM Motors
        .The method refers to the Object Dictionary: 0x3025
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0.01 to 16000
  */
float SOLOMotorControllersCanopen::GetFilterGainBldcPmsmUltrafast(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_FilterGainBldcPmsmUltrafast, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the measured or estimated per-unit angle of the 3-phase motors
        .The method refers to the Object Dictionary: 0x3038
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between -2 to 2
  */
float SOLOMotorControllersCanopen::Get3PhaseMotorAngle(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_3PhaseMotorAngle, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction
        .The method refers to the Object Dictionary: 0x3028
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 1.0
  */
float SOLOMotorControllersCanopen::GetEncoderHallCcwOffset(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_EncoderHallCcwOffset, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction
        .The method refers to the Object Dictionary: 0x3029
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 1.0
  */
float SOLOMotorControllersCanopen::GetEncoderHallCwOffset(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_EncoderHallCwOffset, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line
        .The method refers to the Object Dictionary: 0x3026
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 0 or 1
  */
long SOLOMotorControllersCanopen::GetUartBaudrate(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_UartBaudrate, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the acceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds
        .The method refers to the Object Dictionary: 0x302A
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 1600 Rev/S^2
  */
float SOLOMotorControllersCanopen::GetSpeedAccelerationValue(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_SpeedAccelerationValue, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the deceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds
        .The method refers to the Object Dictionary: 0x302B
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 1600 Rev/S^2
  */
float SOLOMotorControllersCanopen::GetSpeedDecelerationValue(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_SpeedDecelerationValue, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
 * @brief  This command Reads the baud rate of CAN bus in CANOpen network
 *           .The method refers to the Object Dictionary: 0x302C
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval long [kbits/s]
 */
long SOLOMotorControllersCanopen::GetCanbusBaudrate(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_CanbusBaudrate, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
        .The method refers to the Object Dictionary: 0x303E
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long
  */
long SOLOMotorControllersCanopen::GetAnalogueSpeedResolutionDivisionCoefficient(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_ASRDC, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
 *
 * @brief  This command test if the communication is working
 * @retval bool 0 not working / 1 for working
 */
bool SOLOMotorControllersCanopen::CommunicationIsWorking(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  float temperature = SOLOMotorControllersCanopen::GetBoardTemperature(error);
  if (error == SOLOMotorControllers::Error::NO_ERROR_DETECTED)
  {
    return true;
  }
  return false;
}

/**
  * @brief  This Command reads the number of counted index pulses
  *         seen on the Incremental Encoder’s output
        .The method refers to the Object Dictionary: 0x303D
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 0 to 2,147,483,647
  */
long SOLOMotorControllersCanopen::GetEncoderIndexCounts(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_EncoderIndexCounts, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the type of the Embedded Motion profile active in the controller
        .The method refers to the Object Dictionary: 0x303F
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long
  */
long SOLOMotorControllersCanopen::GetMotionProfileMode(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_MotionProfileMode, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable1 set inside the controller
        .The method refers to the Object Dictionary: 0x3040
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable1(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_MotionProfileVariable1, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable2 set inside the controller
        .The method refers to the Object Dictionary: 0x3041
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable2(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_MotionProfileVariable2, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable3 set inside the controller
        .The method refers to the Object Dictionary: 0x3042
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable3(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_MotionProfileVariable3, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable4 set inside the controller
        .The method refers to the Object Dictionary: 0x3043
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable4(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_MotionProfileVariable4, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable5 set inside the controller
        .The method refers to the Object Dictionary: 0x3044
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float
  */
float SOLOMotorControllersCanopen::GetMotionProfileVariable5(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_MotionProfileVariable5, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1;
}

void SOLOMotorControllersCanopen::Mcp2515ReadErrorMode(int &errorMode)
{
  _MCP2515->Mcp2515ReadErrorMode(errorMode);
}
uint8_t SOLOMotorControllersCanopen::Mcp2515ReadReceiveErrorCounter()
{
  return _MCP2515->Mcp2515ReadReceiveErrorCounter();
}
uint8_t SOLOMotorControllersCanopen::Mcp2515ReadTransmitErrorCounter()
{
  return _MCP2515->Mcp2515ReadTransmitErrorCounter();
}

void SOLOMotorControllersCanopen::GenericCanbusReadMcp2515(uint16_t *_ID, uint8_t *_DLC, uint8_t *_Data)
{
  *_ID = 0;
  *_DLC = 0;
  if ((_MCP2515->MCP2515_Read_RX_Status() & (1 << 6)))
  {
    _MCP2515->MCP2515_Receive_Frame(_MCP2515->MCP2515_RX_BUF::RX_BUFFER_0, _ID, _DLC, _Data);
  }
}
void SOLOMotorControllersCanopen::GenericCanbusWriteMcp2515(uint16_t _ID, uint8_t *_DLC, uint8_t *_Data, int &error)
{
  _MCP2515->MCP2515_Transmit_Frame(_MCP2515->MCP2515_TX_BUF::TX_BUFFER_0, _ID, *_DLC, _Data, error);
}

/**
 * @brief  this PDO command give the first in the baffer position of the Motor
 *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses
 *				.The method refers to the Object Dictionary: 0x1814
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval long between -2,147,483,647 to 2,147,483,647 Quad-Pulses
 */
long SOLOMotorControllersCanopen::GetPdoPositionCountsFeedback(int &error)
{
  return GetPdoParameterValueLong(PdoParameterName::POSITION_COUNTS_FEEDBACK, error);
}

/**
 * @brief  this PDO command give the first in the baffer speed of the motor measured or estimated by SOLO in
 *		    sensorless or sensor-based modes respectively
 *				.The method refers to the Object Dictionary: 0x1815
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval long value that rappresent the speed feeback in RPM (positive and negative value)
 */
long SOLOMotorControllersCanopen::GetPdoSpeedFeedback(int &error)
{
  return GetPdoParameterValueLong(PdoParameterName::SPEED_FEEDBACK, error);
}

/**
 * @brief  This PDO command give the first in the baffer monetary value of “Iq” that is
 *         the current acts in torque generation in FOC mode for 3-phase motors
 *				.The method refers to the Object Dictionary: 0x1816
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval float between 0 to 32
 */
float SOLOMotorControllersCanopen::GetPdoQuadratureCurrentIqFeedback(int &error)
{
  return GetPdoParameterValueFloat(PdoParameterName::QUADRATURE_CURRENT_IQ_FEEDBACK, error);
}

/**
  * @brief  This PDO command give the first in the baffer monetary value of Id that is the
  *         direct current acting in FOC
        .The method refers to the Object Dictionary: 0x1817
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 32
  */
float SOLOMotorControllersCanopen::GetPdoMagnetizingCurrentIdFeedback(int &error)
{
  return GetPdoParameterValueFloat(PdoParameterName::MAGNETIZING_CURRENT_ID_FEEDBACK, error);
}

/**
 * @brief  This PDO command reads the error register which is a 32 bit register with
 *         each bit corresponding to specific errors
 *				.The method refers to the Object Dictionary: 0x1818
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersCanopen::GetPdoErrorRegister(int &error)
{
  return GetPdoParameterValueLong(PdoParameterName::ERROR_REGISTER, error);
}

/**
 * @brief  This PDO command reads the momentary temperature of the board in centigrade
 *				.The method refers to the Object Dictionary: 0x1819
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval float between -30 to 150 Celsius
 */
float SOLOMotorControllersCanopen::GetPdoBoardTemperature(int &error)
{
  return GetPdoParameterValueFloat(PdoParameterName::BOARD_TEMPERATURE, error);
}

/**
 * @brief  This command reads the PT1000 Voltage. The method refers to the Object Dictionary: 0x3047
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float
 */
float SOLOMotorControllersCanopen::GetPt1000(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_Pt1000, 0x00, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

uint8_t SOLOMotorControllersCanopen::GetDigitalOutputs(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_MCP2515->CANOpenSdoTransmit(Address, false, Object_DigitalOutput, 0x00, informationToSend, informationReceived, error))
  {
    return informationReceived[3];
  }
  return -1.0;
}
/**
 * @brief  This command reads the Digiatal Ouput pin Status. The method refers to the Object Dictionary: 0x3048
 * @param[out] pinNumber   specify the pin you want to controll. (Ensure your SOLO model support this functions)
 * @param[out] error   pointer to an integer that specify result of function
 * @retval int
 */
int SOLOMotorControllersCanopen::GetDigitalOutput(int pinNumber, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->DigitalInputValidation(pinNumber, error))
  {
    return -1;
  }

  uint8_t informationReceived = GetDigitalOutputs(error);
  if(error != SOLOMotorControllers::Error::NO_ERROR_DETECTED){
    return -1;
  }

  uint8_t mask = 1 << pinNumber;
  return (informationReceived & mask) != 0;
}