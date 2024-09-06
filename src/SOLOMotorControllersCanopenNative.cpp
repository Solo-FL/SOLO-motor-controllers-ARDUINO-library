/**
 *******************************************************************************
 * @file    SOLOMotorControllersCanopenNative.cpp
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the functions for the Solo Drivers
 *          CANopen communications.
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.4.2
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 *******************************************************************************
 */
#if defined(ARDUINO_PORTENTA_C33) || defined(ARDUINO_UNOWIFIR4) || defined(ARDUINO_MINIMA)
#include "SOLOMotorControllersCanopenNative.h"
#include <stdint.h>

CanBus *_canbus;
int SOLOMotorControllersCanopenNative::lastError = 0;

SOLOMotorControllersCanopenNative::SOLOMotorControllersCanopenNative(
  unsigned char _deviceAddress, 
  SOLOMotorControllers::CanbusBaudrate _baudrate, 
  arduino::HardwareCAN &_CAN, 
  long _millisecondsTimeout)
{
  if (_deviceAddress == 0) // Address 0 is reserved for the host
  {
    _deviceAddress = 1;
  }
  Address = _deviceAddress;

  _canbus = new CanBus(_baudrate, _CAN, _millisecondsTimeout);

  InitPdoConfig();
  soloUtils = new SOLOMotorControllersUtils();
}

bool SOLOMotorControllersCanopenNative::SetGuardTime(long guardtime, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetGuardTimeInputValidation(guardtime, error))
  {
    return false;
  }
  soloUtils->ConvertToData(guardtime, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_GUARD_TIME, 0x00, informationToSend, informationToRead, error);
}

bool SOLOMotorControllersCanopenNative::SetLifeTimeFactor(long lifeTimeFactor, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetLifeTimeFactorInputValidation(lifeTimeFactor, error))
  {
    return false;
  }
  soloUtils->ConvertToData(lifeTimeFactor, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_LIFE_TIME_FACTOR, 0x00, informationToSend, informationToRead, error);
}

bool SOLOMotorControllersCanopenNative::SetProducerHeartbeatTime(long producerHeartbeatTime, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetProducerHeartbeatTimeInputValidation(producerHeartbeatTime, error))
  {
    return false;
  }
  soloUtils->ConvertToData(producerHeartbeatTime, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_PRODUCER_HEARTBEAT_TIME, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  This command determine the validity of count of SYNC message
 * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to set CobId value
 * @param[in]  parameterCobbId	CobId value
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetPdoParameterCobbIdInputValidation(PdoParameterName parameterName, int parameterCobbId, int &error)
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
bool SOLOMotorControllersCanopenNative::SetSyncParameterCountInputValidation(uint8_t parameterCount, int &error)
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
bool SOLOMotorControllersCanopenNative::SetPdoParameterConfig(PdoParameterConfig config, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
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
  bool isSuccess = _canbus->CANOpenSdoTransmit(Address, true, pdoParameterObjectByPdoParameterName[config.parameterName], 0x01, informationToSend, informationToRead, error);
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

    isSuccess = _canbus->CANOpenSdoTransmit(Address, true, pdoParameterObjectByPdoParameterName[config.parameterName], 0x02, informationToSend, informationToRead, error);

    return isSuccess;
  }
  return false;
}

/**
 * @brief  This command gets PDO configs for the intended PDO object
 * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to get its config
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval PdoParameterConfig enum @ref SOLOMotorControllersCanopen::PdoParameterConfig
 */
SOLOMotorControllersCanopen::PdoParameterConfig SOLOMotorControllersCanopenNative::GetPdoParameterConfig(SOLOMotorControllersCanopen::PdoParameterName parameterName, int &error)
{
  PdoParameterConfig config;
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  if (_canbus->CANOpenSdoTransmit(Address, true, parameterName, 0x1, informationToSend, informationReceived, error))
  {
    config.parameterName = parameterName;
    config.parameterCobId = (informationReceived[2] << 8) + informationReceived[3];
    config.isPdoParameterEnable = informationReceived[0] & 0x80;
    config.isRrtParameterEnable = informationReceived[0] & 0x40;

    if (_canbus->CANOpenSdoTransmit(Address, true, parameterName, 0x2, informationToSend, informationReceived, error))
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
bool SOLOMotorControllersCanopenNative::SendPdoSync(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return _canbus->SendPdoSync(error);
}

/**
 * @brief  This command returns the CobId value for the intended PDO parameter name
 * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to get parameter CobId
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval long
 */
long SOLOMotorControllersCanopenNative::GetPdoParameterCobId(PdoParameterName parameterName, int &error)
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
void SOLOMotorControllersCanopenNative::InitPdoConfig()
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
bool SOLOMotorControllersCanopenNative::UpdatePdoParameterCobIdByPdoParameterName()
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
bool SOLOMotorControllersCanopenNative::SetPdoParameterValue(PdoParameterName parameterName, long value,
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

  return _canbus->PDOTransmit(adr, informationToSend, error);
}

/**
 * @brief  This command set the intended float value for a PDO command
 * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to write its value
 * @param[in]  value	float value that wants to be set for the PDO parameter
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetPdoParameterValue(PdoParameterName parameterName, float value,
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

  return _canbus->PDOTransmit(adr, informationToSend, error);
}

/**
 * @brief  This command returns the long value of a PDO command
 * @param[in]  parameterName	enum that specifies the name of the parameter that wants to read its value
 * @param[out]  error   optional pointer to an integer that specifies the result of the function
 * @retval long
 */
long SOLOMotorControllersCanopenNative::GetPdoParameterValueLong(PdoParameterName parameterName,
                                                                 int &error)
{
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_ERROR_DETECTED;

  int adr = GetPdoParameterCobId(parameterName, error);
  if (error != SOLOMotorControllers::Error::NO_ERROR_DETECTED)
  {
    return -1;
  }

  if (_canbus->PDOReceive(adr, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetPdoParameterValueFloat(PdoParameterName parameterName,
                                                                   int &error)
{
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_ERROR_DETECTED;

  int adr = GetPdoParameterCobId(parameterName, error);
  if (error != SOLOMotorControllers::Error::NO_ERROR_DETECTED)
  {
    return -1;
  }

  if (_canbus->PDOReceive(adr, informationReceived, error))
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
bool SOLOMotorControllersCanopenNative::SetDeviceAddress(unsigned char deviceAddress, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetDeviceAddressInputValidation(deviceAddress, error))
  {
    return false;
  }
  soloUtils->ConvertToData((long)deviceAddress, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_SET_DEVICE_ADDRESS, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  This command sets the mode of the operation of SOLO
 *         in terms of operating in Analogue mode or Digital
 *				.The method refers to the Object Dictionary: 0x3002
 * @param[in] mode  enum that specify mode of the operation of SOLO
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetCommandMode(SOLOMotorControllers::CommandMode mode, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)mode, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_COMMAND_MODE, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  This command defines the maximum allowed current into the motor in terms of Amps
 *				.The method refers to the Object Dictionary: 0x3003
 * @param[in] currentLimit  a float value between 0 to 32
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetCurrentLimit(float currentLimit, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetCurrentLimitInputValidation(currentLimit, error))
  {
    return false;
  }
  soloUtils->ConvertToData(currentLimit, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_CURRENT_LIMIT, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  This command sets the amount of desired current that acts in torque generation
 *				.The method refers to the Object Dictionary: 0x3004
 * @param[in] torqueReferenceIq  a float value between 0 to 32
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetTorqueReferenceIq(float torqueReferenceIq, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetTorqueReferenceIqInputValidation(torqueReferenceIq, error))
  {
    return false;
  }
  soloUtils->ConvertToData(torqueReferenceIq, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_TORQUE_REFERENCE_IQ, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
 *				.The method refers to the Object Dictionary: 0x3005
 * @param[in] speedReference  a long value between 0 to 30000
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetSpeedReference(long speedReference, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedReferenceInputValidation(speedReference, error))
  {
    return false;
  }
  soloUtils->ConvertToData(speedReference, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_SPEED_REFERENCE, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  This command defines the amount of power percentage during only
 *         Open-loop mode for 3-phase motors
 *				.The method refers to the Object Dictionary: 0x3006
 * @param[in] powerReference  a float value between 0 to 100
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetPowerReference(float powerReference, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPowerReferenceInputValidation(powerReference, error))
  {
    return false;
  }
  soloUtils->ConvertToData(powerReference, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_POWER_REFERENCE, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
 *          identifying the electrical parameters of the Motor connected
 *				.The method refers to the Object Dictionary: 0x3007
 * @param[in] identification  enum that specify Start or Stop of something in SOLO
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::MotorParametersIdentification(SOLOMotorControllers::Action identification, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)identification, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_MOTOR_PARAMETERS_IDENTIFICATION, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  This command Disables or Enables the Controller resulting in deactivation or activation of the
 *            	switching at the output, by disabling the drive, the effect of the Controller on the Motor will be
 *           	almost eliminated ( except for body diodes of the Mosfets) allowing freewheeling
 *            	.The method refers to the Uart Write command: 0x3008
 * @param[in]  action  enum that specify Disable or Enable of something in SOLO
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetDriveDisableEnable(DisableEnable action, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)action, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_DRIVE_DISABLE_ENABLE, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the output switching frequency of the whole power unit on the Motor
        .The method refers to the Object Dictionary: 0x3009
  * @param[in] outputPwmFrequencyKhz  switching frequencies in kHz. a long value between 8 to 80
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetOutputPwmFrequencyKhzInputValidation(outputPwmFrequencyKhz, error))
  {
    return false;
  }
  soloUtils->ConvertToData(outputPwmFrequencyKhz, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_OUTPUT_PWM_FREQUENCY_KHZ, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the Speed controller Kp Gain, and it will
  *         be functional only in Digital Closed-loop mode
        .The method refers to the Object Dictionary: 0x300A
  * @param[in] speedControllerKp  a float value between 0 to 300
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetSpeedControllerKp(float speedControllerKp, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedControllerKpInputValidation(speedControllerKp, error))
  {
    return false;
  }
  soloUtils->ConvertToData(speedControllerKp, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_SPEED_CONTROLLER_KP, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the Speed controller Ki gain, and it will
  *         be functional only in Digital Closed-loop mode
        .The method refers to the Object Dictionary: 0x300B
  * @param[in] speedControllerKi  a float value between 0 to 300
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetSpeedControllerKi(float speedControllerKi, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedControllerKiInputValidation(speedControllerKi, error))
  {
    return false;
  }
  soloUtils->ConvertToData(speedControllerKi, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_SPEED_CONTROLLER_KI, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This commands sets the direction of the rotation of the motor
  *         either to ClockWise rotation or to Counter Clockwise Rotation
        .The method refers to the Object Dictionary: 0x300C
  * @param[in] motorDirection  enum that specify the direction of the rotation of the motor
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)motorDirection, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_MOTOR_DIRECTION, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the amount of the Phase or Armature resistance
  *         for 3-phase or DC Brushed motors respectively
        .The method refers to the Object Dictionary: 0x300D
  * @param[in] motorResistance  a float value between 0.001 to 50
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetMotorResistance(float motorResistance, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotorResistanceInputValidation(motorResistance, error))
  {
    return false;
  }
  soloUtils->ConvertToData(motorResistance, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_MOTOR_RESISTANCE, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the amount of the Phase or Armature Inductance
  *         for 3-phase or DC Brushed motors respectively
        .The method refers to the Object Dictionary: 0x300E
  * @param[in] motorInductance  a float value between 0.00005 to 0.2
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetMotorInductance(float motorInductance, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotorInductanceInputValidation(motorInductance, error))
  {
    return false;
  }
  soloUtils->ConvertToData(motorInductance, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_MOTOR_INDUCTANCE, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
        .The method refers to the Object Dictionary: 0x300F
  * @param[in] motorPolesCounts  a long value between 1 to 254
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetMotorPolesCounts(long motorPolesCounts, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotorPolesCountsInputValidation(motorPolesCounts, error))
  {
    return false;
  }

  soloUtils->ConvertToData(motorPolesCounts, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_MOTOR_POLES_COUNTS, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the pre-quad number of physical lines of an
  *         incremental encoder engraved on its disk
        .The method refers to the Object Dictionary: 0x3010
  * @param[in] incrementalEncoderLines  a long value between 1 to 200000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetIncrementalEncoderLines(long incrementalEncoderLines, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetIncrementalEncoderLinesInputValidation(incrementalEncoderLines, error))
  {
    return false;
  }
  soloUtils->ConvertToData(incrementalEncoderLines, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_INCREMENTAL_ENCODER_LINES, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the allowed speed during trajectory following
  *         in closed-loop position controlling mode
        .The method refers to the Object Dictionary: 0x3011
  * @param[in] speedLimit  a long value between 0 to 30000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetSpeedLimit(long speedLimit, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedLimitInputValidation(speedLimit, error))
  {
    return false;
  }
  soloUtils->ConvertToData(speedLimit, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_SPEED_LIMIT, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the type of the feedback control SOLO has to operate
        .The method refers to the Object Dictionary: 0x3013
  * @param[in] feedbackControlMode  enum that specify the type of the feedback control SOLO
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode feedbackControlMode, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)feedbackControlMode, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_FEEDBACK_CONTROL_MODE, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command resets SOLO to its factory setting to all the default parameters
        .The method refers to the Object Dictionary: 0x3014
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::ResetFactory(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x01};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_RESET_FACTORY, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the Motor type that is connected to SOLO in Digital Mode
        .The method refers to the Object Dictionary: 0x3015
  * @param[in] motorType  enum that specify the Motor type that is connected to SOLO in Digital Mode
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetMotorType(SOLOMotorControllers::MotorType motorType, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)motorType, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_MOTOR_TYPE, 0x00, informationToSend, informationToRead, error);
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
bool SOLOMotorControllersCanopenNative::SetControlMode(SOLOMotorControllers::ControlMode controlMode, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)controlMode, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_CONTROL_MODE, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the value for Current Controller Kp or proportional gain
        .The method refers to the Object Dictionary: 0x3017
  * @param[in] currentControllerKp  a float value between 0 to 16000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetCurrentControllerKp(float currentControllerKp, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetCurrentControllerKpInputValidation(currentControllerKp, error))
  {
    return false;
  }
  soloUtils->ConvertToData(currentControllerKp, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_CURRENT_CONTROLLER_KP, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the value for Current Controller Ki or integral gain
        .The method refers to the Object Dictionary: 0x3018
  * @param[in] currentControllerKi  a float value between 0 to 16000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetCurrentControllerKi(float currentControllerKi, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetCurrentControllerKiInputValidation(currentControllerKi, error))
  {
    return false;
  }
  soloUtils->ConvertToData(currentControllerKi, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_CURRENT_CONTROLLER_KI, 0x00, informationToSend, informationToRead, error);
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
bool SOLOMotorControllersCanopenNative::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMagnetizingCurrentIdReferenceInputValidation(magnetizingCurrentIdReference, error))
  {
    return false;
  }
  soloUtils->ConvertToData(magnetizingCurrentIdReference, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_MAGNETIZING_CURRENT_ID_REFERENCE, 0x00, informationToSend, informationToRead, error);
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
bool SOLOMotorControllersCanopenNative::SetPositionReference(long positionReference, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPositionReferenceInputValidation(positionReference, error))
  {
    return false;
  }
  soloUtils->ConvertToData(positionReference, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_POSITION_REFERENCE, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the value for Position Controller Kp or proportional gain
        .The method refers to the Object Dictionary: 0x301C
  * @param[in] positionControllerKp  a float value between 0 to 16000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetPositionControllerKp(float positionControllerKp, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPositionControllerKpInputValidation(positionControllerKp, error))
  {
    return false;
  }
  soloUtils->ConvertToData(positionControllerKp, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_POSITION_CONTROLLER_KP, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the value for Position Controller Ki or integrator gain
        .The method refers to the Object Dictionary: 0x301D
  * @param[in] positionControllerKi  a float value between 0 to 16000
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetPositionControllerKi(float positionControllerKi, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPositionControllerKiInputValidation(positionControllerKi, error))
  {
    return false;
  }
  soloUtils->ConvertToData(positionControllerKi, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_POSITION_CONTROLLER_KI, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command overwrites the reported errors in Error Register
  *         reported with command code of "0xA1"
        .The method refers to the Object Dictionary: 0x3020
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::OverwriteErrorRegister(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_OVERWRITE_ERROR_REGISTER, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  Once in Zero Speed Full Torque algorithm (ZSFT) for controlling the speed of a BLDC or PMSM
              in sensorless fashion, this parameter defines the strength of signal injection into the motor, the
              user has to make sure this value is not selected too high or too low
        .The method refers to the Object Dictionary: 0x3021
  * @param[in] zsftInjectionAmplitude  a float value between 0.0 to 0.55
  * @param[out]  error   pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetZsftInjectionAmplitude(float zsftInjectionAmplitude, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetZsftInjectionAmplitudeValidation(zsftInjectionAmplitude, error))
  {
    return false;
  }
  soloUtils->ConvertToData(zsftInjectionAmplitude, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_ZSFT_INJECTION_AMPLITUDE, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  Once in Zero Speed Full Torque algorithm (ZSFT) for controlling the speed of a BLDC or PMSM
 *             in sensorless fashion, this parameter defines the strength of signal injection into the motor to
 *            identify the polarity of the Motor at the startup
 *				.The method refers to the Object Dictionary: 0x3022
 * @param[in] zsftPolarityAmplitude  a float value between 0.0 to 0.55
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetZsftPolarityAmplitude(float zsftPolarityAmplitude, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetZsftPolarityAmplitudeValidation(zsftPolarityAmplitude, error))
  {
    return false;
  }
  soloUtils->ConvertToData(zsftPolarityAmplitude, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_ZSFT_POLARITY_AMPLITUDE, 0x00, informationToSend, informationToRead, error);
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
bool SOLOMotorControllersCanopenNative::SetObserverGainDc(float observerGain, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetObserverGainDcInputValidation(observerGain, error))
  {
    return false;
  }
  soloUtils->ConvertToData(observerGain, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_OBSERVER_GAIN_DC, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command defines the frequency of signal injection into the Motor in
        runtime, by selecting zero the full injection frequency will be applied which allows to reach to
        higher speeds, however for some motors, it’s better to increase this value
        .The method refers to the Object Dictionary: 0x3024
  * @param[in] zsftInjectionFrequency  a long value between 0 to 10
  * @param[out]  error   pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetZsftInjectionFrequency(long zsftInjectionFrequency, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetZsftInjectionFrequencyInputValidation(zsftInjectionFrequency, error))
  {
    return false;
  }
  soloUtils->ConvertToData(zsftInjectionFrequency, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_ZSFT_INJECTION_FREQUENCY, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  Once in Sensorless speed or torque controlling of a BLDC or PMSM motors, this parameter
 *				defines the speed in which the Low speed algorithm has to switch to high speed algorithm
 *				.The method refers to the Object Dictionary: 0x3025
 * @param[in] sensorlessTransitionSpeed  a long value between 1 to 5000
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetSensorlessTransitionSpeed(long sensorlessTransitionSpeed, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSensorlessTransitionSpeedInputValidation(sensorlessTransitionSpeed, error))
  {
    return false;
  }
  soloUtils->ConvertToData(sensorlessTransitionSpeed, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_SENSORLESS_TRANSACTION_SPEED, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the baud-rate of the UART line
        .The method refers to the Object Dictionary: 0x3026
  * @param[in] baudrate  enum that specify the baud-rate of the UART line
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)baudrate, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_UART_BAUDRATE, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command starts or stops the process of sensor calibration
        .The method refers to the Object Dictionary: 0x3027
  * @param[in] calibrationAction  enum that specify the process of sensor calibration
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)calibrationAction, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_SENSOR_CALIBRATION, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.C.W direction
        .The method refers to the Object Dictionary: 0x3028
  * @param[in] encoderHallOffset  a float value between 0.0 to 1.0
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetEncoderHallCcwOffset(float encoderHallOffset, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetEncoderHallCcwOffsetInputValidation(encoderHallOffset, error))
  {
    return false;
  }
  soloUtils->ConvertToData(encoderHallOffset, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_ENCODER_HALL_CCW_OFFSET, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.W direction
        .The method refers to the Object Dictionary: 0x3029
  * @param[in] encoderHallOffset  a float value between 0.0 to 1.0
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetEncoderHallCwOffset(float encoderHallOffset, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetEncoderHallCwOffsetInputValidation(encoderHallOffset, error))
  {
    return false;
  }
  soloUtils->ConvertToData(encoderHallOffset, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_ENCODER_HALL_CW_OFFSET, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command defines the acceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
        .The method refers to the Object Dictionary: 0x302A
  * @param[in] speedAccelerationValue  a float value between 0 to 1600
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetSpeedAccelerationValue(float speedAccelerationValue, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedAccelerationValueInputValidation(speedAccelerationValue, error))
  {
    return false;
  }
  soloUtils->ConvertToData(speedAccelerationValue, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_SPEED_ACCELERATION_VALUE, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command defines the deceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
        .The method refers to the Object Dictionary: 0x302B
  * @param[in] speedDecelerationValue  a float value between 0 to 1600
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetSpeedDecelerationValue(float speedDecelerationValue, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedDecelerationValueInputValidation(speedDecelerationValue, error))
  {
    return false;
  }
  soloUtils->ConvertToData(speedDecelerationValue, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_SPEED_DECELERATION_VALUE, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This command sets the baud rate of CAN bus in CANOpen network
        .The method refers to the Object Dictionary: 0x302C
  * @param[in] canbusBaudrate  enum that specify the baud rate of CAN bus in CANOpen network
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate canbusBaudrate, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)canbusBaudrate, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_CANBUS_BAUDRATE, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  This command defines the resolution of the speed at S/T input
 *           while SOLO operates in Analogue mode
 *           .The method refers to the Object Dictionary: 0x303E
 * @param[in] divisionCoefficient  a long value
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetAnalogueSpeedResolutionDivisionCoefficientInputValidation(divisionCoefficient, error))
  {
    return false;
  }
  soloUtils->ConvertToData((long)divisionCoefficient, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_ASRDC, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  This command defines the type of the Motion Profile that is
 *           being used in Speed or Position Modes
 *           .The method refers to the Object Dictionary: 0x3040
 * @param[in] motionProfileMode enum that specify the type of the Motion Profile
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetMotionProfileMode(SOLOMotorControllers::MotionProfileMode motionProfileMode, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  soloUtils->ConvertToData((long)motionProfileMode, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_MOTION_PROFILE_MODE, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
        .The method refers to the Object Dictionary: 0x3041
  * @param[in] MotionProfileVariable1 a float value
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetMotionProfileVariable1(float MotionProfileVariable1, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable1InputValidation(MotionProfileVariable1, error))
  {
    return false;
  }
  soloUtils->ConvertToData((float)MotionProfileVariable1, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_MOTION_PROFILE_VARIABLE1, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 * @param[in] MotionProfileVariable2 a float value
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetMotionProfileVariable2(float MotionProfileVariable2, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable2InputValidation(MotionProfileVariable2, error))
  {
    return false;
  }
  soloUtils->ConvertToData((float)MotionProfileVariable2, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_MOTION_PROFILE_VARIABLE2, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
        .The method refers to the Object Dictionary: 0x3043
  * @param[in] MotionProfileVariable3 a float value
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetMotionProfileVariable3(float MotionProfileVariable3, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable3InputValidation(MotionProfileVariable3, error))
  {
    return false;
  }
  soloUtils->ConvertToData((float)MotionProfileVariable3, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_MOTION_PROFILE_VARIABLE3, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
        .The method refers to the Object Dictionary: 0x3044
  * @param[in] MotionProfileVariable4 a float value
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetMotionProfileVariable4(float MotionProfileVariable4, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable4InputValidation(MotionProfileVariable4, error))
  {
    return false;
  }
  soloUtils->ConvertToData((float)MotionProfileVariable4, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_MOTION_PROFILE_VARIABLE4, 0x00, informationToSend, informationToRead, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
        .The method refers to the Object Dictionary: 0x3045
  * @param[in] MotionProfileVariable5 a float value
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenNative::SetMotionProfileVariable5(float MotionProfileVariable5, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable5InputValidation(MotionProfileVariable5, error))
  {
    return false;
  }
  soloUtils->ConvertToData((float)MotionProfileVariable5, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_MOTION_PROFILE_VARIABLE5, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  This command defines the maximum allowed regeneration current sent back from the Motor to
 *				the Power Supply during decelerations
 *           .The method refers to the Uart Write command: 0x304B
 * @param[in]  current a float value
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetRegenerationCurrentLimit(float current, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetRegenerationCurrentLimitValidation(current, error))
  {
    return false;
  }
  soloUtils->ConvertToData((float)current, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_REGENERATION_CURRENT_LIMIT, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  This value defines the the sampling window of qualification digital filter applied to the output of
 *			the position sensor before being processed by DSP
 *           .The method refers to the Uart Write command: 0x304C
 * @param[in]  level a long value
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetPositionSensorDigitalFilterLevel(long level, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPositionSensorDigitalFilterLevelValidation(level, error))
  {
    return false;
  }
  soloUtils->ConvertToData(level, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_POSITION_SENSOR_DIGITAL_FILTER_LEVEL, 0x00, informationToSend, informationToRead, error);
}

/**
 * @brief  This command Set the Digiatal Ouput pin Status. The method refers to the Object Dictionary: 0x3048
 * @param[in] channel	@ref Channel
 * @param[in] state		@ref DigitalIoState
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetDigitalOutputState(SOLOMotorControllers::Channel channel, SOLOMotorControllers::DigitalIoState state, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationToRead[4] = {0x00, 0x00, 0x00, 0x00};
  long lastOutRegister;

  lastOutRegister = GetDigitalOutputsRegister(error);
  if (error = 0)
    return error;

  if (state == 1)
    lastOutRegister = lastOutRegister | (1 << channel);
  else
    lastOutRegister = lastOutRegister & (~(1 << channel));

  soloUtils->ConvertToData(lastOutRegister, informationToSend);
  return _canbus->CANOpenSdoTransmit(Address, true, OBJECT_DIGITAL_OUTPUT_REGISTER, 0x00, informationToSend, informationToRead, error);
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
bool SOLOMotorControllersCanopenNative::SetPdoPositionReference(long positionReference, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPositionReferenceInputValidation(positionReference, error))
  {
    return false;
  }
  return SOLOMotorControllersCanopenNative::SetPdoParameterValue(PdoParameterName::POSITION_REFERENCE, positionReference, error);
}

/**
 * @brief  This PDO command defines the speed reference for SOLO once it’s in Digital Speed Mode
 *				.The method refers to the Object Dictionary: 0x1415
 * @param[in] speedReference  a long value defining the speed (only positive)
 * @param[out]  error   optional pointer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetPdoSpeedReference(long speedReference, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedReferenceInputValidation(speedReference, error))
  {
    return false;
  }

  return SOLOMotorControllersCanopenNative::SetPdoParameterValue(PdoParameterName::SPEED_REFERENCE, speedReference, error);
}

/**
 * @brief  This PDO command sets the amount of desired current that acts in torque generation
 *				.The method refers to the Object Dictionary: 0x1416
 * @param[in] torqueReferenceIq  a float value between 0 to 32
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetPdoTorqueReferenceIq(float torqueReferenceIq, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetTorqueReferenceIqInputValidation(torqueReferenceIq, error))
  {
    return false;
  }
  return SOLOMotorControllersCanopenNative::SetPdoParameterValue(PdoParameterName::TORQUE_REFERENCE_IQ, torqueReferenceIq, error);
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
bool SOLOMotorControllersCanopenNative::SetPdoMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMagnetizingCurrentIdReferenceInputValidation(magnetizingCurrentIdReference, error))
  {
    return false;
  }
  return SOLOMotorControllersCanopenNative::SetPdoParameterValue(PdoParameterName::MAGNETIZING_CURRENT_ID_REFERENCE, magnetizingCurrentIdReference, error);
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
bool SOLOMotorControllersCanopenNative::SetPdoControlMode(SOLOMotorControllers::ControlMode controlMode, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersCanopenNative::SetPdoParameterValue(PdoParameterName::CONTROL_MODE, (long)controlMode, error);
}

/**
 * @brief  This PDO command sets the direction of the rotation of the motor
 *         either to ClockWise rotation or to Counter Clockwise Rotation
 *				.The method refers to the Object Dictionary: 0x1419
 * @param[in] motorDirection  enum that specify the direction of the rotation of the motor
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersCanopenNative::SetPdoMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersCanopenNative::SetPdoParameterValue(PdoParameterName::MOTOR_DIRECTION, (long)motorDirection, error);
}

//---------------------Read---------------------
long SOLOMotorControllersCanopenNative::GetReadErrorRegister(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_READ_ERROR_REGISTER, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}
long SOLOMotorControllersCanopenNative::GetGuardTime(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_GUARD_TIME, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

long SOLOMotorControllersCanopenNative::GetLifeTimeFactor(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_LIFE_TIME_FACTOR, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

long SOLOMotorControllersCanopenNative::GetProducerHeartbeatTime(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_PRODUCER_HEARTBEAT_TIME, 0x22, informationToSend, informationReceived, error))
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
long SOLOMotorControllersCanopenNative::GetDeviceAddress(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_SET_DEVICE_ADDRESS, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetPhaseAVoltage(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_PHASE_A_VOLTAGE, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetPhaseBVoltage(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_PHASE_B_VOLTAGE, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetPhaseACurrent(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_PHASE_A_CURRENT, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetPhaseBCurrent(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_PHASE_B_CURRENT, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetBusVoltage(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_BUS_VOLTAGE, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetDcMotorCurrentIm(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_DC_MOTOR_CURRENT_IM, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetDcMotorVoltageVm(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_DC_MOTOR_VOLTAGE_VM, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetSpeedControllerKp(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_SPEED_CONTROLLER_KP, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetSpeedControllerKi(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_SPEED_CONTROLLER_KI, 0x22, informationToSend, informationReceived, error))
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
long SOLOMotorControllersCanopenNative::GetOutputPwmFrequencyKhz(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_OUTPUT_PWM_FREQUENCY_KHZ, 0x22, informationToSend, informationReceived, error))
  {
    return soloUtils->ConvertToLong(informationReceived);
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
float SOLOMotorControllersCanopenNative::GetCurrentLimit(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_CURRENT_LIMIT, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetQuadratureCurrentIqFeedback(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_QUADRATURE_CURRENT_IQ_FEEDBACK, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetMagnetizingCurrentIdFeedback(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_MAGNETIZING_CURRENT_ID_FEEDBACK, 0x22, informationToSend, informationReceived, error))
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
long SOLOMotorControllersCanopenNative::GetMotorPolesCounts(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_MOTOR_POLES_COUNTS, 0x22, informationToSend, informationReceived, error))
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
long SOLOMotorControllersCanopenNative::GetIncrementalEncoderLines(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_INCREMENTAL_ENCODER_LINES, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetCurrentControllerKp(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_CURRENT_CONTROLLER_KP, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetCurrentControllerKi(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_CURRENT_CONTROLLER_KI, 0x22, informationToSend, informationReceived, error))
  {
    return soloUtils->ConvertToFloat(informationReceived);
  }
  return -1.0;
}

/**
  * @brief  This command reads the momentary temperature of the board in centigrade
        .The method refers to the Object Dictionary: 0x3039
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between -30 to 150 Celsius
  */
float SOLOMotorControllersCanopenNative::GetBoardTemperature(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_BOARD_TEMPERATURE, 0x00, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetMotorResistance(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_MOTOR_RESISTANCE, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetMotorInductance(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_MOTOR_INDUCTANCE, 0x22, informationToSend, informationReceived, error))
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
long SOLOMotorControllersCanopenNative::GetSpeedFeedback(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_SPEED_FEEDBACK, 0x22, informationToSend, informationReceived, error))
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
SOLOMotorControllers::MotorType SOLOMotorControllersCanopenNative::GetMotorType(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_MOTOR_TYPE, 0x22, informationToSend, informationReceived, error))
  {
    return ((SOLOMotorControllers::MotorType)soloUtils->ConvertToLong(informationReceived));
  }
  return SOLOMotorControllers::MotorType::MOTOR_TYPE_ERROR;
  ;
}

/**
  * @brief  This command reads the feedback control mode selected on SOLO both
  *         for Analogue and Digital operations
        .The method refers to the Object Dictionary: 0x3013
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 0 to 2
  */
SOLOMotorControllers::FeedbackControlMode SOLOMotorControllersCanopenNative::GetFeedbackControlMode(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_FEEDBACK_CONTROL_MODE, 0x22, informationToSend, informationReceived, error))
  {
    return ((SOLOMotorControllers::FeedbackControlMode)soloUtils->ConvertToLong(informationReceived));
  }
  return SOLOMotorControllers::FeedbackControlMode::FEEDBACK_CONTROL_MODE_ERROR;
}

/**
  * @brief  This command reads the actual commanding mode that SOLO is operating
        .The method refers to the Object Dictionary: 0x3002
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 0 or 1
  */
SOLOMotorControllers::CommandMode SOLOMotorControllersCanopenNative::GetCommandMode(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_COMMAND_MODE, 0x22, informationToSend, informationReceived, error))
  {
    return ((SOLOMotorControllers::CommandMode)soloUtils->ConvertToLong(informationReceived));
  }
  return SOLOMotorControllers::CommandMode::COMMAND_MODE_ERROR;
  ;
}

/**
  * @brief  This command reads the Control Mode type in terms of Torque,
  *         Speed or Position in both Digital and Analogue modes
        .The method refers to the Object Dictionary: 0x3013
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 0 to 2
  */
SOLOMotorControllers::ControlMode SOLOMotorControllersCanopenNative::GetControlMode(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_CONTROL_MODE, 0x22, informationToSend, informationReceived, error))
  {
    return ((SOLOMotorControllers::ControlMode)soloUtils->ConvertToLong(informationReceived));
  }
  return SOLOMotorControllers::ControlMode::CONTROL_MODE_ERROR;
  ;
}

/**
  * @brief  This command reads the value of the speed limit set on SOLO
        .The method refers to the Object Dictionary: 0x3011
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval long between 0 to 30000
  */
long SOLOMotorControllersCanopenNative::GetSpeedLimit(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_SPEED_LIMIT, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetPositionControllerKp(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_POSITION_CONTROLLER_KP, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetPositionControllerKi(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_POSITION_CONTROLLER_KI, 0x22, informationToSend, informationReceived, error))
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
long SOLOMotorControllersCanopenNative::GetPositionCountsFeedback(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_POSITION_COUNTS_FEEDBACK, 0x22, informationToSend, informationReceived, error))
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
long SOLOMotorControllersCanopenNative::GetErrorRegister(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_OVERWRITE_ERROR_REGISTER, 0x22, informationToSend, informationReceived, error))
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
long SOLOMotorControllersCanopenNative::GetDeviceFirmwareVersion(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_DEVICE_FIRMWARE_VERSION, 0x22, informationToSend, informationReceived, error))
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
long SOLOMotorControllersCanopenNative::GetDeviceHardwareVersion(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_DEVICE_HARDWARE_VERSION, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetTorqueReferenceIq(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_TORQUE_REFERENCE_IQ, 0x22, informationToSend, informationReceived, error))
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
long SOLOMotorControllersCanopenNative::GetSpeedReference(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_SPEED_REFERENCE, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetMagnetizingCurrentIdReference(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_MAGNETIZING_CURRENT_ID_REFERENCE, 0x22, informationToSend, informationReceived, error))
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
long SOLOMotorControllersCanopenNative::GetPositionReference(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_POSITION_REFERENCE, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetPowerReference(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_POWER_REFERENCE, 0x22, informationToSend, informationReceived, error))
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
SOLOMotorControllers::Direction SOLOMotorControllersCanopenNative::GetMotorDirection(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_MOTOR_DIRECTION, 0x22, informationToSend, informationReceived, error))
  {
    return ((SOLOMotorControllers::Direction)soloUtils->ConvertToLong(informationReceived));
  }
  return SOLOMotorControllers::Direction::DIRECTION_ERROR;
}

/**
 * @brief  This command reads the value of Sensorless Zero Speed Full Torque Injection Amplitude
 *				.The method refers to the Object Dictionary: 0x3021
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval float between 0.01 to 1000
 */
float SOLOMotorControllersCanopenNative::GetZsftInjectionAmplitude(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_ZSFT_POLARITY_AMPLITUDE, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Zero Speed Full Torque Polarity Amplitude
        .The method refers to the Object Dictionary: 0x3022
  * @param[out]  error   pointer to an integer that specify result of function
  * @retval float between 0.0 to 0.55
  */
float SOLOMotorControllersCanopenNative::GetZsftPolarityAmplitude(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_ZSFT_POLARITY_AMPLITUDE, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetObserverGainDc(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_OBSERVER_GAIN_DC, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Zero Speed Full Torque Injection Frequency
        .The method refers to the Object Dictionary: 0x3024
  * @param[out]  error   pointer to an integer that specify result of function
  * @retval long between 0 to 10
  */
long SOLOMotorControllersCanopenNative::GetZsftInjectionFrequency(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_ZSFT_INJECTION_FREQUENCY, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Transition Speed
        .The method refers to the Object Dictionary: 0x3025
  * @param[out]  error   pointer to an integer that specify result of function
  * @retval long between 1 to 5000
  */
long SOLOMotorControllersCanopenNative::GetSensorlessTransitionSpeed(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_SENSORLESS_TRANSACTION_SPEED, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1.0;
}

/**
  * @brief  This command reads the measured or estimated per-unit angle of the 3-phase motors
        .The method refers to the Object Dictionary: 0x3038
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between -2 to 2
  */
float SOLOMotorControllersCanopenNative::Get3PhaseMotorAngle(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_3_PHASE_MOTOR_ANGLE, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetEncoderHallCcwOffset(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_ENCODER_HALL_CCW_OFFSET, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetEncoderHallCwOffset(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_ENCODER_HALL_CW_OFFSET, 0x22, informationToSend, informationReceived, error))
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
SOLOMotorControllers::UartBaudrate SOLOMotorControllersCanopenNative::GetUartBaudrate(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_UART_BAUDRATE, 0x22, informationToSend, informationReceived, error))
  {
    return ((SOLOMotorControllers::UartBaudrate)soloUtils->ConvertToLong(informationReceived));
  }
  return SOLOMotorControllers::UartBaudrate::UART_BAUDRATE_ERROR;
}

/**
  * @brief  This command reads the acceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds
        .The method refers to the Object Dictionary: 0x302A
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float between 0 to 1600 Rev/S^2
  */
float SOLOMotorControllersCanopenNative::GetSpeedAccelerationValue(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_SPEED_ACCELERATION_VALUE, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetSpeedDecelerationValue(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_SPEED_DECELERATION_VALUE, 0x22, informationToSend, informationReceived, error))
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
long SOLOMotorControllersCanopenNative::GetCanbusBaudrate(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_CANBUS_BAUDRATE, 0x22, informationToSend, informationReceived, error))
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
long SOLOMotorControllersCanopenNative::GetAnalogueSpeedResolutionDivisionCoefficient(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_ASRDC, 0x22, informationToSend, informationReceived, error))
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
bool SOLOMotorControllersCanopenNative::CommunicationIsWorking(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  float temperature = SOLOMotorControllersCanopenNative::GetBoardTemperature(error);
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
long SOLOMotorControllersCanopenNative::GetEncoderIndexCounts(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_ENCODER_INDEX_COUNTS, 0x22, informationToSend, informationReceived, error))
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
SOLOMotorControllers::MotionProfileMode SOLOMotorControllersCanopenNative::GetMotionProfileMode(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_MOTION_PROFILE_MODE, 0x22, informationToSend, informationReceived, error))
  {
    return ((SOLOMotorControllers::MotionProfileMode)soloUtils->ConvertToFloat(informationReceived));
  }
  return SOLOMotorControllers::MotionProfileMode::MOTION_PROFILE_MODE_ERROR;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable1 set inside the controller
        .The method refers to the Object Dictionary: 0x3040
  * @param[out]  error   optional pointer to an integer that specify result of function
  * @retval float
  */
float SOLOMotorControllersCanopenNative::GetMotionProfileVariable1(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_MOTION_PROFILE_VARIABLE1, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetMotionProfileVariable2(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_MOTION_PROFILE_VARIABLE2, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetMotionProfileVariable3(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_MOTION_PROFILE_VARIABLE3, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetMotionProfileVariable4(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_MOTION_PROFILE_VARIABLE4, 0x22, informationToSend, informationReceived, error))
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
float SOLOMotorControllersCanopenNative::GetMotionProfileVariable5(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_MOTION_PROFILE_VARIABLE5, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1;
}

long SOLOMotorControllersCanopenNative::GetDigitalOutputsRegister(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, true, OBJECT_DIGITAL_OUTPUT_REGISTER, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

// void SOLOMotorControllersCanopenNative::GenericCanbusRead(uint16_t *_ID, uint8_t *_DLC, uint8_t *_Data)
// {
//   *_ID = 0;
//   *_DLC = 0;
//   if ((_canbus->MCP2515_Read_RX_Status() & (1 << 6)))
//   {
//     _canbus->MCP2515_Receive_Frame(_canbus->Mcp2515RxBuffer::RX_BUFFER_0, _ID, _DLC, _Data);
//   }
// }
// void SOLOMotorControllersCanopenNative::GenericCanbusWrite(uint16_t _ID, uint8_t *_DLC, uint8_t *_Data, int &error)
// {
//   _canbus->MCP2515_Transmit_Frame(_canbus->Mcp2515TxBuffer::TX_BUFFER_0, _ID, *_DLC, _Data, error);
// }

SOLOMotorControllers::DigitalIoState SOLOMotorControllersCanopenNative::GetDigitalOutputState(Channel chaneel, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_DIGITAL_OUTPUT, 0x00, informationToSend, informationReceived, error))
  {
    return (DigitalIoState)informationReceived[3];
  }
  return SOLOMotorControllers::DigitalIoState::DIGITAL_IO_STATE_ERROR;
}
/**
 * @brief  This command reads the Digiatal Ouput pin Status. The method refers to the Object Dictionary: 0x3048
 * @param[out] pinNumber   specify the pin you want to controll. (Ensure your SOLO model support this functions)
 * @param[out] error   pointer to an integer that specify result of function
 * @retval int
 */
int SOLOMotorControllersCanopenNative::GetDigitalOutput(int pinNumber, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->DigitalInputValidation(pinNumber, error))
  {
    return -1;
  }

  uint8_t informationReceived = GetDigitalOutputState((Channel)pinNumber, error);
  if (error != SOLOMotorControllers::Error::NO_ERROR_DETECTED)
  {
    return -1;
  }

  uint8_t mask = 1 << pinNumber;
  return (informationReceived & mask) != 0;
}

/**
 * @brief  This command reads the current state of the controller
 *           .The method refers to the Uart Read command: 0x3008
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval enum @ref DisableEnable
 */
SOLOMotorControllers::DisableEnable SOLOMotorControllersCanopenNative::GetDriveDisableEnable(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_DRIVE_DISABLE_ENABLE, 0x22, informationToSend, informationReceived, error))
  {
    return ((SOLOMotorControllers::DisableEnable)soloUtils->ConvertToLong(informationReceived));
  }
  return SOLOMotorControllers::DisableEnable::DISABLE_ENABLE_ERROR;
}

/**
 * @brief  This command reads the value of the Regeneration Current Limit
 *           .The method refers to the Uart Read command: 0x304B
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval float
 */
float SOLOMotorControllersCanopenNative::GetRegenerationCurrentLimit(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_REGENERATION_CURRENT_LIMIT, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToFloat(informationReceived));
  }
  return -1;
}

/**
 * @brief  This command reads the value of the Position Sensor Digital Filter Level
 *           .The method refers to the Uart Read command: 0x304C
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersCanopenNative::GetPositionSensorDigitalFilterLevel(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_POSITION_SENSOR_DIGITAL_FILTER_LEVEL, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
 * @brief  This command reads the value of the Digital Input Register as a 32 bits register
 *           .The method refers to the Uart Read command: 0x3049
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersCanopenNative::GetDigitalInputRegister(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_DIGITAL_INPUT_REGISTER, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
 * @brief  This command reads the value of the voltage sensed at the output of PT1000 temperature
 *			sensor amplifier, this command can be used only on devices that come with PT1000 input
 *           .The method refers to the Uart Read command: 0x3047
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersCanopenNative::GetPT1000SensorVoltage(int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_PT1000_SENSOR_VOLTAGE, 0x22, informationToSend, informationReceived, error))
  {
    return (soloUtils->ConvertToLong(informationReceived));
  }
  return -1;
}

/**
 * @brief  This command reads the quantized value of an Analogue Input as a number between 0 to 4095
 *				depending on the maximum voltage input possible at the analogue inputs for the controller
 *           .The method refers to the Uart Read command: 0x304A
 * @param[in]  channel  an enum that specify the Channel of Analogue Input
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval enum @ref DigitalIoState
 */
SOLOMotorControllers::DigitalIoState SOLOMotorControllersCanopenNative::GetAnalogueInput(Channel channel, int &error)
{
  uint8_t informationToSend[4] = {0x00, 0x00, 0x00, (uint8_t)channel};
  uint8_t informationReceived[4] = {0x00, 0x00, 0x00, 0x00};
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (_canbus->CANOpenSdoTransmit(Address, false, OBJECT_ANALOGUE_INPUT, 0x22, informationToSend, informationReceived, error))
  {
    return ((SOLOMotorControllers::DigitalIoState)soloUtils->ConvertToLong(informationReceived));
  }
  return SOLOMotorControllers::DigitalIoState::DIGITAL_IO_STATE_ERROR;
}

/**
 * @brief  this PDO command give the first in the baffer position of the Motor
 *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses
 *				.The method refers to the Object Dictionary: 0x1814
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval long between -2,147,483,647 to 2,147,483,647 Quad-Pulses
 */
long SOLOMotorControllersCanopenNative::GetPdoPositionCountsFeedback(int &error)
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
long SOLOMotorControllersCanopenNative::GetPdoSpeedFeedback(int &error)
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
float SOLOMotorControllersCanopenNative::GetPdoQuadratureCurrentIqFeedback(int &error)
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
float SOLOMotorControllersCanopenNative::GetPdoMagnetizingCurrentIdFeedback(int &error)
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
long SOLOMotorControllersCanopenNative::GetPdoErrorRegister(int &error)
{
  return GetPdoParameterValueLong(PdoParameterName::ERROR_REGISTER, error);
}

/**
 * @brief  This PDO command reads the momentary temperature of the board in centigrade
 *				.The method refers to the Object Dictionary: 0x1819
 * @param[out]  error   optional pointer to an integer that specify result of function
 * @retval float between -30 to 150 Celsius
 */
float SOLOMotorControllersCanopenNative::GetPdoBoardTemperature(int &error)
{
  return GetPdoParameterValueFloat(PdoParameterName::BOARD_TEMPERATURE, error);
}

#endif // ARDUINO_PORTENTA_C33 ARDUINO_UNOWIFIR4 ARDUINO_MINIMA
