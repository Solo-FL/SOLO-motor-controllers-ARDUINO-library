/**
 *******************************************************************************
 * @file    SOLOMotorControllersUart.cpp
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the functions for the Solo Drivers
 *          uart communications.
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.1.0
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)
 *******************************************************************************
 */

#include "SOLOMotorControllersUart.h"
#include <stdint.h>

// -------------------- constructor & destructor --------------------
SOLOMotorControllersUart::SOLOMotorControllersUart(unsigned char _deviceAddress, HardwareSerial &_serial, SOLOMotorControllers::UartBaudrate _baudrate, long _millisecondsTimeout, int _packetFailureTrialAttempts)
    : addr(_deviceAddress), serialToUse(&_serial), millisecondsTimeout(_millisecondsTimeout), packetFailureTrialAttempts(_packetFailureTrialAttempts)
{
  switch (_baudrate)
  {
  case SOLOMotorControllers::UartBaudrate::RATE_937500:
    baudrate = 937500;
    break;
  case SOLOMotorControllers::UartBaudrate::RATE_115200:
    baudrate = 115200;
    break;
  default:
    baudrate = 115200;
    break;
  }
  
  //Some Arduino (UNO WIFI R4, MINIMA) have Tx0 and Tx1 at Serial1 not Serial
  #if defined(ARDUINO_UNOWIFIR4) || defined(ARDUINO_MINIMA) // For LEONARDO: || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
    if(_serial == Serial){
      serialToUse=&Serial1;
    }
  #endif

  serialToUse->begin(baudrate);
  serialToUse->setTimeout(millisecondsTimeout);
  soloUtils = new SOLOMotorControllersUtils();
  delay(1000);
}

bool SOLOMotorControllersUart::ExeCMD(unsigned char cmd[], int &error)
{
  unsigned char _cmd[] = {INITIATOR, INITIATOR, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], CRC, ENDING};
  unsigned char _readPacket[10] = {0};

  // DEBUG SENT
  //  Serial.println("packet SENT");
  //  char tmp[16];
  //  for (int i=0; i<10; i++) {
  //      sprintf(tmp, "0x%.2X",_cmd[i]);
  //      Serial.print(tmp); Serial.print(" ");
  //      }
  //  Serial.print("\n");

  bool isPACKET_FAILURE_TRIAL_ATTEMPTS_OVERFLOW = true;
  // FailureTrialAttempts block
  for (int attempts = 0; attempts < packetFailureTrialAttempts; attempts++)
  {

    // ensure buffer to be empty before new comunications
    if (serialToUse->available())
    {
      serialToUse->flush();
      serialToUse->readString();
    }

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

    // https://www.arduino.cc/reference/en/language/functions/communication/serial/
    serialToUse->write(_cmd, 10);
    serialToUse->flush();                                      // Wait for all outgoing data to be sent
    size_t returned = serialToUse->readBytes(_readPacket, 10); // read received data

    // DEBUG
    //  Serial.print("READ ");
    //  char tmp2[16];
    //  for (int r=0; r<10; r++) {
    //      sprintf(tmp2, "0x%.2X",_readPacket[r]);
    //      Serial.print(tmp2); Serial.print(" ");
    //  }

    // momentary situation in comunication or when missing connection
    // NOTE on serial1+ is possible to enfore this situation is not allowed 2 time in row (but happen more on shared serial)
    if (returned < 10)
    {
      attempts++;
      continue;
    }

    // error send async error
    if ((_readPacket[3] == 0xA1 && cmd[1] != 0xA1) || _readPacket[3] == 0xFE)
    {
      continue;
    }

    // packet response correctly
    if (_readPacket[0] == INITIATOR &&
        _readPacket[1] == INITIATOR &&
        _readPacket[3] != 0 &&
        _readPacket[8] == CRC && _readPacket[9] == ENDING)
    {
      isPACKET_FAILURE_TRIAL_ATTEMPTS_OVERFLOW = false;
      break;
    }
  }

  if (isPACKET_FAILURE_TRIAL_ATTEMPTS_OVERFLOW)
  {
    cmd[0] = ERROR;
    cmd[1] = ERROR;
    cmd[2] = ERROR;
    cmd[3] = ERROR;
    cmd[4] = ERROR;
    cmd[5] = ERROR;
    error = SOLOMotorControllers::Error::PACKET_FAILURE_TRIAL_ATTEMPTS_OVERFLOW;
    return false;
  }

  if (_readPacket[0] == _cmd[0] && _readPacket[1] == _cmd[1] && (_readPacket[2] == _cmd[2] || _cmd[2] == 0xFF) && _readPacket[3] == _cmd[3] && _readPacket[8] == _cmd[8] && _readPacket[9] == _cmd[9])
  {
    cmd[0] = _readPacket[2];
    cmd[1] = _readPacket[3];
    cmd[2] = _readPacket[4];
    cmd[3] = _readPacket[5];
    cmd[4] = _readPacket[6];
    cmd[5] = _readPacket[7];
    error = SOLOMotorControllers::Error::NO_ERROR_DETECTED;
    return true;
  }

  cmd[0] = ERROR;
  cmd[1] = ERROR;
  cmd[2] = ERROR;
  cmd[3] = ERROR;
  cmd[4] = ERROR;
  cmd[5] = ERROR;
  error = SOLOMotorControllers::Error::GENERAL_ERROR;
  return false;
}

/**
 * @brief  This command sets the desired device address for a SOLO unit
 *           .The method refers to the Uart Write command: 0x01
 * @param[in] deviceAddress  address want to set for board
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetDeviceAddress(unsigned char deviceAddress, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetDeviceAddressInputValidation(deviceAddress, error))
  {
    return false;
  }

  unsigned char cmd[] = {addr, WriteDeviceAddres, 0x00, 0x00, 0x00, deviceAddress};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the desired device address for a SOLO unit
 *           .The method refers to the Uart Write command: 0x01
 * @param[in] deviceAddress  address want to set for board
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetDeviceAddress(unsigned char deviceAddress)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetDeviceAddress(deviceAddress, error);
}

/**
 * @brief  This command sets the mode of the operation of SOLO
 *         in terms of operating in Analogue mode or Digital
 *           .The method refers to the Uart Write command: 0x02
 * @param[in] mode  enum that specify mode of the operation of SOLO
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetCommandMode(CommandMode mode, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char cmd[] = {addr, WriteCommandMode, 0x00, 0x00, 0x00, mode};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the mode of the operation of SOLO
 *         in terms of operating in Analogue mode or Digital
 *           .The method refers to the Uart Write command: 0x02
 * @param[in] mode  enum that specify mode of the operation of SOLO
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetCommandMode(CommandMode mode)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetCommandMode(mode, error);
}

/**
 * @brief  This command defines the maximum allowed current into the motor in terms of Amps
 *           .The method refers to the Uart Write command: 0x03
 * @param[in] currentLimit  a float value [Amps]
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetCurrentLimit(float currentLimit, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetCurrentLimitInputValidation(currentLimit, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(currentLimit, data);

  unsigned char cmd[] = {addr, WriteCurrentLimit, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the maximum allowed current into the motor in terms of Amps
 *           .The method refers to the Uart Write command: 0x03
 * @param[in] currentLimit  a float value [Amps]
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetCurrentLimit(float currentLimit)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetCurrentLimit(currentLimit, error);
}

/**
 * @brief  This command sets the amount of desired current that acts in torque generation
 *           .The method refers to the Uart Write command: 0x04
 * @param[in] torqueReferenceIq  a float [Amps]
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetTorqueReferenceIq(float torqueReferenceIq, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetTorqueReferenceIqInputValidation(torqueReferenceIq, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(torqueReferenceIq, data);

  unsigned char cmd[] = {addr, WriteTorqueReferenceIq, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the amount of desired current that acts in torque generation
 *           .The method refers to the Uart Write command: 0x04
 * @param[in] torqueReferenceIq  a float [Amps]
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetTorqueReferenceIq(float torqueReferenceIq)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetTorqueReferenceIq(torqueReferenceIq, error);
}

/**
 * @brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
 *           .The method refers to the Uart Write command: 0x05
 * @param[in] speedReference  a long value [RPM]
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetSpeedReference(long speedReference, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedReferenceInputValidation(speedReference, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(speedReference, data);

  unsigned char cmd[] = {addr, WriteSpeedReference, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
 *           .The method refers to the Uart Write command: 0x05
 * @param[in] speedReference  a long value [RPM]
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetSpeedReference(long speedReference)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetSpeedReference(speedReference, error);
}

/**
 * @brief  This command defines the amount of power percentage during only
 *         Open-loop mode for 3-phase motors
 *           .The method refers to the Uart Write command: 0x06
 * @param[in] powerReference  a float value between 0 to 100
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetPowerReference(float powerReference, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPowerReferenceInputValidation(powerReference, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(powerReference, data);

  unsigned char cmd[] = {addr, WritePowerReference, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the amount of power percentage during only
 *         Open-loop mode for 3-phase motors
 *           .The method refers to the Uart Write command: 0x06
 * @param[in] powerReference  a float value between 0 to 100
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetPowerReference(float powerReference)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetPowerReference(powerReference, error);
}

/**
  * @brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
            identifying the electrical parameters of the Motor connected
              .The method refers to the Uart Write command: 0x07
  * @param[in] powerReference  enum that specify Start or Stop of something in SOLO
  * @param[out] error   pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersUart::MotorParametersIdentification(Action identification, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char cmd[] = {addr, WriteMotorParametersIdentification, 0x00, 0x00, 0x00, identification};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
  * @brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
            identifying the electrical parameters of the Motor connected
              .The method refers to the Uart Write command: 0x07
  * @param[in] powerReference  enum that specify Start or Stop of something in SOLO
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersUart::MotorParametersIdentification(Action identification)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::MotorParametersIdentification(identification, error);
}

/**
  * @brief  This command if the DATA is set at zero will stop the whole power and switching system
            connected to the motor and it will cut the current floating into the Motor from SOLO
              .The method refers to the Uart Write command: 0x08
  * @param[out] error   pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersUart::EmergencyStop(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char cmd[] = {addr, WriteEmergencyStop, 0x00, 0x00, 0x00, 0x00};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
  * @brief  This command if the DATA is set at zero will stop the whole power and switching system
            connected to the motor and it will cut the current floating into the Motor from SOLO
              .The method refers to the Uart Write command: 0x08
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersUart::EmergencyStop()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::EmergencyStop(error);
}

/**
 * @brief  This command sets the output switching frequency of the whole power unit on the Motor
 *           .The method refers to the Uart Write command: 0x09
 * @param[in] outputPwmFrequencyKhz  switching frequencies [kHz]
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetOutputPwmFrequencyKhzInputValidation(outputPwmFrequencyKhz, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(outputPwmFrequencyKhz, data);

  unsigned char cmd[] = {addr, WriteOutputPwmFrequencyKhz, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the output switching frequency of the whole power unit on the Motor
 *           .The method refers to the Uart Write command: 0x09
 * @param[in] outputPwmFrequencyKhz  switching frequencies [kHz]
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetOutputPwmFrequencyKhz(outputPwmFrequencyKhz, error);
}

/**
 * @brief  This command sets the Speed controller Kp Gain, and it will
 *         be functional only in Digital Closed-loop mode
 *           .The method refers to the Uart Write command: 0x0A
 * @param[in] speedControllerKp  a float value between 0 to 300
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetSpeedControllerKp(float speedControllerKp, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedControllerKpInputValidation(speedControllerKp, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(speedControllerKp, data);

  unsigned char cmd[] = {addr, WriteSpeedControllerKp, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the Speed controller Kp Gain, and it will
 *         be functional only in Digital Closed-loop mode
 *           .The method refers to the Uart Write command: 0x0A
 * @param[in] speedControllerKp  a float value between 0 to 300
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetSpeedControllerKp(float speedControllerKp)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetSpeedControllerKp(speedControllerKp, error);
}

/**
 * @brief  This command sets the Speed controller Ki gain, and it will
 *         be functional only in Digital Closed-loop mode
 *           .The method refers to the Uart Write command: 0x0B
 * @param[in] speedControllerKi  a float value between 0 to 300
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetSpeedControllerKi(float speedControllerKi, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedControllerKiInputValidation(speedControllerKi, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(speedControllerKi, data);

  unsigned char cmd[] = {addr, WriteSpeedControllerKi, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the Speed controller Ki gain, and it will
 *         be functional only in Digital Closed-loop mode
 *           .The method refers to the Uart Write command: 0x0B
 * @param[in] speedControllerKi  a float value between 0 to 300
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetSpeedControllerKi(float speedControllerKi)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetSpeedControllerKi(speedControllerKi, error);
}

/**
 * @brief  This commands sets the direction of the rotation of the motor
 *         either to ClockWise rotation or to Counter Clockwise Rotation
 *           .The method refers to the Uart Write command: 0x0C
 * @param[in] motorDirection  enum that specify the direction of the rotation of the motor
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotorDirection(Direction motorDirection, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char cmd[] = {addr, WriteMotorDirection, 0x00, 0x00, 0x00, motorDirection};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This commands sets the direction of the rotation of the motor
 *         either to ClockWise rotation or to Counter Clockwise Rotation
 *           .The method refers to the Uart Write command: 0x0C
 * @param[in] motorDirection  enum that specify the direction of the rotation of the motor
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotorDirection(Direction motorDirection)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetMotorDirection(motorDirection, error);
}

/**
 * @brief  This command sets the amount of the Phase or Armature resistance
 *         for 3-phase or DC Brushed motors respectively
 *           .The method refers to the Uart Write command: 0x0D
 * @param[in] motorResistance  a float value [Ohm]
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotorResistance(float motorResistance, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotorResistanceInputValidation(motorResistance, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(motorResistance, data);

  unsigned char cmd[] = {addr, WriteMotorResistance, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the amount of the Phase or Armature resistance
 *         for 3-phase or DC Brushed motors respectively
 *           .The method refers to the Uart Write command: 0x0D
 * @param[in] motorResistance  a float value [Ohm]
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotorResistance(float motorResistance)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetMotorResistance(motorResistance, error);
}

/**
 * @brief  This command sets the amount of the Phase or Armature Inductance
 *         for 3-phase or DC Brushed motors respectively
 *           .The method refers to the Uart Write command: 0x0E
 * @param[in] motorInductance  a float value [Henry]
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotorInductance(float motorInductance, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotorInductanceInputValidation(motorInductance, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(motorInductance, data);

  unsigned char cmd[] = {addr, WriteMotorInductance, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the amount of the Phase or Armature Inductance
 *         for 3-phase or DC Brushed motors respectively
 *           .The method refers to the Uart Write command: 0x0E
 * @param[in] motorInductance  a float value [Henry]
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotorInductance(float motorInductance)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetMotorInductance(motorInductance, error);
}

/**
 * @brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
 *           .The method refers to the Uart Write command: 0x0F
 * @param[in] motorPolesCounts  a long value between 1 to 254
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotorPolesCounts(long motorPolesCounts, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotorPolesCountsInputValidation(motorPolesCounts, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(motorPolesCounts, data);

  unsigned char cmd[] = {addr, WriteMotorPolesCounts, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
 *           .The method refers to the Uart Write command: 0x0F
 * @param[in] motorPolesCounts  a long value between 1 to 254
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotorPolesCounts(long motorPolesCounts)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetMotorPolesCounts(motorPolesCounts, error);
}

/**
 * @brief  This command sets the pre-quad number of physical lines of an
 *         incremental encoder engraved on its disk
 *           .The method refers to the Uart Write command: 0x10
 * @param[in] incrementalEncoderLines  a long value [pre-quad]
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetIncrementalEncoderLines(long incrementalEncoderLines, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetIncrementalEncoderLinesInputValidation(incrementalEncoderLines, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(incrementalEncoderLines, data);

  unsigned char cmd[] = {addr, WriteIncrementalEncoderLines, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the pre-quad number of physical lines of an
 *         incremental encoder engraved on its disk
 *           .The method refers to the Uart Write command: 0x10
 * @param[in] incrementalEncoderLines  a long value [pre-quad]
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetIncrementalEncoderLines(long incrementalEncoderLines)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetIncrementalEncoderLines(incrementalEncoderLines, error);
}

/**
 * @brief  This command sets the allowed speed during trajectory following
 *         in closed-loop position controlling mode
 *           .The method refers to the Uart Write command: 0x11
 * @param[in] speedLimit  a long value [RPM]
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetSpeedLimit(long speedLimit, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedLimitInputValidation(speedLimit, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(speedLimit, data);

  unsigned char cmd[] = {addr, WriteSpeedLimit, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the allowed speed during trajectory following
 *         in closed-loop position controlling mode
 *           .The method refers to the Uart Write command: 0x11
 * @param[in] speedLimit  a long value [RPM]
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetSpeedLimit(long speedLimit)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetSpeedLimit(speedLimit, error);
}

/**
 * @brief  This command resets the device address of any connected SOLO to zero
 *           .The method refers to the Uart Write command: 0x12
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::ResetDeviceAddress(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char cmd[] = {0xFF, WriteDeviceAddres, 0x00, 0x00, 0x00, 0xFF};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command resets the device address of any connected SOLO to zero
 *           .The method refers to the Uart Write command: 0x12
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::ResetDeviceAddress()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::ResetDeviceAddress(error);
}

/**
 * @brief  This command sets the type of the feedback control SOLO has to operate
 *           .The method refers to the Uart Write command: 0x13
 * @param[in] mode  enum that specify the type of the feedback control SOLO
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetFeedbackControlMode(FeedbackControlMode mode, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char data[4];
  soloUtils->ConvertToData((long)mode, data);

  unsigned char cmd[] = {addr, WriteFeedbackControlMode, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the type of the feedback control SOLO has to operate
 *           .The method refers to the Uart Write command: 0x13
 * @param[in] mode  enum that specify the type of the feedback control SOLO
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetFeedbackControlMode(FeedbackControlMode mode)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetFeedbackControlMode(mode, error);
}

/**
 * @brief  This command resets SOLO to its factory setting to all the default parameters
 *           .The method refers to the Uart Write command: 0x14
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::ResetFactory(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char cmd[] = {addr, WriteResetFactory, 0x00, 0x00, 0x00, 0x01};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command resets SOLO to its factory setting to all the default parameters
 *           .The method refers to the Uart Write command: 0x14
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::ResetFactory()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::ResetFactory(error);
}

/**
 * @brief  This command sets the Motor type that is connected to SOLO in Digital Mode
 *           .The method refers to the Uart Write command: 0x15
 * @param[in] motorType  enum that specify the Motor type that is connected to SOLO in Digital Mode
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotorType(MotorType motorType, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char data[4];
  soloUtils->ConvertToData((long)motorType, data);

  unsigned char cmd[] = {addr, WriteMotorType, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the Motor type that is connected to SOLO in Digital Mode
 *           .The method refers to the Uart Write command: 0x15
 * @param[in] motorType  enum that specify the Motor type that is connected to SOLO in Digital Mode
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotorType(MotorType motorType)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetMotorType(motorType, error);
}

/**
 * @brief  This command sets the Control Mode in terms of Torque,
 *         Speed or Position only in Digital Mode
 *           .The method refers to the Uart Write command: 0x16
 * @param[in] controlMode  enum that specify the Control Mode in terms of Torque,
 *                       Speed or Position only in Digital Mode
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetControlMode(ControlMode controlMode, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char data[4];
  soloUtils->ConvertToData((long)controlMode, data);

  unsigned char cmd[] = {addr, WriteControlMode, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the Control Mode in terms of Torque,
 *         Speed or Position only in Digital Mode
 *           .The method refers to the Uart Write command: 0x16
 * @param[in] controlMode  enum that specify the Control Mode in terms of Torque,
 *                       Speed or Position only in Digital Mode
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetControlMode(ControlMode controlMode)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetControlMode(controlMode, error);
}

/**
 * @brief  This command sets the value for Current Controller Kp or proportional gain
 *           .The method refers to the Uart Write command: 0x17
 * @param[in] currentControllerKp  a float value between 0 to 16000
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetCurrentControllerKp(float currentControllerKp, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetCurrentControllerKpInputValidation(currentControllerKp, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(currentControllerKp, data);

  unsigned char cmd[] = {addr, WriteCurrentControllerKp, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the value for Current Controller Kp or proportional gain
 *           .The method refers to the Uart Write command: 0x17
 * @param[in] currentControllerKp  a float value between 0 to 16000
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetCurrentControllerKp(float currentControllerKp)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetCurrentControllerKp(currentControllerKp, error);
}

/**
 * @brief  This command sets the value for Current Controller Ki or integral gain
 *           .The method refers to the Uart Write command: 0x18
 * @param[in] motorInductance  a float value between 0 to 16000
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetCurrentControllerKi(float currentControllerKi, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetCurrentControllerKiInputValidation(currentControllerKi, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(currentControllerKi, data);

  unsigned char cmd[] = {addr, WriteCurrentControllerKi, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the value for Current Controller Ki or integral gain
 *           .The method refers to the Uart Write command: 0x18
 * @param[in] motorInductance  a float value between 0 to 16000
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetCurrentControllerKi(float currentControllerKi)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetCurrentControllerKi(currentControllerKi, error);
}

/**
 * @brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
 *         Weakening current reference to help the motor reaching speeds higher than
 *         nominal values and in case of AC Induction Motors Sets the desired magnetizing
 *         current (Id) required for controlling ACIM motors in FOC in Amps
 *           .The method refers to the Uart Write command: 0x1A
 * @param[in] magnetizingCurrentIdReference  a float value [Amps]
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMagnetizingCurrentIdReferenceInputValidation(magnetizingCurrentIdReference, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(magnetizingCurrentIdReference, data);

  unsigned char cmd[] = {addr, WriteMagnetizingCurrentIdReference, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
 *         Weakening current reference to help the motor reaching speeds higher than
 *         nominal values and in case of AC Induction Motors Sets the desired magnetizing
 *         current (Id) required for controlling ACIM motors in FOC in Amps
 *           .The method refers to the Uart Write command: 0x1A
 * @param[in] magnetizingCurrentIdReference  a float value [Amps]
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetMagnetizingCurrentIdReference(magnetizingCurrentIdReference, error);
}

/**
 * @brief  This command sets the desired Position reference in terms of quadrature
 *         pulses while SOLO operates with the Incremental Encoders or in terms of
 *         pulses while while SOLO operates with Hall sensors
 *           .The method refers to the Uart Write command: 0x1B
 * @param[in] positionReference  a long value [Quad-Pulse]
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetPositionReference(long positionReference, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPositionReferenceInputValidation(positionReference, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(positionReference, data);

  unsigned char cmd[] = {addr, WritePositionReference, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the desired Position reference in terms of quadrature
 *         pulses while SOLO operates with the Incremental Encoders or in terms of
 *         pulses while while SOLO operates with Hall sensors
 *           .The method refers to the Uart Write command: 0x1B
 * @param[in] positionReference  a long value [Quad-Pulse]
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetPositionReference(long positionReference)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetPositionReference(positionReference, error);
}

/**
 * @brief  This command sets the value for Position Controller Kp or proportional gain
 *           .The method refers to the Uart Write command: 0x1C
 * @param[in] positionControllerKp  a float value between 0 to 16000
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetPositionControllerKp(float positionControllerKp, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPositionControllerKpInputValidation(positionControllerKp, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(positionControllerKp, data);

  unsigned char cmd[] = {addr, WritePositionControllerKp, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the value for Position Controller Kp or proportional gain
 *           .The method refers to the Uart Write command: 0x1C
 * @param[in] positionControllerKp  a float value between 0 to 16000
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetPositionControllerKp(float positionControllerKp)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetPositionControllerKp(positionControllerKp, error);
}

/**
 * @brief  This command sets the value for Position Controller Ki or integrator gain
 *           .The method refers to the Uart Write command: 0x1D
 * @param[in] positionControllerKi  a float value between 0 to 16000
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetPositionControllerKi(float positionControllerKi, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPositionControllerKiInputValidation(positionControllerKi, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(positionControllerKi, data);

  unsigned char cmd[] = {addr, WritePositionControllerKi, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the value for Position Controller Ki or integrator gain
 *           .The method refers to the Uart Write command: 0x1D
 * @param[in] positionControllerKi  a float value between 0 to 16000
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetPositionControllerKi(float positionControllerKi)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetPositionControllerKi(positionControllerKi, error);
}

/**
 * @brief  This command overwrites the reported errors in Error Register
 *         reported with command code of "0xA1"
 *           .The method refers to the Uart Write command: 0x20
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::OverwriteErrorRegister(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char cmd[] = {addr, WriteOverwriteErrorRegister, 0x00, 0x00, 0x00, 0x00};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command overwrites the reported errors in Error Register
 *         reported with command code of "0xA1"
 *           .The method refers to the Uart Write command: 0x20
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::OverwriteErrorRegister()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::OverwriteErrorRegister(error);
}

/**
 * @brief  This command sets the observer gain for the Non-linear observer
 *         that estimates the speed and angle of a BLDC or PMSM once the
 *         motor type is selected as normal BLDC-PMSM
 *           .The method refers to the Uart Write command: 0x21
 * @param[in] observerGain  a float value between 0.01 to 1000
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
// SOG => Sensorless Observer Gain
bool SOLOMotorControllersUart::SetObserverGainBldcPmsm(float observerGain, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetObserverGainBldcPmsmInputValidation(observerGain, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(observerGain, data);

  unsigned char cmd[] = {addr, WriteObserverGainBldcPmsm, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the observer gain for the Non-linear observer
 *         that estimates the speed and angle of a BLDC or PMSM once the
 *         motor type is selected as normal BLDC-PMSM
 *           .The method refers to the Uart Write command: 0x21
 * @param[in] observerGain  a float value between 0.01 to 1000
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetObserverGainBldcPmsm(float observerGain)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetObserverGainBldcPmsm(observerGain, error);
}

/**
 * @brief  This command sets the observer gain for the Non-linear observer that
 *         estimates the speed and angle of a BLDC or PMSM once the motor type
 *         is selected as ultra-fast BLDC-PMSM
 *           .The method refers to the Uart Write command: 0x22
 * @param[in] observerGain  a float value between 0.01 to 1000
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetObserverGainBldcPmsmUltrafast(float observerGain, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetObserverGainBldcPmsmUltrafastInputValidation(observerGain, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(observerGain, data);

  unsigned char cmd[] = {addr, WriteObserverGainBldcPmsmUltrafast, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the observer gain for the Non-linear observer that
 *         estimates the speed and angle of a BLDC or PMSM once the motor type
 *         is selected as ultra-fast BLDC-PMSM
 *           .The method refers to the Uart Write command: 0x22
 * @param[in] observerGain  a float value between 0.01 to 1000
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetObserverGainBldcPmsmUltrafast(float observerGain)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetObserverGainBldcPmsmUltrafast(observerGain, error);
}

/**
 * @brief  This command sets the observer gain for the Non-linear observer
 *         that estimates the speed of a DC brushed once the motor type
 *         is selected as DC brushed
 *           .The method refers to the Uart Write command: 0x23
 * @param[in] observerGain  a float value between 0.01 to 1000
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetObserverGainDc(float observerGain, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetObserverGainDcInputValidation(observerGain, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(observerGain, data);

  unsigned char cmd[] = {addr, WriteObserverGainDc, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the observer gain for the Non-linear observer
 *         that estimates the speed of a DC brushed once the motor type
 *         is selected as DC brushed
 *           .The method refers to the Uart Write command: 0x23
 * @param[in] observerGain  a float value between 0.01 to 1000
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetObserverGainDc(float observerGain)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetObserverGainDc(observerGain, error);
}

/**
 * @brief  This command sets how fast the observer should operate once
 *         SOLO is in sensorless mode with normal BLDC-PMSM selected as the Motor type
 *           .The method refers to the Uart Write command: 0x24
 * @param[in] filterGain  a float value between 0.01 to 16000
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
// SOFG => Sensorless Observer Filter Gain
bool SOLOMotorControllersUart::SetFilterGainBldcPmsm(float filterGain, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetFilterGainBldcPmsmInputValidation(filterGain, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(filterGain, data);

  unsigned char cmd[] = {addr, WriteFilterGainBldcPmsm, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets how fast the observer should operate once
 *         SOLO is in sensorless mode with normal BLDC-PMSM selected as the Motor type
 *           .The method refers to the Uart Write command: 0x24
 * @param[in] filterGain  a float value between 0.01 to 16000
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetFilterGainBldcPmsm(float filterGain)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetFilterGainBldcPmsm(filterGain, error);
}

/**
 * @brief  This command sets how fast the observer should operate once SOLO
 *         is in sensorless mode with ultra-fast BLDC-PMSM selected as the Motor type
 *           .The method refers to the Uart Write command: 0x25
 * @param[in] filterGain  a float value between 0.01 to 16000
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetFilterGainBldcPmsmUltrafast(float filterGain, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetFilterGainBldcPmsmUltrafastInputValidation(filterGain, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(filterGain, data);

  unsigned char cmd[] = {addr, WriteFilterGainBldcPmsmUltrafast, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets how fast the observer should operate once SOLO
 *         is in sensorless mode with ultra-fast BLDC-PMSM selected as the Motor type
 *           .The method refers to the Uart Write command: 0x25
 * @param[in] filterGain  a float value between 0.01 to 16000
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetFilterGainBldcPmsmUltrafast(float filterGain)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetFilterGainBldcPmsmUltrafast(filterGain, error);
}

/**
 * @brief  This command sets the baud-rate of the UART line
 *           .The method refers to the Uart Write command: 0x26
 * @param[in] baudrate  enum that specify the baud-rate of the UART line
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetUartBaudrate(UartBaudrate baudrate, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char data[4];
  soloUtils->ConvertToData((long)baudrate, data);

  unsigned char cmd[] = {addr, WriteUartBaudrate, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the baud-rate of the UART line
 *           .The method refers to the Uart Write command: 0x26
 * @param[in] baudrate  enum that specify the baud-rate of the UART line
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetUartBaudrate(UartBaudrate baudrate)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetUartBaudrate(baudrate, error);
}

/**
 * @brief  This command starts or stops the process of sensor calibration
 *           .The method refers to the Uart Write command: 0x27
 * @param[in] calibrationAction  enum that specify the process of sensor calibration
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SensorCalibration(PositionSensorCalibrationAction calibrationAction, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char data[4];
  soloUtils->ConvertToData((long)calibrationAction, data);

  unsigned char cmd[] = {addr, WriteSensorCalibration, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command starts or stops the process of sensor calibration
 *           .The method refers to the Uart Write command: 0x27
 * @param[in] calibrationAction  enum that specify the process of sensor calibration
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SensorCalibration(PositionSensorCalibrationAction calibrationAction)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SensorCalibration(calibrationAction, error);
}

/**
 * @brief  This command sets the per-unit offset identified after sensor calibration
 *         for Encoder or Hall sensors in C.C.W direction
 *           .The method refers to the Uart Write command: 0x28
 * @param[in] encoderHallOffset  a float value between 0.0 to 1.0
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetEncoderHallCcwOffset(float encoderHallOffset, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetEncoderHallCcwOffsetInputValidation(encoderHallOffset, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(encoderHallOffset, data);

  unsigned char cmd[] = {addr, WriteEncoderHallCcwOffset, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the per-unit offset identified after sensor calibration
 *         for Encoder or Hall sensors in C.C.W direction
 *           .The method refers to the Uart Write command: 0x28
 * @param[in] encoderHallOffset  a float value between 0.0 to 1.0
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetEncoderHallCcwOffset(float encoderHallOffset)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetEncoderHallCcwOffset(encoderHallOffset, error);
}

/**
 * @brief  This command sets the per-unit offset identified after sensor calibration
 *         for Encoder or Hall sensors in C.W direction
 *           .The method refers to the Uart Write command: 0x29
 * @param[in] encoderHallOffset  a float value between 0.0 to 1.0
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetEncoderHallCwOffset(float encoderHallOffset, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetEncoderHallCwOffsetInputValidation(encoderHallOffset, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(encoderHallOffset, data);

  unsigned char cmd[] = {addr, WriteEncoderHallCwOffset, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the per-unit offset identified after sensor calibration
 *         for Encoder or Hall sensors in C.W direction
 *           .The method refers to the Uart Write command: 0x29
 * @param[in] encoderHallOffset  a float value between 0.0 to 1.0
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetEncoderHallCwOffset(float encoderHallOffset)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetEncoderHallCwOffset(encoderHallOffset, error);
}

/**
 * @brief  This command defines the acceleration value of the Speed for speed controller
 *         both in Analogue and Digital modes in Revolution per square seconds
 *           .The method refers to the Uart Write command: 0x2A
 * @param[in] speedAccelerationValue  a float value [Rev/S^2]
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetSpeedAccelerationValue(float speedAccelerationValue, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedAccelerationValueInputValidation(speedAccelerationValue, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(speedAccelerationValue, data);

  unsigned char cmd[] = {addr, WriteSpeedAccelerationValue, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the acceleration value of the Speed for speed controller
 *         both in Analogue and Digital modes in Revolution per square seconds
 *           .The method refers to the Uart Write command: 0x2A
 * @param[in] speedAccelerationValue  a float value [Rev/S^2]
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetSpeedAccelerationValue(float speedAccelerationValue)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetSpeedAccelerationValue(speedAccelerationValue, error);
}

/**
 * @brief  This command defines the deceleration value of the Speed for speed controller
 *         both in Analogue and Digital modes in Revolution per square seconds
 *           .The method refers to the Uart Write command: 0x2B
 * @param[in] speedDecelerationValue  a float value [Rev/S^2]
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetSpeedDecelerationValue(float speedDecelerationValue, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSpeedDecelerationValueInputValidation(speedDecelerationValue, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(speedDecelerationValue, data);

  unsigned char cmd[] = {addr, WriteSpeedDecelerationValue, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the deceleration value of the Speed for speed controller
 *         both in Analogue and Digital modes in Revolution per square seconds
 *           .The method refers to the Uart Write command: 0x2B
 * @param[in] speedDecelerationValue  a float value [Rev/S^2]
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetSpeedDecelerationValue(float speedDecelerationValue)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetSpeedDecelerationValue(speedDecelerationValue, error);
}

/**
 * @brief  This command sets the baud rate of CAN bus in CANOpen network
 *           .The method refers to the Uart Write command: 0x2C
 * @param[in] canbusBaudrate  enum that specify the baud rate of CAN bus in CANOpen network
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetCanbusBaudrate(CanbusBaudrate canbusBaudrate, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char data[4];
  soloUtils->ConvertToData((long)canbusBaudrate, data);

  unsigned char cmd[] = {addr, WriteUartBaudrate, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the baud rate of CAN bus in CANOpen network
 *           .The method refers to the Uart Write command: 0x2C
 * @param[in] canbusBaudrate  enum that specify the baud rate of CAN bus in CANOpen network
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetCanbusBaudrate(CanbusBaudrate canbusBaudrate)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetCanbusBaudrate(canbusBaudrate, error);
}

/**
 * @brief  This command defines the resolution of the speed at S/T input
 *           while SOLO operates in Analogue mode
 *           .The method refers to the Uart Write command: 0x2D
 * @param[in] divisionCoefficient  a long value
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetAnalogueSpeedResolutionDivisionCoefficientInputValidation(divisionCoefficient, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(divisionCoefficient, data);

  unsigned char cmd[] = {addr, WriteASRDC, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the resolution of the speed at S/T input
 *           while SOLO operates in Analogue mode
 *           .The method refers to the Uart Write command: 0x2D
 * @param[in] divisionCoefficient  a long value
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetAnalogueSpeedResolutionDivisionCoefficient(divisionCoefficient, error);
}

/**
 * @brief  This command defines the type of the Motion Profile that is
 *           being used in Speed or Position Modes
 *           .The method refers to the Uart Write command: 0x30
 * @param[in] motionProfileMode enum that specify the type of the Motion Profile
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotionProfileMode(MotionProfileMode motionProfileMode, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char data[4];
  soloUtils->ConvertToData((long)motionProfileMode, data);

  unsigned char cmd[] = {addr, WriteMotionProfileMode, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the type of the Motion Profile that is
 *           being used in Speed or Position Modes
 *           .The method refers to the Uart Write command: 0x30
 * @param[in] motionProfileMode enum that specify the type of the Motion Profile
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotionProfileMode(MotionProfileMode motionProfileMode)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetMotionProfileMode(motionProfileMode, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x31
 * @param[in] MotionProfileVariable1 a float value
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotionProfileVariable1(float MotionProfileVariable1, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable1InputValidation(MotionProfileVariable1, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(MotionProfileVariable1, data);

  unsigned char cmd[] = {addr, WriteMotionProfileVariable1, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x31
 * @param[in] MotionProfileVariable1 a float value
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotionProfileVariable1(float MotionProfileVariable1)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetMotionProfileVariable1(MotionProfileVariable1, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x32
 * @param[in] MotionProfileVariable2 a float value
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotionProfileVariable2(float MotionProfileVariable2, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable2InputValidation(MotionProfileVariable2, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(MotionProfileVariable2, data);

  unsigned char cmd[] = {addr, WriteMotionProfileVariable2, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x32
 * @param[in] MotionProfileVariable2 a float value
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotionProfileVariable2(float MotionProfileVariable2)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetMotionProfileVariable2(MotionProfileVariable2, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x33
 * @param[in] MotionProfileVariable3 a float value
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotionProfileVariable3(float MotionProfileVariable3, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable3InputValidation(MotionProfileVariable3, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(MotionProfileVariable3, data);

  unsigned char cmd[] = {addr, WriteMotionProfileVariable3, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x31
 * @param[in] MotionProfileVariable3 a float value
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotionProfileVariable3(float MotionProfileVariable3)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetMotionProfileVariable3(MotionProfileVariable3, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x34
 * @param[in] MotionProfileVariable4 a float value
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotionProfileVariable4(float MotionProfileVariable4, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable4InputValidation(MotionProfileVariable4, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(MotionProfileVariable4, data);

  unsigned char cmd[] = {addr, WriteMotionProfileVariable4, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x34
 * @param[in] MotionProfileVariable4 a float value
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotionProfileVariable4(float MotionProfileVariable4)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetMotionProfileVariable4(MotionProfileVariable4, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x35
 * @param[in] MotionProfileVariable5 a float value
 * @param[out] error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotionProfileVariable5(float MotionProfileVariable5, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetMotionProfileVariable5InputValidation(MotionProfileVariable5, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(MotionProfileVariable5, data);

  unsigned char cmd[] = {addr, WriteMotionProfileVariable5, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x35
 * @param[in] MotionProfileVariable5 a float value
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotionProfileVariable5(float MotionProfileVariable5)
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::SetMotionProfileVariable5(MotionProfileVariable5, error);
}

//----------Read----------

/**
 * @brief  This command reads the device address connected on the line
 *           .The method refers to the Uart Read command: 0x81
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long device address connected on the line
 */
long SOLOMotorControllersUart::GetDeviceAddress(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {0xFF, ReadDeviceAddress, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the device address connected on the line
 *           .The method refers to the Uart Read command: 0x81
 * @retval long device address connected on the line
 */
long SOLOMotorControllersUart::GetDeviceAddress()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetDeviceAddress(error);
}

/**
 * @brief  This command reads the phase-A voltage of the motor connected to the
 *         "A" pin output of SOLO for 3-phase Motors
 *           .The method refers to the Uart Read command: 0x82
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float phase-A voltage of the motor [Volts]
 */
float SOLOMotorControllersUart::GetPhaseAVoltage(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadPhaseAVoltage, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the phase-A voltage of the motor connected to the
 *         "A" pin output of SOLO for 3-phase Motors
 *           .The method refers to the Uart Read command: 0x82
 * @retval float phase-A voltage of the motor [Volts]
 */
float SOLOMotorControllersUart::GetPhaseAVoltage()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetPhaseAVoltage(error);
}

/**
 * @brief  This command reads the phase-B voltage of the motor connected to the
 *         "B" pin output of SOLO for 3-phase Motors
 *           .The method refers to the Uart Read command: 0x83
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float 0 phase-A voltage of the motor [Volts]
 */
float SOLOMotorControllersUart::GetPhaseBVoltage(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadPhaseBVoltage, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the phase-B voltage of the motor connected to the
 *         "B" pin output of SOLO for 3-phase Motors
 *           .The method refers to the Uart Read command: 0x83
 * @retval float 0 phase-A voltage of the motor [Volts]
 */
float SOLOMotorControllersUart::GetPhaseBVoltage()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetPhaseBVoltage(error);
}

/**
 * @brief  This command reads the phase-A current of the motor connected to the
 *         "A" pin output of SOLO for 3-phase Motors
 *           .The method refers to the Uart Read command: 0x84
 * @param[out] error   pointer to an integer that specify result of function
 * @retval phase-A current of the motor [Amps]
 */
float SOLOMotorControllersUart::GetPhaseACurrent(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadPhaseACurrent, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the phase-A current of the motor connected to the
 *         "A" pin output of SOLO for 3-phase Motors
 *           .The method refers to the Uart Read command: 0x84
 * @retval float phase-A current of the motor [Amps]
 */
float SOLOMotorControllersUart::GetPhaseACurrent()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetPhaseACurrent(error);
}

/**
 * @brief  This command reads the phase-B current of the motor connected to the
 *         "B" pin output of SOLO for 3-phase Motors
 *           .The method refers to the Uart Read command: 0x85
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float phase-B current of the motor [Amps]
 */
float SOLOMotorControllersUart::GetPhaseBCurrent(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadPhaseBCurrent, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the phase-B current of the motor connected to the
 *         "B" pin output of SOLO for 3-phase Motors
 *           .The method refers to the Uart Read command: 0x85
 * @retval float phase-B current of the motor [Amps]
 */
float SOLOMotorControllersUart::GetPhaseBCurrent()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetPhaseBCurrent(error);
}

/**
 * @brief  This command reads the input BUS voltage
 *           .The method refers to the Uart Read command: 0x86
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float  BUS voltage [Volts]
 */
// Battery Voltage
float SOLOMotorControllersUart::GetBusVoltage(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadBusVoltage, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the input BUS voltage
 *           .The method refers to the Uart Read command: 0x86
 * @retval float  BUS voltage [Volts]
 */
float SOLOMotorControllersUart::GetBusVoltage()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetBusVoltage(error);
}

/**
 * @brief  This command reads the current inside the DC brushed motor connected to
 *         "B" and "C" outputs of SOLO
 *           .The method refers to the Uart Read command: 0x87
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float between [Amps]
 */
float SOLOMotorControllersUart::GetDcMotorCurrentIm(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadDcMotorCurrentIm, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the current inside the DC brushed motor connected to
 *         "B" and "C" outputs of SOLO
 *           .The method refers to the Uart Read command: 0x87
 * @retval float between [Amps]
 */
float SOLOMotorControllersUart::GetDcMotorCurrentIm()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetDcMotorCurrentIm(error);
}

/**
 * @brief  This command reads the voltage of the DC brushed motor connected to
 *         "B" and "C" outputs of SOLO
 *           .The method refers to the Uart Read command: 0x88
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [Volts]
 */
float SOLOMotorControllersUart::GetDcMotorVoltageVm(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadDcMotorVoltageVm, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the voltage of the DC brushed motor connected to
 *         "B" and "C" outputs of SOLO
 *           .The method refers to the Uart Read command: 0x88
 * @retval float [Volts]
 */
float SOLOMotorControllersUart::GetDcMotorVoltageVm()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetDcMotorVoltageVm(error);
}

/**
 * @brief  This command reads the value of the Speed controller Kp gain,
 *         set for Digital mode operations
 *           .The method refers to the Uart Read command: 0x89
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetSpeedControllerKp(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadSpeedControllerKp, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of the Speed controller Kp gain,
 *         set for Digital mode operations
 *           .The method refers to the Uart Read command: 0x89
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetSpeedControllerKp()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetSpeedControllerKp(error);
}

/**
 * @brief  This command reads the value of the Speed controller Ki gain,
 *         set for Digital mode operations
 *           .The method refers to the Uart Read command: 0x8A
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetSpeedControllerKi(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadSpeedControllerKi, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of the Speed controller Ki gain,
 *         set for Digital mode operations
 *           .The method refers to the Uart Read command: 0x8A
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetSpeedControllerKi()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetSpeedControllerKi(error);
}

/**
 * @brief  This command reads the output switching frequency of SOLO in Hertz
 *           .The method refers to the Uart Read command: 0x8B
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long [Hz]
 */
long SOLOMotorControllersUart::GetOutputPwmFrequencyKhz(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadOutputPwmFrequencyHz, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return (soloUtils->ConvertToLong(data) / 1000L); // PWM reading is in Hz
  }
  return -1;
}

/**
 * @brief  This command reads the output switching frequency of SOLO in Hertz
 *           .The method refers to the Uart Read command: 0x8B
 * @retval long [Hz]
 */
long SOLOMotorControllersUart::GetOutputPwmFrequencyKhz()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetOutputPwmFrequencyKhz(error);
}

/**
 * @brief  This command reads the value of the current limit set for SOLO in
 *         closed-loop digital operation mode
 *           .The method refers to the Uart Read command: 0x8C
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetCurrentLimit(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadCurrentLimit, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of the current limit set for SOLO in
 *         closed-loop digital operation mode
 *           .The method refers to the Uart Read command: 0x8C
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetCurrentLimit()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetCurrentLimit(error);
}

/**
 * @brief  This command reads the actual monetary value of “Iq” that is
 *         the current acts in torque generation in FOC mode for 3-phase motors
 *           .The method refers to the Uart Read command: 0x8D
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetQuadratureCurrentIqFeedback(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadQuadratureCurrentIqFeedback, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the actual monetary value of “Iq” that is
 *         the current acts in torque generation in FOC mode for 3-phase motors
 *           .The method refers to the Uart Read command: 0x8D
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetQuadratureCurrentIqFeedback()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetQuadratureCurrentIqFeedback(error);
}

/**
 * @brief  This command reads the actual monetary value of Id that is the
 *         direct current acting in FOC
 *           .The method refers to the Uart Read command: 0x8E
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetMagnetizingCurrentIdFeedback(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadMagnetizingCurrentIdFeedback, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the actual monetary value of Id that is the
 *         direct current acting in FOC
 *           .The method refers to the Uart Read command: 0x8E
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetMagnetizingCurrentIdFeedback()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetMagnetizingCurrentIdFeedback(error);
}

/**
 * @brief  This command reads the number of Poles set for 3-phase motors
 *           .The method refers to the Uart Read command: 0x8F
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long between 1 to 254
 */
long SOLOMotorControllersUart::GetMotorPolesCounts(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadMotorPolesCounts, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the number of Poles set for 3-phase motors
 *           .The method refers to the Uart Read command: 0x8F
 * @retval long between 1 to 254
 */
long SOLOMotorControllersUart::GetMotorPolesCounts()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetMotorPolesCounts(error);
}

/**
 * @brief  This command reads the number of physical Incremental encoder lines set on SOLO
 *           .The method refers to the Uart Read command: 0x90
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long between 1 to 200000
 */
long SOLOMotorControllersUart::GetIncrementalEncoderLines(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadIncrementalEncoderLines, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the number of physical Incremental encoder lines set on SOLO
 *           .The method refers to the Uart Read command: 0x90
 * @retval long between 1 to 200000
 */
long SOLOMotorControllersUart::GetIncrementalEncoderLines()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetIncrementalEncoderLines(error);
}

/**
 * @brief  This command reads the amount of value set for Current controller
 *         Kp or proportional gain
 *           .The method refers to the Uart Read command: 0x91
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetCurrentControllerKp(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadCurrentControllerKp, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the amount of value set for Current controller
 *         Kp or proportional gain
 *           .The method refers to the Uart Read command: 0x91
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetCurrentControllerKp()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetCurrentControllerKp(error);
}

/**
 * @brief  This command reads the amount of value set for Current controller
 *         Ki or integrator gain
 *           .The method refers to the Uart Read command: 0x92
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetCurrentControllerKi(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadCurrentControllerKi, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data) * 0.00005;
  }
  return -1;
}

/**
 * @brief  This command reads the amount of value set for Current controller
 *         Ki or integrator gain
 *           .The method refers to the Uart Read command: 0x92
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetCurrentControllerKi()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetCurrentControllerKi(error);
}

/**
 * @brief  This command reads the momentary temperature of the board in centigrade
 *           .The method refers to the Uart Read command: 0x93
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [°C]
 */
float SOLOMotorControllersUart::GetBoardTemperature(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadBoardTemperature, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the momentary temperature of the board in centigrade
 *           .The method refers to the Uart Read command: 0x93
 * @retval float [°C]
 */
float SOLOMotorControllersUart::GetBoardTemperature()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetBoardTemperature(error);
}

/**
 * @brief  This command reads the Phase or Armature resistance of
 *         the 3-phase or DC brushed motor connected to SOLO respectively
 *           .The method refers to the Uart Read command: 0x94
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [Ohms]
 */
float SOLOMotorControllersUart::GetMotorResistance(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadMotorResistance, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the Phase or Armature resistance of
 *         the 3-phase or DC brushed motor connected to SOLO respectively
 *           .The method refers to the Uart Read command: 0x94
 * @retval float [Ohms]
 */
float SOLOMotorControllersUart::GetMotorResistance()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetMotorResistance(error);
}

/**
 * @brief  This command reads the Phase or Armature Inductance of
 *         the 3-phase or DC brushed motor connected to SOLO respectively
 *           .The method refers to the Uart Read command: 0x95
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [Henry]
 */
float SOLOMotorControllersUart::GetMotorInductance(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadMotorInductance, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the Phase or Armature Inductance of
 *         the 3-phase or DC brushed motor connected to SOLO respectively
 *           .The method refers to the Uart Read command: 0x95
 * @retval float [Henry]
 */
float SOLOMotorControllersUart::GetMotorInductance()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetMotorInductance(error);
}

/**
  * @brief  his command reads the actual speed of the motor measured or estimated by SOLO in
            sensorless or sensor-based modes respectively
              .The method refers to the Uart Read command: 0x96
  * @param[out] error   pointer to an integer that specify result of function
  * @retval long [RPM]
  */
long SOLOMotorControllersUart::GetSpeedFeedback(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadSpeedFeedback, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
  * @brief  his command reads the actual speed of the motor measured or estimated by SOLO in
            sensorless or sensor-based modes respectively
              .The method refers to the Uart Read command: 0x96
  * @retval long [RPM]
  */
long SOLOMotorControllersUart::GetSpeedFeedback()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetSpeedFeedback(error);
}

/**
 * @brief  This command reads the Motor type selected for Digital or Analogue mode operations
 *           .The method refers to the Uart Read command: 0x97
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long between 0 to 3
 */
long SOLOMotorControllersUart::GetMotorType(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadMotorType, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the Motor type selected for Digital or Analogue mode operations
 *           .The method refers to the Uart Read command: 0x97
 * @retval long between 0 to 3
 */
long SOLOMotorControllersUart::GetMotorType()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetMotorType(error);
}

/**
 * @brief  This command reads the feedback control mode selected on SOLO both
 *         for Analogue and Digital operations
 *           .The method refers to the Uart Read command: 0x99
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long between 0 to 2
 */
long SOLOMotorControllersUart::GetFeedbackControlMode(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadFeedbackControlMode, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the feedback control mode selected on SOLO both
 *         for Analogue and Digital operations
 *           .The method refers to the Uart Read command: 0x99
 * @retval long between 0 to 2
 */
long SOLOMotorControllersUart::GetFeedbackControlMode()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetFeedbackControlMode(error);
}

/**
 * @brief  This command reads the actual commanding mode that SOLO is operating
 *           .The method refers to the Uart Read command: 0x9A
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long between 0 or 1
 */
long SOLOMotorControllersUart::GetCommandMode(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadCommandMode, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the actual commanding mode that SOLO is operating
 *           .The method refers to the Uart Read command: 0x9A
 * @retval long between 0 or 1
 */
long SOLOMotorControllersUart::GetCommandMode()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetCommandMode(error);
}

/**
 * @brief  This command reads the Control Mode type in terms of Torque,
 *         Speed or Position in both Digital and Analogue modes
 *           .The method refers to the Uart Read command: 0x9B
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long between 0 to 2
 */
long SOLOMotorControllersUart::GetControlMode(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadControlMode, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the Control Mode type in terms of Torque,
 *         Speed or Position in both Digital and Analogue modes
 *           .The method refers to the Uart Read command: 0x9B
 * @retval long between 0 to 2
 */
long SOLOMotorControllersUart::GetControlMode()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetControlMode(error);
}

/**
 * @brief  This command reads the value of the speed limit set on SOLO
 *           .The method refers to the Uart Read command: 0x9C
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long [RPM]
 */
long SOLOMotorControllersUart::GetSpeedLimit(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadSpeedLimit, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of the speed limit set on SOLO
 *           .The method refers to the Uart Read command: 0x9C
 * @retval long [RPM]
 */
long SOLOMotorControllersUart::GetSpeedLimit()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetSpeedLimit(error);
}

/**
 * @brief  This command reads the amount of value set for Position
 *         controller Kp or proportional gain
 *           .The method refers to the Uart Read command: 0x9D
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetPositionControllerKp(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadPositionControllerKp, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the amount of value set for Position
 *         controller Kp or proportional gain
 *           .The method refers to the Uart Read command: 0x9D
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetPositionControllerKp()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetPositionControllerKp(error);
}

/**
 * @brief  This command reads the amount of value set for Position
 *         controller Ki or integrator gain
 *           .The method refers to the Uart Read command: 0x9E
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetPositionControllerKi(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadPositionControllerKi, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the amount of value set for Position
 *         controller Ki or integrator gain
 *           .The method refers to the Uart Read command: 0x9E
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetPositionControllerKi()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetPositionControllerKi(error);
}

/**
 * @brief  This command reads the number of counted pulses from the
 *         Incremental Encoder or Hall sensors
 *           .The method refers to the Uart Read command: 0xA0
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long [Quad-Pulses]
 */
long SOLOMotorControllersUart::GetPositionCountsFeedback(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadPositionCountsFeedback, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the number of counted pulses from the
 *         Incremental Encoder or Hall sensors
 *           .The method refers to the Uart Read command: 0xA0
 * @retval long [Quad-Pulses]
 */
long SOLOMotorControllersUart::GetPositionCountsFeedback()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetPositionCountsFeedback(error);
}

/**
 * @brief  This command reads the error register which is a 32 bit register with
 *         each bit corresponding to specific errors
 *           .The method refers to the Uart Read command: 0xA1
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersUart::GetErrorRegister(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadErrorRegister, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the error register which is a 32 bit register with
 *         each bit corresponding to specific errors
 *           .The method refers to the Uart Read command: 0xA1
 * @retval long
 */
long SOLOMotorControllersUart::GetErrorRegister()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetErrorRegister(error);
}

/**
 * @brief  This command reads the Firmware version existing currently on the SOLO unit
 *           .The method refers to the Uart Read command: 0xA2
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersUart::GetDeviceFirmwareVersion(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadDeviceFirmwareVersion, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the Firmware version existing currently on the SOLO unit
 *           .The method refers to the Uart Read command: 0xA2
 * @retval long
 */
long SOLOMotorControllersUart::GetDeviceFirmwareVersion()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetDeviceFirmwareVersion(error);
}

/**
 * @brief  This command reads the Hardware version of the SOLO unit connected
 *           .The method refers to the Uart Read command: 0xA3
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersUart::GetDeviceHardwareVersion(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadDeviceHardwareVersion, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the Hardware version of the SOLO unit connected
 *           .The method refers to the Uart Read command: 0xA3
 * @retval long
 */
long SOLOMotorControllersUart::GetDeviceHardwareVersion()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetDeviceHardwareVersion(error);
}

/**
 * @brief  This command reads the amount of desired Torque reference (Iq or IM)
 *         already set for the Motor to follow in Digital Closed-loop Torque control mode
 *           .The method refers to the Uart Read command: 0xA4
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetTorqueReferenceIq(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadTorqueReferenceIq, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the amount of desired Torque reference (Iq or IM)
 *         already set for the Motor to follow in Digital Closed-loop Torque control mode
 *           .The method refers to the Uart Read command: 0xA4
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetTorqueReferenceIq()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetTorqueReferenceIq(error);
}

/**
 * @brief  This command reads the amount of desired Speed reference already set for
 *         the Motor to follow in Digital Closed-loop Speed control mode
 *           .The method refers to the Uart Read command: 0xA5
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long [RPM]
 */
long SOLOMotorControllersUart::GetSpeedReference(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadSpeedReference, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the amount of desired Speed reference already set for
 *         the Motor to follow in Digital Closed-loop Speed control mode
 *           .The method refers to the Uart Read command: 0xA5
 * @retval long [RPM]
 */
long SOLOMotorControllersUart::GetSpeedReference()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetSpeedReference(error);
}

/**
 * @brief  This command reads the amount of desired Id (direct current) or
 *         Magnetizing current reference already set for the Motor to follow
 *         in Digital Closed-loop Speed control mode for ACIM motors
 *           .The method refers to the Uart Read command: 0xA6
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetMagnetizingCurrentIdReference(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadMagnetizingCurrentIdReference, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the amount of desired Id (direct current) or
 *         Magnetizing current reference already set for the Motor to follow
 *         in Digital Closed-loop Speed control mode for ACIM motors
 *           .The method refers to the Uart Read command: 0xA6
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetMagnetizingCurrentIdReference()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetMagnetizingCurrentIdReference(error);
}

/**
 * @brief  This command reads the desired position reference set for the Motor
 *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses
 *           .The method refers to the Uart Read command: 0xA7
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long [Quad-Pulses]
 */
long SOLOMotorControllersUart::GetPositionReference(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadPositionReference, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the desired position reference set for the Motor
 *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses
 *           .The method refers to the Uart Read command: 0xA7
 * @retval long [Quad-Pulses]
 */
long SOLOMotorControllersUart::GetPositionReference()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetPositionReference(error);
}

/**
 * @brief  This command reads the desired Power reference for SOLO to apply in
 *         Digital Open-loop speed control mode for 3-phase motors in terms of percentage
 *           .The method refers to the Uart Read command: 0xA8
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [%]
 */
float SOLOMotorControllersUart::GetPowerReference(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadPowerReference, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the desired Power reference for SOLO to apply in
 *         Digital Open-loop speed control mode for 3-phase motors in terms of percentage
 *           .The method refers to the Uart Read command: 0xA8
 * @retval float [%]
 */
float SOLOMotorControllersUart::GetPowerReference()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetPowerReference(error);
}

/**
 * @brief  This commands reads the desired direction of rotation set for the Motor
 *           .The method refers to the Uart Read command: 0xA9
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long 0 Counter ClockWise / 1 ClockWise
 */
long SOLOMotorControllersUart::GetMotorDirection(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadMotorDirection, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This commands reads the desired direction of rotation set for the Motor
 *           .The method refers to the Uart Read command: 0xA9
 * @retval long 0 Counter ClockWise / 1 ClockWise
 */
long SOLOMotorControllersUart::GetMotorDirection()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetMotorDirection(error);
}

/**
 * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors
 *           .The method refers to the Uart Read command: 0xAA
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float between 0.01 to 1000
 */
float SOLOMotorControllersUart::GetObserverGainBldcPmsm(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadObserverGainBldcPmsm, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors
 *           .The method refers to the Uart Read command: 0xAA
 * @retval float between 0.01 to 1000
 */
float SOLOMotorControllersUart::GetObserverGainBldcPmsm()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetObserverGainBldcPmsm(error);
}

/**
 * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors
 *           .The method refers to the Uart Read command: 0xAB
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float between 0.01 to 1000
 */
float SOLOMotorControllersUart::GetObserverGainBldcPmsmUltrafast(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadObserverGainBldcPmsmUltrafast, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors
 *           .The method refers to the Uart Read command: 0xAB
 * @retval float between 0.01 to 1000
 */
float SOLOMotorControllersUart::GetObserverGainBldcPmsmUltrafast()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetObserverGainBldcPmsmUltrafast(error);
}

/**
 * @brief  This command reads the value of Sensorless Observer Gain for DC Motor
 *           .The method refers to the Uart Read command: 0xAC
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float between 0.01 to 1000
 */
float SOLOMotorControllersUart::GetObserverGainDc(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadObserverGainDc, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of Sensorless Observer Gain for DC Motor
 *           .The method refers to the Uart Read command: 0xAC
 * @retval float between 0.01 to 1000
 */
float SOLOMotorControllersUart::GetObserverGainDc()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetObserverGainDc(error);
}

/**
 * @brief  This command reads the value of Sensorless Observer
 *         Filter Gain for Normal BLDC-PMSM Motors
 *           .The method refers to the Uart Read command: 0xAD
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float between 0.01 to 16000
 */
float SOLOMotorControllersUart::GetFilterGainBldcPmsm(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadFilterGainBldcPmsm, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of Sensorless Observer
 *         Filter Gain for Normal BLDC-PMSM Motors
 *           .The method refers to the Uart Read command: 0xAD
 * @retval float between 0.01 to 16000
 */
float SOLOMotorControllersUart::GetFilterGainBldcPmsm()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetFilterGainBldcPmsm(error);
}

/**
 * @brief  This command reads the value of Sensorless Observer
 *         Filter Gain for Ultra Fast BLDC-PMSM Motors
 *           .The method refers to the Uart Read command: 0xAE
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float between 0.01 to 16000
 */
float SOLOMotorControllersUart::GetFilterGainBldcPmsmUltrafast(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadFilterGainBldcPmsmUltrafast, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of Sensorless Observer
 *         Filter Gain for Ultra Fast BLDC-PMSM Motors
 *           .The method refers to the Uart Read command: 0xAE
 * @retval float between 0.01 to 16000
 */
float SOLOMotorControllersUart::GetFilterGainBldcPmsmUltrafast()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetFilterGainBldcPmsmUltrafast(error);
}

/**
 * @brief  This command reads the measured or estimated per-unit angle of the 3-phase motors
 *           .The method refers to the Uart Read command: 0xB0
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [Per Unit]
 */
float SOLOMotorControllersUart::Get3PhaseMotorAngle(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, Read3PhaseMotorAngle, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the measured or estimated per-unit angle of the 3-phase motors
 *           .The method refers to the Uart Read command: 0xB0
 * @retval float [Per Unit]
 */
float SOLOMotorControllersUart::Get3PhaseMotorAngle()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::Get3PhaseMotorAngle(error);
}

/**
 * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction
 *           .The method refers to the Uart Read command: 0xB1
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [Per Unit]
 */
float SOLOMotorControllersUart::GetEncoderHallCcwOffset(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadEncoderHallCcwOffset, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction
 *           .The method refers to the Uart Read command: 0xB1
 * @retval float [Per Unit]
 */
float SOLOMotorControllersUart::GetEncoderHallCcwOffset()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetEncoderHallCcwOffset(error);
}

/**
 * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction
 *           .The method refers to the Uart Read command: 0xB2
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [Per Unit]
 */
float SOLOMotorControllersUart::GetEncoderHallCwOffset(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadEncoderHallCwOffset, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction
 *           .The method refers to the Uart Read command: 0xB2
 * @retval float [Per Unit]
 */
float SOLOMotorControllersUart::GetEncoderHallCwOffset()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetEncoderHallCwOffset(error);
}

/**
 * @brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line
 *           .The method refers to the Uart Read command: 0xB3
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long [Bits/s]
 */
long SOLOMotorControllersUart::GetUartBaudrate(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadUartBaudrate, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line
 *           .The method refers to the Uart Read command: 0xB3
 * @retval long [Bits/s]
 */
long SOLOMotorControllersUart::GetUartBaudrate()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetUartBaudrate(error);
}

/**
 * @brief  This command reads the acceleration value of the Speed for
 *         speed controller both in Analogue and Digital modes
 *         in Revolution per square seconds
 *           .The method refers to the Uart Read command: 0xB4
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [Rev/S^2]
 */
float SOLOMotorControllersUart::GetSpeedAccelerationValue(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadSpeedAccelerationValue, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the acceleration value of the Speed for
 *         speed controller both in Analogue and Digital modes
 *         in Revolution per square seconds
 *           .The method refers to the Uart Read command: 0xB4
 * @retval float [Rev/S^2]
 */
float SOLOMotorControllersUart::GetSpeedAccelerationValue()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetSpeedAccelerationValue(error);
}

/**
 * @brief  This command reads the deceleration value of the Speed for
 *         speed controller both in Analogue and Digital modes
 *         in Revolution per square seconds
 *           .The method refers to the Uart Read command: 0xB5
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float [Rev/S^2]
 */
float SOLOMotorControllersUart::GetSpeedDecelerationValue(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadSpeedDecelerationValue, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the deceleration value of the Speed for
 *         speed controller both in Analogue and Digital modes
 *         in Revolution per square seconds
 *           .The method refers to the Uart Read command: 0xB5
 * @retval float [Rev/S^2]
 */
float SOLOMotorControllersUart::GetSpeedDecelerationValue()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetSpeedDecelerationValue(error);
}

/**
 * @brief  This command Reads the baud rate of CAN bus in CANOpen network
 *           .The method refers to the Uart Read command: 0xB6
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long [kbits/s]
 */
long SOLOMotorControllersUart::GetCanbusBaudrate(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadCanbusBaudrate, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command Reads the baud rate of CAN bus in CANOpen network
 *           .The method refers to the Uart Read command: 0xB6
 * @retval long [kbits/s]
 */
long SOLOMotorControllersUart::GetCanbusBaudrate()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetCanbusBaudrate(error);
}

/**
 * @brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
 *           .The method refers to the Uart Read command: 0xB7
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersUart::GetAnalogueSpeedResolutionDivisionCoefficient(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadASRDC, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
 *           .The method refers to the Uart Read command: 0xB7
 * @retval long
 */
long SOLOMotorControllersUart::GetAnalogueSpeedResolutionDivisionCoefficient()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetAnalogueSpeedResolutionDivisionCoefficient(error);
}

/**
 * @brief  This Command reads the number of counted index pulses
 *         seen on the Incremental Encoder’s output
 *           .The method refers to the Uart Read command: 0xB8
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long [Pulses]
 */
long SOLOMotorControllersUart::GetEncoderIndexCounts(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadEncoderIndexCounts, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This Command reads the number of counted index pulses
 *         seen on the Incremental Encoder’s output
 *           .The method refers to the Uart Read command: 0xB8
 * @retval long [Pulses]
 */
long SOLOMotorControllersUart::GetEncoderIndexCounts()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetEncoderIndexCounts(error);
}

/**
 *
 * @brief  This command test if the communication is working
 * @retval bool 0 not working / 1 for working
 */
bool SOLOMotorControllersUart::CommunicationIsWorking(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  float temperature = GetBoardTemperature(error);
  if (error == SOLOMotorControllers::Error::NO_ERROR_DETECTED)
  {
    return true;
  }
  return false;
}

/**
 *
 * @brief  This command test if the communication is working
 * @retval bool 0 not working / 1 for working
 */
bool SOLOMotorControllersUart::CommunicationIsWorking()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::CommunicationIsWorking(error);
}

/**
 * @brief  This command reads the type of the Embedded Motion profile active in the controller
 *           .The method refers to the Uart Read command: 0xBB
 * @param[out] error   pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersUart::GetMotionProfileMode(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadMotionProfileMode, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the type of the Embedded Motion profile active in the controller
 *           .The method refers to the Uart Read command: 0xBB
 * @retval long
 */
long SOLOMotorControllersUart::GetMotionProfileMode()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetMotionProfileMode(error);
}

/**
 * @brief  This command reads the value of the Motion Profile Variable1 set inside the controller
 *           .The method refers to the Uart Read command: 0xBC
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable1(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadMotionProfileVariable1, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of the Motion Profile Variable1 set inside the controller
 *           .The method refers to the Uart Read command: 0xBC
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable1()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetMotionProfileVariable1(error);
}

/**
 * @brief  This command reads the value of the Motion Profile Variable2 set inside the controller
 *           .The method refers to the Uart Read command: 0xBD
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable2(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadMotionProfileVariable2, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of the Motion Profile Variable2 set inside the controller
 *           .The method refers to the Uart Read command: 0xBD
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable2()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetMotionProfileVariable2(error);
}

/**
 * @brief  This command reads the value of the Motion Profile Variable3 set inside the controller
 *           .The method refers to the Uart Read command: 0xBE
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable3(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadMotionProfileVariable3, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of the Motion Profile Variable3 set inside the controller
 *           .The method refers to the Uart Read command: 0xBE
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable3()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetMotionProfileVariable3(error);
}

/**
 * @brief  This command reads the value of the Motion Profile Variable4 set inside the controller
 *           .The method refers to the Uart Read command: 0xBF
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable4(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadMotionProfileVariable4, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of the Motion Profile Variable4 set inside the controller
 *           .The method refers to the Uart Read command: 0xBF
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable4()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetMotionProfileVariable4(error);
}

/**
 * @brief  This command reads the value of the Motion Profile Variable5 set inside the controller
 *           .The method refers to the Uart Read command: 0xC0
 * @param[out] error   pointer to an integer that specify result of function
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable5(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, ReadMotionProfileVariable5, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of the Motion Profile Variable5 set inside the controller
 *           .The method refers to the Uart Read command: 0xC0
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable5()
{
  int error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  return SOLOMotorControllersUart::GetMotionProfileVariable5(error);
}