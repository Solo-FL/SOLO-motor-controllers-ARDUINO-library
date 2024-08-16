/**
 *******************************************************************************
 * @file    SOLOMotorControllersUart.cpp
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the functions for the Solo Drivers
 *          uart communications.
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.4.1
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 *******************************************************************************
 */

#include "SOLOMotorControllersUart.h"
#include <stdint.h>

int SOLOMotorControllersUart::lastError = 0;
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

// Some Arduino (UNO WIFI R4, MINIMA) have Tx0 and Tx1 at Serial1 not Serial
#if defined(ARDUINO_UNOWIFIR4) || defined(ARDUINO_MINIMA) // For LEONARDO: || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
  if (_serial == Serial)
  {
    serialToUse = &Serial1;
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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetDeviceAddress(unsigned char deviceAddress, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetDeviceAddressInputValidation(deviceAddress, error))
  {
    return false;
  }

  unsigned char cmd[] = {addr, WRITE_DEVICE_ADDRESS, 0x00, 0x00, 0x00, deviceAddress};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the mode of the operation of SOLO
 *         in terms of operating in Analogue mode or Digital
 *           .The method refers to the Uart Write command: 0x02
 * @param[in] mode  enum that specify mode of the operation of SOLO
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetCommandMode(SOLOMotorControllers::CommandMode mode, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char cmd[] = {addr, WRITE_COMMAND_MODE, 0x00, 0x00, 0x00, (unsigned char)mode};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the maximum allowed current into the motor in terms of Amps
 *           .The method refers to the Uart Write command: 0x03
 * @param[in] currentLimit  a float value [Amps]
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_CURRENT_LIMIT, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the amount of desired current that acts in torque generation
 *           .The method refers to the Uart Write command: 0x04
 * @param[in] torqueReferenceIq  a float [Amps]
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_TORQUE_REFERENCE_IQ, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
 *           .The method refers to the Uart Write command: 0x05
 * @param[in] speedReference  a long value [RPM]
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_SPEED_REFERENCE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the amount of power percentage during only
 *         Open-loop mode for 3-phase motors
 *           .The method refers to the Uart Write command: 0x06
 * @param[in] powerReference  a float value between 0 to 100
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_POWER_REFERENCE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
  * @brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
            identifying the electrical parameters of the Motor connected
              .The method refers to the Uart Write command: 0x07
  * @param[in] identification  enum that specify Start or Stop of something in SOLO
  * @param[out] error   optional pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersUart::MotorParametersIdentification(SOLOMotorControllers::Action identification, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char cmd[] = {addr, WRITE_MOTOR_PARAMETERS_IDENTIFICATION, 0x00, 0x00, 0x00, (unsigned char)identification};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command Disables or Enables the Controller resulting in deactivation or activation of the
 *           switching at the output, by disabling the drive, the effect of the Controller on the Motor will be
 *            almost eliminated ( except for body diodes of the Mosfets) allowing freewheeling
 *           .The method refers to the Uart Write command: 0x08
 * @param[in]  action  enum that specify Disable or Enable of something in SOLO
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetDriveDisableEnable(SOLOMotorControllers::DisableEnable action, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, WRITE_DRIVE_DISABLE_ENABLE, 0x00, 0x00, 0x00, (unsigned char)action};

  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the output switching frequency of the whole power unit on the Motor
 *           .The method refers to the Uart Write command: 0x09
 * @param[in] outputPwmFrequencyKhz  switching frequencies [kHz]
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_OUTPUT_PWM_FREQUENCY_KHZ, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the Speed controller Kp Gain, and it will
 *         be functional only in Digital Closed-loop mode
 *           .The method refers to the Uart Write command: 0x0A
 * @param[in] speedControllerKp  a float value between 0 to 300
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_SPEED_CONTROLLER_KP, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the Speed controller Ki gain, and it will
 *         be functional only in Digital Closed-loop mode
 *           .The method refers to the Uart Write command: 0x0B
 * @param[in] speedControllerKi  a float value between 0 to 300
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_SPEED_CONTROLLER_KI, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This commands sets the direction of the rotation of the motor
 *         either to ClockWise rotation or to Counter Clockwise Rotation
 *           .The method refers to the Uart Write command: 0x0C
 * @param[in] motorDirection  enum that specify the direction of the rotation of the motor
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char cmd[] = {addr, WRITE_MOTOR_DIRECTION, 0x00, 0x00, 0x00, (unsigned char)motorDirection};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the amount of the Phase or Armature resistance
 *         for 3-phase or DC Brushed motors respectively
 *           .The method refers to the Uart Write command: 0x0D
 * @param[in] motorResistance  a float value [Ohm]
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_MOTOR_RESISTANCE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the amount of the Phase or Armature Inductance
 *         for 3-phase or DC Brushed motors respectively
 *           .The method refers to the Uart Write command: 0x0E
 * @param[in] motorInductance  a float value [Henry]
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_MOTOR_INDUCTANCE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
 *           .The method refers to the Uart Write command: 0x0F
 * @param[in] motorPolesCounts  a long value between 1 to 254
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_MOTOR_POLES_COUNTS, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the pre-quad number of physical lines of an
 *         incremental encoder engraved on its disk
 *           .The method refers to the Uart Write command: 0x10
 * @param[in] incrementalEncoderLines  a long value [pre-quad]
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_INCREMENTAL_ENCODER_LINES, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the allowed speed during trajectory following
 *         in closed-loop position controlling mode
 *           .The method refers to the Uart Write command: 0x11
 * @param[in] speedLimit  a long value [RPM]
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_SPEED_LIMIT, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the type of the feedback control SOLO has to operate
 *           .The method refers to the Uart Write command: 0x13
 * @param[in] feedbackControlMode  enum that specify the type of the feedback control SOLO
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode feedbackControlMode, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char data[4];
  soloUtils->ConvertToData((long)feedbackControlMode, data);

  unsigned char cmd[] = {addr, WRITE_FEEDBACK_CONTROL_MODE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command resets SOLO to its factory setting to all the default parameters
 *           .The method refers to the Uart Write command: 0x14
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::ResetFactory(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char cmd[] = {addr, WRITE_RESET_FACTORY, 0x00, 0x00, 0x00, 0x01};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the Motor type that is connected to SOLO in Digital Mode
 *           .The method refers to the Uart Write command: 0x15
 * @param[in] motorType  enum that specify the Motor type that is connected to SOLO in Digital Mode
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotorType(SOLOMotorControllers::MotorType motorType, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char data[4];
  soloUtils->ConvertToData((long)motorType, data);

  unsigned char cmd[] = {addr, WRITE_MOTOR_TYPE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the Control Mode in terms of Torque,
 *         Speed or Position only in Digital Mode
 *           .The method refers to the Uart Write command: 0x16
 * @param[in] controlMode  enum that specify the Control Mode in terms of Torque,
 *                       Speed or Position only in Digital Mode
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetControlMode(SOLOMotorControllers::ControlMode controlMode, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char data[4];
  soloUtils->ConvertToData((long)controlMode, data);

  unsigned char cmd[] = {addr, WRITE_CONTROL_MODE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the value for Current Controller Kp or proportional gain
 *           .The method refers to the Uart Write command: 0x17
 * @param[in] currentControllerKp  a float value between 0 to 16000
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_CURRENT_CONTROLLER_KP, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the value for Current Controller Ki or integral gain
 *           .The method refers to the Uart Write command: 0x18
 * @param[in] currentControllerKi  a float value between 0 to 16000
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_CURRENT_CONTROLLER_KI, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
 *         Weakening current reference to help the motor reaching speeds higher than
 *         nominal values and in case of AC Induction Motors Sets the desired magnetizing
 *         current (Id) required for controlling ACIM motors in FOC in Amps
 *           .The method refers to the Uart Write command: 0x1A
 * @param[in] magnetizingCurrentIdReference  a float value [Amps]
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_MAGNETIZING_CURRENT_ID_REFERENCE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the desired Position reference in terms of quadrature
 *         pulses while SOLO operates with the Incremental Encoders or in terms of
 *         pulses while while SOLO operates with Hall sensors
 *           .The method refers to the Uart Write command: 0x1B
 * @param[in] positionReference  a long value [Quad-Pulse]
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_POSITION_REFERENCE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the value for Position Controller Kp or proportional gain
 *           .The method refers to the Uart Write command: 0x1C
 * @param[in] positionControllerKp  a float value between 0 to 16000
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_POSITION_CONTROLLER_KP, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the value for Position Controller Ki or integrator gain
 *           .The method refers to the Uart Write command: 0x1D
 * @param[in] positionControllerKi  a float value between 0 to 16000
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_POSITION_CONTROLLER_KI, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command overwrites the reported errors in Error Register
 *         reported with command code of "0xA1"
 *           .The method refers to the Uart Write command: 0x20
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::OverwriteErrorRegister(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char cmd[] = {addr, WRITE_OVERWRITE_ERROR_REGISTER, 0x00, 0x00, 0x00, 0x00};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  Once in Zero Speed Full Torque algorithm (ZSFT) for controlling the speed of a BLDC or PMSM
 *            in sensorless fashion, this parameter defines the strength of signal injection into the motor, the
 *            user has to make sure this value is not selected too high or too low
 *           .The method refers to the Uart Write command: 0x21
 * @param[in]  zsftInjectionAmplitude  a float value between 0.0 to 0.55
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetZsftInjectionAmplitude(float zsftInjectionAmplitude, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetZsftInjectionAmplitudeValidation(zsftInjectionAmplitude, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(zsftInjectionAmplitude, data);
  unsigned char cmd[] = {addr, WRITE_ZSFT_INJECTION_AMPLITUDE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  Once in Zero Speed Full Torque algorithm (ZSFT) for controlling the speed of a BLDC or PMSM
 *             in sensorless fashion, this parameter defines the strength of signal injection into the motor to
 *            identify the polarity of the Motor at the startup
 *           .The method refers to the Uart Write command: 0x22
 * @param[in]  zsftPolarityAmplitude  a float value between 0.0 to 0.55
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetZsftPolarityAmplitude(float zsftPolarityAmplitude, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetZsftPolarityAmplitudeValidation(zsftPolarityAmplitude, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(zsftPolarityAmplitude, data);
  unsigned char cmd[] = {addr, WRITE_ZSFT_POLARITY_AMPLITUDE, data[0], data[1], data[2], data[3]};

  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the observer gain for the Non-linear observer
 *         that estimates the speed of a DC brushed once the motor type
 *         is selected as DC brushed
 *           .The method refers to the Uart Write command: 0x23
 * @param[in] observerGain  a float value between 0.01 to 1000
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_OBSERVER_GAIN_DC, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
  * @brief This command defines the frequency of signal injection into the Motor in
        runtime, by selecting zero the full injection frequency will be applied which allows to reach to
        higher speeds, however for some motors, it’s better to increase this value
 *           .The method refers to the Uart Write command: 0x24
  * @param[in]  zsftInjectionFrequency  a long value between 0 to 10
  * @param[out]  error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetZsftInjectionFrequency(long zsftInjectionFrequency, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetZsftInjectionFrequencyInputValidation(zsftInjectionFrequency, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(zsftInjectionFrequency, data);
  unsigned char cmd[] = {addr, WRITE_ZSFT_INJECTION_FREQUENCY, data[0], data[1], data[2], data[3]};

  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  Once in Sensorless speed or torque controlling of a BLDC or PMSM motors, this parameter
 *				defines the speed in which the Low speed algorithm has to switch to high speed algorithm
 *           .The method refers to the Uart Write command: 0x25
 * @param[in]  sensorlessTransitionSpeed  a long value between 1 to 5000
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetSensorlessTransitionSpeed(long sensorlessTransitionSpeed, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetSensorlessTransitionSpeedInputValidation(sensorlessTransitionSpeed, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(sensorlessTransitionSpeed, data);
  unsigned char cmd[] = {addr, WRITE_SENSORLESS_TRANSITION_SPEED, data[0], data[1], data[2], data[3]};

  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the baud-rate of the UART line
 *           .The method refers to the Uart Write command: 0x26
 * @param[in] baudrate  enum that specify the baud-rate of the UART line
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char data[4];
  soloUtils->ConvertToData((long)baudrate, data);

  unsigned char cmd[] = {addr, WRITE_UART_BAUDRATE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command starts or stops the process of sensor calibration
 *           .The method refers to the Uart Write command: 0x27
 * @param[in] calibrationAction  enum that specify the process of sensor calibration
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char data[4];
  soloUtils->ConvertToData((long)calibrationAction, data);

  unsigned char cmd[] = {addr, WRITE_SENSOR_CALIBRATION, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the per-unit offset identified after sensor calibration
 *         for Encoder or Hall sensors in C.C.W direction
 *           .The method refers to the Uart Write command: 0x28
 * @param[in] encoderHallOffset  a float value between 0.0 to 1.0
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_ENCODER_HALL_CCW_OFFSET, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the per-unit offset identified after sensor calibration
 *         for Encoder or Hall sensors in C.W direction
 *           .The method refers to the Uart Write command: 0x29
 * @param[in] encoderHallOffset  a float value between 0.0 to 1.0
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_ENCODER_HALL_CW_OFFSET, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the acceleration value of the Speed for speed controller
 *         both in Analogue and Digital modes in Revolution per square seconds
 *           .The method refers to the Uart Write command: 0x2A
 * @param[in] speedAccelerationValue  a float value [Rev/S^2]
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_SPEED_ACCELERATION_VALUE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the deceleration value of the Speed for speed controller
 *         both in Analogue and Digital modes in Revolution per square seconds
 *           .The method refers to the Uart Write command: 0x2B
 * @param[in] speedDecelerationValue  a float value [Rev/S^2]
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_SPEED_DECELERATION_VALUE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command sets the baud rate of CAN bus in CANOpen network
 *           .The method refers to the Uart Write command: 0x2C
 * @param[in] canbusBaudrate  enum that specify the baud rate of CAN bus in CANOpen network
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate canbusBaudrate, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char data[4];
  soloUtils->ConvertToData((long)canbusBaudrate, data);

  unsigned char cmd[] = {addr, WRITE_UART_BAUDRATE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the resolution of the speed at S/T input
 *           while SOLO operates in Analogue mode
 *           .The method refers to the Uart Write command: 0x2D
 * @param[in] divisionCoefficient  a long value
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_ASRDC, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the type of the Motion Profile that is
 *           being used in Speed or Position Modes
 *           .The method refers to the Uart Write command: 0x30
 * @param[in] motionProfileMode enum that specify the type of the Motion Profile
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetMotionProfileMode(SOLOMotorControllers::MotionProfileMode motionProfileMode, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;

  unsigned char data[4];
  soloUtils->ConvertToData((long)motionProfileMode, data);

  unsigned char cmd[] = {addr, WRITE_MOTION_PROFILE_MODE, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x31
 * @param[in] MotionProfileVariable1 a float value
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_MOTION_PROFILE_VARIABLE1, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x32
 * @param[in] MotionProfileVariable2 a float value
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_MOTION_PROFILE_VARIABLE2, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x33
 * @param[in] MotionProfileVariable3 a float value
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_MOTION_PROFILE_VARIABLE3, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x34
 * @param[in] MotionProfileVariable4 a float value
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_MOTION_PROFILE_VARIABLE4, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
 *           .The method refers to the Uart Write command: 0x35
 * @param[in] MotionProfileVariable5 a float value
 * @param[out] error   optional pointer to an integer that specify result of function
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

  unsigned char cmd[] = {addr, WRITE_MOTION_PROFILE_VARIABLE5, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command Set the Digiatal Ouput pin Status. The method refers to the Uart Write command: 0x38
 * @param[in] channel	@ref Channel
 * @param[in] state   @ref DigitalIoState
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetDigitalOutputState(Channel channel, DigitalIoState state, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char data[4];
  long lastOutRegister;

  lastOutRegister = GetDigitalOutputsRegister(error);
  if (error = 0)
    return error;

  if (state == 1)
    lastOutRegister = lastOutRegister | (1 << channel);
  else
    lastOutRegister = lastOutRegister & (~(1 << channel));

  soloUtils->ConvertToData(lastOutRegister, data);

  unsigned char cmd[] = {addr, WRITE_DIGITAL_OUTPUTS_REGISTER, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This command defines the maximum allowed regeneration current sent back from the Motor to
 *				the Power Supply during decelerations
 *           .The method refers to the Uart Write command: 0x39
 * @param[in]  current a float value
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetRegenerationCurrentLimit(float current, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetRegenerationCurrentLimitValidation(current, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(current, data);

  unsigned char cmd[] = {addr, WRITE_REGENERATION_CURRENT_LIMIT, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}

/**
 * @brief  This value defines the the sampling window of qualification digital filter applied to the output of
 *			the position sensor before being processed by DSP
 *           .The method refers to the Uart Write command: 0x3A
 * @param[in]  level a long value
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval bool 0 fail / 1 for success
 */
bool SOLOMotorControllersUart::SetPositionSensorDigitalFilterLevel(long level, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->SetPositionSensorDigitalFilterLevelValidation(level, error))
  {
    return false;
  }

  unsigned char data[4];
  soloUtils->ConvertToData(level, data);

  unsigned char cmd[] = {addr, WRITE_POSITION_SENSOR_DIGITAL_FILTER_LEVEL, data[0], data[1], data[2], data[3]};
  return SOLOMotorControllersUart::ExeCMD(cmd, error);
}
//----------Read----------

/**
 * @brief  This command reads the device address connected on the line
 *           .The method refers to the Uart Read command: 0x81
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long device address connected on the line
 */
long SOLOMotorControllersUart::GetDeviceAddress(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {0xFF, READ_DEVICE_ADDRESS, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the phase-A voltage of the motor connected to the
 *         "A" pin output of SOLO for 3-phase Motors
 *           .The method refers to the Uart Read command: 0x82
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float phase-A voltage of the motor [Volts]
 */
float SOLOMotorControllersUart::GetPhaseAVoltage(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_PHASE_A_VOLTAGE, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float 0 phase-A voltage of the motor [Volts]
 */
float SOLOMotorControllersUart::GetPhaseBVoltage(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_PHASE_B_VOLTAGE, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval phase-A current of the motor [Amps]
 */
float SOLOMotorControllersUart::GetPhaseACurrent(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_PHASE_A_CURRENT, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float phase-B current of the motor [Amps]
 */
float SOLOMotorControllersUart::GetPhaseBCurrent(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_PHASE_B_CURRENT, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float  BUS voltage [Volts]
 */
// Battery Voltage
float SOLOMotorControllersUart::GetBusVoltage(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_BUS_VOLTAGE, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float between [Amps]
 */
float SOLOMotorControllersUart::GetDcMotorCurrentIm(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_DC_MOTOR_CURRENT_IM, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [Volts]
 */
float SOLOMotorControllersUart::GetDcMotorVoltageVm(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_DC_MOTOR_VOLTAGE_VM, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetSpeedControllerKp(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_SPEED_CONTROLLER_KP, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetSpeedControllerKi(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_SPEED_CONTROLLER_KI, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the output switching frequency of SOLO in Hertz
 *           .The method refers to the Uart Read command: 0x8B
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long [Hz]
 */
long SOLOMotorControllersUart::GetOutputPwmFrequencyKhz(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_OUTPUT_PWM_FREQUENCY_HZ, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data); // PWM reading is in Hz
  }
  return -1;
}

/**
 * @brief  This command reads the value of the current limit set for SOLO in
 *         closed-loop digital operation mode
 *           .The method refers to the Uart Read command: 0x8C
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetCurrentLimit(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_CURRENT_LIMIT, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetQuadratureCurrentIqFeedback(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_QUADRATURE_CURRENT_IQ_FEEDBACK, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetMagnetizingCurrentIdFeedback(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_MAGNETIZING_CURRENT_ID_FEEDBACK, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the number of Poles set for 3-phase motors
 *           .The method refers to the Uart Read command: 0x8F
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long between 1 to 254
 */
long SOLOMotorControllersUart::GetMotorPolesCounts(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_MOTOR_POLES_COUNTS, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long between 1 to 200000
 */
long SOLOMotorControllersUart::GetIncrementalEncoderLines(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_INCREMENTAL_ENCODER_LINES, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the amount of value set for Current controller
 *         Kp or proportional gain
 *           .The method refers to the Uart Read command: 0x91
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetCurrentControllerKp(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_CURRENT_CONTROLLER_KP, 0x00, 0x00, 0x00, 0x00};

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
 *         Ki or integrator gain
 *           .The method refers to the Uart Read command: 0x92
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetCurrentControllerKi(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_CURRENT_CONTROLLER_KI, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [°C]
 */
float SOLOMotorControllersUart::GetBoardTemperature(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_BOARD_TEMPERATURE, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [Ohms]
 */
float SOLOMotorControllersUart::GetMotorResistance(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_MOTOR_RESISTANCE, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [Henry]
 */
float SOLOMotorControllersUart::GetMotorInductance(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_MOTOR_INDUCTANCE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
  * @brief  his command reads the actual speed of the motor measured or estimated by SOLO in
            sensorless or sensor-based modes respectively
              .The method refers to the Uart Read command: 0x96
  * @param[out] error   optional pointer to an integer that specify result of function
  * @retval long [RPM]
  */
long SOLOMotorControllersUart::GetSpeedFeedback(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_SPEED_FEEDBACK, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long between 0 to 3
 */
SOLOMotorControllers::MotorType SOLOMotorControllersUart::GetMotorType(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_MOTOR_TYPE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return (SOLOMotorControllers::MotorType)soloUtils->ConvertToLong(data);
  }
  return SOLOMotorControllers::MotorType::MOTOR_TYPE_ERROR;
  ;
}

/**
 * @brief  This command reads the feedback control mode selected on SOLO both
 *         for Analogue and Digital operations
 *           .The method refers to the Uart Read command: 0x99
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long between 0 to 2
 */
SOLOMotorControllers::FeedbackControlMode SOLOMotorControllersUart::GetFeedbackControlMode(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_FEEDBACK_CONTROL_MODE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return (SOLOMotorControllers::FeedbackControlMode)soloUtils->ConvertToLong(data);
  }
  return SOLOMotorControllers::FeedbackControlMode::FEEDBACK_CONTROL_MODE_ERROR;
}

/**
 * @brief  This command reads the actual commanding mode that SOLO is operating
 *           .The method refers to the Uart Read command: 0x9A
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long between 0 or 1
 */
SOLOMotorControllers::CommandMode SOLOMotorControllersUart::GetCommandMode(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_COMMAND_MODE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return (SOLOMotorControllers::CommandMode)soloUtils->ConvertToLong(data);
  }
  return SOLOMotorControllers::CommandMode::COMMAND_MODE_ERROR;
}

/**
 * @brief  This command reads the Control Mode type in terms of Torque,
 *         Speed or Position in both Digital and Analogue modes
 *           .The method refers to the Uart Read command: 0x9B
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long between 0 to 2
 */
SOLOMotorControllers::ControlMode SOLOMotorControllersUart::GetControlMode(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_CONTROL_MODE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return (SOLOMotorControllers::ControlMode)soloUtils->ConvertToLong(data);
  }
  return SOLOMotorControllers::ControlMode::CONTROL_MODE_ERROR;
  ;
}

/**
 * @brief  This command reads the value of the speed limit set on SOLO
 *           .The method refers to the Uart Read command: 0x9C
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long [RPM]
 */
long SOLOMotorControllersUart::GetSpeedLimit(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_SPEED_LIMIT, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the amount of value set for Position
 *         controller Kp or proportional gain
 *           .The method refers to the Uart Read command: 0x9D
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetPositionControllerKp(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_POSITION_CONTROLLER_KP, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float between 0 to 16000
 */
float SOLOMotorControllersUart::GetPositionControllerKi(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_POSITION_CONTROLLER_KI, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the number of counted pulses from the
 *         Incremental Encoder or Hall sensors
 *           .The method refers to the Uart Read command: 0xA0
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long [Quad-Pulses]
 */
long SOLOMotorControllersUart::GetPositionCountsFeedback(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_POSITION_COUNTS_FEEDBACK, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersUart::GetErrorRegister(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_ERROR_REGISTER, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersUart::GetDeviceFirmwareVersion(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_DEVICE_FIRMWARE_VERSION, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersUart::GetDeviceHardwareVersion(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_DEVICE_HARDWARE_VERSION, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the amount of desired Torque reference (Iq or IM)
 *         already set for the Motor to follow in Digital Closed-loop Torque control mode
 *           .The method refers to the Uart Read command: 0xA4
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetTorqueReferenceIq(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_TORQUE_REFERENCE_IQ, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the amount of desired Speed reference already set for
 *         the Motor to follow in Digital Closed-loop Speed control mode
 *           .The method refers to the Uart Read command: 0xA5
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long [RPM]
 */
long SOLOMotorControllersUart::GetSpeedReference(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_SPEED_REFERENCE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the amount of desired Id (direct current) or
 *         Magnetizing current reference already set for the Motor to follow
 *         in Digital Closed-loop Speed control mode for ACIM motors
 *           .The method refers to the Uart Read command: 0xA6
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [Amps]
 */
float SOLOMotorControllersUart::GetMagnetizingCurrentIdReference(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_MAGNETIZING_CURRENT_ID_REFERENCE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the desired position reference set for the Motor
 *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses
 *           .The method refers to the Uart Read command: 0xA7
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long [Quad-Pulses]
 */
long SOLOMotorControllersUart::GetPositionReference(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_POSITION_REFERENCE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the desired Power reference for SOLO to apply in
 *         Digital Open-loop speed control mode for 3-phase motors in terms of percentage
 *           .The method refers to the Uart Read command: 0xA8
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [%]
 */
float SOLOMotorControllersUart::GetPowerReference(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_POWER_REFERENCE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This commands reads the desired direction of rotation set for the Motor
 *           .The method refers to the Uart Read command: 0xA9
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long 0 Counter ClockWise / 1 ClockWise
 */
SOLOMotorControllers::Direction SOLOMotorControllersUart::GetMotorDirection(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_MOTOR_DIRECTION, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return (SOLOMotorControllers::Direction)soloUtils->ConvertToLong(data);
  }
  return SOLOMotorControllers::Direction::DIRECTION_ERROR;
}

/**
 * @brief  This command reads the value of Sensorless Zero Speed Full Torque Injection Amplitude
 *           .The method refers to the Uart Read command: 0xAA
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval float between 0.0 to 0.55
 */
float SOLOMotorControllersUart::GetZsftInjectionAmplitude(int &error)
{
  error = Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_ZSFT_INJECTION_AMPLITUDE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of Sensorless Zero Speed Full Torque Polarity Amplitude
 *           .The method refers to the Uart Read command: 0xAB
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval float between 0.0 to 0.55
 */
float SOLOMotorControllersUart::GetZsftPolarityAmplitude(int &error)
{
  error = Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_ZSFT_POLARITY_AMPLITUDE, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float between 0.01 to 1000
 */
float SOLOMotorControllersUart::GetObserverGainDc(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_OBSERVER_GAIN_DC, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of Sensorless Zero Speed Full Torque Injection Frequency
 *           .The method refers to the Uart Read command: 0xAD
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval float between 0 to 10
 */
long SOLOMotorControllersUart::GetZsftInjectionFrequency(int &error)
{
  error = Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_ZSFT_INJECTION_FREQUENCY, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of Sensorless Transition Speed
 *           .The method refers to the Uart Read command: 0xAE
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval long between 1 to 5000
 */
long SOLOMotorControllersUart::GetSensorlessTransitionSpeed(int &error)
{
  error = Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_SENSORLESS_TRANSITION_SPEED, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the measured or estimated per-unit angle of the 3-phase motors
 *           .The method refers to the Uart Read command: 0xB0
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [Per Unit]
 */
float SOLOMotorControllersUart::Get3PhaseMotorAngle(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_3_PHASE_MOTOR_ANGLE, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [Per Unit]
 */
float SOLOMotorControllersUart::GetEncoderHallCcwOffset(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_ENCODER_HALL_CCW_OFFSET, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [Per Unit]
 */
float SOLOMotorControllersUart::GetEncoderHallCwOffset(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_ENCODER_HALL_CW_OFFSET, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line
 *           .The method refers to the Uart Read command: 0xB3
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long [Bits/s]
 */
SOLOMotorControllers::UartBaudrate SOLOMotorControllersUart::GetUartBaudrate(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_UART_BAUDRATE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return (SOLOMotorControllers::UartBaudrate)soloUtils->ConvertToLong(data);
  }
  return SOLOMotorControllers::UartBaudrate::UART_BAUDRATE_ERROR;
}

/**
 * @brief  This command reads the acceleration value of the Speed for
 *         speed controller both in Analogue and Digital modes
 *         in Revolution per square seconds
 *           .The method refers to the Uart Read command: 0xB4
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [Rev/S^2]
 */
float SOLOMotorControllersUart::GetSpeedAccelerationValue(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_SPEED_ACCELERATION_VALUE, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float [Rev/S^2]
 */
float SOLOMotorControllersUart::GetSpeedDecelerationValue(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_SPEED_DECELERATION_VALUE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command Reads the baud rate of CAN bus in CANOpen network
 *           .The method refers to the Uart Read command: 0xB6
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long [kbits/s]
 */
long SOLOMotorControllersUart::GetCanbusBaudrate(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_CANBUS_BAUDRATE, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersUart::GetAnalogueSpeedResolutionDivisionCoefficient(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_ASRDC, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long [Pulses]
 */
long SOLOMotorControllersUart::GetEncoderIndexCounts(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_ENCODER_INDEX_COUNTS, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
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
 * @brief  This command reads the type of the Embedded Motion profile active in the controller
 *           .The method refers to the Uart Read command: 0xBB
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval long
 */
SOLOMotorControllers::MotionProfileMode SOLOMotorControllersUart::GetMotionProfileMode(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_MOTION_PROFILE_MODE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return (SOLOMotorControllers::MotionProfileMode)soloUtils->ConvertToLong(data);
  }
  return SOLOMotorControllers::MotionProfileMode::MOTION_PROFILE_MODE_ERROR;
}

/**
 * @brief  This command reads the value of the Motion Profile Variable1 set inside the controller
 *           .The method refers to the Uart Read command: 0xBC
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable1(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_MOTION_PROFILE_VARIABLE1, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable2(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_MOTION_PROFILE_VARIABLE2, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable3(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_MOTION_PROFILE_VARIABLE3, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable4(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_MOTION_PROFILE_VARIABLE4, 0x00, 0x00, 0x00, 0x00};

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
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval float
 */
float SOLOMotorControllersUart::GetMotionProfileVariable5(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_MOTION_PROFILE_VARIABLE5, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of the Digital Outputs Register as a 32 bits register
 *           .The method refers to the Uart Read command: 0xC4
 * @param[in]  channel  @ref Channel
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval enum @ref DigitalIoState
 */
SOLOMotorControllers::DigitalIoState SOLOMotorControllersUart::GetDigitalOutputsState(Channel channel, int &error)
{
  long lastOutRegister;
  lastOutRegister = GetDigitalOutputsRegister(error);
  if (error = SOLOMotorControllers::Error::NO_ERROR_DETECTED)
    return SOLOMotorControllers::DigitalIoState::DIGITAL_IO_STATE_ERROR;
  return (SOLOMotorControllers::DigitalIoState)((lastOutRegister >> channel) & 0x00000001);
}

long SOLOMotorControllersUart::GetDigitalOutputsRegister(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_DIGITAL_OUTPUT_REGISTER, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the Digiatal Ouput pin Status. The method refers to the Uart Read command: 0xC4
 * @param[out] pinNumber   specify the pin you want to controll. (Ensure your SOLO model support this functions)
 * @param[out] error   optional pointer to an integer that specify result of function
 * @retval int
 */
int SOLOMotorControllersUart::GetDigitalOutput(int pinNumber, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  if (!soloUtils->DigitalInputValidation(pinNumber, error))
  {
    return -1;
  }

  uint8_t informationReceived = (uint8_t)GetDigitalOutputsState((Channel)pinNumber, error);
  if (error != SOLOMotorControllers::Error::NO_ERROR_DETECTED)
  {
    return -1;
  }

  uint8_t mask = 1 << pinNumber;
  return (informationReceived & mask) != 0;
}

/**
 * @brief  This command reads the current state of the controller
 *           .The method refers to the Uart Read command: 0xC7
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval enum @ref DisableEnable
 */
SOLOMotorControllers::DisableEnable SOLOMotorControllersUart::GetDriveDisableEnable(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_DRIVE_DISABLE_ENABLE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return (SOLOMotorControllers::DisableEnable)soloUtils->ConvertToLong(data);
  }
  return SOLOMotorControllers::DisableEnable::DISABLE_ENABLE_ERROR;
}

/**
 * @brief  This command reads the value of the Regeneration Current Limit
 *           .The method refers to the Uart Read command: 0xC8
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval float
 */
float SOLOMotorControllersUart::GetRegenerationCurrentLimit(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_REGENERATION_CURRENT_LIMIT, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToFloat(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of the Position Sensor Digital Filter Level
 *           .The method refers to the Uart Read command: 0xC9
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersUart::GetPositionSensorDigitalFilterLevel(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_POSITION_SENSOR_DIGITAL_FILTER_LEVEL, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of the Digital Input Register as a 32 bits register
 *           .The method refers to the Uart Read command: 0xC5
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersUart::GetDigitalInputRegister(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_DIGITAL_INPUT_REGISTER, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the value of the voltage sensed at the output of PT1000 temperature
 *			sensor amplifier, this command can be used only on devices that come with PT1000 input
 *           .The method refers to the Uart Read command: 0xC3
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval long
 */
long SOLOMotorControllersUart::GetPT1000SensorVoltage(int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_PT1000_SENSOR_VOLTAGE, 0x00, 0x00, 0x00, 0x00};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return soloUtils->ConvertToLong(data);
  }
  return -1;
}

/**
 * @brief  This command reads the quantized value of an Analogue Input as a number between 0 to 4095
 *				depending on the maximum voltage input possible at the analogue inputs for the controller
 *           .The method refers to the Uart Read command: 0xC6
 * @param[in]  channel  an enum that specify the Channel of Analogue Input
 * @param[out]  error   pointer to an integer that specify result of function
 * @retval enum @ref DigitalIoState
 */
SOLOMotorControllers::DigitalIoState SOLOMotorControllersUart::GetAnalogueInput(Channel channel, int &error)
{
  error = SOLOMotorControllers::Error::NO_PROCESSED_COMMAND;
  unsigned char cmd[] = {addr, READ_ANALOGUE_INPUT, 0x00, 0x00, 0x00, (unsigned char)channel};

  if (SOLOMotorControllersUart::ExeCMD(cmd, error))
  {
    unsigned char data[4];
    soloUtils->SplitData(data, cmd);
    return (SOLOMotorControllers::DigitalIoState)soloUtils->ConvertToLong(data);
  }
  return SOLOMotorControllers::DigitalIoState::DIGITAL_IO_STATE_ERROR;
}