/**
 *******************************************************************************
 * @file    SOLOMotorControllersCanopenNative.h
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
#ifndef SOLO_MOTOR_CONTROLLERS_CANOPEN_NATIVE_H
#define SOLO_MOTOR_CONTROLLERS_CANOPEN_NATIVE_H

#include "SOLOMotorControllersCanopen.h"
#include "SOLOMotorControllersUtils.h"
#include "CanBus.hpp"
#include <Arduino.h>

/**
 * @brief a class for handle CANopen native communication
 * */
class SOLOMotorControllersCanopenNative : public SOLOMotorControllersCanopen
{
private:
  uint8_t Address;
  SOLOMotorControllersUtils *soloUtils;
  long millisecondsTimeout;
  // used to list internally object value for every PDO
  int pdoParameterObjectByPdoParameterName[PDO_PARAMETER_NAME_COUNT];
  SOLOMotorControllers::DigitalIoState GetDigitalOutputState(SOLOMotorControllers::Channel chaneel, int &error);

public:
  #ifdef ARDUINO_PORTENTA_C33
    SOLOMotorControllersCanopenNative(
        unsigned char _deviceAddress = 0,
        SOLOMotorControllers::CanbusBaudrate _baudrate = SOLOMotorControllers::CanbusBaudrate::RATE_1000,
        arduino::HardwareCAN &_CAN = CAN1,
        long _millisecondsTimeout = 200);
  #else
      SOLOMotorControllersCanopenNative(
        unsigned char _deviceAddress = 0,
        SOLOMotorControllers::CanbusBaudrate _baudrate = SOLOMotorControllers::CanbusBaudrate::RATE_1000,
        arduino::HardwareCAN &_CAN = CAN,
        long _millisecondsTimeout = 200);
  #endif

  // Cob-ID user value for every PDO
  int pdoParameterCobIdByPdoParameterName[PDO_PARAMETER_NAME_COUNT];
  static int lastError;

private:
  bool SetPdoParameterCobbIdInputValidation(SOLOMotorControllersCanopen::PdoParameterName parameterName, int parameterCobbId, int &error = lastError);
  bool SetSyncParameterCountInputValidation(uint8_t parameterCount, int &error = lastError);
  long GetPdoParameterValueLong(SOLOMotorControllersCanopen::PdoParameterName parameterName, int &error = lastError);
  float GetPdoParameterValueFloat(SOLOMotorControllersCanopen::PdoParameterName parameterName, int &error = lastError);
  void InitPdoConfig();
  long GetPdoParameterCobId(SOLOMotorControllersCanopen::PdoParameterName parameterName, int &error = lastError);
  bool SetPdoParameterValue(SOLOMotorControllersCanopen::PdoParameterName parameterName, long value, int &error = lastError);
  bool SetPdoParameterValue(SOLOMotorControllersCanopen::PdoParameterName parameterName, float value, int &error = lastError);

public:
  //----------Write  SOLOMotorControllersCanopen----------
  bool SetGuardTime(long guardtime, int &error = lastError);
  bool SetLifeTimeFactor(long lifeTimeFactor, int &error = lastError);
  bool SetProducerHeartbeatTime(long producerHeartbeatTime, int &error = lastError);

  bool UpdatePdoParameterCobIdByPdoParameterName();
  bool SetPdoParameterConfig(PdoParameterConfig config, int &error = lastError);
  bool SendPdoSync(int &error = lastError);

  bool SetPdoPositionReference(long positionReference, int &error = lastError);
  bool SetPdoSpeedReference(long speedReference, int &error = lastError);
  bool SetPdoTorqueReferenceIq(float torqueReferenceIq, int &error = lastError);
  bool SetPdoMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error = lastError);
  bool SetPdoControlMode(SOLOMotorControllers::ControlMode controlMode, int &error = lastError);
  bool SetPdoMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error = lastError);

  long GetPdoPositionCountsFeedback(int &error = lastError);
  long GetPdoSpeedFeedback(int &error = lastError);
  float GetPdoQuadratureCurrentIqFeedback(int &error = lastError);
  float GetPdoMagnetizingCurrentIdFeedback(int &error = lastError);
  long GetPdoErrorRegister(int &error = lastError);
  float GetPdoBoardTemperature(int &error = lastError);
  SOLOMotorControllersCanopen::PdoParameterConfig GetPdoParameterConfig(SOLOMotorControllersCanopen::PdoParameterName parameterName, int &error = lastError);

  //----------Write  SOLOMotorControllers----------
  bool SetDeviceAddress(unsigned char deviceAddress, int &error = lastError);
  bool SetCommandMode(SOLOMotorControllers::CommandMode mode, int &error = lastError);
  bool SetCurrentLimit(float currentLimit, int &error = lastError);
  bool SetTorqueReferenceIq(float torqueReferenceIq, int &error = lastError);
  bool SetSpeedReference(long speedReference, int &error = lastError);
  bool SetPowerReference(float powerReference, int &error = lastError);
  bool MotorParametersIdentification(SOLOMotorControllers::Action identification, int &error = lastError);
  bool SetDriveDisableEnable(SOLOMotorControllers::DisableEnable action, int &error = lastError);
  bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int &error = lastError);
  bool SetSpeedControllerKp(float speedControllerKp, int &error = lastError);
  bool SetSpeedControllerKi(float speedControllerKi, int &error = lastError);
  bool SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error = lastError);
  bool SetMotorResistance(float motorResistance, int &error = lastError);
  bool SetMotorInductance(float motorInductance, int &error = lastError);
  bool SetMotorPolesCounts(long motorPolesCounts, int &error = lastError);
  bool SetIncrementalEncoderLines(long incrementalEncoderLines, int &error = lastError);
  bool SetSpeedLimit(long speedLimit, int &error = lastError);
  bool SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode feedbackControlMode, int &error = lastError);
  bool ResetFactory(int &error = lastError);
  bool SetMotorType(SOLOMotorControllers::MotorType motorType, int &error = lastError);
  bool SetControlMode(SOLOMotorControllers::ControlMode controlMode, int &error = lastError);
  bool SetCurrentControllerKp(float currentControllerKp, int &error = lastError);
  bool SetCurrentControllerKi(float currentControllerKi, int &error = lastError);
  bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error = lastError);
  bool SetPositionReference(long positionReference, int &error = lastError);
  bool SetPositionControllerKp(float positionControllerKp, int &error = lastError);
  bool SetPositionControllerKi(float positionControllerKi, int &error = lastError);
  bool OverwriteErrorRegister(int &error = lastError);
  bool SetZsftInjectionAmplitude(float zsftInjectionAmplitude, int &error = lastError);
  bool SetZsftPolarityAmplitude(float zsftPolarityAmplitude, int &error = lastError);
  bool SetObserverGainDc(float observerGain, int &error = lastError);
  bool SetZsftInjectionFrequency(long zsftInjectionFrequency, int &error = lastError);
  bool SetSensorlessTransitionSpeed(long sensorlessTransitionSpeed, int &error = lastError);
  bool SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate, int &error = lastError);
  bool SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction, int &error = lastError);
  bool SetEncoderHallCcwOffset(float encoderHallOffset, int &error = lastError);
  bool SetEncoderHallCwOffset(float encoderHallOffset, int &error = lastError);
  bool SetSpeedAccelerationValue(float speedAccelerationValue, int &error = lastError);
  bool SetSpeedDecelerationValue(float speedDecelerationValue, int &error = lastError);
  bool SetCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate canbusBaudrate, int &error = lastError);
  bool SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int &error = lastError);
  bool SetMotionProfileMode(SOLOMotorControllers::MotionProfileMode motionProfileMode, int &error = lastError);
  bool SetMotionProfileVariable1(float MotionProfileVariable1, int &error = lastError);
  bool SetMotionProfileVariable2(float MotionProfileVariable2, int &error = lastError);
  bool SetMotionProfileVariable3(float MotionProfileVariable3, int &error = lastError);
  bool SetMotionProfileVariable4(float MotionProfileVariable4, int &error = lastError);
  bool SetMotionProfileVariable5(float MotionProfileVariable5, int &error = lastError);
  bool SetDigitalOutputState(SOLOMotorControllers::Channel channel, SOLOMotorControllers::DigitalIoState state, int &error = lastError);
  bool SetRegenerationCurrentLimit(float current, int &error = lastError);
  bool SetPositionSensorDigitalFilterLevel(long level, int &error = lastError);

  long GetGuardTime(int &error = lastError);
  long GetLifeTimeFactor(int &error = lastError);
  long GetProducerHeartbeatTime(int &error = lastError);

  //----------Read SOLOMotorControllers----------
  long GetReadErrorRegister(int &error = lastError);
  long GetDeviceAddress(int &error = lastError);
  float GetPhaseAVoltage(int &error = lastError);
  float GetPhaseBVoltage(int &error = lastError);
  float GetPhaseACurrent(int &error = lastError);
  float GetPhaseBCurrent(int &error = lastError);
  float GetBusVoltage(int &error = lastError); // Battery Voltage
  float GetDcMotorCurrentIm(int &error = lastError);
  float GetDcMotorVoltageVm(int &error = lastError);
  float GetSpeedControllerKp(int &error = lastError);
  float GetSpeedControllerKi(int &error = lastError);
  long GetOutputPwmFrequencyKhz(int &error = lastError);
  float GetCurrentLimit(int &error = lastError);
  float GetQuadratureCurrentIqFeedback(int &error = lastError);
  float GetMagnetizingCurrentIdFeedback(int &error = lastError); // Magnetizing
  long GetMotorPolesCounts(int &error = lastError);
  long GetIncrementalEncoderLines(int &error = lastError);
  float GetCurrentControllerKp(int &error = lastError);
  float GetCurrentControllerKi(int &error = lastError);
  float GetBoardTemperature(int &error = lastError);
  float GetMotorResistance(int &error = lastError);
  float GetMotorInductance(int &error = lastError);
  long GetSpeedFeedback(int &error = lastError);
  SOLOMotorControllers::MotorType GetMotorType(int &error = lastError);
  SOLOMotorControllers::FeedbackControlMode GetFeedbackControlMode(int &error = lastError);
  SOLOMotorControllers::CommandMode GetCommandMode(int &error = lastError);
  SOLOMotorControllers::ControlMode GetControlMode(int &error = lastError);
  long GetSpeedLimit(int &error = lastError);
  float GetPositionControllerKp(int &error = lastError);
  float GetPositionControllerKi(int &error = lastError);
  long GetPositionCountsFeedback(int &error = lastError);
  long GetErrorRegister(int &error = lastError);
  long GetDeviceFirmwareVersion(int &error = lastError);
  long GetDeviceHardwareVersion(int &error = lastError);
  float GetTorqueReferenceIq(int &error = lastError);
  long GetSpeedReference(int &error = lastError);
  float GetMagnetizingCurrentIdReference(int &error = lastError);
  long GetPositionReference(int &error = lastError);
  float GetPowerReference(int &error = lastError);
  SOLOMotorControllers::Direction GetMotorDirection(int &error = lastError);
  float GetZsftInjectionAmplitude(int &error = lastError);
  float GetZsftPolarityAmplitude(int &error = lastError);
  float GetObserverGainDc(int &error = lastError);
  long GetZsftInjectionFrequency(int &error = lastError);
  long GetSensorlessTransitionSpeed(int &error = lastError);
  float Get3PhaseMotorAngle(int &error = lastError); // Read Estimated or Measured Rotor Angle
  float GetEncoderHallCcwOffset(int &error = lastError);
  float GetEncoderHallCwOffset(int &error = lastError);
  SOLOMotorControllers::UartBaudrate GetUartBaudrate(int &error = lastError);
  float GetSpeedAccelerationValue(int &error = lastError);
  float GetSpeedDecelerationValue(int &error = lastError);
  long GetCanbusBaudrate(int &error = lastError);
  long GetAnalogueSpeedResolutionDivisionCoefficient(int &error = lastError);
  bool CommunicationIsWorking(int &error = lastError);
  SOLOMotorControllers::MotionProfileMode GetMotionProfileMode(int &error = lastError);
  float GetMotionProfileVariable1(int &error = lastError);
  float GetMotionProfileVariable2(int &error = lastError);
  float GetMotionProfileVariable3(int &error = lastError);
  float GetMotionProfileVariable4(int &error = lastError);
  float GetMotionProfileVariable5(int &error = lastError);
  long GetDigitalOutputsRegister(int &error);
  SOLOMotorControllers::DisableEnable GetDriveDisableEnable(int &error = lastError);
  long GetPT1000SensorVoltage(int &error = lastError);
  SOLOMotorControllers::DigitalIoState GetAnalogueInput(Channel channel, int &error = lastError);
  long GetEncoderIndexCounts(int &error = lastError);
  //void GenericCanbusRead(uint16_t *_ID, uint8_t *_DLC, uint8_t *_Data);
  //void GenericCanbusWrite(uint16_t _ID, uint8_t *_DLC, uint8_t *_Data, int &error = lastError);
  float GetRegenerationCurrentLimit(int &error = lastError);
  long GetPositionSensorDigitalFilterLevel(int &error = lastError);
  long GetDigitalInputRegister(int &error = lastError);
  int GetDigitalOutput(int pinNumber, int &error = lastError);
};
#endif // SOLO_MOTOR_CONTROLLERS_CANOPEN_NATIVE_H
#endif // ARDUINO_PORTENTA_C33 ARDUINO_UNOWIFIR4 ARDUINO_MINIMA