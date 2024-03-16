/**
 *******************************************************************************
 * @file    SOLOMotorControllersCanopen.h
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

#ifndef SOLO_MOTOR_CONTROLLERS_CANOPEN_H
#define SOLO_MOTOR_CONTROLLERS_CANOPEN_H

#include "SOLOMotorControllers.h"
#include "SOLOMotorControllersUtils.h"
#include "MCP2515.hpp"
#include <Arduino.h>

// SOLO Object Index
/** @addtogroup NMT_Control_Objects NMT Control Objects
 * @{
 */
#define Object_ReadErrorRegister 0x1001
#define Object_GuardTime 0x100C
#define Object_LifeTimeFactor 0x100D
#define Object_ProducerHeartbeatTime 0x1017
/**
 * @}
 */

/** @addtogroup SOLO_UNO_CANOPEN_Objects SOLO UNO CANOPEN Objects
 * @{
 */
#define Object_SetDeviceAddress 0x3001
#define Object_CommandMode 0x3002
#define Object_CurrentLimit 0x3003
#define Object_TorqueReferenceIq 0x3004
#define Object_SpeedReference 0x3005
#define Object_PowerReference 0x3006
#define Object_MotorParametersIdentification 0x3007
#define Object_EmergencyStop 0x3008
#define Object_OutputPwmFrequencyKhz 0x3009
#define Object_SpeedControllerKp 0x300A
#define Object_SpeedControllerKi 0x300B
#define Object_MotorDirection 0x300C
#define Object_MotorResistance 0x300D
#define Object_MotorInductance 0x300E
#define Object_MotorPolesCounts 0x300F
#define Object_IncrementalEncoderLines 0x3010
#define Object_SpeedLimit 0x3011
// Reserved Object                                      0x3012
#define Object_FeedbackControlMode 0x3013
#define Object_ResetFactory 0x3014
#define Object_MotorType 0x3015
#define Object_ControlMode 0x3016
#define Object_CurrentControllerKp 0x3017
#define Object_CurrentControllerKi 0x3018
// Reserved Object                                      0x3019
#define Object_MagnetizingCurrentIdReference 0x301A
#define Object_PositionReference 0x301B
#define Object_PositionControllerKp 0x301C
#define Object_PositionControllerKi 0x301D
// Reserved Object                                      0x301E
#define Object_OverwriteErrorRegister 0x3020
#define Object_ObserverGainBldcPmsm 0x3021
#define Object_ObserverGainBldcPmsmUltrafast 0x3022
#define Object_ObserverGainDc 0x3023
#define Object_FilterGainBldcPmsm 0x3024
#define Object_FilterGainBldcPmsmUltrafast 0x3025
#define Object_UartBaudrate 0x3026
#define Object_SensorCalibration 0x3027
#define Object_EncoderHallCcwOffset 0x3028
#define Object_EncoderHallCwOffset 0x3029
#define Object_SpeedAccelerationValue 0x302A
#define Object_SpeedDecelerationValue 0x302B
#define Object_CanbusBaudrate 0x302C
#define Object_PhaseAVoltage 0x302D
#define Object_PhaseBVoltage 0x302E
#define Object_PhaseACurrent 0x302F
#define Object_PhaseBCurrent 0x3030
#define Object_BusVoltage 0x3031
#define Object_DcMotorCurrentIm 0x3032
#define Object_DcMotorVoltageVm 0x3033
#define Object_QuadratureCurrentIqFeedback 0x3034
#define Object_MagnetizingCurrentIdFeedback 0x3035
#define Object_SpeedFeedback 0x3036
#define Object_PositionCountsFeedback 0x3037
#define Object_3PhaseMotorAngle 0x3038
#define Object_BoardTemperature 0x3039
#define Object_DeviceFirmwareVersion 0x303A
#define Object_DeviceHardwareVersion 0x303B
#define Object_EncoderIndexCounts 0x303D
#define Object_Pt1000 0x3047
#define Object_DigitalOutput 0x3048

#define Object_ASRDC 0x303E
#define Object_MotionProfileMode 0x303F
#define Object_MotionProfileVariable1 0x3040
#define Object_MotionProfileVariable2 0x3041
#define Object_MotionProfileVariable3 0x3042
#define Object_MotionProfileVariable4 0x3043
#define Object_MotionProfileVariable5 0x3044
/**
 * @}
 */

#define PDO_PARAMETER_NAME_COUNT 12

#define RPDO_MIN_COBIB 0x200
#define RPDO_MAX_COBIB 0x27F
#define TPDO_MIN_COBIB 0x280
#define TPDO_MAX_COBIB 0x2FF



/**
 * @brief a class for handle canopen communication
 * */
class SOLOMotorControllersCanopen : public SOLOMotorControllers
{

public:
  /**
   * @brief  Pdo Parameter Name enumeration
   */
  enum PdoParameterName
  {
    POSITION_REFERENCE = 0,               /*!< target position [RPDO] */
    SPEED_REFERENCE = 1,                  /*!< target velocity [RPDO] */
    TORQUE_REFERENCE_IQ = 2,              /*!< target torque [RPDO] */
    MAGNETIZING_CURRENT_ID_REFERENCE = 3, /*!< target direct current [RPDO] */
    CONTROL_MODE = 4,                     /*!< control mode [RPDO] */
    MOTOR_DIRECTION = 5,                  /*!< motor direction [RPDO] */
    POSITION_COUNTS_FEEDBACK = 6,         /*!< feedback position [TPDO] */
    SPEED_FEEDBACK = 7,                   /*!< feedback velocity [TPDO] */
    QUADRATURE_CURRENT_IQ_FEEDBACK = 8,   /*!< feedback lq [TPDO] */
    MAGNETIZING_CURRENT_ID_FEEDBACK = 9,  /*!< feedback ld [TPDO] */
    ERROR_REGISTER = 10,                  /*!< error register [TPDO] */
    BOARD_TEMPERATURE = 11                /*!< board temperature [TPDO] */
  };

  /**
   * @brief a struct that include all the Parameter used during PDO configuration
   */
  typedef struct
  {
    PdoParameterName parameterName;
    int parameterCobId;
    bool isPdoParameterEnable;
    bool isRrtParameterEnable;
    int syncParameterCount;
  } PdoParameterConfig;

private:
  uint8_t Address;
  SOLOMotorControllersUtils *soloUtils;
  long millisecondsTimeout;
  // used to list internally object value for every PDO
  int pdoParameterObjectByPdoParameterName[PDO_PARAMETER_NAME_COUNT];
  uint8_t GetDigitalOutputs(int &error);

public:
  SOLOMotorControllersCanopen(unsigned char _deviceAddress = 0, unsigned char _chipSelectPin = 9, SOLOMotorControllers::CanbusBaudrate _baudrate = SOLOMotorControllers::CanbusBaudrate::RATE_1000,unsigned char _interruptPin = 2, SOLOMotorControllers::Frequency _frequency = SOLOMotorControllers::Frequency::RATE_16, long _millisecondsTimeout = 200);
  // Cob-ID user value for every PDO
  int pdoParameterCobIdByPdoParameterName[PDO_PARAMETER_NAME_COUNT];
  static int lastError;
  
public:
  /** @addtogroup SOLOMotorControllersCanopen_Write_Functions SOLOMotorControllersCanopen Write Functions
   * @{
   */
  //----------Write  SOLOMotorControllersCanopen----------
  bool SetGuardTime(long guardtime, int &error = lastError);
  bool SetLifeTimeFactor(long lifeTimeFactor, int &error = lastError);
  bool SetProducerHeartbeatTime(long producerHeartbeatTime, int &error = lastError);
  /**
   * @}
   */

  /** @addtogroup CANOpen_PDO_Functions CANOpen Functions for Work with PDO Objects
   * @{
   */

  bool SetPdoParameterConfig(PdoParameterConfig config, int &error = lastError);
  bool SetPdoParameterCobbIdInputValidation(PdoParameterName parameterName, int parameterCobbId, int &error = lastError);
  bool SetSyncParameterCountInputValidation(uint8_t parameterCount, int &error = lastError);
  long GetPdoParameterValueLong(PdoParameterName parameterName, int &error = lastError);
  float GetPdoParameterValueFloat(PdoParameterName parameterName, int &error = lastError);
  bool PdoRtrValidParameter(PdoParameterName parameterName, int &error = lastError);
  void InitPdoConfig();
  long GetPdoParameterCobId(PdoParameterName parameterName, int &error = lastError);
  bool SetPdoParameterValue(PdoParameterName parameterName, long value, int &error = lastError);
  bool SetPdoParameterValue(PdoParameterName parameterName, float value, int &error = lastError);

  bool SendPdoSync(int &error = lastError);
  bool SendPdoRtr(PdoParameterName parameterName, int &error = lastError);

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
  bool UpdatePdoParameterCobIdByPdoParameterName();

  /**
   * @}
   */

  /** @addtogroup SOLOMotorControllers_Write_Functions SOLOMotorControllers Write Functions
   * @{
   */
  //----------Write  SOLOMotorControllers----------
  bool SetDeviceAddress(unsigned char deviceAddress, int &error = lastError);
  bool SetCommandMode(SOLOMotorControllers::CommandMode mode, int &error = lastError);
  bool SetCurrentLimit(float currentLimit, int &error = lastError);
  bool SetTorqueReferenceIq(float torqueReferenceIq, int &error = lastError);
  bool SetSpeedReference(long speedReference, int &error = lastError);
  bool SetPowerReference(float powerReference, int &error = lastError);
  bool MotorParametersIdentification(SOLOMotorControllers::Action identification, int &error = lastError);
  bool EmergencyStop(int &error = lastError);
  bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int &error = lastError);
  bool SetSpeedControllerKp(float speedControllerKp, int &error = lastError);
  bool SetSpeedControllerKi(float speedControllerKi, int &error = lastError);
  bool SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error = lastError);
  bool SetMotorResistance(float motorResistance, int &error = lastError);
  bool SetMotorInductance(float motorInductance, int &error = lastError);
  bool SetMotorPolesCounts(long motorPolesCounts, int &error = lastError);
  bool SetIncrementalEncoderLines(long incrementalEncoderLines, int &error = lastError);
  bool SetSpeedLimit(long speedLimit, int &error = lastError);
  bool SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode, int &error = lastError);
  bool ResetFactory(int &error = lastError);
  bool ResetDeviceAddress(int &error = lastError);
  bool SetMotorType(SOLOMotorControllers::MotorType motorType, int &error = lastError);
  bool SetControlMode(SOLOMotorControllers::ControlMode controlMode, int &error = lastError);
  bool SetCurrentControllerKp(float currentControllerKp, int &error = lastError);
  bool SetCurrentControllerKi(float currentControllerKi, int &error = lastError);
  bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error = lastError);
  bool SetPositionReference(long positionReference, int &error = lastError);
  bool SetPositionControllerKp(float positionControllerKp, int &error = lastError);
  bool SetPositionControllerKi(float positionControllerKi, int &error = lastError);
  bool OverwriteErrorRegister(int &error = lastError);
  bool SetObserverGainBldcPmsm(float observerGain, int &error = lastError);
  bool SetObserverGainBldcPmsmUltrafast(float observerGain, int &error = lastError);
  bool SetObserverGainDc(float observerGain, int &error = lastError);
  bool SetFilterGainBldcPmsm(float filterGain, int &error = lastError);
  bool SetFilterGainBldcPmsmUltrafast(float filterGain, int &error = lastError);
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
  bool SetDigitalOutput(int pinNumber, SOLOMotorControllers::DigitalStatus digitalStatus, int &error = lastError);
  /**
   * @}
   */

  /** @addtogroup CANOpen_Read_Functions Specific CANOpen Read Functions
   * @{
   */
  long GetGuardTime(int &error = lastError);
  long GetLifeTimeFactor(int &error = lastError);
  long GetProducerHeartbeatTime(int &error = lastError);
  /**
   * @}
   */

  /** @addtogroup SOLOMotorControllersCanopen_Read_Functions SOLOMotorControllersCanopen Read Functions
   * @{
   */
  //----------Read SOLOMotorControllersCanopen----------
  void Mcp2515ReadErrorMode(int &errorMode = lastError);
  uint8_t Mcp2515ReadReceiveErrorCounter();
  uint8_t Mcp2515ReadTransmitErrorCounter();
  /**
   * @}
   */

  /** @addtogroup SOLOMotorControllers_Read_Functions SOLOMotorControllers Read Functions
   * @{
   */
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
  long GetMotorType(int &error = lastError);
  long GetFeedbackControlMode(int &error = lastError);
  long GetCommandMode(int &error = lastError);
  long GetControlMode(int &error = lastError);
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
  long GetMotorDirection(int &error = lastError);
  float GetObserverGainBldcPmsm(int &error = lastError);
  float GetObserverGainBldcPmsmUltrafast(int &error = lastError);
  float GetObserverGainDc(int &error = lastError);
  float GetFilterGainBldcPmsm(int &error = lastError);
  float GetFilterGainBldcPmsmUltrafast(int &error = lastError);
  float Get3PhaseMotorAngle(int &error = lastError); // Read Estimated or Measured Rotor Angle
  float GetEncoderHallCcwOffset(int &error = lastError);
  float GetEncoderHallCwOffset(int &error = lastError);
  long GetUartBaudrate(int &error = lastError);
  float GetSpeedAccelerationValue(int &error = lastError);
  float GetSpeedDecelerationValue(int &error = lastError);
  long GetCanbusBaudrate(int &error = lastError);
  long GetAnalogueSpeedResolutionDivisionCoefficient(int &error = lastError);
  bool CommunicationIsWorking(int &error = lastError);
  long GetMotionProfileMode(int &error = lastError);
  float GetMotionProfileVariable1(int &error = lastError);
  float GetMotionProfileVariable2(int &error = lastError);
  float GetMotionProfileVariable3(int &error = lastError);
  float GetMotionProfileVariable4(int &error = lastError);
  float GetMotionProfileVariable5(int &error = lastError);
  long GetEncoderIndexCounts(int &error = lastError);
  void GenericCanbusReadMcp2515(uint16_t *_ID, uint8_t *_DLC, uint8_t *_Data);
  void GenericCanbusWriteMcp2515(uint16_t _ID, uint8_t *_DLC, uint8_t *_Data, int &error = lastError);
  float GetPt1000(int &error = lastError);
  int GetDigitalOutput(int pinNumber, int &error = lastError);
};
/**
 * @}
 */

#endif // SOLO_MOTOR_CONTROLLERS_CANOPEN_H