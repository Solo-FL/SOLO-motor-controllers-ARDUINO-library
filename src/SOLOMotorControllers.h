/**
 *******************************************************************************
 * @file    SOLOMotorControllers.h
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the base functions prototypes for the Solo Drivers
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.0.0
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)
 *******************************************************************************
 */

#ifndef SOLO_MOTOR_CONTROLLERS_H
#define SOLO_MOTOR_CONTROLLERS_H

class SOLOMotorControllers
{
public:
  /**
   * @brief  Error enumeration definition
   */
  enum Error
  {
    NO_ERROR_DETECTED = 0,                      /*!< no error detected */
    GENERAL_ERROR = 1,                          /*!< general error */
    NO_PROCESSED_COMMAND = 2,                   /*!< command is not valid */
    OUT_OF_RANGE_SETTING = 3,                   /*!< setting is out of range */
    PACKET_FAILURE_TRIAL_ATTEMPTS_OVERFLOW = 4, /*!< trial attempt overflow */
    RECEIVE_TIMEOUT_ERROR = 5,                  /*!< receive time error */
    ABORT_OBJECT = 6,                           /*!< abort object */
    ABORT_VALUE = 7,                            /*!< abort value */
    MCP2515_TRANSMIT_ARBITRATION_LOST = 8,      /*!< MCP2515 transmit arbitration lost */
    MCP2515_TRANSMIT_ERROR = 9,                 /*!< MCP2515 transmit error */
    OBJECT_NOT_INITIALIZE = 10,                 /*!< Kvaser object not initialize */
    CAN_EMPTY_BUFFER = 11,                      /*!< Kvaser buffer have no data for the defined COBID */
    PDO_PARAMETER_ID_OUT_OF_RANGE = 12,         /*!< PDO configuration id out of range */
    PDO_SYNC_OUT_OF_RANGE = 13,                 /*!< PDO configuration sync out of range */
    PDO_MISSING_COB_ID = 14,                    /*!< PDO no specific CobId for the specified pdo*/
    PDO_RTR_COMMAND_NOT_ALLOWED = 15,           /*!< PDO RTR specific command not allowed*/
  };

  /**
   * @brief  Command Mode enumeration definition
   */
  enum CommandMode
  {
    ANALOGUE = 0, /*!< Analogue Mode */
    DIGITAL = 1   /*!< Digital Mode */
  };

  /**
   * @brief  Direction enumeration definition
   */
  enum Direction
  {
    COUNTERCLOCKWISE = 0, /*!< counter-clockwise direction */
    CLOCKWISE = 1         /*!< clockwise direction */
  };

  /**
   * @brief  Feedback Control Mode enumeration definition
   */
  enum FeedbackControlMode
  {
    SENSORLESS = 0,  /*!< sensorless mode */
    ENCODERS = 1,    /*!< encoders mode */
    HALL_SENSORS = 2 /*!< hall sensors mode */
  };

  /**
   * @brief  Control Mode enumeration definition
   */
  enum ControlMode
  {
    SPEED_MODE = 0,   /*!< speed mode */
    TORQUE_MODE = 1,  /*!< torque mode */
    POSITION_MODE = 2 /*!< position mode */
  };

  /**
   * @brief  Motor Type enumeration definition
   */
  enum MotorType
  {
    DC = 0,                 /*!< dc motor */
    BLDC_PMSM = 1,          /*!< brushless dc motor  */
    ACIM = 2,               /*!< acim motor */
    BLDC_PMSM_ULTRAFAST = 3 /*!< brushless dc motor fast */
  };

  /**
   * @brief  Uart Baudrate enumeration definition
   */
  enum UartBaudrate //[bits/s]
  {
    RATE_937500 = 0, /*!< baud rate 937500 */
    RATE_115200 = 1  /*!< baud rate 115200 */
  };

  /**
   * @brief  Canbus Baudrate enumeration definition
   */
  enum CanbusBaudrate //[kbits/s]
  {
    RATE_1000 = 0, /*!< Baudrate 1000 kbits/s */
    RATE_500 = 1,  /*!< Baudrate 500 kbits/s */
    RATE_250 = 2,  /*!< Baudrate 250 kbits/s */
    RATE_125 = 3,  /*!< Baudrate 125 kbits/s */
    RATE_100 = 4   /*!< Baudrate 100 kbits/s */
  };

  /**
   * @brief  Action enumeration definition
   */
  enum Action
  {
    STOP = 0, /*!< stop */
    START = 1 /*!< start */
  };

  /**
   * @brief  Position Sensor Calibration Action enumeration definition
   */
  enum PositionSensorCalibrationAction
  {
    STOP_CALIBRATION = 0,                      /*!< stop colibration */
    INCREMENTAL_ENCODER_START_CALIBRATION = 1, /*!< incremental encoder start calibration */
    HALL_SENSOR_START_CALIBRATION = 2          /*!< hall sensor start calibration */
  };

  /**
   * @brief  Motion Profile Mode enumeration definition
   */
  enum MotionProfileMode
  {
    STEP_RAMP_RESPONSE = 0,   /*!< step ramp service */
    TIME_BASED_ST_CURVE = 1,  /*!< time based st curve */
    TIME_OPTIMAL_ST_CURVE = 2 /*!< time optimal st curve */
  };

public:
  //----------Write----------
  virtual bool SetDeviceAddress(unsigned char deviceAddress, int &error);
  virtual bool SetDeviceAddress(unsigned char deviceAddress);
  virtual bool SetCommandMode(SOLOMotorControllers::CommandMode mode, int &error);
  virtual bool SetCommandMode(SOLOMotorControllers::CommandMode mode);
  virtual bool SetCurrentLimit(float currentLimit, int &error);
  virtual bool SetCurrentLimit(float currentLimit);
  virtual bool SetTorqueReferenceIq(float torqueReferenceIq, int &error);
  virtual bool SetTorqueReferenceIq(float torqueReferenceIq);
  virtual bool SetSpeedReference(long speedReference, int &error);
  virtual bool SetSpeedReference(long speedReference);
  virtual bool SetPowerReference(float powerReference, int &error);
  virtual bool SetPowerReference(float powerReference);
  virtual bool MotorParametersIdentification(SOLOMotorControllers::Action identification, int &error);
  virtual bool MotorParametersIdentification(SOLOMotorControllers::Action identification);
  virtual bool EmergencyStop(int &error);
  virtual bool EmergencyStop();
  virtual bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int &error);
  virtual bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz);
  virtual bool SetSpeedControllerKp(float speedControllerKp, int &error);
  virtual bool SetSpeedControllerKp(float speedControllerKp);
  virtual bool SetSpeedControllerKi(float speedControllerKi, int &error);
  virtual bool SetSpeedControllerKi(float speedControllerKi);
  virtual bool SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error);
  virtual bool SetMotorDirection(SOLOMotorControllers::Direction motorDirection);
  virtual bool SetMotorResistance(float motorResistance, int &error);
  virtual bool SetMotorResistance(float motorResistance);
  virtual bool SetMotorInductance(float motorInductance, int &error);
  virtual bool SetMotorInductance(float motorInductance);
  virtual bool SetMotorPolesCounts(long motorPolesCounts, int &error);
  virtual bool SetMotorPolesCounts(long motorPolesCounts);
  virtual bool SetIncrementalEncoderLines(long incrementalEncoderLines, int &error);
  virtual bool SetIncrementalEncoderLines(long incrementalEncoderLines);
  virtual bool SetSpeedLimit(long speedLimit, int &error);
  virtual bool SetSpeedLimit(long speedLimit);
  virtual bool SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode, int &error);
  virtual bool SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode);
  virtual bool ResetFactory(int &error);
  virtual bool ResetFactory();
  virtual bool SetMotorType(SOLOMotorControllers::MotorType motorType, int &error);
  virtual bool SetMotorType(SOLOMotorControllers::MotorType motorType);
  virtual bool SetControlMode(SOLOMotorControllers::ControlMode controlMode, int &error);
  virtual bool SetControlMode(SOLOMotorControllers::ControlMode controlMode);
  virtual bool SetCurrentControllerKp(float currentControllerKp, int &error);
  virtual bool SetCurrentControllerKp(float currentControllerKp);
  virtual bool SetCurrentControllerKi(float currentControllerKi, int &error);
  virtual bool SetCurrentControllerKi(float currentControllerKi);
  virtual bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error);
  virtual bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference);
  virtual bool SetPositionReference(long positionReference, int &error);
  virtual bool SetPositionReference(long positionReference);
  virtual bool SetPositionControllerKp(float positionControllerKp, int &error);
  virtual bool SetPositionControllerKp(float positionControllerKp);
  virtual bool SetPositionControllerKi(float positionControllerKi, int &error);
  virtual bool SetPositionControllerKi(float positionControllerKi);
  virtual bool OverwriteErrorRegister(int &error);
  virtual bool OverwriteErrorRegister();
  virtual bool SetObserverGainBldcPmsm(float observerGain, int &error);
  virtual bool SetObserverGainBldcPmsm(float observerGain);
  virtual bool SetObserverGainBldcPmsmUltrafast(float observerGain, int &error);
  virtual bool SetObserverGainBldcPmsmUltrafast(float observerGain);
  virtual bool SetObserverGainDc(float observerGain, int &error);
  virtual bool SetObserverGainDc(float observerGain);
  virtual bool SetFilterGainBldcPmsm(float filterGain, int &error);
  virtual bool SetFilterGainBldcPmsm(float filterGain);
  virtual bool SetFilterGainBldcPmsmUltrafast(float filterGain, int &error);
  virtual bool SetFilterGainBldcPmsmUltrafast(float filterGain);
  virtual bool SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate, int &error);
  virtual bool SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate);
  virtual bool SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction, int &error);
  virtual bool SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction);
  virtual bool SetEncoderHallCcwOffset(float encoderHallOffset, int &error);
  virtual bool SetEncoderHallCcwOffset(float encoderHallOffset);
  virtual bool SetEncoderHallCwOffset(float encoderHallOffset, int &error);
  virtual bool SetEncoderHallCwOffset(float encoderHallOffset);
  virtual bool SetSpeedAccelerationValue(float speedAccelerationValue, int &error);
  virtual bool SetSpeedAccelerationValue(float speedAccelerationValue);
  virtual bool SetSpeedDecelerationValue(float speedDecelerationValue, int &error);
  virtual bool SetSpeedDecelerationValue(float speedDecelerationValue);
  virtual bool SetCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate canbusBaudrate, int &error);
  virtual bool SetCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate canbusBaudrate);
  virtual bool SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int &error);
  virtual bool SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient);
  virtual bool SetMotionProfileMode(MotionProfileMode motionProfileMode, int &error);
  virtual bool SetMotionProfileMode(MotionProfileMode motionProfileMode);
  virtual bool SetMotionProfileVariable1(float MotionProfileVariable1, int &error);
  virtual bool SetMotionProfileVariable1(float MotionProfileVariable1);
  virtual bool SetMotionProfileVariable2(float MotionProfileVariable2, int &error);
  virtual bool SetMotionProfileVariable2(float MotionProfileVariable2);
  virtual bool SetMotionProfileVariable3(float MotionProfileVariable3, int &error);
  virtual bool SetMotionProfileVariable3(float MotionProfileVariable3);
  virtual bool SetMotionProfileVariable4(float MotionProfileVariable4, int &error);
  virtual bool SetMotionProfileVariable4(float MotionProfileVariable4);
  virtual bool SetMotionProfileVariable5(float MotionProfileVariable5, int &error);
  virtual bool SetMotionProfileVariable5(float MotionProfileVariable5);

  //----------Read----------
  virtual long GetDeviceAddress(int &error);
  virtual long GetDeviceAddress();
  virtual float GetPhaseAVoltage(int &error);
  virtual float GetPhaseAVoltage();
  virtual float GetPhaseBVoltage(int &error);
  virtual float GetPhaseBVoltage();
  virtual float GetPhaseACurrent(int &error);
  virtual float GetPhaseACurrent();
  virtual float GetPhaseBCurrent(int &error);
  virtual float GetPhaseBCurrent();
  virtual float GetBusVoltage(int &error); // Battery Voltage
  virtual float GetBusVoltage();
  virtual float GetDcMotorCurrentIm(int &error);
  virtual float GetDcMotorCurrentIm();
  virtual float GetDcMotorVoltageVm(int &error);
  virtual float GetDcMotorVoltageVm();
  virtual float GetSpeedControllerKp(int &error);
  virtual float GetSpeedControllerKp();
  virtual float GetSpeedControllerKi(int &error);
  virtual float GetSpeedControllerKi();
  virtual long GetOutputPwmFrequencyKhz(int &error);
  virtual long GetOutputPwmFrequencyKhz();
  virtual float GetCurrentLimit(int &error);
  virtual float GetCurrentLimit();
  virtual float GetQuadratureCurrentIqFeedback(int &error);
  virtual float GetQuadratureCurrentIqFeedback();
  virtual float GetMagnetizingCurrentIdFeedback(int &error); // Magnetizing
  virtual float GetMagnetizingCurrentIdFeedback();
  virtual long GetMotorPolesCounts(int &error);
  virtual long GetMotorPolesCounts();
  virtual long GetIncrementalEncoderLines(int &error);
  virtual long GetIncrementalEncoderLines();
  virtual float GetCurrentControllerKp(int &error);
  virtual float GetCurrentControllerKp();
  virtual float GetCurrentControllerKi(int &error);
  virtual float GetCurrentControllerKi();
  virtual float GetBoardTemperature(int &error);
  virtual float GetBoardTemperature();
  virtual float GetMotorResistance(int &error);
  virtual float GetMotorResistance();
  virtual float GetMotorInductance(int &error);
  virtual float GetMotorInductance();
  virtual long GetSpeedFeedback(int &error);
  virtual long GetSpeedFeedback();
  virtual long GetMotorType(int &error);
  virtual long GetMotorType();
  virtual long GetFeedbackControlMode(int &error);
  virtual long GetFeedbackControlMode();
  virtual long GetCommandMode(int &error);
  virtual long GetCommandMode();
  virtual long GetControlMode(int &error);
  virtual long GetControlMode();
  virtual long GetSpeedLimit(int &error);
  virtual long GetSpeedLimit();
  virtual float GetPositionControllerKp(int &error);
  virtual float GetPositionControllerKp();
  virtual float GetPositionControllerKi(int &error);
  virtual float GetPositionControllerKi();
  virtual long GetPositionCountsFeedback(int &error);
  virtual long GetPositionCountsFeedback();
  virtual long GetErrorRegister(int &error);
  virtual long GetErrorRegister();
  virtual long GetDeviceFirmwareVersion(int &error);
  virtual long GetDeviceFirmwareVersion();
  virtual long GetDeviceHardwareVersion(int &error);
  virtual long GetDeviceHardwareVersion();
  virtual float GetTorqueReferenceIq(int &error);
  virtual float GetTorqueReferenceIq();
  virtual long GetSpeedReference(int &error);
  virtual long GetSpeedReference();
  virtual float GetMagnetizingCurrentIdReference(int &error);
  virtual float GetMagnetizingCurrentIdReference();
  virtual long GetPositionReference(int &error);
  virtual long GetPositionReference();
  virtual float GetPowerReference(int &error);
  virtual float GetPowerReference();
  virtual long GetMotorDirection(int &error);
  virtual long GetMotorDirection();
  virtual float GetObserverGainBldcPmsm(int &error);
  virtual float GetObserverGainBldcPmsm();
  virtual float GetObserverGainBldcPmsmUltrafast(int &error);
  virtual float GetObserverGainBldcPmsmUltrafast();
  virtual float GetObserverGainDc(int &error);
  virtual float GetObserverGainDc();
  virtual float GetFilterGainBldcPmsm(int &error);
  virtual float GetFilterGainBldcPmsm();
  virtual float GetFilterGainBldcPmsmUltrafast(int &error);
  virtual float GetFilterGainBldcPmsmUltrafast();
  virtual float Get3PhaseMotorAngle(int &error); // Read Estimated or Measured Rotor Angle
  virtual float Get3PhaseMotorAngle();
  virtual float GetEncoderHallCcwOffset(int &error);
  virtual float GetEncoderHallCcwOffset();
  virtual float GetEncoderHallCwOffset(int &error);
  virtual float GetEncoderHallCwOffset();
  virtual long GetUartBaudrate(int &error);
  virtual long GetUartBaudrate();
  virtual float GetSpeedAccelerationValue(int &error);
  virtual float GetSpeedAccelerationValue();
  virtual float GetSpeedDecelerationValue(int &error);
  virtual float GetSpeedDecelerationValue();
  virtual long GetCanbusBaudrate(int &error);
  virtual long GetCanbusBaudrate();
  virtual long GetAnalogueSpeedResolutionDivisionCoefficient(int &error);
  virtual long GetAnalogueSpeedResolutionDivisionCoefficient();
  virtual long GetEncoderIndexCounts(int &error);
  virtual long GetEncoderIndexCounts();
  virtual bool CommunicationIsWorking(int &error);
  virtual bool CommunicationIsWorking();
  virtual long GetMotionProfileMode(int &error);
  virtual long GetMotionProfileMode();
  virtual float GetMotionProfileVariable1(int &error);
  virtual float GetMotionProfileVariable1();
  virtual float GetMotionProfileVariable2(int &error);
  virtual float GetMotionProfileVariable2();
  virtual float GetMotionProfileVariable3(int &error);
  virtual float GetMotionProfileVariable3();
  virtual float GetMotionProfileVariable4(int &error);
  virtual float GetMotionProfileVariable4();
  virtual float GetMotionProfileVariable5(int &error);
  virtual float GetMotionProfileVariable5();
};
#endif // SOLO_MOTOR_CONTROLLERS_H