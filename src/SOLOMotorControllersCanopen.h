/**
 *******************************************************************************
 * @file    SOLOMotorControllersCanopen.h
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the base functions prototypes for the CANopen
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2025
 * @version 5.5.0
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 *******************************************************************************
 */

#ifndef SOLO_MOTOR_CONTROLLERS_CANOPEN_H
#define SOLO_MOTOR_CONTROLLERS_CANOPEN_H

#include "SOLOMotorControllers.h"

// SOLO Object Index NMT_Control_Objects NMT Control Objects
#define OBJECT_READ_ERROR_REGISTER 0x1001
#define OBJECT_GUARD_TIME 0x100C
#define OBJECT_LIFE_TIME_FACTOR 0x100D
#define OBJECT_PRODUCER_HEARTBEAT_TIME 0x1017

// SOLO_UNO_CANOPEN_Objects SOLO UNO CANOPEN Objects
#define OBJECT_SET_DEVICE_ADDRESS 0x3001
#define OBJECT_COMMAND_MODE 0x3002
#define OBJECT_CURRENT_LIMIT 0x3003
#define OBJECT_TORQUE_REFERENCE_IQ 0x3004
#define OBJECT_SPEED_REFERENCE 0x3005
#define OBJECT_POWER_REFERENCE 0x3006
#define OBJECT_MOTOR_PARAMETERS_IDENTIFICATION 0x3007
#define OBJECT_DRIVE_DISABLE_ENABLE 0x3008
#define OBJECT_OUTPUT_PWM_FREQUENCY_KHZ 0x3009
#define OBJECT_SPEED_CONTROLLER_KP 0x300A
#define OBJECT_SPEED_CONTROLLER_KI 0x300B
#define OBJECT_MOTOR_DIRECTION 0x300C
#define OBJECT_MOTOR_RESISTANCE 0x300D
#define OBJECT_MOTOR_INDUCTANCE 0x300E
#define OBJECT_MOTOR_POLES_COUNTS 0x300F
#define OBJECT_INCREMENTAL_ENCODER_LINES 0x3010
#define OBJECT_SPEED_LIMIT 0x3011
// Reserved Object                                      0x3012
#define OBJECT_FEEDBACK_CONTROL_MODE 0x3013
#define OBJECT_RESET_FACTORY 0x3014
#define OBJECT_RESET_POSITION 0x301F
#define OBJECT_MOTOR_TYPE 0x3015
#define OBJECT_CONTROL_MODE 0x3016
#define OBJECT_CURRENT_CONTROLLER_KP 0x3017
#define OBJECT_CURRENT_CONTROLLER_KI 0x3018
// Reserved Object                                      0x3019
#define OBJECT_MAGNETIZING_CURRENT_ID_REFERENCE 0x301A
#define OBJECT_POSITION_REFERENCE 0x301B
#define OBJECT_POSITION_CONTROLLER_KP 0x301C
#define OBJECT_POSITION_CONTROLLER_KI 0x301D
// Reserved Object                                      0x301E
#define OBJECT_OVERWRITE_ERROR_REGISTER 0x3020
#define OBJECT_ZSFT_INJECTION_AMPLITUDE 0x3021
#define OBJECT_ZSFT_POLARITY_AMPLITUDE 0x3022
#define OBJECT_OBSERVER_GAIN_DC 0x3023
#define OBJECT_ZSFT_INJECTION_FREQUENCY 0x3024
#define OBJECT_SENSORLESS_TRANSACTION_SPEED 0x3025
#define OBJECT_UART_BAUDRATE 0x3026
#define OBJECT_SENSOR_CALIBRATION 0x3027
#define OBJECT_ENCODER_HALL_CCW_OFFSET 0x3028
#define OBJECT_ENCODER_HALL_CW_OFFSET 0x3029
#define OBJECT_SPEED_ACCELERATION_VALUE 0x302A
#define OBJECT_SPEED_DECELERATION_VALUE 0x302B
#define OBJECT_CANBUS_BAUDRATE 0x302C
#define OBJECT_PHASE_A_VOLTAGE 0x302D
#define OBJECT_PHASE_B_VOLTAGE 0x302E
#define OBJECT_PHASE_A_CURRENT 0x302F
#define OBJECT_PHASE_B_CURRENT 0x3030
#define OBJECT_BUS_VOLTAGE 0x3031
#define OBJECT_DC_MOTOR_CURRENT_IM 0x3032
#define OBJECT_DC_MOTOR_VOLTAGE_VM 0x3033
#define OBJECT_QUADRATURE_CURRENT_IQ_FEEDBACK 0x3034
#define OBJECT_MAGNETIZING_CURRENT_ID_FEEDBACK 0x3035
#define OBJECT_SPEED_FEEDBACK 0x3036
#define OBJECT_POSITION_COUNTS_FEEDBACK 0x3037
#define OBJECT_3_PHASE_MOTOR_ANGLE 0x3038
#define OBJECT_BOARD_TEMPERATURE 0x3039
#define OBJECT_DEVICE_FIRMWARE_VERSION 0x303A
#define OBJECT_DEVICE_HARDWARE_VERSION 0x303B
#define OBJECT_ENCODER_INDEX_COUNTS 0x303D
#define OBJECT_PT1000 0x3047
#define OBJECT_DIGITAL_OUTPUT 0x3048

#define OBJECT_ASRDC 0x303E
#define OBJECT_MOTION_PROFILE_MODE 0x303F
#define OBJECT_MOTION_PROFILE_VARIABLE1 0x3040
#define OBJECT_MOTION_PROFILE_VARIABLE2 0x3041
#define OBJECT_MOTION_PROFILE_VARIABLE3 0x3042
#define OBJECT_MOTION_PROFILE_VARIABLE4 0x3043
#define OBJECT_MOTION_PROFILE_VARIABLE5 0x3044
#define OBJECT_PT1000_SENSOR_VOLTAGE 0x3047
#define OBJECT_DIGITAL_OUTPUT_REGISTER 0x3048
#define OBJECT_DIGITAL_INPUT_REGISTER 0x3049
#define OBJECT_ANALOGUE_INPUT 0x304A
#define OBJECT_REGENERATION_CURRENT_LIMIT 0x304B
#define OBJECT_POSITION_SENSOR_DIGITAL_FILTER_LEVEL 0x304C

#define PDO_PARAMETER_NAME_COUNT 12

#define RPDO_MIN_COBIB 0x200
#define RPDO_MAX_COBIB 0x27F
#define TPDO_MIN_COBIB 0x280
#define TPDO_MAX_COBIB 0x2FF

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

public:
  /** @addtogroup SOLOMotorControllersCanopen_Write_Functions SOLOMotorControllersCanopen Write Functions
   * @{
   */
  //----------Write  SOLOMotorControllersCanopen----------
  virtual bool SetGuardTime(long guardtime, int &error);
  virtual bool SetLifeTimeFactor(long lifeTimeFactor, int &error);
  virtual bool SetProducerHeartbeatTime(long producerHeartbeatTime, int &error);
  /**
   * @}
   */

  /** @addtogroup CANOpen_PDO_Functions CANOpen Functions for Work with PDO Objects
   * @{
   */
  #if defined(ARDUINO_PORTENTA_C33) || defined(ARDUINO_UNOWIFIR4) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_PORTENTA_H7_M7)
    virtual bool SetPdoParameterConfig(PdoParameterConfig config, int &error);
    virtual bool SendPdoSync(int &error);

    virtual bool SetPdoPositionReference(long positionReference, int &error);
    virtual bool SetPdoSpeedReference(long speedReference, int &error);
    virtual bool SetPdoTorqueReferenceIq(float torqueReferenceIq, int &error);
    virtual bool SetPdoMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error);
    virtual bool SetPdoControlMode(SOLOMotorControllers::ControlMode controlMode, int &error);
    virtual bool SetPdoMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error);

    virtual long GetPdoPositionCountsFeedback(int &error);
    virtual long GetPdoSpeedFeedback(int &error);
    virtual float GetPdoQuadratureCurrentIqFeedback(int &error);
    virtual float GetPdoMagnetizingCurrentIdFeedback(int &error);
    virtual long GetPdoErrorRegister(int &error);
    virtual float GetPdoBoardTemperature(int &error);
    virtual PdoParameterConfig GetPdoParameterConfig(PdoParameterName parameterName, int &error);
    virtual bool UpdatePdoParameterCobIdByPdoParameterName();
  #endif // ARDUINO_PORTENTA_C33 ARDUINO_UNOWIFIR4 ARDUINO_MINIMA ARDUINO_PORTENTA_H7_M7

  /**
   * @}
   */

  /** @addtogroup CANOpen_Read_Functions Specific CANOpen Read Functions
   * @{
   */
  virtual long GetGuardTime(int &error);
  virtual long GetLifeTimeFactor(int &error);
  virtual long GetProducerHeartbeatTime(int &error);
  /**
   * @}
   */
};

#endif // SOLO_MOTOR_CONTROLLERS_CANOPEN_H