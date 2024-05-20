/**
 *******************************************************************************
 * @file    SOLOMotorControllersCanopen.h
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the base functions prototypes for the CANopen
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.3.0
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)
 *******************************************************************************
 */

#ifndef SOLO_MOTOR_CONTROLLERS_CANOPEN_H
#define SOLO_MOTOR_CONTROLLERS_CANOPEN_H

#include "SOLOMotorControllers.h"

// SOLO Object Index NMT_Control_Objects NMT Control Objects
#define Object_ReadErrorRegister 0x1001
#define Object_GuardTime 0x100C
#define Object_LifeTimeFactor 0x100D
#define Object_ProducerHeartbeatTime 0x1017

//SOLO_UNO_CANOPEN_Objects SOLO UNO CANOPEN Objects
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