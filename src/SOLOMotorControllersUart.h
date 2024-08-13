/**
 *******************************************************************************
 * @file    SOLOMotorControllersUart.h
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the functions prototypes for the Solo Drivers
 *          uart communications.
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.4.0
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 *******************************************************************************
 */

#ifndef SOLO_MOTOR_CONTROLLERS_UART_H
#define SOLO_MOTOR_CONTROLLERS_UART_H

#include "SOLOMotorControllers.h"
#include "SOLOMotorControllersUtils.h"
#include <HardwareSerial.h>
#include "Arduino.h"

// UART_Commands UART Commands
// All uart command hex code
#define INITIATOR 0xFF // 0xFFFF
#define BROADCAST_ADDRESS 0xFF
#define ENDING 0xFE
#define ERROR 0xEE // 0xEEEEEEEE
#define CRC 0x00
#define WRITE_DEVICE_ADDRESS 0x01
#define WRITE_COMMAND_MODE 0x02
#define WRITE_CURRENT_LIMIT 0x03
#define WRITE_TORQUE_REFERENCE_IQ 0x04
#define WRITE_SPEED_REFERENCE 0x05
#define WRITE_POWER_REFERENCE 0x06
#define WRITE_MOTOR_PARAMETERS_IDENTIFICATION 0x07
#define WRITE_DRIVE_DISABLE_ENABLE 0x08
#define WRITE_OUTPUT_PWM_FREQUENCY_KHZ 0x09
#define WRITE_SPEED_CONTROLLER_KP 0x0A
#define WRITE_SPEED_CONTROLLER_KI 0x0B
#define WRITE_MOTOR_DIRECTION 0x0C
#define WRITE_MOTOR_RESISTANCE 0x0D
#define WRITE_MOTOR_INDUCTANCE 0x0E
#define WRITE_MOTOR_POLES_COUNTS 0x0F
#define WRITE_INCREMENTAL_ENCODER_LINES 0x10
#define WRITE_SPEED_LIMIT 0x11
#define WRITE_RESET_DEVICE_ADDRESS 0x12
#define WRITE_FEEDBACK_CONTROL_MODE 0x13
#define WRITE_RESET_FACTORY 0x14
#define WRITE_MOTOR_TYPE 0x15
#define WRITE_CONTROL_MODE 0x16
#define WRITE_CURRENT_CONTROLLER_KP 0x17
#define WRITE_CURRENT_CONTROLLER_KI 0x18
#define WRITE_MONITORING_MODE 0x19
#define WRITE_MAGNETIZING_CURRENT_ID_REFERENCE 0x1A
#define WRITE_POSITION_REFERENCE 0x1B
#define WRITE_POSITION_CONTROLLER_KP 0x1C
#define WRITE_POSITION_CONTROLLER_KI 0x1D
#define WRITE_RESET_POSITION_TO_ZERO 0x1F // Home
#define WRITE_OVERWRITE_ERROR_REGISTER 0x20
#define WRITE_ZSFT_INJECTION_AMPLITUDE 0x21
#define WRITE_ZSFT_POLARITY_AMPLITUDE 0x22
#define WRITE_OBSERVER_GAIN_DC 0x23
#define WRITE_ZSFT_INJECTION_FREQUENCY 0x24
#define WRITE_SENSORLESS_TRANSITION_SPEED 0x25
#define WRITE_UART_BAUDRATE 0x26 // Set UART line baud-rate - 937500 / 115200 [ bits/s]
#define WRITE_SENSOR_CALIBRATION 0x27
#define WRITE_ENCODER_HALL_CCW_OFFSET 0x28
#define WRITE_ENCODER_HALL_CW_OFFSET 0x29
#define WRITE_SPEED_ACCELERATION_VALUE 0x2A
#define WRITE_SPEED_DECELERATION_VALUE 0x2B
#define WRITE_CANBUS_BAUDRATE 0x2C
#define WRITE_ASRDC 0x2D
#define WRITE_MOTION_PROFILE_MODE 0x30
#define WRITE_MOTION_PROFILE_VARIABLE1 0x31
#define WRITE_MOTION_PROFILE_VARIABLE2 0x32
#define WRITE_MOTION_PROFILE_VARIABLE3 0x33
#define WRITE_MOTION_PROFILE_VARIABLE4 0x34
#define WRITE_MOTION_PROFILE_VARIABLE5 0x35
#define WRITE_DIGITAL_OUTPUTS_REGISTER 0x38
#define WRITE_REGENERATION_CURRENT_LIMIT 0x39
#define WRITE_POSITION_SENSOR_DIGITAL_FILTER_LEVEL 0x3A

#define READ_DEVICE_ADDRESS 0x81
#define READ_PHASE_A_VOLTAGE 0x82
#define READ_PHASE_B_VOLTAGE 0x83
#define READ_PHASE_A_CURRENT 0x84
#define READ_PHASE_B_CURRENT 0x85
#define READ_BUS_VOLTAGE 0x86
#define READ_DC_MOTOR_CURRENT_IM 0x87
#define READ_DC_MOTOR_VOLTAGE_VM 0x88
#define READ_SPEED_CONTROLLER_KP 0x89
#define READ_SPEED_CONTROLLER_KI 0x8A
#define READ_OUTPUT_PWM_FREQUENCY_HZ 0x8B
#define READ_CURRENT_LIMIT 0x8C
#define READ_QUADRATURE_CURRENT_IQ_FEEDBACK 0x8D
#define READ_MAGNETIZING_CURRENT_ID_FEEDBACK 0x8E // Magnetizing
#define READ_MOTOR_POLES_COUNTS 0x8F
#define READ_INCREMENTAL_ENCODER_LINES 0x90
#define READ_CURRENT_CONTROLLER_KP 0x91
#define READ_CURRENT_CONTROLLER_KI 0x92
#define READ_BOARD_TEMPERATURE 0x93
#define READ_MOTOR_RESISTANCE 0x94
#define READ_MOTOR_INDUCTANCE 0x95
#define READ_SPEED_FEEDBACK 0x96
#define READ_MOTOR_TYPE 0x97
#define READ_FEEDBACK_CONTROL_MODE 0x99
#define READ_COMMAND_MODE 0x9A
#define READ_CONTROL_MODE 0x9B
#define READ_SPEED_LIMIT 0x9C
#define READ_POSITION_CONTROLLER_KP 0x9D
#define READ_POSITION_CONTROLLER_KI 0x9E
#define READ_POSITION_COUNTS_FEEDBACK 0xA0
#define READ_ERROR_REGISTER 0xA1
#define READ_DEVICE_FIRMWARE_VERSION 0xA2
#define READ_DEVICE_HARDWARE_VERSION 0xA3
#define READ_TORQUE_REFERENCE_IQ 0xA4              // Read Torque /Iq Reference
#define READ_SPEED_REFERENCE 0xA5                  // Read Speed Reference
#define READ_MAGNETIZING_CURRENT_ID_REFERENCE 0xA6 // Read Magnetizing Current / Id Reference
#define READ_POSITION_REFERENCE 0xA7
#define READ_POWER_REFERENCE 0xA8
#define READ_MOTOR_DIRECTION 0xA9
#define READ_ZSFT_INJECTION_AMPLITUDE 0xAA
#define READ_ZSFT_POLARITY_AMPLITUDE 0xAB
#define READ_OBSERVER_GAIN_DC 0xAC
#define READ_ZSFT_INJECTION_FREQUENCY 0xAD
#define READ_SENSORLESS_TRANSITION_SPEED 0xAE
#define READ_3_PHASE_MOTOR_ANGLE 0xB0
#define READ_ENCODER_HALL_CCW_OFFSET 0xB1
#define READ_ENCODER_HALL_CW_OFFSET 0xB2
#define READ_UART_BAUDRATE 0xB3 // 0 / 1 ( 937500 / 115200 [bits/s] )
#define READ_SPEED_ACCELERATION_VALUE 0xB4
#define READ_SPEED_DECELERATION_VALUE 0xB5
#define READ_CANBUS_BAUDRATE 0xB6
#define READ_ASRDC 0xB7
#define READ_ENCODER_INDEX_COUNTS 0xB8
#define READ_MOTION_PROFILE_MODE 0xBB
#define READ_MOTION_PROFILE_VARIABLE1 0xBC
#define READ_MOTION_PROFILE_VARIABLE2 0xBD
#define READ_MOTION_PROFILE_VARIABLE3 0xBE
#define READ_MOTION_PROFILE_VARIABLE4 0xBF
#define READ_MOTION_PROFILE_VARIABLE5 0xC0
#define READ_PT1000_SENSOR_VOLTAGE 0xC3
#define READ_DIGITAL_OUTPUT_REGISTER 0xC4
#define READ_DIGITAL_INPUT_REGISTER 0xC5
#define READ_ANALOGUE_INPUT 0xC6
#define READ_DRIVE_DISABLE_ENABLE 0xC7
#define READ_REGENERATION_CURRENT_LIMIT 0xC8
#define READ_POSITION_SENSOR_DIGITAL_FILTER_LEVEL 0x3A

/**
 * @brief a class for handle uart communication
 * */
class SOLOMotorControllersUart : public SOLOMotorControllers
{
private:
    unsigned char addr;
    HardwareSerial *serialToUse;
    long baudrate;
    long millisecondsTimeout;
    int packetFailureTrialAttempts;
    SOLOMotorControllersUtils *soloUtils;

public:
    SOLOMotorControllersUart(unsigned char _deviceAddress = 0, HardwareSerial &_serial = Serial, SOLOMotorControllers::UartBaudrate _baudrate = SOLOMotorControllers::UartBaudrate::RATE_115200, long _millisecondsTimeout = 50, int _packetFailureTrialAttempts = 5);
    static int lastError;

private:
    bool ExeCMD(unsigned char cmd[], int &error = lastError);

public:
    /** @addtogroup SOLOMotorControllersUart_Write_Functions SOLOMotorControllersUart Write Functions
     * @{
     */
    //----------Write----------
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
    bool SetRegenerationCurrentLimit(float current, int &error);
    bool SetPositionSensorDigitalFilterLevel(long level, int &error);
    bool SetDigitalOutputState(SOLOMotorControllers::Channel channel, SOLOMotorControllers::DigitalIoState state, int &error = lastError);
    /**
     * @}
     */

    /** @addtogroup SOLOMotorControllersUart_Read_Functions SOLOMotorControllersUart Read Functions
     * @{
     */
    //----------Read----------
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
    long GetEncoderIndexCounts(int &error = lastError);
    bool CommunicationIsWorking(int &error = lastError);
    SOLOMotorControllers::MotionProfileMode GetMotionProfileMode(int &error = lastError);
    float GetMotionProfileVariable1(int &error = lastError);
    float GetMotionProfileVariable2(int &error = lastError);
    float GetMotionProfileVariable3(int &error = lastError);
    float GetMotionProfileVariable4(int &error = lastError);
    float GetMotionProfileVariable5(int &error = lastError);
    SOLOMotorControllers::DigitalIoState GetDigitalOutputsState(SOLOMotorControllers::Channel chaneel, int &error = lastError);
    long GetDigitalOutputsRegister(int &error = lastError);
    long GetPT1000SensorVoltage(int &error = lastError);
    SOLOMotorControllers::DisableEnable GetDriveDisableEnable(int &error = lastError);
    float GetRegenerationCurrentLimit(int &error = lastError);
    long GetPositionSensorDigitalFilterLevel(int &error = lastError);
    long GetDigitalInputRegister(int &error = lastError);
    SOLOMotorControllers::DigitalIoState GetAnalogueInput(SOLOMotorControllers::Channel channel, int &error = lastError);
    int GetDigitalOutput(int pinNumber, int &error = lastError);
    /**
     * @}
     */
};

#endif // SOLO_MOTOR_CONTROLLERS_UART_H