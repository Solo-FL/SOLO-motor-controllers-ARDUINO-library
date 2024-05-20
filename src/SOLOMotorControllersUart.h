/**
 *******************************************************************************
 * @file    SOLOMotorControllersUart.h
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the functions prototypes for the Solo Drivers
 *          uart communications.
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

#ifndef SOLO_MOTOR_CONTROLLERS_UART_H
#define SOLO_MOTOR_CONTROLLERS_UART_H

#include "SOLOMotorControllers.h"
#include "SOLOMotorControllersUtils.h"
#include <HardwareSerial.h>
#include "Arduino.h"

// UART_Commands UART Commands
// All uart command hex code
#define ReadData 0x00  // 0x00000000
#define INITIATOR 0xFF // 0xFFFF
#define BroadcastAddress 0xFF
#define ENDING 0xFE
#define ERROR 0xEE // 0xEEEEEEEE
#define CRC 0x00
#define WriteDeviceAddres 0x01
#define WriteCommandMode 0x02
#define WriteCurrentLimit 0x03
#define WriteTorqueReferenceIq 0x04
#define WriteSpeedReference 0x05
#define WritePowerReference 0x06
#define WriteMotorParametersIdentification 0x07
#define WriteEmergencyStop 0x08
#define WriteOutputPwmFrequencyKhz 0x09
#define WriteSpeedControllerKp 0x0A
#define WriteSpeedControllerKi 0x0B
#define WriteMotorDirection 0x0C
#define WriteMotorResistance 0x0D
#define WriteMotorInductance 0x0E
#define WriteMotorPolesCounts 0x0F
#define WriteIncrementalEncoderLines 0x10
#define WriteSpeedLimit 0x11
#define WriteFeedbackControlMode 0x13
#define WriteResetFactory 0x14
#define WriteMotorType 0x15
#define WriteControlMode 0x16
#define WriteCurrentControllerKp 0x17
#define WriteCurrentControllerKi 0x18
#define WriteMonitoringMode 0x19
#define WriteMagnetizingCurrentIdReference 0x1A
#define WritePositionReference 0x1B
#define WritePositionControllerKp 0x1C
#define WritePositionControllerKi 0x1D
#define WriteOverwriteErrorRegister 0x20
#define WriteObserverGainBldcPmsm 0x21          // Set Sensorless Observer Gain for Normal Brushless Motor
#define WriteObserverGainBldcPmsmUltrafast 0x22 // Set Sensorless Observer Gain for Ultra-Fast Brushless Motor
#define WriteObserverGainDc 0x23                // Set Sensorless Observer Gain for DC Motor
#define WriteFilterGainBldcPmsm 0x24            // Set Sensorless Observer Filter Gain for Normal Brushless Motor
#define WriteFilterGainBldcPmsmUltrafast 0x25   // Set Sensorless Observer Filter Gain for ultra-fast Brushless Motor
#define WriteUartBaudrate 0x26                  // Set UART line baud-rate - 937500 / 115200 [ bits/s]
#define WriteSensorCalibration 0x27
#define WriteEncoderHallCcwOffset 0x28
#define WriteEncoderHallCwOffset 0x29
#define WriteSpeedAccelerationValue 0x2A
#define WriteSpeedDecelerationValue 0x2B
#define WriteCanbusBaudrate 0x2C
#define WriteASRDC 0x2D
#define WriteMotionProfileMode 0x30
#define WriteMotionProfileVariable1 0x31
#define WriteMotionProfileVariable2 0x32
#define WriteMotionProfileVariable3 0x33
#define WriteMotionProfileVariable4 0x34
#define WriteMotionProfileVariable5 0x35
#define WriteDigitalOutput 0x38

#define ReadDeviceAddress 0x81
#define ReadPhaseAVoltage 0x82
#define ReadPhaseBVoltage 0x83
#define ReadPhaseACurrent 0x84
#define ReadPhaseBCurrent 0x85
#define ReadBusVoltage 0x86
#define ReadDcMotorCurrentIm 0x87
#define ReadDcMotorVoltageVm 0x88
#define ReadSpeedControllerKp 0x89
#define ReadSpeedControllerKi 0x8A
#define ReadOutputPwmFrequencyHz 0x8B
#define ReadCurrentLimit 0x8C
#define ReadQuadratureCurrentIqFeedback 0x8D
#define ReadMagnetizingCurrentIdFeedback 0x8E // Magnetizing
#define ReadMotorPolesCounts 0x8F
#define ReadIncrementalEncoderLines 0x90
#define ReadCurrentControllerKp 0x91
#define ReadCurrentControllerKi 0x92
#define ReadBoardTemperature 0x93
#define ReadMotorResistance 0x94
#define ReadMotorInductance 0x95
#define ReadSpeedFeedback 0x96
#define ReadMotorType 0x97
#define ReadFeedbackControlMode 0x99
#define ReadCommandMode 0x9A
#define ReadControlMode 0x9B
#define ReadSpeedLimit 0x9C
#define ReadPositionControllerKp 0x9D
#define ReadPositionControllerKi 0x9E
#define ReadPositionCountsFeedback 0xA0
#define ReadErrorRegister 0xA1
#define ReadDeviceFirmwareVersion 0xA2
#define ReadDeviceHardwareVersion 0xA3
#define ReadTorqueReferenceIq 0xA4             // Read Torque /“Iq” Reference
#define ReadSpeedReference 0xA5                // Read Speed Reference
#define ReadMagnetizingCurrentIdReference 0xA6 // Read Magnetizing Current / “Id” Reference
#define ReadPositionReference 0xA7
#define ReadPowerReference 0xA8
#define ReadMotorDirection 0xA9
#define ReadObserverGainBldcPmsm 0xAA          // Read the Non-linear observer Gain for Normal Brushless motor in Sensorless mode
#define ReadObserverGainBldcPmsmUltrafast 0xAB // Read the Non-linear observer Gain for Ultra-fast Brushless motor in Sensorless mode
#define ReadObserverGainDc 0xAC                // Read the Non-linear observer Gain for DC motor in Sensorless mode
#define ReadFilterGainBldcPmsm 0xAD            // Read the Non-linear observer Filter Gain for Normal Brushless motor in Sensorless mode
#define ReadFilterGainBldcPmsmUltrafast 0xAE   // Read the Non-linear Filter Gain for Ultra-fast Brushless motor in Sensorless mode
#define Read3PhaseMotorAngle 0xB0              // Read Estimated or Measured Rotor Angle
#define ReadEncoderHallCcwOffset 0xB1
#define ReadEncoderHallCwOffset 0xB2
#define ReadUartBaudrate 0xB3 // 0 / 1 ( 937500 / 115200 [bits/s] )
#define ReadSpeedAccelerationValue 0xB4
#define ReadSpeedDecelerationValue 0xB5
#define ReadCanbusBaudrate 0xB6
#define ReadASRDC 0xB7
#define ReadEncoderIndexCounts 0xB8
#define ReadMotionProfileMode 0xBB
#define ReadMotionProfileVariable1 0xBC
#define ReadMotionProfileVariable2 0xBD
#define ReadMotionProfileVariable3 0xBE
#define ReadMotionProfileVariable4 0xBF
#define ReadMotionProfileVariable5 0xC0
#define ReadPt1000 0xC3
#define ReadDigitalOutput 0xC4


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
    uint8_t GetDigitalOutputs(int &error);


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
    bool ResetDeviceAddress(int &error = lastError);
    bool SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode, int &error = lastError);
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
    long GetEncoderIndexCounts(int &error = lastError);
    bool CommunicationIsWorking(int &error = lastError);
    long GetMotionProfileMode(int &error = lastError);
    float GetMotionProfileVariable1(int &error = lastError);
    float GetMotionProfileVariable2(int &error = lastError);
    float GetMotionProfileVariable3(int &error = lastError);
    float GetMotionProfileVariable4(int &error = lastError);
    float GetMotionProfileVariable5(int &error = lastError);
    float GetPt1000(int &error = lastError);
    int GetDigitalOutput(int pinNumber, int &error = lastError);
    /**
     * @}
     */
};

#endif // SOLO_MOTOR_CONTROLLERS_UART_H