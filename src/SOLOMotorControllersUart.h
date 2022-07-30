// Copyright: (c) 2021, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers Arduino Library
*    Author: SOLOMotorControllers
*    Date: 2022
*    Code version: 4.0.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library

This Library is made by SOLOMotorControllers.COM
please visit:  https://www.SOLOMotorControllers.com/

*/
#include <stdint.h>
#include <HardwareSerial.h>
#include "Arduino.h"
#include "SOLOMotorControllers.h"
#include "SOLOMotorControllersUtils.h"

#define ReadData                            0x00 // 0x00000000
#define INITIATOR                           0xFF //0xFFFF
#define BroadcastAddress                    0xFF
#define ENDING                              0xFE
#define ERROR                               0xEE //0xEEEEEEEE
#define CRC                                 0x00
#define WriteDeviceAddres                   0x01
#define WriteCommandMode                    0x02
#define WriteCurrentLimit                   0x03
#define WriteTorqueReferenceIq              0x04
#define WriteSpeedReference                 0x05
#define WritePowerReference                 0x06
#define WriteMotorParametersIdentification  0x07
#define WriteEmergencyStop                  0x08
#define WriteOutputPwmFrequencyKhz          0x09
#define WriteSpeedControllerKp              0x0A
#define WriteSpeedControllerKi              0x0B
#define WriteMotorDirection                 0x0C
#define WriteMotorResistance                0x0D
#define WriteMotorInductance                0x0E
#define WriteMotorPolesCounts               0x0F
#define WriteIncrementalEncoderLines        0x10
#define WriteSpeedLimit                     0x11
#define WriteFeedbackControlMode            0x13
#define WriteResetFactory                   0x14
#define WriteMotorType                      0x15
#define WriteControlMode                    0x16
#define WriteCurrentControllerKp            0x17
#define WriteCurrentControllerKi            0x18
#define WriteMonitoringMode                 0x19
#define WriteMagnetizingCurrentIdReference  0x1A
#define WritePositionReference              0x1B
#define WritePositionControllerKp           0x1C
#define WritePositionControllerKi           0x1D
#define WriteResetPositionToZero            0x1F //Home
#define WriteOverwriteErrorRegister         0x20
#define WriteObserverGainBldcPmsm           0x21 //Set Sensorless Observer Gain for Normal Brushless Motor
#define WriteObserverGainBldcPmsmUltrafast  0x22 //Set Sensorless Observer Gain for Ultra-Fast Brushless Motor
#define WriteObserverGainDc                 0x23 //Set Sensorless Observer Gain for DC Motor
#define WriteFilterGainBldcPmsm             0x24 //Set Sensorless Observer Filter Gain for Normal Brushless Motor
#define WriteFilterGainBldcPmsmUltrafast    0x25 //Set Sensorless Observer Filter Gain for ultra-fast Brushless Motor
#define WriteUartBaudrate                   0x26 //Set UART line baud-rate - 937500 / 115200 [ bits/s]
#define WriteSensorCalibration              0x27
#define WriteEncoderHallCcwOffset           0x28
#define WriteEncoderHallCwOffset            0x29
#define WriteSpeedAccelerationValue         0x2A
#define WriteSpeedDecelerationValue         0x2B
#define WriteCanbusBaudrate                 0x2C

#define ReadDeviceAddress                   0x81
#define ReadPhaseAVoltage                   0x82
#define ReadPhaseBVoltage                   0x83
#define ReadPhaseACurrent                   0x84
#define ReadPhaseBCurrent                   0x85
#define ReadBusVoltage                      0x86
#define ReadDcMotorCurrentIm                0x87
#define ReadDcMotorVoltageVm                0x88
#define ReadSpeedControllerKp               0x89
#define ReadSpeedControllerKi               0x8A
#define ReadOutputPwmFrequencyHz            0x8B
#define ReadCurrentLimit                    0x8C
#define ReadQuadratureCurrentIqFeedback     0x8D
#define ReadMagnetizingCurrentIdFeedback    0x8E //Magnetizing
#define ReadMotorPolesCounts                0x8F
#define ReadIncrementalEncoderLines         0x90
#define ReadCurrentControllerKp             0x91
#define ReadCurrentControllerKi             0x92
#define ReadBoardTemperature                0x93
#define ReadMotorResistance                 0x94
#define ReadMotorInductance                 0x95
#define ReadSpeedFeedback                   0x96
#define ReadMotorType                       0x97
#define ReadFeedbackControlMode             0x99
#define ReadCommandMode                     0x9A
#define ReadControlMode                     0x9B
#define ReadSpeedLimit                      0x9C
#define ReadPositionControllerKp            0x9D
#define ReadPositionControllerKi            0x9E
#define ReadPositionCountsFeedback          0xA0
#define ReadErrorRegister                   0xA1
#define ReadDeviceFirmwareVersion           0xA2
#define ReadDeviceHardwareVersion           0xA3
#define ReadTorqueReferenceIq               0xA4 // Read Torque /“Iq” Reference
#define ReadSpeedReference                  0xA5 // Read Speed Reference
#define ReadMagnetizingCurrentIdReference   0xA6 // Read Magnetizing Current / “Id” Reference
#define ReadPositionReference               0xA7
#define ReadPowerReference                  0xA8
#define ReadMotorDirection                  0xA9
#define ReadObserverGainBldcPmsm            0xAA // Read the Non-linear observer Gain for Normal Brushless motor in Sensorless mode
#define ReadObserverGainBldcPmsmUltrafast   0xAB // Read the Non-linear observer Gain for Ultra-fast Brushless motor in Sensorless mode
#define ReadObserverGainDc                  0xAC // Read the Non-linear observer Gain for DC motor in Sensorless mode
#define ReadFilterGainBldcPmsm              0xAD // Read the Non-linear observer Filter Gain for Normal Brushless motor in Sensorless mode
#define ReadFilterGainBldcPmsmUltrafast     0xAE // Read the Non-linear Filter Gain for Ultra-fast Brushless motor in Sensorless mode
#define Read3PhaseMotorAngle                0xB0 // Read Estimated or Measured Rotor Angle
#define ReadEncoderHallCcwOffset            0xB1
#define ReadEncoderHallCwOffset             0xB2
#define ReadUartBaudrate                    0xB3 // 0 / 1 ( 937500 / 115200 [bits/s] )
#define ReadSpeedAccelerationValue          0xB4
#define ReadSpeedDecelerationValue          0xB5
#define ReadEncoderIndexCounts              0xB8

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
        SOLOMotorControllersUart(unsigned char _deviceAddress = 0, HardwareSerial &_serial = Serial ,  SOLOMotorControllers::UartBaudrate _baudrate = SOLOMotorControllers::UartBaudrate::rate115200, long _millisecondsTimeout = 200, int _packetFailureTrialAttempts = 5);

    private:
        bool ExeCMD(unsigned char cmd[], int &error);
        float ConvertToFloat(unsigned char data[]);
        void  ConvertToData(float f, unsigned char data[]);
        long ConvertToLong(unsigned char data[]);
        void ConvertToData(long l, unsigned char data[]);
        void SplitData(unsigned char data[], unsigned char cmd[]);

    public:
        //----------Write----------
        bool SetDeviceAddress(unsigned char deviceAddress, int &error);
        bool SetDeviceAddress(unsigned char deviceAddress);
        bool SetCommandMode(CommandMode mode, int &error);
        bool SetCommandMode(CommandMode mode);
        bool SetCurrentLimit(float currentLimit, int &error);
        bool SetCurrentLimit(float currentLimit);
        bool SetTorqueReferenceIq(float torqueReferenceIq, int &error);
        bool SetTorqueReferenceIq(float torqueReferenceIq);
        bool SetSpeedReference(long speedReference, int &error);
        bool SetSpeedReference(long speedReference);
        bool SetPowerReference(float powerReference, int &error);
        bool SetPowerReference(float powerReference);
        bool MotorParametersIdentification(Action identification, int &error);
        bool MotorParametersIdentification(Action identification);
        bool EmergencyStop(int &error);
        bool EmergencyStop();
        bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int &error);
        bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz);
        bool SetSpeedControllerKp(float speedControllerKp, int &error);
        bool SetSpeedControllerKp(float speedControllerKp);
        bool SetSpeedControllerKi(float speedControllerKi, int &error);
        bool SetSpeedControllerKi(float speedControllerKi);
        bool SetMotorDirection(Direction motorDirection, int &error); 
        bool SetMotorDirection(Direction motorDirection); 
        bool SetMotorResistance(float motorResistance, int &error); 
        bool SetMotorResistance(float motorResistance);
        bool SetMotorInductance(float motorInductance, int &error);
        bool SetMotorInductance(float motorInductance);
        bool SetMotorPolesCounts(long motorPolesCounts, int &error);
        bool SetMotorPolesCounts(long motorPolesCounts);
        bool SetIncrementalEncoderLines(long incrementalEncoderLines, int &error);
        bool SetIncrementalEncoderLines(long incrementalEncoderLines);
        bool SetSpeedLimit(long speedLimit, int &error);
        bool SetSpeedLimit(long speedLimit);
        bool ResetDeviceAddress(int &error);
        bool ResetDeviceAddress();
        bool SetFeedbackControlMode(FeedbackControlMode mode, int &error); 
        bool SetFeedbackControlMode(FeedbackControlMode mode); 
        bool ResetFactory(int &error);
        bool ResetFactory();
        bool SetMotorType(MotorType motorType, int &error); 
        bool SetMotorType(MotorType motorType); 
        bool SetControlMode(ControlMode controlMode, int &error); 
        bool SetControlMode(ControlMode controlMode); 
        bool SetCurrentControllerKp(float currentControllerKp, int &error);
        bool SetCurrentControllerKp(float currentControllerKp);
        bool SetCurrentControllerKi(float currentControllerKi, int &error);
        bool SetCurrentControllerKi(float currentControllerKi);
        bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error);
        bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference);
        bool SetPositionReference(long positionReference, int &error);
        bool SetPositionReference(long positionReference);
        bool SetPositionControllerKp(float positionControllerKp, int &error);
        bool SetPositionControllerKp(float positionControllerKp);
        bool SetPositionControllerKi(float positionControllerKi, int &error);
        bool SetPositionControllerKi(float positionControllerKi);
        bool ResetPositionToZero(int &error); //Home
        bool ResetPositionToZero();
        bool OverwriteErrorRegister(int &error);
        bool OverwriteErrorRegister();
        bool SetObserverGainBldcPmsm(float observerGain, int &error);
        bool SetObserverGainBldcPmsm(float observerGain);
        bool SetObserverGainBldcPmsmUltrafast(float observerGain, int &error);
        bool SetObserverGainBldcPmsmUltrafast(float observerGain);
        bool SetObserverGainDc(float observerGain, int &error);
        bool SetObserverGainDc(float observerGain);
        bool SetFilterGainBldcPmsm(float filterGain, int &error);
        bool SetFilterGainBldcPmsm(float filterGain);
        bool SetFilterGainBldcPmsmUltrafast(float filterGain, int &error);
        bool SetFilterGainBldcPmsmUltrafast(float filterGain);
        bool SetUartBaudrate(UartBaudrate baudrate, int &error);
        bool SetUartBaudrate(UartBaudrate baudrate);
        bool SensorCalibration(PositionSensorCalibrationAction calibrationAction, int &error);
        bool SensorCalibration(PositionSensorCalibrationAction calibrationAction);
        bool SetEncoderHallCcwOffset(float encoderHallOffset, int &error);
        bool SetEncoderHallCcwOffset(float encoderHallOffset);
        bool SetEncoderHallCwOffset(float encoderHallOffset, int &error);
        bool SetEncoderHallCwOffset(float encoderHallOffset);
        bool SetSpeedAccelerationValue(float speedAccelerationValue, int &error);
        bool SetSpeedAccelerationValue(float speedAccelerationValue);
        bool SetSpeedDecelerationValue(float speedDecelerationValue, int &error);
        bool SetSpeedDecelerationValue(float speedDecelerationValue);
        bool SetCanbusBaudrate(CanbusBaudrate canbusBaudrate, int &error);
        bool SetCanbusBaudrate(CanbusBaudrate canbusBaudrate);
        //----------Read----------
        long  GetDeviceAddress(int &error);
        long  GetDeviceAddress();
        float GetPhaseAVoltage(int &error);
        float GetPhaseAVoltage();
        float GetPhaseBVoltage(int &error);
        float GetPhaseBVoltage();
        float GetPhaseACurrent(int &error);
        float GetPhaseACurrent();
        float GetPhaseBCurrent(int &error);
        float GetPhaseBCurrent();
        float GetBusVoltage(int &error); //Battery Voltage
        float GetBusVoltage();
        float GetDcMotorCurrentIm(int &error);
        float GetDcMotorCurrentIm();
        float GetDcMotorVoltageVm(int &error);
        float GetDcMotorVoltageVm();
        float GetSpeedControllerKp(int &error);
        float GetSpeedControllerKp();
        float GetSpeedControllerKi(int &error);
        float GetSpeedControllerKi();
        long  GetOutputPwmFrequencyKhz(int &error);
        long  GetOutputPwmFrequencyKhz();
        float GetCurrentLimit(int &error);
        float GetCurrentLimit();
        float GetQuadratureCurrentIqFeedback(int &error);
        float GetQuadratureCurrentIqFeedback();
        float GetMagnetizingCurrentIdFeedback(int &error); //Magnetizing
        float GetMagnetizingCurrentIdFeedback();
        long  GetMotorPolesCounts(int &error);
        long  GetMotorPolesCounts();
        long  GetIncrementalEncoderLines(int &error);
        long  GetIncrementalEncoderLines();
        float GetCurrentControllerKp(int &error);
        float GetCurrentControllerKp();
        float GetCurrentControllerKi(int &error);
        float GetCurrentControllerKi();
        float GetBoardTemperature(int &error);
        float GetBoardTemperature();
        float GetMotorResistance(int &error);
        float GetMotorResistance();
        float GetMotorInductance(int &error);
        float GetMotorInductance();
        long  GetSpeedFeedback(int &error);
        long  GetSpeedFeedback();
        long  GetMotorType(int &error);
        long  GetMotorType();
        long  GetFeedbackControlMode(int &error);
        long  GetFeedbackControlMode();
        long  GetCommandMode(int &error);
        long  GetCommandMode();
        long  GetControlMode(int &error);
        long  GetControlMode();
        long  GetSpeedLimit(int &error);
        long  GetSpeedLimit();
        float GetPositionControllerKp(int &error);
        float GetPositionControllerKp();
        float GetPositionControllerKi(int &error);
        float GetPositionControllerKi();
        long  GetPositionCountsFeedback(int &error);
        long  GetPositionCountsFeedback();
        long  GetErrorRegister(int &error);
        long  GetErrorRegister();
        long  GetDeviceFirmwareVersion(int &error);
        long  GetDeviceFirmwareVersion();
        long  GetDeviceHardwareVersion(int &error);
        long  GetDeviceHardwareVersion();
        float GetTorqueReferenceIq(int &error);
        float GetTorqueReferenceIq();
        long  GetSpeedReference(int &error);
        long  GetSpeedReference();
        float GetMagnetizingCurrentIdReference(int &error);
        float GetMagnetizingCurrentIdReference();
        long  GetPositionReference(int &error);
        long  GetPositionReference();
        float GetPowerReference(int &error);
        float GetPowerReference();
        long  GetMotorDirection(int &error);
        long  GetMotorDirection();
        float GetObserverGainBldcPmsm(int &error);
        float GetObserverGainBldcPmsm();
        float GetObserverGainBldcPmsmUltrafast(int &error);
        float GetObserverGainBldcPmsmUltrafast();
        float GetObserverGainDc(int &error);
        float GetObserverGainDc();
        float GetFilterGainBldcPmsm(int &error);
        float GetFilterGainBldcPmsm();
        float GetFilterGainBldcPmsmUltrafast(int &error);
        float GetFilterGainBldcPmsmUltrafast();
        float Get3PhaseMotorAngle(int &error); // Read Estimated or Measured Rotor Angle
        float Get3PhaseMotorAngle();
        float GetEncoderHallCcwOffset(int &error);
        float GetEncoderHallCcwOffset();
        float GetEncoderHallCwOffset(int &error);
        float GetEncoderHallCwOffset();
        long  GetUartBaudrate(int &error);
        long  GetUartBaudrate();
        float GetSpeedAccelerationValue(int &error);
        float GetSpeedAccelerationValue();
        float GetSpeedDecelerationValue(int &error);
        float GetSpeedDecelerationValue();
        long  GetEncoderIndexCounts(int &error);
        long  GetEncoderIndexCounts();
        bool CommunicationIsWorking(int &error);
        bool CommunicationIsWorking();
};
