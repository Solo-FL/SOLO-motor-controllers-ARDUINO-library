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
#ifndef GRANDPARENT_H
#define GRANDPARENT_H

class SOLOMotorControllers {   
public:
enum Error
    {
        noErrorDetected = 0,
        generalError = 1,
        noProcessedCommand = 2,
        outOfRengeSetting = 3,
        packetFailureTrialAttemptsOverflow = 4,
        recieveTimeOutError = 5,
        Abort_Object = 6,
        Abort_Value  = 7,
        MCP2515_Transmit_ArbitrationLost = 8,
        MCP2515_Transmit_Error = 9
    };
    enum CommandMode
    {
        analogue = 0,
        digital = 1
    };
    enum Direction
    {
        clockwise = 0,
        counterclockwise = 1
    };
    enum FeedbackControlMode
    {
        sensorLess = 0,
        encoders = 1,
        hallSensors = 2
    };
    enum ControlMode
    {
        speedMode = 0,
        torqueMode = 1,
        positionMode = 2
    };
    enum MotorType
    {
        dc = 0,
        bldcPmsm = 1,
        acim = 2,
        bldcPmsmUltrafast = 3 
    }; 
    enum UartBaudrate //[bits/s]
    {
        rate937500 = 0,
        rate115200 = 1
    };
    enum CanbusBaudrate //[kbits/s]
    {
        rate1000 = 0,
        rate500 = 1,
        rate250 = 2,
        rate125 = 3,
        rate100 = 4
    };
    enum Action
    {
        stop = 0,
        start = 1
    }; 
    enum PositionSensorCalibrationAction
    {
        stopCalibration = 0,
        incrementalEncoderStartCalibration = 1,
        hallSensorStartCalibration = 2
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
    virtual bool ResetPositionToZero(int &error); //Home
    virtual bool ResetPositionToZero();
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

    //----------Read----------
    virtual long  GetDeviceAddress(int &error);
    virtual long  GetDeviceAddress();
    virtual float GetPhaseAVoltage(int &error);
    virtual float GetPhaseAVoltage();
    virtual float GetPhaseBVoltage(int &error);
    virtual float GetPhaseBVoltage();
    virtual float GetPhaseACurrent(int &error);
    virtual float GetPhaseACurrent();
    virtual float GetPhaseBCurrent(int &error);
    virtual float GetPhaseBCurrent();
    virtual float GetBusVoltage(int &error); //Battery Voltage
    virtual float GetBusVoltage();
    virtual float GetDcMotorCurrentIm(int &error);
    virtual float GetDcMotorCurrentIm();
    virtual float GetDcMotorVoltageVm(int &error);
    virtual float GetDcMotorVoltageVm();
    virtual float GetSpeedControllerKp(int &error);
    virtual float GetSpeedControllerKp();
    virtual float GetSpeedControllerKi(int &error);
    virtual float GetSpeedControllerKi();
    virtual long  GetOutputPwmFrequencyKhz(int &error);
    virtual long  GetOutputPwmFrequencyKhz();
    virtual float GetCurrentLimit(int &error);
    virtual float GetCurrentLimit();
    virtual float GetQuadratureCurrentIqFeedback(int &error);
    virtual float GetQuadratureCurrentIqFeedback();
    virtual float GetMagnetizingCurrentIdFeedback(int &error); //Magnetizing
    virtual float GetMagnetizingCurrentIdFeedback();
    virtual long  GetMotorPolesCounts(int &error);
    virtual long  GetMotorPolesCounts();
    virtual long  GetIncrementalEncoderLines(int &error);
    virtual long  GetIncrementalEncoderLines();
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
    virtual long  GetSpeedFeedback(int &error);
    virtual long  GetSpeedFeedback();
    virtual long  GetMotorType(int &error);
    virtual long  GetMotorType();
    virtual long  GetFeedbackControlMode(int &error);
    virtual long  GetFeedbackControlMode();
    virtual long  GetCommandMode(int &error);
    virtual long  GetCommandMode();
    virtual long  GetControlMode(int &error);
    virtual long  GetControlMode();
    virtual long  GetSpeedLimit(int &error);
    virtual long  GetSpeedLimit();
    virtual float GetPositionControllerKp(int &error);
    virtual float GetPositionControllerKp();
    virtual float GetPositionControllerKi(int &error);
    virtual float GetPositionControllerKi();
    virtual long  GetPositionCountsFeedback(int &error);
    virtual long  GetPositionCountsFeedback();
    virtual long  GetErrorRegister(int &error);
    virtual long  GetErrorRegister();
    virtual long  GetDeviceFirmwareVersion(int &error);
    virtual long  GetDeviceFirmwareVersion();
    virtual long  GetDeviceHardwareVersion(int &error);
    virtual long  GetDeviceHardwareVersion();
    virtual float GetTorqueReferenceIq(int &error);
    virtual float GetTorqueReferenceIq();
    virtual long  GetSpeedReference(int &error);
    virtual long  GetSpeedReference();
    virtual float GetMagnetizingCurrentIdReference(int &error);
    virtual float GetMagnetizingCurrentIdReference();
    virtual long  GetPositionReference(int &error);
    virtual long  GetPositionReference();
    virtual float GetPowerReference(int &error);
    virtual float GetPowerReference();
    virtual long  GetMotorDirection(int &error);
    virtual long  GetMotorDirection();
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
    virtual long  GetUartBaudrate(int &error);
    virtual long  GetUartBaudrate();
    virtual float GetSpeedAccelerationValue(int &error);
    virtual float GetSpeedAccelerationValue();
    virtual float GetSpeedDecelerationValue(int &error);
    virtual float GetSpeedDecelerationValue();
    virtual long  GetEncoderIndexCounts(int &error);
    virtual long  GetEncoderIndexCounts();
    virtual bool  CommunicationIsWorking(int &error);
    virtual bool  CommunicationIsWorking();
};
#endif