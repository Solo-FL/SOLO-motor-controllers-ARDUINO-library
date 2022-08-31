// Copyright: (c) 2021, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers Arduino Library
*    Author: SOLOMotorControllers
*    Date: 2022
*    Code version: 3.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library

This Library is made by SOLOMotorControllers.COM
please visit:  https://www.SOLOMotorControllers.com/

*/
#include <stdint.h>
#include "MCP2515.hpp"
#include "Arduino.h"
#include "SOLOMotorControllers.h"
#include "SOLOMotorControllersUtils.h"

#ifndef SOLOMOTORCONTROLLERSCANOPEN_H       //Avoid loading SOLOMotorControllersCanopen.h more than once
#define SOLOMOTORCONTROLLERSCANOPEN_H

//SOLO Object Index
//***** NMT Control Objects
#define Object_ReadErrorRegister                       0x1001
#define Object_GuardTime                               0x100C
#define Object_LifeTimeFactor                          0x100D
#define Object_ProducerHeartbeatTime                   0x1017
//***** SOLO UNO CANOPEN Objects
#define Object_SetDeviceAddress                        0x3001
#define Object_CommandMode                             0x3002
#define Object_CurrentLimit                            0x3003
#define Object_TorqueReferenceIq                       0x3004
#define Object_SpeedReference                          0x3005
#define Object_PowerReference                          0x3006
#define Object_MotorParametersIdentification           0x3007
#define Object_EmergencyStop                           0x3008
#define Object_OutputPwmFrequencyKhz                   0x3009
#define Object_SpeedControllerKp                       0x300A
#define Object_SpeedControllerKi                       0x300B
#define Object_MotorDirection                          0x300C
#define Object_MotorResistance                         0x300D
#define Object_MotorInductance                         0x300E
#define Object_MotorPolesCounts                        0x300F
#define Object_IncrementalEncoderLines                 0x3010
#define Object_SpeedLimit                              0x3011
//Reserved Object                                      0x3012
#define Object_FeedbackControlMode                     0x3013
#define Object_ResetFactory                            0x3014
#define Object_MotorType                               0x3015
#define Object_ControlMode                             0x3016
#define Object_CurrentControllerKp                     0x3017
#define Object_CurrentControllerKi                     0x3018
//Reserved Object                                      0x3019
#define Object_MagnetizingCurrentIdReference           0x301A
#define Object_PositionReference                       0x301B
#define Object_PositionControllerKp                    0x301C
#define Object_PositionControllerKi                    0x301D
//Reserved Object                                      0x301E
#define Object_ResetPositionToZero                     0x301F
#define Object_OverwriteErrorRegister                  0x3020
#define Object_ObserverGainBldcPmsm                    0x3021
#define Object_ObserverGainBldcPmsmUltrafast           0x3022
#define Object_ObserverGainDc                          0x3023
#define Object_FilterGainBldcPmsm                      0x3024
#define Object_FilterGainBldcPmsmUltrafast             0x3025
#define Object_UartBaudrate                            0x3026
#define Object_SensorCalibration                       0x3027
#define Object_EncoderHallCcwOffset                    0x3028
#define Object_EncoderHallCwOffset                     0x3029
#define Object_SpeedAccelerationValue                  0x302A
#define Object_SpeedDecelerationValue                  0x302B
#define Object_CanbusBaudrate                          0x302C
#define Object_PhaseAVoltage                           0x302D
#define Object_PhaseBVoltage                           0x302E
#define Object_PhaseACurrent                           0x302F
#define Object_PhaseBCurrent                           0x3030
#define Object_BusVoltage                              0x3031
#define Object_DcMotorCurrentIm                        0x3032
#define Object_DcMotorVoltageVm                        0x3033
#define Object_QuadratureCurrentIqFeedback             0x3034
#define Object_MagnetizingCurrentIdFeedback            0x3035
#define Object_SpeedFeedback                           0x3036  
#define Object_PositionCountsFeedback                  0x3037
#define Object_3PhaseMotorAngle                        0x3038
#define Object_BoardTemperature                        0x3039
#define Object_DeviceFirmwareVersion                   0x303A
#define Object_DeviceHardwareVersion                   0x303B
#define Object_EncoderIndexCounts                      0x303D

class SOLOMotorControllersCanopen : public SOLOMotorControllers
{

private:
    uint8_t   Address;
    MCP2515 *_MCP2515;
    SOLOMotorControllersUtils* soloUtils;
    long countTimeout;

public:
    SOLOMotorControllersCanopen(unsigned char _deviceAddress = 0, unsigned char _chipSelectPin = 9, SOLOMotorControllers::CanbusBaudrate _baudrate = SOLOMotorControllers::CanbusBaudrate::rate1000,  long _countTimeout = 6000) ; //SOLOMotorControllers::CanbusBaudrate , ? 
    
private :
    float ConvertToFloat(unsigned char data[]);
    long  ConvertToLong(unsigned char data[]);
    void  ConvertToData(float f, unsigned char data[]);
    void  ConvertToData(long l, unsigned char data[]);
    void  ExtractData(unsigned char _Data[] , unsigned char _ExtractedData[]);
public:
    //----------Write  SOLOMotorControllersCanopen----------
    bool SetGuardTime(long guardtime, int &error);
    bool SetGuardTime(long guardtime);
    bool SetLifeTimeFactor(long lifeTimeFactor, int &error);
    bool SetLifeTimeFactor(long lifeTimeFactor);
    bool SetProducerHeartbeatTime(long producerHeartbeatTime, int &error);
    bool SetProducerHeartbeatTime(long producerHeartbeatTime);
    //----------Write  SOLOMotorControllers----------   
    bool SetDeviceAddress(unsigned char deviceAddress, int &error);
    bool SetDeviceAddress(unsigned char deviceAddress);
    bool SetCommandMode(SOLOMotorControllers::CommandMode mode, int &error);
    bool SetCommandMode(SOLOMotorControllers::CommandMode mode);
    bool SetCurrentLimit(float currentLimit, int &error);
    bool SetCurrentLimit(float currentLimit);
    bool SetTorqueReferenceIq(float torqueReferenceIq, int &error);
    bool SetTorqueReferenceIq(float torqueReferenceIq);
    bool SetSpeedReference(long speedReference, int &error);
    bool SetSpeedReference(long speedReference);
    bool SetPowerReference(float powerReference, int &error);
    bool SetPowerReference(float powerReference);
    bool MotorParametersIdentification(SOLOMotorControllers::Action identification, int &error);
    bool MotorParametersIdentification(SOLOMotorControllers::Action identification);
    bool EmergencyStop(int &error);
    bool EmergencyStop();
    bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int &error);
    bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz);
    bool SetSpeedControllerKp(float speedControllerKp, int &error);
    bool SetSpeedControllerKp(float speedControllerKp);
    bool SetSpeedControllerKi(float speedControllerKi, int &error);
    bool SetSpeedControllerKi(float speedControllerKi);
    bool SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error); 
    bool SetMotorDirection(SOLOMotorControllers::Direction motorDirection); 
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
    bool SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode, int &error); 
    bool SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode); 
    bool ResetFactory(int &error);
    bool ResetFactory();
    bool ResetDeviceAddress(int &error);
    bool ResetDeviceAddress();
    bool SetMotorType(SOLOMotorControllers::MotorType motorType, int &error); 
    bool SetMotorType(SOLOMotorControllers::MotorType motorType);  
    bool SetControlMode(SOLOMotorControllers::ControlMode controlMode, int &error); 
    bool SetControlMode(SOLOMotorControllers::ControlMode controlMode); 
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
    bool SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate, int &error);
    bool SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate);
    bool SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction, int &error);
    bool SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction);
    bool SetEncoderHallCcwOffset(float encoderHallOffset, int &error);
    bool SetEncoderHallCcwOffset(float encoderHallOffset);
    bool SetEncoderHallCwOffset(float encoderHallOffset, int &error);
    bool SetEncoderHallCwOffset(float encoderHallOffset);
    bool SetSpeedAccelerationValue(float speedAccelerationValue, int &error);
    bool SetSpeedAccelerationValue(float speedAccelerationValue);
    bool SetSpeedDecelerationValue(float speedDecelerationValue, int &error);
    bool SetSpeedDecelerationValue(float speedDecelerationValue);
    bool SetCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate canbusBaudrate, int &error);
    bool SetCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate canbusBaudrate);
    //----------Read SOLOMotorControllersCanopen----------
    void  MCP2515_ReadErrorMode(int &errorMode);
    uint8_t MCP2515_Read_ReceiveErrorCounter();
    uint8_t MCP2515_Read_TransmitErrorCounter();
    //----------Read SOLOMotorControllers----------
    long  GetReadErrorRegister(int &error);
    long  GetReadErrorRegister();
    long  GetGuardTime(int &error);
    long  GetGuardTime();
    long  GetLifeTimeFactor(int &error);
    long  GetLifeTimeFactor();
    long  GetProducerHeartbeatTime(int &error);
    long  GetProducerHeartbeatTime();
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
    bool  CommunicationIsWorking(int &error);
    bool  CommunicationIsWorking();
    long  GetEncoderIndexCounts(int &error);
    long  GetEncoderIndexCounts();
    void  Generic_Canbus_Read_MCP2515(uint16_t *_ID, uint8_t *_DLC, uint8_t *_Data);
    void  Generic_Canbus_Write_MCP2515(uint16_t _ID, uint8_t *_DLC, uint8_t *_Data, int &error);
};



#endif
