/**
 *******************************************************************************
 * @file    SOLOMotorControllersUart.h
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the utility common functions
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

#ifndef SOLO_MOTOR_CONTROLLERS_UTILS_H
#define SOLO_MOTOR_CONTROLLERS_UTILS_H

#include "SOLOMotorControllers.h"

class SOLOMotorControllersUtils
{
public:
        float ConvertToFloat(unsigned char data[]);
        long ConvertToLong(unsigned char data[]);
        void ConvertToData(float f, unsigned char data[]);
        void ConvertToData(long l, unsigned char data[]);
        void SplitData(unsigned char data[], unsigned char cmd[]);
        void ExtractData(unsigned char _Data[], unsigned char _ExtractedData[]);

        //-- input Validation
        bool SetGuardTimeInputValidation(long guardtime, int &error);
        bool SetLifeTimeFactorInputValidation(long lifeTimeFactor, int &error);
        bool SetProducerHeartbeatTimeInputValidation(long producerHeartbeatTime, int &error);
        bool SetDeviceAddressInputValidation(unsigned char deviceAddress, int &error);
        bool SetCurrentLimitInputValidation(float currentLimit, int &error);
        bool SetTorqueReferenceIqInputValidation(float torqueReferenceIq, int &error);
        bool SetSpeedReferenceInputValidation(long speedReference, int &error);
        bool SetPowerReferenceInputValidation(float powerReference, int &error);
        bool SetOutputPwmFrequencyKhzInputValidation(long outputPwmFrequencyKhz, int &error);
        bool SetSpeedControllerKpInputValidation(float speedControllerKp, int &error);
        bool SetSpeedControllerKiInputValidation(float speedControllerKi, int &error);
        bool SetMotorResistanceInputValidation(float motorResistance, int &error);
        bool SetMotorInductanceInputValidation(float motorInductance, int &error);
        bool SetMotorPolesCountsInputValidation(long motorPolesCounts, int &error);
        bool SetIncrementalEncoderLinesInputValidation(long incrementalEncoderLines, int &error);
        bool SetSpeedLimitInputValidation(long speedLimit, int &error);
        bool SetCurrentControllerKpInputValidation(float currentControllerKp, int &error);
        bool SetCurrentControllerKiInputValidation(float currentControllerKi, int &error);
        bool SetMagnetizingCurrentIdReferenceInputValidation(float magnetizingCurrentIdReference, int &error);
        bool SetPositionReferenceInputValidation(long positionReference, int &error);
        bool SetPositionControllerKpInputValidation(float positionControllerKp, int &error);
        bool SetPositionControllerKiInputValidation(float positionControllerKi, int &error);
        bool SetZsftInjectionAmplitudeValidation(float amplitude, int &error);
        bool SetZsftPolarityAmplitudeValidation(float amplitude, int &error);
        bool SetObserverGainDcInputValidation(float observerGain, int &error);
        bool SetZsftInjectionFrequencyInputValidation(long frequency, int &error);
        bool SetSensorlessTransitionSpeedInputValidation(long speed, int &error);
        bool SetFilterGainBldcPmsmInputValidation(float filterGain, int &error);
        bool SetFilterGainBldcPmsmUltrafastInputValidation(float filterGain, int &error);
        bool SetEncoderHallCcwOffsetInputValidation(float encoderHallOffset, int &error);
        bool SetEncoderHallCwOffsetInputValidation(float encoderHallOffset, int &error);
        bool SetSpeedAccelerationValueInputValidation(float speedAccelerationValue, int &error);
        bool SetSpeedDecelerationValueInputValidation(float speedDecelerationValue, int &error);
        bool SetAnalogueSpeedResolutionDivisionCoefficientInputValidation(float divisionCoefficient, int &error);
        bool SetMotionProfileVariable1InputValidation(float MotionProfileVariable1, int &error);
        bool SetMotionProfileVariable2InputValidation(float MotionProfileVariable2, int &error);
        bool SetMotionProfileVariable3InputValidation(float MotionProfileVariable3, int &error);
        bool SetMotionProfileVariable4InputValidation(float MotionProfileVariable4, int &error);
        bool SetMotionProfileVariable5InputValidation(float MotionProfileVariable5, int &error);
        bool SetRegenerationCurrentLimitValidation(float current, int &error);
        bool SetPositionSensorDigitalFilterLevelValidation(long level, int &error);
        bool DigitalInputValidation(int pinNumber, int &error);
};

#endif // SOLO_MOTOR_CONTROLLERS_UTILS_H