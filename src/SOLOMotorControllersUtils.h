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
#include "SOLOMotorControllers.h"

class SOLOMotorControllersUtils {   
public:
        float ConvertToFloat(unsigned char data[]);
        long  ConvertToLong(unsigned char data[]);
        void  ConvertToData(float f, unsigned char data[]);
        void  ConvertToData(long l, unsigned char data[]);
        void  SplitData(unsigned char data[], unsigned char cmd[]);
        void  ExtractData(unsigned char _Data[] , unsigned char _ExtractedData[]);

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
        bool SetObserverGainBldcPmsmInputValidation(float observerGain, int &error);
        bool SetObserverGainBldcPmsmUltrafastInputValidation(float observerGain, int &error);
        bool SetObserverGainDcInputValidation(float observerGain, int &error);
        bool SetFilterGainBldcPmsmInputValidation(float filterGain, int &error);
        bool SetFilterGainBldcPmsmUltrafastInputValidation(float filterGain, int &error);
        bool SetEncoderHallCcwOffsetInputValidation(float encoderHallOffset, int &error);
        bool SetEncoderHallCwOffsetInputValidation(float encoderHallOffset, int &error);
        bool SetSpeedAccelerationValueInputValidation(float speedAccelerationValue, int &error);
        bool SetSpeedDecelerationValueInputValidation(float speedDecelerationValue, int &error);
};