/**
 *******************************************************************************
 * @file    SOLOMotorControllersUtils.cpp
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the utility common functions
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.1.0
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)
 *******************************************************************************
 */

#include "SOLOMotorControllersUtils.h"

float SOLOMotorControllersUtils::ConvertToFloat(unsigned char data[])
{
    float dec = 0;
    dec = (long)data[0] << 24;
    dec += (long)data[1] << 16;
    dec += (long)data[2] << 8;
    dec += (long)data[3];

    if (dec <= 0x7FFE0000)
    {
        return (float)dec / 131072.0;
    }
    else
    {
        dec = 0xFFFFFFFF - dec + 1;
        return ((float)dec / 131072.0) * -1;
    }
}
void SOLOMotorControllersUtils::ConvertToData(float f, unsigned char data[])
{
    long dec = (long)(f * 131072);
    if (dec < 0)
    {
        dec *= -1;
        dec = 0xFFFFFFFF - dec;
    }
    data[0] = dec >> 24;
    dec = dec % 16777216;
    data[1] = dec >> 16;
    dec = dec % 65536;
    data[2] = dec >> 8;
    data[3] = dec % 256;
}
long SOLOMotorControllersUtils::ConvertToLong(unsigned char data[])
{
    long dec = 0;
    dec = (long)data[0] << 24;
    dec += (long)data[1] << 16;
    dec += (long)data[2] << 8;
    dec += (long)data[3];

    if (dec <= 2147483647 /*0x7FFFFFFF*/)
    {
        return dec;
    }
    else
    {
        dec = /*0xFFFFFFFF*/ 4294967295 - dec + 1;
        return dec * -1;
    }
}
void SOLOMotorControllersUtils::ConvertToData(long l, unsigned char data[])
{
    long dec = l;
    if (dec < 0)
    {
        dec *= -1;
        dec = 0xFFFFFFFF - dec + 1;
    }
    data[0] = dec >> 24;
    dec = dec % 16777216;
    data[1] = dec >> 16;
    dec = dec % 65536;
    data[2] = dec >> 8;
    data[3] = dec % 256;
}
void SOLOMotorControllersUtils::SplitData(unsigned char data[], unsigned char cmd[])
{
    data[0] = cmd[2];
    data[1] = cmd[3];
    data[2] = cmd[4];
    data[3] = cmd[5];
}

void SOLOMotorControllersUtils::ExtractData(unsigned char _Data[], unsigned char _ExtractedData[])
{

    _ExtractedData[0] = _Data[7];
    _ExtractedData[1] = _Data[6];
    _ExtractedData[2] = _Data[5];
    _ExtractedData[3] = _Data[4];
}
bool SOLOMotorControllersUtils::SetGuardTimeInputValidation(long guardtime, int &error)
{
    if (guardtime < 0 || guardtime > 65535)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetLifeTimeFactorInputValidation(long lifeTimeFactor, int &error)
{
    if (lifeTimeFactor < 0 || lifeTimeFactor > 255)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetProducerHeartbeatTimeInputValidation(long producerHeartbeatTime, int &error)
{
    if (producerHeartbeatTime < 0 || producerHeartbeatTime > 65535)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}

bool SOLOMotorControllersUtils::SetDeviceAddressInputValidation(unsigned char deviceAddress, int &error)
{
    if (deviceAddress < 0 || deviceAddress > 254)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetCurrentLimitInputValidation(float currentLimit, int &error)
{
    if (currentLimit < 0 || currentLimit > 32)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetTorqueReferenceIqInputValidation(float torqueReferenceIq, int &error)
{
    if (torqueReferenceIq < 0 || torqueReferenceIq > 32)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetSpeedReferenceInputValidation(long speedReference, int &error)
{
    if (speedReference < 0 || speedReference > 30000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetPowerReferenceInputValidation(float powerReference, int &error)
{
    if (powerReference < 0 || powerReference > 100)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetOutputPwmFrequencyKhzInputValidation(long outputPwmFrequencyKhz, int &error)
{
    if (outputPwmFrequencyKhz < 8 || outputPwmFrequencyKhz > 80)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetSpeedControllerKpInputValidation(float speedControllerKp, int &error)
{
    if (speedControllerKp < 0 || speedControllerKp > 300)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetSpeedControllerKiInputValidation(float speedControllerKi, int &error)
{
    if (speedControllerKi < 0 || speedControllerKi > 300)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetMotorResistanceInputValidation(float motorResistance, int &error)
{
    if (motorResistance < 0.001 || motorResistance > 50)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetMotorInductanceInputValidation(float motorInductance, int &error)
{
    if (motorInductance < 0.00001 || motorInductance > 0.2)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetMotorPolesCountsInputValidation(long motorPolesCounts, int &error)
{
    if (motorPolesCounts < 1 || motorPolesCounts > 254)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetIncrementalEncoderLinesInputValidation(long incrementalEncoderLines, int &error)
{
    if (incrementalEncoderLines < 1 || incrementalEncoderLines > 200000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetSpeedLimitInputValidation(long speedLimit, int &error)
{
    if (speedLimit < 1 || speedLimit > 30000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetCurrentControllerKpInputValidation(float currentControllerKp, int &error)
{
    if (currentControllerKp < 0 || currentControllerKp > 16000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetCurrentControllerKiInputValidation(float currentControllerKi, int &error)
{
    if (currentControllerKi < 0 || currentControllerKi > 16000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetMagnetizingCurrentIdReferenceInputValidation(float magnetizingCurrentIdReference, int &error)
{
    if (magnetizingCurrentIdReference < 0 || magnetizingCurrentIdReference > 32)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetPositionReferenceInputValidation(long positionReference, int &error)
{
    if (positionReference < -2147483647 || positionReference > 2147483647)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetPositionControllerKpInputValidation(float positionControllerKp, int &error)
{
    if (positionControllerKp < 0 || positionControllerKp > 16000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetPositionControllerKiInputValidation(float positionControllerKi, int &error)
{
    if (positionControllerKi < 0 || positionControllerKi > 16000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetObserverGainBldcPmsmInputValidation(float observerGain, int &error)
{
    if (observerGain < 0.01 || observerGain > 1000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetObserverGainBldcPmsmUltrafastInputValidation(float observerGain, int &error)
{
    if (observerGain < 0.01 || observerGain > 1000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetObserverGainDcInputValidation(float observerGain, int &error)
{
    if (observerGain < 0.01 || observerGain > 1000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetFilterGainBldcPmsmInputValidation(float filterGain, int &error)
{
    if (filterGain < 0.01 || filterGain > 16000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetFilterGainBldcPmsmUltrafastInputValidation(float filterGain, int &error)
{
    if (filterGain < 0.01 || filterGain > 16000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetEncoderHallCcwOffsetInputValidation(float encoderHallOffset, int &error)
{
    if (encoderHallOffset <= 0 || encoderHallOffset >= 1)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetEncoderHallCwOffsetInputValidation(float encoderHallOffset, int &error)
{
    if (encoderHallOffset <= 0 || encoderHallOffset >= 1)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetSpeedAccelerationValueInputValidation(float speedAccelerationValue, int &error)
{
    if (speedAccelerationValue < 0 || speedAccelerationValue > 1600)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetSpeedDecelerationValueInputValidation(float speedDecelerationValue, int &error)
{
    if (speedDecelerationValue < 0 || speedDecelerationValue > 1600)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}

bool SOLOMotorControllersUtils::SetAnalogueSpeedResolutionDivisionCoefficientInputValidation(float divisionCoefficient, int &error)
{
    if (divisionCoefficient < 0.0001 || divisionCoefficient > 10000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}

bool SOLOMotorControllersUtils::SetMotionProfileVariable1InputValidation(float MotionProfileVariable1, int &error)
{
    if (MotionProfileVariable1 < 0 || MotionProfileVariable1 > 16000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetMotionProfileVariable2InputValidation(float MotionProfileVariable2, int &error)
{
    if (MotionProfileVariable2 < 0 || MotionProfileVariable2 > 16000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetMotionProfileVariable3InputValidation(float MotionProfileVariable3, int &error)
{
    if (MotionProfileVariable3 < 0 || MotionProfileVariable3 > 16000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetMotionProfileVariable4InputValidation(float MotionProfileVariable4, int &error)
{
    if (MotionProfileVariable4 < 0 || MotionProfileVariable4 > 16000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}
bool SOLOMotorControllersUtils::SetMotionProfileVariable5InputValidation(float MotionProfileVariable5, int &error)
{
    if (MotionProfileVariable5 < 0 || MotionProfileVariable5 > 16000)
    {
        error = SOLOMotorControllers::Error::OUT_OF_RANGE_SETTING;
        return false;
    }
    return true;
}