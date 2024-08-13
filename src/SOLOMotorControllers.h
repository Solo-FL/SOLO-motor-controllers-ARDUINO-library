/**
 *******************************************************************************
 * @file    SOLOMotorControllers.h
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the base functions prototypes for the Solo Drivers
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

#ifndef SOLO_MOTOR_CONTROLLERS_H
#define SOLO_MOTOR_CONTROLLERS_H

class SOLOMotorControllers
{

public:
	/**
	 * @brief  Error enumeration definition
	 */
	enum Error
	{
		NO_ERROR_DETECTED = 0,						/*!< no error detected */
		GENERAL_ERROR = 1,							/*!< general error */
		NO_PROCESSED_COMMAND = 2,					/*!< command is not valid */
		OUT_OF_RANGE_SETTING = 3,					/*!< setting is out of range */
		PACKET_FAILURE_TRIAL_ATTEMPTS_OVERFLOW = 4, /*!< trial attempt overflow */
		RECEIVE_TIMEOUT_ERROR = 5,					/*!< receive time error */
		ABORT_OBJECT = 6,							/*!< abort object */
		ABORT_VALUE = 7,							/*!< abort value */
		MCP2515_TRANSMIT_ARBITRATION_LOST = 8,		/*!< MCP2515 transmit arbitration lost */
		MCP2515_TRANSMIT_ERROR = 9,					/*!< MCP2515 transmit error */
		OBJECT_NOT_INITIALIZE = 10,					/*!< Kvaser object not initialize */
		CAN_EMPTY_BUFFER = 11,						/*!< Kvaser buffer have no data for the defined COBID */
		PDO_PARAMETER_ID_OUT_OF_RANGE = 12,			/*!< PDO configuration id out of range */
		PDO_SYNC_OUT_OF_RANGE = 13,					/*!< PDO configuration sync out of range */
		PDO_MISSING_COB_ID = 14,					/*!< PDO no specific CobId for the specified pdo*/
		PDO_RTR_COMMAND_NOT_ALLOWED = 15,			/*!< PDO RTR specific command not allowed*/
	};

	/**
	 * @brief  Command Mode enumeration definition
	 */
	enum CommandMode
	{
		ANALOGUE = 0,						  /*!< Analogue Mode */
		DIGITAL = 1,						  /*!< Digital Mode */
		ANALOGUE_WITH_DIGITAL_SPEED_GAIN = 2, /*!<Analogue Mode with Speed controller gains taken from the Digital setup*/
		COMMAND_MODE_ERROR = -1				  /*!< error */
	};

	/**
	 * @brief  Direction enumeration definition
	 */
	enum Direction
	{
		COUNTERCLOCKWISE = 0, /*!< counter-clockwise direction */
		CLOCKWISE = 1,		  /*!< clockwise direction */
		DIRECTION_ERROR = -1  /*!< error */
	};

	/**
	 * @brief  Feedback Control Mode enumeration definition
	 */
	enum FeedbackControlMode
	{
		SENSORLESS_HSO = 0,				 /*!< sensorless mode */
		ENCODERS = 1,					 /*!< encoders mode */
		HALL_SENSORS = 2,				 /*!< hall sensors mode */
		SENSORLESS_ZSFT = 3,			 /*!< Sensor-less Zero Speed Full Torque feedback Mode (ZSFT) */
		FEEDBACK_CONTROL_MODE_ERROR = -1 /*!< error */

	};

	/**
	 * @brief  Control Mode enumeration definition
	 */
	enum ControlMode
	{
		SPEED_MODE = 0,			/*!< speed mode */
		TORQUE_MODE = 1,		/*!< torque mode */
		POSITION_MODE = 2,		/*!< position mode */
		CONTROL_MODE_ERROR = -1 /*!< error */
	};

	/**
	 * @brief  Motor Type enumeration definition
	 */
	enum MotorType
	{
		DC = 0,					 /*!< dc motor */
		BLDC_PMSM = 1,			 /*!< brushless dc motor  */
		ACIM = 2,				 /*!< acim motor */
		BLDC_PMSM_ULTRAFAST = 3, /*!< brushless dc motor fast */
		MOTOR_TYPE_ERROR = -1	 /*!< error */
	};

	/**
	 * @brief  Uart Baudrate enumeration definition
	 */
	enum UartBaudrate //[bits/s]
	{
		RATE_937500 = 0,		/*!< baud rate 937500 */
		RATE_115200 = 1,		/*!< baud rate 115200 */
		UART_BAUDRATE_ERROR = 2 /*!< error */
	};

	/**
	 * @brief  Canbus Baudrate enumeration definition
	 */
	enum CanbusBaudrate //[kbits/s]
	{
		RATE_1000 = 0, /*!< Baudrate 1000 kbits/s */
		RATE_500 = 1,  /*!< Baudrate 500 kbits/s */
		RATE_250 = 2,  /*!< Baudrate 250 kbits/s */
		RATE_125 = 3,  /*!< Baudrate 125 kbits/s */
		RATE_100 = 4   /*!< Baudrate 100 kbits/s */
	};

	/**
	 * @brief  Frequency enumeration definition
	 */
	enum Frequency //[Mbits/s] 16000
	{
		RATE_8 = 0,	 /*!< Baudrate 8 Mbits/s */
		RATE_16 = 1, /*!< Baudrate 16 Mbits/s */
		RATE_20 = 2, /*!< Baudrate 20 Mbits/s */
	};

	/**
	 * @brief  Action enumeration definition
	 */
	enum Action
	{
		STOP = 0,		  /*!< stop */
		START = 1,		  /*!< start */
		ACTION_ERROR = -1 /*!< error */
	};

	/**
	 * @brief  Disable/Enable enumeration definition
	 */
	enum DisableEnable
	{
		DISABLE = 0,			  /*!< Disable */
		ENABLE = 1,				  /*!< Enable */
		DISABLE_ENABLE_ERROR = -1 /*!< error */
	};

	/**
	 * @brief  Position Sensor Calibration Action enumeration definition
	 */
	enum PositionSensorCalibrationAction
	{
		STOP_CALIBRATION = 0,						  /*!< stop colibration */
		INCREMENTAL_ENCODER_START_CALIBRATION = 1,	  /*!< incremental encoder start calibration */
		HALL_SENSOR_START_CALIBRATION = 2,			  /*!< hall sensor start calibration */
		POSITION_SENSOR_CALIBRATION_ACTION_ERROR = -1 /*!< error */
	};

	/**
	 * @brief  Motion Profile Mode enumeration definition
	 */
	enum MotionProfileMode
	{
		STEP_RAMP_RESPONSE = 0,		   /*!< step ramp profile */
		TIME_BASED_ST_CURVE = 1,	   /*!< time based st curve */
		TIME_OPTIMAL_ST_CURVE = 2,	   /*!< time optimal st curve */
		MOTION_PROFILE_MODE_ERROR = -1 /*!< error */
	};

	/**
	 * @brief  Digital Input and Output State enumeration definition
	 */
	enum DigitalIoState
	{
		LOW_IO_STATE = 0,					/*!< GPIO Low State */
		HIGH_IO_STATE = 1,					/*!< GPIO High State */
		DIGITAL_IO_STATE_ERROR = -1 /*!< error */
	};

	/**
	 * @brief  Channel enumeration definition
	 */
	enum Channel
	{
		CHANNEL0 = 0,	   /*!< Channel 0 */
		CHANNEL1 = 1,	   /*!< Channel 1 */
		CHANNEL2 = 2,	   /*!< Channel 2 */
		CHANNEL3 = 3,	   /*!< Channel 3 */
		CHANNEL_ERROR = -1 /*!< error */
	};

	
	//----------Write----------

	virtual bool SetDeviceAddress(unsigned char deviceAddress, int &error );

	virtual bool SetCommandMode(SOLOMotorControllers::CommandMode mode, int &error );

	virtual bool SetCurrentLimit(float currentLimit, int &error );

	virtual bool SetTorqueReferenceIq(float torqueReferenceIq, int &error );

	virtual bool SetSpeedReference(long speedReference, int &error );

	virtual bool SetPowerReference(float powerReference, int &error );

	virtual bool MotorParametersIdentification(SOLOMotorControllers::Action identification, int &error );

	virtual bool SetDriveDisableEnable(SOLOMotorControllers::DisableEnable action, int &error );

	virtual bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int &error );

	virtual bool SetSpeedControllerKp(float speedControllerKp, int &error );

	virtual bool SetSpeedControllerKi(float speedControllerKi, int &error );

	virtual bool SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error );

	virtual bool SetMotorResistance(float motorResistance, int &error );

	virtual bool SetMotorInductance(float motorInductance, int &error );

	virtual bool SetMotorPolesCounts(long motorPolesCounts, int &error );

	virtual bool SetIncrementalEncoderLines(long incrementalEncoderLines, int &error );

	virtual bool SetSpeedLimit(long speedLimit, int &error );

	virtual bool SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode feedbackControlMode, int &error );

	virtual bool ResetFactory(int &error );

	virtual bool SetMotorType(SOLOMotorControllers::MotorType motorType, int &error );

	virtual bool SetControlMode(SOLOMotorControllers::ControlMode controlMode, int &error );

	virtual bool SetCurrentControllerKp(float currentControllerKp, int &error );

	virtual bool SetCurrentControllerKi(float currentControllerKi, int &error );

	virtual bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error );

	virtual bool SetPositionReference(long positionReference, int &error );

	virtual bool SetPositionControllerKp(float positionControllerKp, int &error );

	virtual bool SetPositionControllerKi(float positionControllerKi, int &error );

	virtual bool OverwriteErrorRegister(int &error );

	virtual bool SetZsftInjectionAmplitude(float zsftInjectionAmplitude, int &error );

	virtual bool SetZsftPolarityAmplitude(float zsftPolarityAmplitude, int &error );

	virtual bool SetObserverGainDc(float observerGain, int &error );

	virtual bool SetZsftInjectionFrequency(long zsftInjectionFrequency, int &error );

	virtual bool SetSensorlessTransitionSpeed(long sensorlessTransitionSpeed, int &error );

	virtual bool SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate, int &error );

	virtual bool SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction, int &error );

	virtual bool SetEncoderHallCcwOffset(float encoderHallOffset, int &error );

	virtual bool SetEncoderHallCwOffset(float encoderHallOffset, int &error );

	virtual bool SetSpeedAccelerationValue(float speedAccelerationValue, int &error );

	virtual bool SetSpeedDecelerationValue(float speedDecelerationValue, int &error );

	virtual bool SetCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate canbusBoudrate, int &error );

	virtual bool SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int &error );

	virtual bool SetMotionProfileMode(SOLOMotorControllers::MotionProfileMode motionProfileMode, int &error );

	virtual bool SetMotionProfileVariable1(float MotionProfileVariable1, int &error );

	virtual bool SetMotionProfileVariable2(float MotionProfileVariable2, int &error );

	virtual bool SetMotionProfileVariable3(float MotionProfileVariable3, int &error );

	virtual bool SetMotionProfileVariable4(float MotionProfileVariable4, int &error );

	virtual bool SetMotionProfileVariable5(float MotionProfileVariable5, int &error );

	virtual bool SetDigitalOutputState(SOLOMotorControllers::Channel channel, SOLOMotorControllers::DigitalIoState state, int &error );

	virtual bool SetRegenerationCurrentLimit(float current, int &error );

	virtual bool SetPositionSensorDigitalFilterLevel(long level, int &error );

	////----------Read----------
	virtual long GetDeviceAddress(int &error );

	virtual float GetPhaseAVoltage(int &error );

	virtual float GetPhaseBVoltage(int &error );

	virtual float GetPhaseACurrent(int &error );

	virtual float GetPhaseBCurrent(int &error );

	virtual float GetBusVoltage(int &error ); // Battery Voltage

	virtual float GetDcMotorCurrentIm(int &error );

	virtual float GetDcMotorVoltageVm(int &error );

	virtual float GetSpeedControllerKp(int &error );

	virtual float GetSpeedControllerKi(int &error );

	virtual long GetOutputPwmFrequencyKhz(int &error );

	virtual float GetCurrentLimit(int &error );

	virtual float GetQuadratureCurrentIqFeedback(int &error );

	virtual float GetMagnetizingCurrentIdFeedback(int &error ); // Magnetizing

	virtual long GetMotorPolesCounts(int &error );

	virtual long GetIncrementalEncoderLines(int &error );

	virtual float GetCurrentControllerKp(int &error );

	virtual float GetCurrentControllerKi(int &error );

	virtual float GetBoardTemperature(int &error );

	virtual float GetMotorResistance(int &error );

	virtual float GetMotorInductance(int &error );

	virtual long GetSpeedFeedback(int &error );

	virtual SOLOMotorControllers::MotorType GetMotorType(int &error );

	virtual SOLOMotorControllers::FeedbackControlMode GetFeedbackControlMode(int &error );

	virtual SOLOMotorControllers::CommandMode GetCommandMode(int &error );

	virtual SOLOMotorControllers::ControlMode GetControlMode(int &error );

	virtual long GetSpeedLimit(int &error );

	virtual float GetPositionControllerKp(int &error );

	virtual float GetPositionControllerKi(int &error );

	virtual long GetPositionCountsFeedback(int &error );

	virtual long GetErrorRegister(int &error );

	virtual long GetDeviceFirmwareVersion(int &error );

	virtual long GetDeviceHardwareVersion(int &error );

	virtual float GetTorqueReferenceIq(int &error );

	virtual long GetSpeedReference(int &error );

	virtual float GetMagnetizingCurrentIdReference(int &error );

	virtual long GetPositionReference(int &error );

	virtual float GetPowerReference(int &error );

	virtual SOLOMotorControllers::Direction GetMotorDirection(int &error );

	virtual float GetZsftInjectionAmplitude(int &error );

	virtual float GetZsftPolarityAmplitude(int &error );

	virtual float GetObserverGainDc(int &error );

	virtual long GetZsftInjectionFrequency(int &error );

	virtual long GetSensorlessTransitionSpeed(int &error );

	virtual float Get3PhaseMotorAngle(int &error ); // Read Estimated or Measured Rotor Angle

	virtual float GetEncoderHallCcwOffset(int &error );

	virtual float GetEncoderHallCwOffset(int &error );

	virtual SOLOMotorControllers::UartBaudrate GetUartBaudrate(int &error );

	virtual float GetSpeedAccelerationValue(int &error );

	virtual float GetSpeedDecelerationValue(int &error );

	virtual long GetEncoderIndexCounts(int &error );

	virtual bool CommunicationIsWorking(int &error );

	virtual long GetAnalogueSpeedResolutionDivisionCoefficient(int &error );

	virtual SOLOMotorControllers::MotionProfileMode GetMotionProfileMode(int &error );

	virtual float GetMotionProfileVariable1(int &error );

	virtual float GetMotionProfileVariable2(int &error );

	virtual float GetMotionProfileVariable3(int &error );

	virtual float GetMotionProfileVariable4(int &error );

	virtual float GetMotionProfileVariable5(int &error );

	virtual SOLOMotorControllers::DigitalIoState GetDigitalOutputsState(SOLOMotorControllers::Channel chaneel, int &error );

	virtual SOLOMotorControllers::DisableEnable GetDriveDisableEnable(int &error );

	virtual float GetRegenerationCurrentLimit(int &error );

	virtual long GetPositionSensorDigitalFilterLevel(int &error );

	virtual long GetDigitalInputRegister(int &error );

	virtual long GetPT1000SensorVoltage(int &error );

	virtual SOLOMotorControllers::DigitalIoState GetAnalogueInput(SOLOMotorControllers::Channel channel, int &error );
};

#endif // SOLO_MOTOR_CONTROLLERS_H