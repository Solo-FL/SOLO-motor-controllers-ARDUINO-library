# Changelog

All notable changes to this project will be documented in this file.

## [5.3.1] - 2024-05-30
### Added
- for CANopen native code usage added ARDUINO_CAN_NATIVE_SUPPORTED 

### Changed
- example name for CANopen native general read and write
- example code for CANopen native library import

### Deprecated

### Removed

### Fixed
- CAN back-compability over older arduino version

## [5.3.0] - 2024-05-18
### Added
- Add Native support over Can Native (TX RX) of Arduino (SOLOMotorControllersCanopenNative class)
- Add examples

### Changed
- Renamed SOLOMotorControllersCanopen files to SOLOMotorControllersCanopenMcp2515
- Improved CANopen resource management and performance

### Deprecated

### Removed

### Fixed

## [5.2.0] - 2024-03-16
### Added
- New function for PT100 and Digital-Pin Set/Get
- Enhanced CANopen: Increased flexibility for using various MCP2515 boards by customization of interrupt pins and frequencies

### Changed
- Replaced CANopen Timeout based on count to milliseconds for better accuracy
- Improved CANopen resource management and performance

### Deprecated

### Removed
- Reduced public method by over 40% while maintaining all existing functionalities
- Reduced library src size by 25% 

### Fixed

## [5.1.0] - 2024-02-12
### Added
- Support over ARDUINO UNO R4 WIFI
- Support over ARDUINO UNO MINIMA
- Changelog file

### Changed
- Internal UART SPI layer
- Internal CANopen layer

### Deprecated

### Removed

### Fixed
- Fixed a Minor CANopen PRO Bug  

## [5.0.0] - 2024-01-15
### Added
- CANopen PDO Support
- CANopen PDO Examples

### Changed
- Internal UART layer
- Style Refinements

### Deprecated

### Removed

### Fixed
