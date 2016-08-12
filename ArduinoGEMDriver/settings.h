// A lot of programs that connect to the serial port to talk LX200 send an ACK immediatley while the bootloader is 
// still running - the define ensures the G# reply to the ACK command is sent at startup to fix the issue
#define _LX200_TIMEOUT_FIX
// -------------------------------------------------------------
// Compile-Time mount setup
// -------------------------------------------------------------
#define _RA_ENCODER_PPR 32320   //32320
#define _DEC_ENCODER_PPR 32320    //32320
#define _RA_ENCODER_GEAR_TEETH 70 //rbw 64 orig
#define _DEC_ENCODER_GEAR_TEETH 65 //rbw 64 orig
#define _RA_MOTOR_PORT 1
#define _DEC_MOTOR_PORT 2
#define _FOCUS_MOTOR_PORT 3

// -------------------------------------------------------------
// Defaults for user adjustable constants
// -------------------------------------------------------------
// eeprom variables repurposed from old version for PId loop in tracking speed.
#define _default_RASlowSpeed 5 //RA Derivitive 
#define _default_DECSlowSpeed 65 //RA endcoder geartooth adjustment
#define _default_RAErrorTolerance 52 //RA P gain
#define _default_DECErrorTolerance 6 //RA integral
#define _default_Brightnessvalue 128///LCD backlight value -not implimented yet
// --------------------------------------------------------------------------------------------
// USE The DS1307 for Settings storage, more flexible then EEPROM as can be written frequently
// This will enable dynamic storage of the motor calibration and axis positions in a future release
// --------------------------------------------------------------------------------------------
#define _USE_DS1307_NVRAM

// -------------------------------------------------------------
// Debug Flags - turn these off if you want to use SPI0 as an LX200 interface
// -------------------------------------------------------------
//#define debugOut_I2CTimeouts
//#define debugOut_shaftPositions
//#define debugOut_motorResponseTimeOuts
//#define debugOut_showSPI1commandsOnSPI0
//#define debugOut_startuplog
//#define debugOut_joystick
// -------------------------------------------------------------
// Motor
// -------------------------------------------------------------
#define _DEC_MOTOR_MAX_SPEED 250
#define _RA_MOTOR_MAX_SPEED 250
//Be sure to also reverse feedback inputs -rbw
#define RA_ENC_REVERSE // use alternate encoder state table to reverse encoder input for RA - rbw 3-16-16
#define _RA_DIRECTION_REVERSE //define "east/west" motor direction
//#define DEC_ENC_REVERSE // use alternate encoder state table to reverse encoder input for DEC - rbw 3-16-16
//#define _DEC_DIRECTION_REVERSE //define "north/south" motor direction

// RBW New motor section 
// Defines motor I/O ports for use with external L298 module. Vars reflect inputs to module and port # of arduino.
#define RAIN1 3 //motor out RA - A
#define RAIN2 2 //motor out RA - B
#define RAEN 8 // RA Analog out - Speed PWM
#define DECIN1 5 //motor out DEC - A
#define DECIN2 4 //motor out DEC - B
#define DECEN 10 // DEC analog out - Speed PWM
#define FMIN1 6 //focus feedback - not used
#define FMIN2 7 //focus feedback - not used
#define FMEN 12 //focus speed - not used
//define mega2560 to use serial1 for bluetooth??
#define __AVR_ATmega2560__

