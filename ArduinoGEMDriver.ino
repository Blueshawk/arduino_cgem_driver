// -------------------------------------------------------------
// Telescope Driver
// -------------------------------------------------------------
#include <ds3231.h>
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "settings.h"
#include "PID_v1.h"


// LCD & LCD I2C Libraries
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
// This replaces original Arduino LCD Library, remove the original
// library to avoid confilicts
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include "DateTime.h"
#include "types.h"
#include "DS1307_RTC.h"
#include "Camera.h"
#include "EEPROMHandler.h"
#include "axis.h"
#include "mount.h"
#include "encoders.h"
#include "joystickcontrol.h"
#include "LX200Serial.h"
#include "UI.h"

DS1307_RTC_c* DS1307_RTC;
EEPROMHandler_c* EEPROMHandler;
mount_c* mount;
LX200SerialHandler_c* LX200SerialHandler;
UI_c* UI;


PID myPID(&pidInput, &pidOutput, &pidSetpoint,kp,ki,kd, DIRECT); //set up pid library

void setup() {
  delay(2000);//in case something goes horribly wrong, this will let you download a new program.

  myPID.SetSampleTime(50); 
  myPID.SetOutputLimits (-255,255);
  myPID.SetMode(AUTOMATIC);
 
//set pwm frequency for the outputs
//For Arduino Mega1280, Mega2560, MegaADK, Spider or any other board using ATmega1280 or ATmega2560**
//---------------------------------------------- Set PWM frequency for D11 & D12 -----------------------------
 
//TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to 64 for PWM frequency of   490.20 Hz //default rbw -is this right?
//TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
 
//-------------------------------------(DEC=10)---- Set PWM frequency for D9 & D10 ------------------------------
 
//TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
//TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz //default rbw 
//TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
 
 
//--------------------------------------(RA=8)---- Set PWM frequency for D6, D7 & D8 ---------------------------
 
//TCCR4B = TCCR4B & B11111000 | B00000001;    // set timer 4 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR4B = TCCR4B & B11111000 | B00000010;    // set timer 4 divisor to     8 for PWM frequency of  3921.16 Hz
TCCR4B = TCCR4B & B11111000 | B00000011;    // set timer 4 divisor to    64 for PWM frequency of   490.20 Hz //defaul rbw
//TCCR4B = TCCR4B & B11111000 | B00000100;    // set timer 4 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR4B = TCCR4B & B11111000 | B00000101;    // set timer 4 divisor to  1024 for PWM frequency of    30.64 Hz
 
//rbw set the new motor ports as outputs
pinMode ((RAIN1), OUTPUT); 
pinMode ((RAIN2), OUTPUT) ;
pinMode ((RAEN), OUTPUT) ;
pinMode ((DECIN1), OUTPUT); 
pinMode ((DECIN2), OUTPUT) ;
pinMode ((DECEN), OUTPUT) ;
pinMode ((FMIN1), OUTPUT) ;
pinMode ((FMIN2), OUTPUT) ;
pinMode ((FMEN), OUTPUT) ;
pinMode (19,INPUT_PULLUP); // for bluetooth tx output hc-06 

	Wire.begin(); // called in classes that use it but seems to hang if its not here
  Serial.begin(9600); //I think 9600 is needed for lx200 emulation?
	#ifdef _LX200_TIMEOUT_FIX
		Serial.print("G#");
	#endif
	#ifdef debugOut_startuplog
		delay(500);
	#endif
        #ifdef debugOut_startuplog
		Serial.println("Starting up");
	#endif
	#ifdef debugOut_startuplog
		delay(500);
	#endif
	#ifdef debugOut_startuplog
		Serial.println("Starting wire");
	#endif
	#ifdef debugOut_startuplog
		Serial.println("Starting DS1307 RTC");
        #endif
	           DS1307_RTC = new DS1307_RTC_c();
	#ifdef debugOut_startuplog
		Serial.println("Starting EEPROM handler");
                delay(500);
	#endif
	#ifndef _USE_DS1307_NVRAM
		EEPROMHandler = new EEPROMHandler_c();
	#else
		EEPROMHandler = new EEPROMHandler_c(DS1307_RTC);
	#endif
            
	#ifdef debugOut_startuplog
		Serial.println(); 
                Serial.println("Starting Mount handler");
	        delay(400);
               
        #endif
	mount = new mount_c((uint8_t)_RA_MOTOR_PORT, (uint8_t)_DEC_MOTOR_PORT,
						_RA_ENCODER_GEAR_TEETH, _DEC_ENCODER_GEAR_TEETH,
						_RA_ENCODER_PPR, _DEC_ENCODER_PPR,
						DS1307_RTC, EEPROMHandler);
	#ifdef debugOut_startuplog
		Serial.println("Starting serial command parser");
                delay(500);
	#endif
	LX200SerialHandler = new LX200SerialHandler_c(mount, DS1307_RTC, EEPROMHandler);
	#ifdef debugOut_startuplog
		Serial.println("Starting UI");
                delay(400);
	#endif
	UI = new UI_c(mount, DS1307_RTC, EEPROMHandler);
	#ifdef debugOut_startuplog
		Serial.println("Startup complete");
	#endif

 

}

void loop() {
          
	LX200SerialHandler->update();
	mount->update();
        UI->update();
	mount->update();
        EEPROMHandler->update();
  mount->update();       
}
