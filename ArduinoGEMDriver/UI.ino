 // ------------------------------------------------------------------------
// User Interface
// ------------------------------------------------------------------------
#include "UI.h"
#include "joystickcontrol.h"
UI_c::UI_c(mount_c* mount_obj, DS1307_RTC_c* DS1307_RTC_Obj, EEPROMHandler_c* EEPROM_obj) {
	Wire.begin();
	mount = mount_obj;
	EEPROMHandler = EEPROM_obj;
	camera = new camera_c();
	DS1307_RTC = DS1307_RTC_Obj;
	lcd = new LiquidCrystal_I2C(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
	joystick = new joystick_c();
	lcd->begin (24, 4);
	lcd->home();
	lcd->setBacklightPin(BACKLIGHT_PIN,POSITIVE); //to do make a backlight adjustment in setup
	lcd->setBacklight(HIGH); //LCD backlight value - orig HIGH
  // dimming from another program -- doesn't work with this library. ---more later RBW  -- lcd.setBacklight(brightnessvalue);
	//rbw focuser_motor = new AF_DCMotor(3, MOTOR12_1KHZ);
}

// -------------------------------------------------------------
// LCD/Helper Functions
// -------------------------------------------------------------
void UI_c::progmemLCDPrint (PGM_P s) {
	char c;
	while ((c = pgm_read_byte(s++)) != 0) {
		lcd->print(c);
	}
}

void UI_c::LCDPrintbyte (byte val) {
	lcd->print(val, DEC);
	progmemLCDPrint(PSTR("  "));
}

void UI_c::LCDPrintbool (bool val) {
	if(val) {
		progmemLCDPrint(PSTR("Yes"));
	} else {
		progmemLCDPrint(PSTR("No "));
	}
}

int UI_c::freeRAM() { // copied this function from somewhere online, way over my head!
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

// -------------------------------------------------------------
// Main loop
// -------------------------------------------------------------

void UI_c::update() {
	static byte state = 0;
	
	// update joystick class
	joystick->update();

	// joystick data refreshed triggers UI update
	bool UIUpdateFlag = joystick->newDataAvailible_event();
	if(joystick->CommunicationTimeout()) {
		lcd->clear();
		lcd->print("Lost Nunchuck");

	}

	// Jump to handler for current menu item
	switch(state) {
		case 0: // Main Menu
			state = mainMenu(UIUpdateFlag);
			break;
		case 1: // Track And Pan
			if(!menuItem_TrackPan(UIUpdateFlag)) {
				state = 0;
			}
			break;
		case 2: // Fine Track & Pan
			if(!menuItem_FineTrackPan(UIUpdateFlag)) {
				state = 0;
			}
			break;
		case 3: // Settings
			if(!menuItem_Settings(UIUpdateFlag)) {
				state = 0;
			}
			break;
		case 4: // focuser control
			if(!menuItem_Focuser(UIUpdateFlag)) {
				state = 0;
			}
			break;
		default:
			state = 0;
			break;

	}
}

// -------------------------------------------------------------
// Main Menu
// -------------------------------------------------------------
byte UI_c::mainMenu(bool UIUpdateFlag) {
	static byte menuItem = 1;
	const byte menuLength = 4;
	// refresh LCD
	if(UIUpdateFlag) {
		lcd->home();
		
		lcd->setCursor(0, 0); // row 1,left
    lcd->print(DS1307_RTC->TimeString());
		progmemLCDPrint(PSTR("             "));
    lcd->setCursor(3, 1); // row 2,left
		//lcd->print(menuItem);
    //lcd->print("> ");
    	
    		switch(menuItem) {
			
			case 1 :
				progmemLCDPrint(PSTR("Track & Pan   "));
              break;

			case 2 :
				progmemLCDPrint(PSTR("Fine Track&Pan"));
              break;
			case 3 :
				progmemLCDPrint(PSTR("Settings      "));
              break;
			case 4 :
				progmemLCDPrint(PSTR("Focuser       "));
              break;
		    }
    lcd->setCursor(0, 2); //row 3,left
    lcd->print("Spt "); 
    lcd->print((byte)speedSetpoint);
    lcd->print(" ");
    lcd->print(" Err ");
    lcd->print(mount -> ra_axis -> gearPositionError());
    lcd->print("        ");
    lcd->setCursor(0,3); //row 4 left
    lcd->print("PID On:");
    lcd->print(myPID.GetMode());
  	
	}

	// handle joystick up/down
	if(joystick->up_event()) {
		if(menuItem>1) {
			menuItem--;
		} else {
			menuItem=menuLength;
		}
	}
	if(joystick->down_event()) {
		if(menuItem<menuLength) {
			 menuItem++;
		} else {
			 menuItem=1;
		}
	}

	// Z+C selects a menu item
	if(joystick->zc_longpress_event()) {
		lcd->clear();
		return menuItem;
	}

	return 0;
}

// -------------------------------------------------------------
// Functions to handle each menu item
// -------------------------------------------------------------

bool UI_c::menuItem_TrackPan(bool UIUpdateFlag) {
	static bool initialised = false;
	// startup
	if(!initialised) {
		initialised=true;
	}
	// send joystick info to mount class
	mount->joystickControl(joystick->filteredX,joystick->filteredY,joystick->nunchuck_zbutton);
	// cbutton down press event
	//camera->setPin(joystick->nunchuck_cbutton && !joystick->nunchuck_zbutton);
	// write RA/DEC position + error
	if(UIUpdateFlag) {
		lcd->setCursor(0, 0); // top left
		lcd->print(mount -> ra_axis -> currentAngle() -> RA_Angle_text());
            //.println(mount -> ra_axis -> currentAngle() -> RA_Angle_text());//rbw
		progmemLCDPrint(PSTR(" "));
		lcd->print(mount -> ra_axis -> gearPositionError());
		progmemLCDPrint(PSTR("e   "));
		lcd->setCursor(0, 1); // bottom left
		lcd->print(mount -> dec_axis -> currentAngle() -> DEC_Angle_text());
   	progmemLCDPrint(PSTR(" "));
		lcd->print(mount -> dec_axis -> gearPositionError());
   	progmemLCDPrint(PSTR("e   "));
      lcd->setCursor(0,3); //row 4 left
      lcd->print("PID On:");
      lcd->print(myPID.GetMode());
    
	}
	// if long press on z+c for 3 seconds enter menu
	if(joystick->zc_longpress_event()) {
		initialised = false;
		mount->joystickControl(0,0,false);
		lcd->clear();
		return false;
	}
	return true;
}

bool UI_c::menuItem_FineTrackPan(bool UIUpdateFlag) {
	static bool initialised = false;
	// startup
	if(!initialised) {
		initialised = true;
	}

	// at intervals read joystick position and pass to axis target positions
	if(UIUpdateFlag) {
		byte speedMultiplier = 10;
		if(joystick->nunchuck_zbutton) {
			speedMultiplier = 15; //orig 10 rbw
		}
            //rbw all were 50 or -50
		if(joystick->filteredX>20) {
			mount->moveTarget((int)speedMultiplier,0); 
		}
		if(joystick->filteredX<-20) {
			mount->moveTarget(0 - (int)speedMultiplier,0); 
		}
		if(joystick->filteredY>20) {
			mount->moveTarget(0, (int)speedMultiplier); 
		}
		if(joystick->filteredY<-20) {
			mount->moveTarget(0, 0 - (int)speedMultiplier);
		}
	}

	// write RA/DEC position + error
	if(UIUpdateFlag) {
		lcd->setCursor(0, 0); // top left
		lcd->print(mount -> ra_axis -> currentAngle() -> RA_Angle_text());
		progmemLCDPrint(PSTR(" "));
		lcd->print(mount -> ra_axis -> gearPositionError());
		progmemLCDPrint(PSTR("e   "));

		lcd->setCursor(0, 1); // bottom left
		lcd->print(mount -> dec_axis -> currentAngle() -> DEC_Angle_text());
		progmemLCDPrint(PSTR(" "));
		lcd->print(mount -> dec_axis -> gearPositionError());
		progmemLCDPrint(PSTR("e   "));
	}

	// cbutton down press event
	camera->setPin(joystick->nunchuck_cbutton && !joystick->nunchuck_zbutton);

	// if long press on z+c for 3 seconds enter menu
	if(joystick->zc_longpress_event()) {
		initialised = false;
    return false;
	}
	return true;
}

bool UI_c::menuItem_Settings(bool UIUpdateFlag) {
	static bool initialised = false;
	static byte currentSettingItem;
	const byte settingsMenuLength = 3;
	// startup
	if(!initialised) {
		initialised = true;
		currentSettingItem = 0;
	}
	// update LCD with current item and value
	if(UIUpdateFlag) {
		lcd->home();
		lcd->setCursor(0, 0); // top left
		progmemLCDPrint(PSTR("Set:"));
		switch(currentSettingItem) {
			case 0 :
				progmemLCDPrint(PSTR("Proportional"));
				lcd->setCursor(0, 1); // bottom left
				LCDPrintbyte(EEPROMHandler->readRAErrorTolerance());
				  lcd->setCursor(0, 3); //row 4,left
          lcd->print("P=");
          lcd->print(((int)kp));
          lcd->print(" I=");
          lcd->print(int(ki));
          lcd->print(" D=");
          lcd->print(int(kd));				
				break;
			case 1 :
				progmemLCDPrint(PSTR("Integral    "));
				lcd->setCursor(0, 1); // bottom left
				LCDPrintbyte(EEPROMHandler->readDECErrorTolerance());
          lcd->setCursor(0, 3); //row 4,left
          lcd->print("P=");
          lcd->print(((int)kp));
          lcd->print(" I=");
          lcd->print(int(ki));
          lcd->print(" D=");
          lcd->print(int(kd));
          lcd->print("     ");  			
				break;
			case 2 :
				progmemLCDPrint(PSTR("Derivative  "));
				lcd->setCursor(0, 1); // bottom left
				LCDPrintbyte(EEPROMHandler->readRASlowSpeed());
          lcd->setCursor(0, 3); //row 4,left
          lcd->print("P=");
          lcd->print(((int)kp));
          lcd->print(" I=");
          lcd->print(int(ki));
          lcd->print(" D=");
          lcd->print(int(kd));  				
		      lcd->print("     ");          
        		
				break;
			case 3 :
				progmemLCDPrint(PSTR("RA fine tune   "));
				lcd->setCursor(0, 1); // bottom left
				LCDPrintbyte(EEPROMHandler->readDECSlowSpeed());
				break;
			
		}
	}

	// handle up/down events to change item
	if(joystick->up_event()) {
		if(currentSettingItem > 0) {
			currentSettingItem--;
		}  else {
			currentSettingItem = settingsMenuLength;
		}
	}
	if(joystick->down_event()) {
		if(currentSettingItem < settingsMenuLength) {
			currentSettingItem++;
		} else {
			currentSettingItem = 0;
		}
	}
	// handle left right events to change value
	bool LREvent=false;
	bool right=false;
	if(joystick->right_event()) {
		LREvent = true;
		right=true;
	}
	if(joystick->left_event()) {
		LREvent = true;
		right=false;
	}
	if(LREvent) {
		switch(currentSettingItem) {
			case 0 : {
				byte curVal = EEPROMHandler->readRAErrorTolerance();
				if(curVal<200 && right) {
					 byte newVal = curVal + 1;
					 EEPROMHandler->setRAErrorTolerance(curVal + 1);
				}
				if(curVal>0 && !right) {
					 byte newVal = curVal - 1;
					 EEPROMHandler->setRAErrorTolerance(curVal - 1);
				}
				break;
			}
			case 1 : {
				byte curVal = EEPROMHandler->readDECErrorTolerance();
				if(curVal<200 && right) {
					 EEPROMHandler->setDECErrorTolerance(curVal + 1);
				}
				if(curVal>0 && !right) {
					 EEPROMHandler->setDECErrorTolerance(curVal - 1);
				}
				break;
			}
			case 2 : {
				byte curVal = EEPROMHandler->readRASlowSpeed();
				if(curVal<200 && right) {
					 EEPROMHandler->setRASlowSpeed(curVal + 1);
				}
				if(curVal>0 && !right) {
					 EEPROMHandler->setRASlowSpeed(curVal - 1);
				}
				break;
			}
			case 3 : {
				byte curVal = EEPROMHandler->readDECSlowSpeed();
				if(curVal<200 && right) {
					 EEPROMHandler->setDECSlowSpeed(curVal + 1);
				}
				if(curVal>0 && !right) {
					 EEPROMHandler->setDECSlowSpeed(curVal - 1);
				}
				break;
			}
		}
	}

	// if long press on z+c for 3 seconds enter menu
	if(joystick->zc_longpress_event()) {
		lcd->clear();
   	initialised = false;
		return false;
	}
	return true;
}

bool UI_c::menuItem_Focuser(bool UIUpdateFlag) {
	static bool initialised = false;
	// startup
	if(!initialised) {
		initialised=true;
	}
	byte setSpd;
	//rbw focuser_motor->run(RELEASE);
	if(joystick->filteredX>20) { //rbw orig 10
		setSpd = (byte)joystick->filteredX;
		//rbw focuser_motor->setSpeed(setSpd);
		//rbw focuser_motor->run(FORWARD);
	        digitalWrite (FMIN1, LOW);
                digitalWrite (FMIN2, HIGH);
                analogWrite (FMEN, (setSpd));
        }
	if(joystick->filteredX<30) {
		setSpd = (byte)(0- joystick->filteredX);
		//rbw focuser_motor->setSpeed(setSpd);
		//rbw focuser_motor->run(BACKWARD);
	        digitalWrite (FMIN1, HIGH);
                digitalWrite (FMIN2, LOW);
                analogWrite (FMEN, (setSpd));

	}
	// if long press on z+c for 3 seconds enter menu
	if(joystick->zc_longpress_event()) {
		initialised = false;
		//rbw focuser_motor->run(RELEASE);
	        digitalWrite (FMIN1, LOW);
                digitalWrite (FMIN2, LOW);
                analogWrite (FMEN, 0);
		return false;
	}
	return true;
}



