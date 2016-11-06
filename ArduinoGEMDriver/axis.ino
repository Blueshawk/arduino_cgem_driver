// ------------------------------------------------------------------------
// Telescope Axis Class
// Handle The control, positioning and upkeep of a telescope axis
// equiv methods for ra and dec variants are paired up here
// ------------------------------------------------------------------------
#include "axis.h"
#include "settings.h"
#include "PID_v1.h"
// -------------------------------------------------------------
// Init 
// -------------------------------------------------------------

byte speedSetpoint;
double pidSetpoint=0, pidInput=0, pidOutput=0;
double kp = 35, ki=20, kd=5;

ra_axis_c::ra_axis_c(byte imotorport, int encoder_PPR, int igearTeeth, EEPROMHandler_c* EEPROMObj) {
	
	EEPROMHandler = EEPROMObj;
	EEPROMUpdateIndex = EEPROMHandler->EEPROMUpdates;
	shaftPPR = (unsigned int)encoder_PPR;
	gearTeeth=(EEPROMHandler->readDECSlowSpeed());//igearTeeth;
	firstZrecorded=false;
	shaftPosition = -5000;
	gearPosition_max = (signed long)encoder_PPR * (signed long)gearTeeth;
	gearPosition_max_half = (signed long)((float)gearPosition_max / (float)2);
	gearPosition_target = gearPosition_max_half;
	gearPosition_raw = gearPosition_max_half;
	newGearPositon_raw_value_event = false;
	axisAngle = new Angle;
	setupTrackingRates();
	slewingEast = false;
	slewingWest = false;
	goingTo = false;
	resetgearPosErrHist();

}

dec_axis_c::dec_axis_c(byte imotorport, int encoder_PPR, int igearTeeth, EEPROMHandler_c* EEPROMObj) {
	EEPROMHandler = EEPROMObj;
	EEPROMUpdateIndex = EEPROMHandler->EEPROMUpdates;
	shaftPPR = (unsigned int)encoder_PPR;
	gearTeeth=igearTeeth;
	gearPosition_max = (signed long)encoder_PPR * (signed long)gearTeeth;
	gearPosition_max_half = gearPosition_max / 2;
	gearPosition_target = gearPosition_max_half;
	gearPosition_raw = gearPosition_max_half;
	newGearPositon_raw_value_event = false;
	setupTrackingRates();
	slewingNorth = false;
	slewingSouth = false;
	slewSpeed = 0;
	axisAngle = new Angle;
	resetgearPosErrHist();
}

void axis_c::resetgearPosErrHist() {
	for(byte loop = 0; loop<(byte)_HISTORY_BYTES; loop++) {
		gearPosErrHist[loop] = _ERROR_ZERO;
	}
	gearPosErrHist_index = 0;
}

// Tracking Rates
void ra_axis_c::setupTrackingRates() {
	// seconds in a sidereal day
	float sideRealDayInSeconds = 23.9344699 * 60.0 * 60.0;  //float sideRealDayInSeconds = 23.9344699 * 60.0 * 60.0;
	// seconds shaft should take to rotate at sidereal speed
	float sideRealshaftPeriod_float = (sideRealDayInSeconds / (float)gearTeeth);
	// save shaft period
	sideReelShaftPeriod = (unsigned int)sideRealshaftPeriod_float;
	// get pulses per second moving at sidereal speed
	float pulseLengthInSeconds = sideRealshaftPeriod_float / (float)shaftPPR;
	// seconds to microseconds
	float pulseLengthInMicros = pulseLengthInSeconds * 1000000.0;
	// save rounded pulse time
	sideRealPulseTime = (unsigned long)pulseLengthInMicros;
	sideRealPulseTime_lastPulse=micros();
	minimumMotorSpeed = EEPROMHandler->readRASlowSpeed();
	errorTolerance = EEPROMHandler->readRAErrorTolerance();
	
	// minimumMotorSpeed = 0;
	motorRange = (byte)_RA_MOTOR_MAX_SPEED - (byte)minimumMotorSpeed;
}

void dec_axis_c::setupTrackingRates() {
	errorTolerance = 10;//EEPROMHandler->readDECErrorTolerance();
	minimumMotorSpeed = EEPROMHandler->readDECSlowSpeed();
	motorRange = (byte)_RA_MOTOR_MAX_SPEED - (byte)minimumMotorSpeed;
}

// -------------------------------------------------------------
// Main loop - High Priority - called at very regular intervals
// -------------------------------------------------------------
void ra_axis_c::priorityUpdate() {
   
	//signed long errorTolerance_local = (signed long)(errorTolerance);
	//signed long errorTolerance_local_neg = (signed long)(errorTolerance);
	
	// update gear position
	boolean zWasAlreadyKnown = firstZrecorded;
	gearPosition_raw += (signed long)encoders.readRAencoderMovement();
	if(newGearPositon_raw_value_event) {
	  gearPosition_raw = newGearPositon_raw_value;
	  newGearPositon_raw_value_event = false;
	}
	
	// check for flags where control of the motors via this routine is not desireable
	if(slewingEast || slewingWest) {
		return;
	}
	// get gear position error
	signed long gearPosErr = gearPositionError();
 
	/*set RA speed and direction based on gear postion error feedback 
	pid controlled with a bidirectional setpoint */
	    //pid setup moved to main setup routine
      myPID.SetTunings(kp, ki, kd); 
      //myPID.SetControllerDirection(DIRECT); //now set in main setup
      pidSetpoint = 0;                         //aiming for 0 error to follow tracking
        pidInput = ((.01*gearPosErr));
                    myPID.Compute();
        speedSetpoint = ((byte)pidOutput);
       
       if (speedSetpoint > 0) {runMotorEast ((byte)speedSetpoint);}  
       if (speedSetpoint < 0) {runMotorWest ((byte)speedSetpoint);} 
 
	
	// get error 'state'
	byte currentErrorState;
	if(gearPosErr == (signed long)0) { currentErrorState = (byte)_ERROR_ZERO; }
	if(gearPosErr < (signed long)0) { currentErrorState = (byte)_ERROR_WEST; } 
	if(gearPosErr > (signed long)0) { currentErrorState = (byte)_ERROR_EAST; } 
	if(gearPosErr < (signed long)-200) { currentErrorState = (byte)_ERROR_BIG_WEST; } 
	if(gearPosErr > (signed long)200) { currentErrorState = (byte)_ERROR_BIG_EAST; } 
	
	if(!goingTo) {
	        // update history
		gearPosErrHist_index++; 
		if(gearPosErrHist_index >= _HISTORY_BYTES) { gearPosErrHist_index = 0; }
		gearPosErrHist[gearPosErrHist_index] = currentErrorState;
    myPID.SetMode(AUTOMATIC);	
	
	}
	
	switch(currentErrorState) {
	  case _ERROR_ZERO:
		/* commented on RA to let motor keep old speed and make output smoother between sidereal updates
    analogWrite (RAEN,0);	         // digitalWrite (RAIN1,LOW); //rbw
	  digitalWrite (RAIN2, LOW); //rbw 
		*/
		goingTo = false;
		  myPID.SetMode(AUTOMATIC);    //turn pid off when large error present
		break;
	  case _ERROR_WEST:
      myPID.SetMode(AUTOMATIC);    //turn pid off when large error present
			goingTo = false;
		break;
	  case _ERROR_BIG_EAST:
		runMotorWest(254);
		 myPID.SetMode(MANUAL);    //turn pid off when large error present
		goingTo = true;
		break;
	  case _ERROR_EAST:
     goingTo = false;
		myPID.SetMode(AUTOMATIC);    //turn pid off when large error present
		break;
	  case _ERROR_BIG_WEST:
		goingTo = true;
    myPID.SetMode(MANUAL);    //turn pid off when large error present
		runMotorEast(254);
		break;
	} 
}

void dec_axis_c::priorityUpdate() {
	// update gear position
	gearPosition_raw += (signed long)encoders.readDECencoderMovement();
	if(newGearPositon_raw_value_event) {
	  gearPosition_raw = newGearPositon_raw_value;
	  newGearPositon_raw_value_event = false;
	}
	
	// no tracking when slewing
	if(slewingNorth || slewingSouth) {
		return;
	}
	
	// get error 'state'
	byte currentErrorState;
	signed long gearPosErr = gearPositionError();
	if(gearPosErr == (signed long)0) { currentErrorState = (byte)_ERROR_ZERO; }
	if(gearPosErr < (signed long)0) { currentErrorState = (byte)_ERROR_SOUTH; }
	if(gearPosErr > (signed long)0) { currentErrorState = (byte)_ERROR_NORTH; }
	if(gearPosErr < (signed long)-200) { currentErrorState = (byte)_ERROR_BIG_SOUTH; }
	if(gearPosErr > (signed long)200) { currentErrorState = (byte)_ERROR_BIG_NORTH; }
	
	if(!goingTo) {
	        // update history
		gearPosErrHist_index++; 
		if(gearPosErrHist_index >= _HISTORY_BYTES) { gearPosErrHist_index = 0; }
		gearPosErrHist[gearPosErrHist_index] = currentErrorState;
	}
	
	switch(currentErrorState) {
	  case _ERROR_ZERO:
		  analogWrite (DECEN,0);//rbw
	    digitalWrite (DECIN1, LOW); //rbw
	    digitalWrite (DECIN2, LOW); //rbw 
                goingTo = false;
		break;
	  case _ERROR_NORTH:
		runMotorSouth(1); //1 rbw
		goingTo = false;
		break;
	  case _ERROR_BIG_NORTH:
		runMotorSouth(255);
		goingTo = true;
		break;
	  case _ERROR_SOUTH:
		runMotorNorth(1); //1 rbw
		goingTo = false;
		break;
	  case _ERROR_BIG_SOUTH:
		goingTo = true;
		runMotorNorth(255);
		break;
	}
}


// -------------------------------------------------------------
// Main loop - Low Priority
// -------------------------------------------------------------
void ra_axis_c::update() {
	priorityUpdate();
//get pid variables from eeprom
kd=(EEPROMHandler->readRASlowSpeed()); //read pid derivitive from eeprom
kp=(EEPROMHandler->readRAErrorTolerance()); //read proportional from eeprom
ki=(EEPROMHandler->readDECErrorTolerance()); //read intergral from eeprom ---reused settings.
gearTeeth=(EEPROMHandler->readDECSlowSpeed());//igearTeeth;
	// Update Shaft Position
	boolean zWasAlreadyKnown = firstZrecorded;
	switch(encoders.readRAshaftReset()) {
		case -1 :
			firstZrecorded = true;
			shaftPosition = 0 + (signed int)encoders.readRAshaftMovement();
			break;
		case 0 :
			shaftPosition += (signed int)encoders.readRAshaftMovement();
			break;
		case 1 :
			firstZrecorded = true;
			shaftPosition = shaftPPR + (signed int)encoders.readRAshaftMovement();
			break;
	}
	
	// if slewing just run the motors and return from function
	if(slewingEast || slewingWest) {
		setMotorSpeed(slewSpeed);
		if(slewingEast) {
			runMotorEast();
		}
		if(slewingWest) {
			runMotorWest();
		}
		sync();
		return;
	}
 
	// get a tally of error history
	byte history_count[3]= {0,0,0};
	for(byte loop=0; loop<(byte)_HISTORY_BYTES; loop++) {
		if(gearPosErrHist[loop] < _ERROR_BIG_WEST) {
			history_count[gearPosErrHist[loop]]++;
		}
	}
	// use error history tally to see if any motor adjustments may help
	if(history_count[_ERROR_ZERO] < _HISTORY_BYTES_QUATER) {
		if(history_count[_ERROR_EAST]>((byte)_HISTORY_BYTES_QUATER - 1 ) && 
		    history_count[_ERROR_WEST]>((byte)_HISTORY_BYTES_QUATER - 1 )) {
			// wiggling around target a lot - slow down motor
			//minimumMotorSpeed--;
			motorRange = (byte)_RA_MOTOR_MAX_SPEED - (byte)minimumMotorSpeed;
			resetgearPosErrHist();
		}
		if(history_count[_ERROR_EAST]>((byte)_HISTORY_BYTES - 1 ) ||
		    history_count[_ERROR_WEST]>((byte)_HISTORY_BYTES - 1 )) {
			// losing target - increase speed
			//minimumMotorSpeed++; // orig ++ rbw 
			motorRange = (byte)_RA_MOTOR_MAX_SPEED - (byte)minimumMotorSpeed;
			resetgearPosErrHist();
		}
	} 
  
	
	// if "going to" check target distance if close then unset  
	if(goingTo) {
		if((gearPositionError() > -200) || gearPositionError() < 200) {
			goingTo = false;
		}
	}
	
	  // Check for EEPROM Updates - if settings have changed then reload them
	if(EEPROMHandler->EEPROMUpdates != EEPROMUpdateIndex) {
		EEPROMUpdateIndex = EEPROMHandler->EEPROMUpdates;
		setupTrackingRates();
	}
	
	// sidereal motion
	// increment gearPosition_raw every sideReelPulseTime microseconds to counter sidereal motion
	// time of next event
	
	unsigned long nextPulse = (sideRealPulseTime_lastPulse + sideRealPulseTime);
	// check for overflow
	boolean microsOverflow = (micros() < 0x000000FF) && (nextPulse > 0xFFFFFF00);
	// timer
	if((micros() > nextPulse) | microsOverflow) {
		sideRealPulseTime_lastPulse += sideRealPulseTime;
		// sidereal motion has effectivley 'pushed' the gears effective position back West
		// use inverted position updates and encoder tables if motor/encoder mounted on same side of shaft as motor - Rbw 3-16-16

   //based on encoder gear position? *head spinning* 
#ifdef RA_ENC_REVERSE
                gearPosition_raw++; //motor mounted encoder 
#else
                gearPosition_raw--; //Far side slow motion shaft mounted encoder
                
#endif                
		if(gearPosition_raw<0) {
			gearPosition_raw=(gearPosition_max-1);
		}
	}
}


void dec_axis_c::update() {
	priorityUpdate();
	
	// if slewing just run the motors, sync position and return from function
	if(slewingNorth || slewingSouth) {
		setMotorSpeed(slewSpeed);
		if(slewingNorth) {
			runMotorNorth();
		}
		if(slewingSouth) {
			runMotorSouth();
		}
		sync();
		return;
	}
	
	// Check for EEPROM Updates
	if(EEPROMHandler->EEPROMUpdates != EEPROMUpdateIndex) {
		EEPROMUpdateIndex = EEPROMHandler->EEPROMUpdates;
		setupTrackingRates();
	}
	
// get a tally of error history
/*	byte history_count[3]= {0,0,0};
	for(byte loop=0; loop<(byte)_HISTORY_BYTES; loop++) {
		if(gearPosErrHist[loop] < _ERROR_BIG_WEST) {
			history_count[gearPosErrHist[loop]]++;
		}
	} */
 // use error history tally to see if any motor adjustments may help
/*	if(history_count[_ERROR_ZERO] < _HISTORY_BYTES_QUATER) {
		if(history_count[_ERROR_NORTH]>((byte)_HISTORY_BYTES_QUATER - 1 ) && 
		    history_count[_ERROR_SOUTH]>((byte)_HISTORY_BYTES_QUATER - 1 ))
		    {
			// wiggling around target a lot - slow down motor
			minimumMotorSpeed--;
			motorRange = (byte)_DEC_MOTOR_MAX_SPEED - (byte)minimumMotorSpeed;
			resetgearPosErrHist();
		} */
	/*	if(history_count[_ERROR_NORTH]>((byte)_HISTORY_BYTES - 1 ) ||
		    history_count[_ERROR_SOUTH]>((byte)_HISTORY_BYTES - 1 )) {
		    // loosing target - increase speed
			minimumMotorSpeed++;
			motorRange = (byte)_DEC_MOTOR_MAX_SPEED - (byte)minimumMotorSpeed;
			resetgearPosErrHist();
		}*/ 
  } 

// -------------------------------------------------------------
// Position
// -------------------------------------------------------------
signed long ra_axis_c::gearPosition() {
		return gearPosition_raw;
}

signed long dec_axis_c::gearPosition() {
		return gearPosition_raw;
}

void axis_c::setgearPosition_raw(signed long newValue) {
	newGearPositon_raw_value = newValue;
	newGearPositon_raw_value_event = true;
}

signed long ra_axis_c::gearPositionError() {
	return getDiff(gearPosition(), gearPosition_target);
}

signed long dec_axis_c::gearPositionError() {
	return getDiff(gearPosition(), gearPosition_target);
}


Angle* ra_axis_c::currentAngle() {
	axisAngle -> SetRAFromGearPos((double)gearPosition(), (double)gearPosition_max);
	return axisAngle;
}

Angle* dec_axis_c::currentAngle() {
	axisAngle -> DecimalToDEC_fullCircle((float)gearPosition() / (float)gearPosition_max);
	return axisAngle;
}

bool dec_axis_c::DecWestSide() {
  if(gearPosition_raw > gearPosition_max_half) {
	axisAngle -> decWestSide = false;
	axisAngle -> decEastSide = true;
	return false;
  } else {
	axisAngle -> decWestSide = true;
	axisAngle -> decEastSide = false;
	return true;
  }
}

// get difference from A to B paying regards to the fact that the shortest route between the two may cross 0
signed long axis_c::getDiff(unsigned long A, unsigned long B) {
  // cache pervious result to avoid wasting cycles recalcing
	static signed long lastResult;
	static signed long lastA;
	static signed long lastB;

	// check if result has already been calculated and return if it has
	if((lastA==A)&&(lastB==B)) {
		return lastResult;
	}

	// otherwise re-calc
	lastA = A;
	lastB = B;
	signed long rawDiff = B - A; // the RAW difference

	// if less than half the circle away from each other the 0 point has not been crossed so rawDiff is result
	if(!((rawDiff > gearPosition_max_half) || ((0-rawDiff) > gearPosition_max_half))) {
		lastResult=rawDiff;
		return lastResult;
	}
	if(A > B) {
		lastResult = (gearPosition_max - A) + B;    // B is in front of A over the 0 line
		return lastResult;
	}
	// B must be greater then A
	lastResult = 0-((gearPosition_max - B) + A); // B is behind A over the 0 line
	return lastResult;
}

signed long ra_axis_c::AngleToGearPos(Angle posAngle) {
	return (signed long)(posAngle.RAToDecimal() * (float)gearPosition_max);
}

signed long dec_axis_c::AngleToGearPos(Angle posAngle) {
	return (signed long)(posAngle.DECToDecimal() * (float)gearPosition_max);
}

// -------------------------------------------------------------
// Control
// -------------------------------------------------------------
// make current position the target
void ra_axis_c::sync() {
	gearPosition_target = gearPosition();
}

void dec_axis_c::sync() {
	gearPosition_target = gearPosition();
}

// make specified Angle object the current position
void ra_axis_c::sync(Angle syncPos) {
	gearPosition_raw = AngleToGearPos(syncPos);
	sync();
}

void dec_axis_c::sync(Angle syncPos) {
	gearPosition_raw = AngleToGearPos(syncPos);
	sync();
}

// make specified Angle object the target
void ra_axis_c::setTarget(Angle syncPos) {
	gearPosition_target = AngleToGearPos(syncPos);
}

void dec_axis_c::setTarget(Angle syncPos) {
	gearPosition_target = AngleToGearPos(syncPos);
}



//RA MIN/MAX check and final speed output

void ra_axis_c::setMotorSpeed(byte newSpeed) {        
//Minimum speeds have been removed from my design which uses a pid library to set tracking speed - rbw
// if new speed setting would overflow byte set to max speed
   if(((int)newSpeed > 255))
    {
    analogWrite (RAEN, ((byte)_RA_MOTOR_MAX_SPEED));
        return;
    } 
// limit with max speed
  if(newSpeed > (byte)_RA_MOTOR_MAX_SPEED)            
    {
    analogWrite (RAEN, ((byte)_RA_MOTOR_MAX_SPEED));
        return; 
    }
//output final speed command
  analogWrite (RAEN, (newSpeed)); //rbw motor->setSpeed(newSpeed);
}

//**************************DEC SPEED + MIN/MAX CHECK before final speed output changes******************

void dec_axis_c::setMotorSpeed(byte newSpeed) {
	// check newSpeed overflow 8bit limit
	if(((int)newSpeed > 255)) {
		analogWrite (DECEN,((byte)_DEC_MOTOR_MAX_SPEED));
return;
	}
	// limit with max speed
	if(newSpeed > (byte)_DEC_MOTOR_MAX_SPEED) {
			analogWrite (DECEN,((byte)_DEC_MOTOR_MAX_SPEED));
return;
	}
	// otherwise output final speed
	//rbw motor->setSpeed(newSpeed);
          analogWrite (DECEN, (newSpeed));

#ifdef debugOut_shaftPositions 
      Serial.println((newSpeed));
          delay(20); 
#endif

}

//****************MANUAL SLEW CONTROLS*****************

// ra slewing/manual control
void ra_axis_c::stopSlew() {
  if(slewingEast || slewingWest) {
    slewSpeed = 0;
    // unset slewing flags
    slewingEast = false;
    slewingWest = false;
    stopMotor();
  myPID.SetMode(AUTOMATIC);
  }
}
// dec slewing/manual control
void dec_axis_c::stopSlew() {
  if(slewingNorth || slewingSouth) {
    slewSpeed = 0;
    // unset slewing flags/
    slewingNorth = false;
    slewingSouth = false;
    stopMotor();
  }
}

bool ra_axis_c::isSlewing() {
  return (slewingWest || slewingEast);
myPID.SetMode(MANUAL);
}

bool dec_axis_c::isSlewing() {
  return (slewingNorth || slewingSouth);
}

void ra_axis_c::slewEast(byte newSlewSpeed) {
  slewSpeed = newSlewSpeed;
  slewingEast = true;
  slewingWest = false;

}

void ra_axis_c::slewWest(byte newSlewSpeed) {
  slewSpeed = newSlewSpeed;
  slewingEast = false;
  slewingWest = true;
}

void dec_axis_c::slewNorth(byte newSlewSpeed) {
  slewSpeed = newSlewSpeed;
  slewingNorth = true;
  slewingSouth = false;
}

void dec_axis_c::slewSouth(byte newSlewSpeed) {
  slewSpeed = newSlewSpeed;
  slewingNorth = false;
  slewingSouth = true;
}



//*******************MOTOR ENABLES***********************
void ra_axis_c::stopMotor() {
	//rbw motor->run(RELEASE);
	analogWrite (RAEN,0);//rbw
	digitalWrite (RAIN1, LOW); //rbw
	digitalWrite (RAIN2, LOW); //rbw
}

void dec_axis_c::stopMotor() {
	//rbw motor->run(RELEASE);
	analogWrite (RAEN,0);//rbw
	digitalWrite (DECIN1, LOW); //rbw
	digitalWrite (DECIN2, LOW); //rbw
}

void dec_axis_c::runMotorNorth() {
	#ifndef _DEC_DIRECTION_REVERSE
		//rbw motor->run(BACKWARD);
		digitalWrite (DECIN1, HIGH); //rbw
		digitalWrite (DECIN2, LOW); //rbw
	#else
		//rbw motor->run(FORWARD);
		digitalWrite (DECIN1, LOW); //rbw
		digitalWrite (DECIN2, HIGH); //rbw
	
	#endif
}

void dec_axis_c::runMotorSouth() {
	#ifndef _DEC_DIRECTION_REVERSE
		//rbw motor->run(FORWARD);
		digitalWrite (DECIN1, LOW); //rbw
		digitalWrite (DECIN2, HIGH); //rbw
	#else
		//rbw motor->run(BACKWARD);
		digitalWrite (DECIN1, HIGH); //rbw
		digitalWrite (DECIN2, LOW); //rbw
	#endif
}

void ra_axis_c::runMotorEast() {
	#ifndef _RA_DIRECTION_REVERSE
		//rbw motor->run(BACKWARD);
		digitalWrite (RAIN1, HIGH); //rbw
		digitalWrite (RAIN2, LOW); //rbw
	#else
		//rbw motor->run(FORWARD);
		digitalWrite (RAIN1, LOW); //rbw
		digitalWrite (RAIN2, HIGH); //rbw
	#endif
}

void ra_axis_c::runMotorWest() {
	#ifndef _RA_DIRECTION_REVERSE
		//rbw motor->run(FORWARD);
		digitalWrite (RAIN1, LOW); //rbw
		digitalWrite (RAIN2, HIGH); //rbw
		#else
		//rbw motor->run(BACKWARD);
		digitalWrite (RAIN1, HIGH); //rbw
		digitalWrite (RAIN2, LOW); //rbw
	#endif
} 


