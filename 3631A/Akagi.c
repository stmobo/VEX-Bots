/* NOTE: old auto code is in autodrive.c */

#ifndef AKAGI_C
#define AKAGI_C

/* Control parameter constants. */
const int deadband = 25;
const int manualTurnOut = 32;

int fastSpeedLimit = 96;
int slowSpeedLimit = 48; // = 0.5 * fastSpeedLimit

const bool limSwitchEnabled = true;
const bool catStateEnabled = true;

struct control_t {
	signed char yAxis;	/* Raw Ch2 from stick */
	signed char zAxis;	/* Raw Ch1 from stick */

	bool catUp;	       	/* Button 6D */
	bool catDown;		/* Button 6U */
	bool catReset;		/* Button 7U */

	bool hangUp;		/* Button 5U */
	bool hangDown;		/* Button 5D */

	bool turnRight;		/* Button 8U */
	bool turnLeft;		/* Button 8D */
    bool slowDown;      /* Button 7L (tentative) */

	unsigned int catState;
    unsigned int speedLimit;
};

/* Reset state (for when switching from auto->driver) */
void resetState(control_t* state) {
	state->yAxis = 0;
	state->zAxis = 0;

	state->catUp = false;
	state->catDown = false;
	state->catReset = false;
	state->hangUp = false;
	state->hangDown = false;
	state->turnLeft = false;
	state->turnRight = false;
  state->slowDown = false;

  state->speedLimit = fastSpeedLimit;
}

/* Completely initialize state (from preauto->auto) */
void initState(control_t* state) {
	resetState(state);
	state->catState = 0;
    state->speedLimit = fastSpeedLimit;
}

void catapultDown() {
	motor[rightLowerIntake] = 127;
	motor[rightUpperIntake] = 127;
	motor[leftLowerIntake] = 127;
	motor[leftUpperIntake] = 127;
}

void catapultUp() {
	if(!limSwitchEnabled || (sensorValue[upperLim] == 0)) {
		motor[rightLowerIntake] = -127;
		motor[rightUpperIntake] = -127;
		motor[leftLowerIntake] = -127;
		motor[leftUpperIntake] = -127;
	}
}

void catapultStop() {
	motor[rightLowerIntake] = 0;
	motor[rightUpperIntake] = 0;
	motor[leftLowerIntake] = 0;
	motor[leftUpperIntake] = 0;
}

void stopMotors() {
	motor[LBack] = motor[LFront] = motor[RBack] = motor[RFront] = 0;
}

void stopAllMotorsCustom() {
	stopMotors();
	catapultStop();
	motor[hangMotor] = 0;
}

void intakeReset(control_t* state) {
		if (state->catReset && !SensorValue[catapultLim])
		{
			motor[rightLowerIntake] = 127;
			motor[leftLowerIntake] = 127;
			motor[rightUpperIntake] = 127;
			motor[leftUpperIntake] = 127;
		}
}

void fireRoutine() {
	clearTimer(T4);
	catapultDown();
	while(true) {
		if((sensorValue[catapultLim] == 0) || (time1[T4] > 50)) {
			catapultStop();
			return;
		}
		sleep(2);
	}
}

void fireControl(control_t* state) {
	if(!limSwitchEnabled || !catStateEnabled) {
		if(state->catDown && !state->catUp) {
			catapultDown();
		} else if(!state->catDown && state->catUp) {
			catapultUp();
		} else if(!state->catDown && !state->catUp) {
			catapultStop();
		}
	} else {
		/*
		 * CATAPULT STATE MACHINE:
		 *
		 * state 0 -> catapult moving to switch
		 * state 1 -> catapult halted at switch
		 * state 2 -> catapult ready to fire
		 */
		if(state->catState == 0) {
			if(state->catDown) {
				catapultDown();
			} else if(state->catUp) {
				catapultUp();
			} else {
				catapultStop();
			}

			if(sensorValue[catapultLim] == 1) {
				catapultStop();
				state->catState = 1;
				clearTimer(T3);
			}
		} else if(state->catState == 1) {
			if(!state->catDown) {
				if(time1[T3] > 150) {
					state->catState = 2;
				}
			} else if(state->catUp) {
				catapultUp();
			} else {
				catapultStop();
			}

			if(sensorValue[catapultLim] == 0) {
				state->catState = 0;
			}
		} else if(state->catState == 2) {
			if(state->catDown) {
				fireRoutine();
			} else if(state->catUp) {
				catapultUp();
			} else {
				catapultStop();
			}

			if(sensorValue[catapultLim] == 0) {
				state->catState = 0;
			}
		}
	}
}

void hangControl(control_t* state) {
	if( state->hangUp && !state->hangDown ) {
		motor[hangMotor] = 127;
	} else if( state->hangDown && !state->hangUp ) {
		motor[hangMotor] = -127;
	} else if( !state->hangDown && !state->hangUp ) {
		motor[hangMotor] = 0;
	}
}

void moveControl(control_t* state) {
	if( state->turnLeft || state->turnRight ) {
		/* Rotation inputs: */
		motor[LBack] = motor[LFront] = (state->turnLeft ? -1*manualTurnOut : manualTurnOut);
		motor[RBack] = motor[RFront] = (state->turnLeft ? manualTurnOut : -1*manualTurnOut);
	} else {
    /* Set speedlimit as appropriate: */
    float speedMult = 1.0;
    if(state->slowDown) {
        state->speedLimit = fastSpeedLimit;
        speedMult = 0.5;
    } else {
        state->speedLimit = fastSpeedLimit;
    }

		short yAxis = (abs(state->yAxis) < deadband) ? 0 : -state->yAxis;
		short zAxis = (abs(state->zAxis) < deadband) ? 0 : -state->zAxis;

		short right = yAxis - zAxis;
		short left = yAxis + zAxis;

		right = (abs(right) > state->speedLimit) ? (sgn(right)*state->speedLimit) : right;
		left = (abs(left) > state->speedLimit) ? (sgn(left)*state->speedLimit) : left;

		motor[RFront] = motor[RBack] = (signed char)((float)right * speedMult);
		motor[LFront] = motor[LBack] = (signed char)((float)left * speedMult);
	}
}

void controlLoopIteration(control_t* state) {
	intakeReset(state);
	fireControl(state);
	hangControl(state);
	moveControl(state);
}

void controllerToControlState(control_t* state) {
	state->yAxis = vexRT[Ch2];
	state->zAxis = vexRT[Ch1];

	state->catUp = (vexRT[Btn6D] > 0);
	state->catDown = (vexRT[Btn6U] > 0);
	state->catReset = (vexRT[Btn7U] > 0);
	state->hangUp = (vexRT[Btn5U] > 0);
	state->hangDown = (vexRT[Btn5D] > 0);
	state->turnRight = (vexRT[Btn8U] > 0);
	state->turnLeft = (vexRT[Btn8D] > 0);
    state->slowDown = (vexRT[Btn7L] > 0);
}

void replayToControlState(control_t* state, replay_t* replay) {
	state->yAxis = (signed char)readNextByte(replay);
	state->zAxis = (signed char)readNextByte(replay);

	/* Bit | Button
	 *  0  | Catapult Up (6D)
	 *  1  | Catapult Down (6U)
	 *  2  | Catapult Reset (7U)
	 *  3  | Hang Up (5U)
	 *  4  | Hang Down (5D)
	 *  5  | Turn Right (8U)
	 *  6  | Turn Left (8D)
	 *  7  | Slow Down (7L)
	 */
	unsigned char buttonState = readNextByte(replay);

	state->catUp = TEST_BIT(buttonState, 0);
	state->catDown = TEST_BIT(buttonState, 1);
	state->catReset = TEST_BIT(buttonState, 2);
	state->hangUp = TEST_BIT(buttonState, 3);
	state->hangDown = TEST_BIT(buttonState, 4);
	state->turnRight = TEST_BIT(buttonState, 5);
	state->turnLeft = TEST_BIT(buttonState, 6);
	state->slowDown = TEST_BIT(buttonState, 7);
}

void controlStateToReplay(control_t* state, replay_t* replay) {
	writeByte(replay, (unsigned char)state->yAxis);
	writeByte(replay, (unsigned char)state->zAxis);

	unsigned char buttonState = 0;
	buttonState |= (state->catUp ? 1 : 0);
	buttonState |= (state->catDown ? 1 : 0) << 1;
	buttonState |= (state->catReset ? 1 : 0) << 2;
	buttonState |= (state->hangUp ? 1 : 0) << 3;
	buttonState |= (state->hangDown ? 1 : 0) << 4;
	buttonState |= (state->turnRight ? 1 : 0) << 5;
	buttonState |= (state->turnLeft ? 1 : 0) << 6;
	buttonState |= (state->slowDown ? 1 : 0) << 7;

	writeByte(replay, buttonState);
}

int getReplayTime(replay_t* replay) {
    if(replay->streamSize > 0) {
        int nReplayFrames = (replay->streamSize - 2) / 3;
        return nReplayFrames * deltaT;
    }

    return 0;
}

bool doingReplayAuton = true;

void loadAutonomous(replay_t* replay) {
	int pos = sensorValue[autoSelector];

	if(pos < 727) {		// Illuminati Skills
		doingReplayAuton = false;
	} else if(pos < 1920) {	// Illuminati routine
		doingReplayAuton = false;
	} else if(pos < 2678) {	// Off
   		 clearLCDLine(0);
   		 displayLCDCenteredString(0, "Auto: None");

		return;
	} else if(pos < 3200) {	// A1
		writeDebugStreamLine("Loading: slot1");
		loadReplayFromFile("slot1", replay);

        clearLCDLine(0);
        displayLCDCenteredString(0, "Auto: Slot 1");
	} else if(pos < 3768) { // A2
		writeDebugStreamLine("Loading: slot2");
		loadReplayFromFile("slot2", replay);

        clearLCDLine(0);
        displayLCDCenteredString(0, "Auto: Slot 2");
	} else if(pos > 4080) {	// A3
		writeDebugStreamLine("Loading: slot3");
		loadReplayFromFile("slot3", replay);

        clearLCDLine(0);
        displayLCDCenteredString(0, "Auto: Slot 3");
	}

	clearLCDLine(1);
	displayLCDCenteredString(1, "Load done.");
	writeDebugStreamLine("Loading done.");
}

#endif /* end of include guard: AKAGI_C */
