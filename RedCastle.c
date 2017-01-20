#include "./Enterprise.c"

/* NOTE: old auto code is in autodrive.c */

/* Control parameter constants. */
const int deadband = 25;
int manualTurnOut = 32;
int absoluteMaxDrive = 96;
int maxMotorOut = 63;
int turnMotorOut = 32;

bool limSwitchEnabled = false;

struct controlState {
	signed char yAxis;	/* Raw Ch2 from stick */
	signed char zAxis;	/* Raw Ch1 from stick */
	
	bool catUp;		/* Button 6D */
	bool catDown;		/* Button 6U */
	bool catReset;		/* Button 7U */
	
	bool hangUp;		/* Button 5U */
	bool hangDown;		/* Button 5D */
	
	bool turnRight;		/* Button 8U */
	bool turnLeft;		/* Button 8D */
	
	unsigned int catState;
};
controlState currentState;
replayData replay;

/* Reset state (for when switching from auto->driver) */
void resetState() {
	currentState.yAxis = 0;
	currentState.zAxis = 0;
	
	currentState.catUp = false;
	currentState.catDown = false;
	currentState.catReset = false;
	currentState.hangUp = false;
	currentState.hangDown = false;
	currentState.turnLeft = false;
	currentState.turnRight = false;
}

/* Completely initialize state (from preauto->auto) */
void initState() {
	resetState();
	currentState.catState = 0;

}

void clearReplay() {
	for(unsigned int i=0;i<10802;i++) {
		replay.streamData[i] = 0;
	}
	
	replay.streamIndex = 0;
	replay.streamSize = 0;
}

void catapultDown() {
	motor[rightLowerIntake] = 127;
	motor[rightUpperIntake] = 127;
	motor[leftLowerIntake] = 127;
	motor[leftUpperIntake] = 127;
}

void catapultUp() {
	motor[rightLowerIntake] = -127;
	motor[rightUpperIntake] = -127;
	motor[leftLowerIntake] = -127;
	motor[leftUpperIntake] = -127;
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

void stopAllMotors() {
	stopMotors();
	catapultStop();
	motor[hangMotor] = 0;
}

void intakeReset() {
		if (currentState.catReset && !SensorValue[catapultLim])
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

void fireControl() {
	if(!limSwitchEnabled) {
		if(currentState.catDown && !currentState.catUp) {
			catapultDown();
		} else if(!currentState.catDown && currentState.catUp) {
			catapultUp();
		} else if(!currentState.catDown && !currentState.catUp) {
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
		if(currentState.catState == 0) {
			if(currentState.catDown) {
				catapultDown();
			} else if(currentState.catUp) {
				catapultUp();
			} else {
				catapultStop();
			}

			if(sensorValue[catapultLim] == 1) {
				catapultStop();
				currentState.catState = 1;
				clearTimer(T3);
			}
		} else if(currentState.catState == 1) {
			if(!currentState.catDown) {
				if(time1[T3] > 150) {
					currentState.catState = 2;
				}
			} else if(currentState.catUp) {
				catapultUp();
			} else {
				catapultStop();
			}

			if(sensorValue[catapultLim] == 0) {
				currentState.catState = 0;
			}
		} else if(currentState.catState == 2) {
			if(currentState.catDown) {
				fireRoutine();
			} else if(currentState.catUp) {
				catapultUp();
			} else {
				catapultStop();
			}

			if(sensorValue[catapultLim] == 0) {
				currentState.catState = 0;
			}
		}
	}
}

void hangControl() {
	if( currentState.hangUp && !currentState.hangDown ) {
		motor[hangMotor] = 127;
	} else if( currentState.hangDown && !currentState.hangUp ) {
		motor[hangMotor] = -127;
	} else if( !currentState.hangDown && !currentState.hangUp ) {
		motor[hangMotor] = 0;
	}
}

void moveControl() {
	if( currentState.turnLeft || currentState.turnRight ) {
		/* Rotation inputs: */
		motor[LBack] = motor[LFront] = (currentState.turnLeft ? -1*manualTurnOut : manualTurnOut);
		motor[RBack] = motor[RFront] = (currentState.turnLeft ? manualTurnOut : -1*manualTurnOut);
	} else {
		short yAxis = (abs(currentState.yAxis) < deadband) ? 0 : -currentState.yAxis;
		short zAxis = (abs(currentState.zAxis) < deadband) ? 0 : -currentState.zAxis;

		short right = yAxis - zAxis;
		short left = yAxis + zAxis;

		right = (abs(right) > absoluteMaxDrive) ? (sgn(right)*absoluteMaxDrive) : right;
		left = (abs(left) > absoluteMaxDrive) ? (sgn(left)*absoluteMaxDrive) : left;

		motor[RFront] = motor[RBack] = right;
		motor[LFront] = motor[LBack] = left;
	}
}

void controlLoopIteration() {
	intakeReset();
	fireControl();
	hangControl();
	moveControl();
}

#define TEST_BIT(x, i) (((x)&(1<<(i))) > 0)

void replayToControlState() {
	currentState.yAxis = (signed char)readNextByte(&replay);
	currentState.zAxis = (signed char)readNextByte(&replay);

	/* Bit | Button
	 *  0  | Catapult Up (6D)
	 *  1  | Catapult Down (6U)
	 *  2  | Catapult Reset (7U)
	 *  3  | Hang Up (5U)
	 *  4  | Hang Down (5D)
	 *  5  | Turn Right (8U)
	 *  6  | Turn Left (8D)
	 *  7  | <reserved>
	 */
	unsigned char buttonState = readNextByte(&replay);
	
	currentState.catUp = TEST_BIT(buttonState, 0);
	currentState.catDown = TEST_BIT(buttonState, 1);
	currentState.catReset = TEST_BIT(buttonState, 2);
	currentState.hangUp = TEST_BIT(buttonState, 3);
	currentState.hangDown = TEST_BIT(buttonState, 4);
	currentState.turnRight = TEST_BIT(buttonState, 5);
	currentState.turnLeft = TEST_BIT(buttonState, 6);
}

void controllerToControlState() {
	currentState.yAxis = vexRT[Ch2];
	currentState.zAxis = vexRT[Ch1];
	
	currentState.catUp = (vexRT[Btn6D] > 0);
	currentState.catDown = (vexRT[Btn6U] > 0);
	currentState.catReset = (vexRT[Btn7U] > 0);
	currentState.hangUp = (vexRT[Btn5U] > 0);
	currentState.hangDown = (vexRT[Btn5D] > 0);
	currentState.turnRight = (vexRT[Btn8U] > 0);
	currentState.turnLeft = (vexRT[Btn8D] > 0);
}

void controlStateToReplay() {
	writeByte(&replay, (unsigned char)currentState.yAxis);
	writeByte(&replay, (unsigned char)currentState.zAxis);
	
	unsigned char buttonState = 0;
	buttonState |= (currentState.catUp ? 1 : 0);
	buttonState |= (currentState.catDown ? 1 : 0) << 1;
	buttonState |= (currentState.catReset ? 1 : 0) << 2;
	buttonState |= (currentState.hangUp ? 1 : 0) << 3;
	buttonState |= (currentState.hangDown ? 1 : 0) << 4;
	buttonState |= (currentState.turnRight ? 1 : 0) << 5;
	buttonState |= (currentState.turnLeft ? 1 : 0) << 6;
	
	writeByte(&replay, buttonState);
}