/*
 * Replay Stream Format Specification:
 *  - Saves state at 30Hz for 120sec (2 minutes) == 3600 state snapshots.
 *  - At 1 byte/snapshot this == 3.5 KiB per replay.
 *
 * - The following aspects of robot state are replicated:
 *  o Left / Right Drivetrain state
 *  o Hang motor position (full up/down)
 *  o Catapult controls
 * 
 * - To do this, we save:
 *  o Y/Z joystick input deltas OR L/R drivetrain output signal deltas - 4*2 bits == 1 byte (8 bits)
 *   Deltas are constrained to 16 values (4 bits) for each: [-255, -127, -63, -31, -15, -7, -3, 0, +3, +7, +15, +31, +63, +127, +255, <invalid>]
 *   Both the joystick and motor output deltas use the same format (+-127 shorts) so the format and size is the same no matter which we go with
 *  o Controller button state:
 *   - Hang state (driving up & driving down) - 2 bits
 *   - Cat controller state - up button state + down button state (2 bits)
 *  o Delta presence indicators for each set (2 bits).
 *
 *
 *
 * Final format:
 *  [1 bit] Joysticks changed?
 *   if so:
 *    [4 bits] Y joystick axis output delta
 *    [4 bits] Z joystick axis output delta
 *  [1 bit] Boolean states changed?
 *   if so:
 *    [1 bit] Hang motors being driven up?
 *    [1 bit] Hang motors being driven down?
 *    [1 bit] Catapult controller up button state. (pressed/not pressed)
 *    [1 bit] Catapult controller down button state. (ditto)
 *
 * Data usage per snapshot:
 *  Minimum 2 bits to indicate no state changes.
 *  Maximum 14 bits if everything changes at once.
 *  9 bits for drive-only snapshots.
 *  5 bits for button-only snapshots.
 *
 * Thus, the maximum size for a snapshot is (14 bits * 30 snapshots/sec * 120 seconds) = 50400 bits / 8 bits/byte = 6300 bytes total.
 *
 * Note that all-zero bits indicate a snapshot with unchanged values (no deltas).
 * This fact is used here-- this means we don't need to know the actual number of snapshots recorded when replaying.
 */
 
/* Powers of 2, minus 1.
 * value&(progressiveBitmasks[n]) == first <n> bits of <value> */
const unsigned char progressiveBitmasks[9] = {
	0x00,
	0x01,
	0x03,
	0x07,
	0x0F,
	0x1F,
	0x3F,
	0x7F,
	0xFF,
};
 

unsigned char currentStream[6300];
unsigned int currentByteIndex = 0;
unsigned short currentBitIndex = 0;

void writeBoolean(bool value) {
	if(value)
		currentStream[currentByteIndex] |= (1<<currentBitIndex);
		
	if(currentBitIndex == 7) {
		currentBitIndex = 0;
		currentByteIndex++;
	} else {
		currentBitIndex++;
	}
}

// writes a 4-bit integer.
void writeSmallInt(unsigned char value) {
	if(currentBitIndex < 5) {
		// value will fit within current byte.
		currentStream[currentByteIndex] |= ((value&0x0F)<<currentBitIndex);
		currentByteIndex += (currentBitIndex == 4 ? 1 : 0);
		currentBitIndex = (currentBitIndex == 4 ? 0 : currentBitIndex+4);
	} else {
		// value will not fit within current byte.
		// (currentBitIndex-4) == number of overflow bits.
		// 8 - currentBitIndex == number of bits that can be saved in the current byte.
		currentStream[currentByteIndex] |= ( (value&progressiveBitmasks[8-currentBitIndex]) << currentBitIndex);
		currentStream[currentByteIndex+1] |= (value&(progressiveBitmasks[currentBitIndex-4]<<(8-currentBitIndex)))>>(8-currentBitIndex);
		
		currentByteIndex += 1;
		currentBitIndex = currentBitIndex-4;
	}
}

bool readBoolean() {
	bool val = (bool)(currentStream[currentByteIndex] & (1<<currentBitIndex));
	
	if(currentBitIndex == 7) {
		currentBitIndex = 0;
		currentByteIndex++;
	} else {
		currentBitIndex++;
	}
	
	return val;
}

unsigned char readSmallInt() {
	unsigned char res = 0;
	if(currentBitIndex < 5) {
		// value should fit within current byte.
		res = (currentStream[currentByteIndex] & (0xF<<currentBitIndex))>>currentBitIndex;
		currentByteIndex += (currentBitIndex == 4 ? 1 : 0);
		currentBitIndex = (currentBitIndex == 4 ? 0 : currentBitIndex+4);
	} else {
		// value will not fit within current byte.
		res = (currentStream[currentByteIndex] >> currentBitIndex); // get the rest of the bits in this byte
		res |= (currentStream[currentByteIndex+1] & progressiveBitmasks[currentBitIndex-4]) << (8-currentBitIndex); // get overflow bits
		
		currentByteIndex += 1;
		currentBitIndex = currentBitIndex-4;
	}
	
	return res;
}

signed short deltas[14] = {
	-254,
	-127,
	-63,
	-31,
	-15,
	-7,
	-3,
	0,
	3,
	7,
	15,
	31,
	63,
	127,
	254
};

signed short applyDelta(signed short in, unsigned char encDelta) {
	return in + deltas[encDelta];
}

unsigned char getDelta(signed short fin, signed short init) {
	signed short delta = fin-init;
	
	/* I'm pretty sure there's some tricky bit-level arithmetic I could use here, */
	/* But this is just easier to write. */
	if(delta <= -254) {
		return 0;
	} else if(delta <= -127) {
		return 1;
	} else if(delta <= -63) {
		return 2;
	} else if(delta <= -31) {
		return 3;
	} else if(delta <= -15) {
		return 4;
	} else if(delta <= -7) {
		return 5;
	} else if(delta <= -3) {
		return 6;
	} else if(delta > -3 && delta < 3) {
		return 7;
	} else if(delta <= 7) {
		return 8;
	} else if(delta <= 15) {
		return 9;
	} else if(delta <= 31) {
		return 10;
	} else if(delta <= 63) {
		return 11;
	} else if(delta <= 127) {
		return 12;
	} else if(delta <= 254) {
		return 13;
	} else {
		// shouldn't get here
		return 7; // return encoded delta of 0.
	}
}

struct stateStruct = {
	signed short left;
	signed short right;
	
	bool catUp;	// Corresponds to vexRT[Btn6U]
	bool catDown;	// Corresponds to vexRT[Btn6D]
	bool hangUp;	// corresponds to vexRT[Btn5U]
	bool hangDown;  // corresponds to vexRT[Btn5D]
};

stateStruct currentState;

void updateAndWriteSnapshot() {
	short yAxis = (abs(vexRT[Ch2]) < deadband) ? 0 : -vexRT[Ch2];
	short zAxis = (abs(vexRT[Ch1]) < deadband) ? 0 : -vexRT[Ch1];
	
	short right = yAxis - zAxis;
	short left = yAxis + zAxis;

	right = (abs(right) > absoluteMaxDrive) ? (sgn(right)*absoluteMaxDrive) : right;
	left = (abs(left) > absoluteMaxDrive) ? (sgn(left)*absoluteMaxDrive) : left;
	
	unsigned char deltaL = getDelta(left, currentState.left);
	unsigned char deltaR = getDelta(right, currentState.right);
	
	if(deltaL == 7 && deltaR == 7) {
		// both deltas are 0-- no need to save them.
		writeBoolean(false);
	} else {
		// save joystick deltas:
		writeBoolean(true);
		writeSmallInt(deltaL);
		writeSmallInt(deltaR);
	}
	
	/* Get button state. */
	bool catUpChanged = ((bool)vexRT[Btn6U] != currentState.catUp);
	bool catDnChanged = ((bool)vexRT[Btn6D] != currentState.catDown);
	bool hngUpChanged = ((bool)vexRT[Btn5U] != currentState.hangUp);
	bool hngDnChanged = ((bool)vexRT[Btn5D] != currentState.hangDown);
	
	if(hngUpChanged || hngDnChanged || upChanged || dnChanged) {
		/* Something's changed-- save new state */
		writeBoolean(true);
		writeBoolean(vexRT[Btn5U]);
		writeBoolean(vexRT[Btn5D]);
		writeBoolean(vexRT[Btn6U]);
		writeBoolean(vexRT[Btn6D]);
	} else {
		/* Nothing's changed. */
		writeBoolean(false);
	}
	
	/* Save current state. */
	currentState.left = left;
	currentState.right = right;
	currentState.hang = hangState;
	currentState.catUp = vexRT[Btn6U];
	currentState.catDown = vexRT[Btn6D];
}

// load from replay stream to currentState object
void replayToStruct() {
	bool joystickChanged = readBoolean();
	if(joystickChanged) {
		signed short left = applyDelta(currentState.left, readSmallInt());
		signed short right = applyDelta(currentState.right, readSmallInt());
		
		// apply voltage scaling here possibly
		
		currentState.left = left;
		currentState.right = right;
	}
	
	bool boolsChanged = readBoolean();
	if(boolsChanged) {
		currentState.hangUp = readBoolean();
		currentState.hangDown = readBoolean();
		currentState.catUp = readBoolean();
		currentState.catDown = readBoolean();
	}
}

// load from currentState object to actual motor outputs
bool drivingHangMotors = false;
void loadMotorOutputs() {
	motor[LBack] = motor[LFront] = currentState.left;
	motor[RBack] = motor[RFront] = currentState.right;
	
	if(!limSwitchEnabled) {
		if(currentState.catUp && !currentState.catDown) {
			catapultDown();
		} else if(!currentState.catUp && currentState.catDown) {
			catapultUp();
		} else if(!currentState.catUp && !currentState.catDown) {
			catapultStop();
		}
	} else {
		if(catState == 0) {
			if(currentState.catUp) {
				catapultDown();
			} else if(currentState.catDown) {
				catapultUp();
			} else {
				catapultStop();
			}

			if(sensorValue[catapultLim] == 1) {
				catapultStop();
				catState = 1;
				clearTimer(T3);
			}
		} else if(catState == 1) {
			if(!currentState.catUp) {
				if(time1[T3] > 150) {
					catState = 2;
				}
			} else if(currentState.catDown) {
				catapultUp();
			} else {
				catapultStop();
			}

			if(sensorValue[catapultLim] == 0) {
				catState = 0;
			}
		} else if(catState == 2) {
			if(currentState.catUp) {
				fireRoutine();
			} else if(currentState.catDown) {
				catapultUp();
			} else {
				catapultStop();
			}

			if(sensorValue[catapultLim] == 0) {
				catState = 0;
			}
		}
	}
	
	if( currentState.hangUp && !currentState.hangDown ) {
		motor[hangMotor] = 127;
	} else if( currentState.hangDown && !currentState.hangUp ) {
		motor[hangMotor] = -127;
	} else if( !currentState.hangDown && !currentState.hangUp ) {
		motor[hangMotor] = 0;
	}
}

const float snapshotFreq = 30; // Hz
const float deltaT = (1/snapshotFreq) * 1000; // time between snapshots in milliseconds

// Replay an entire snapshot.
// Note that currentByteIndex and currentBitIndex are both clobbered.
void autoReplaySnapshot() {
	currentByteIndex = 0;
	currentBitIndex = 0;
	while(true) {
		replayToStruct();
		loadMotorOutputs();
		sleep((int)deltaT);
	}
}

/* Size of the stream in bytes is written first as a decimal number,
 * then the actual stream data is written as hex. Spaces are put in between each hex byte.
 */
void dumpReplayToDebugStream() {
	writeDebugStream("%d", currentByteIndex);
	for(int i=0;i<currentByteIndex;i++) {
		writeDebugStream("%x ", currentStream[i]);
	}
}

