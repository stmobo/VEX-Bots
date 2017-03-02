#ifndef WARSPITE_C
#define WARSPITE_C

struct control_t {
    /* CONTROL INPUTS: */
    signed char left;   // Analog Ch. 3
    signed char right;  // Analog Ch. 2

    bool armUp;         // Button 6U
    bool armDown;       // Button 6D

    bool clawOpen;      // Button 5U
    bool clawClose;     // Button 5D

    /* CONTROL STATE: */
    float clawErr;          // Claw control error
    float clawLErr;         // Claw control last error (prev. iteration)
    float clawSP;           // Claw control setpoint
    int   clawCrossings;    // Number of times claw err has changed sign

    int speedLimit = 96;
};

/* Button -> Replay Bitfield mapping
 * Bit | Function
 *  0  | Arm Up (6U)
 *  1  | Arm Down (6D)
 *  2  | Claw Open (5U)
 *  3  | Claw Closed (5D)
 *  4  | <reserved>
 *  5  | <reserved>
 *  6  | <reserved>
 *  7  | <reserved>
 */
const float clawKp      = 45;       // Arm porportionality control constant
const int clawCtrlSpeed = 32;       // Rate to add to claw target every loop iteration (in potentiometer units)
const int clawOpenPos   = 1024;     // Potentiometer output when claw open
const int clawClosedPos = 0;        // Potentiometer output when claw open
const bool enableClawTBH = false;   // Enable enhanced claw control?

void clawControl(control_t* state) {
    /* Adjust setpoint based on control input: */
    if(state->clawOpen) {
        state->clawSP += clawCtrlSpeed;
        state->clawSP = (state->clawSP > clawOpenPos) ? clawOpenPos : state->clawSP;
        state->clawCrossings = 0;

        motor[clawL] = -127;
        motor[clawR] = 127;
    } else if(state->clawClosed) {
        state->clawSP -= clawCtrlSpeed;
        state->clawSP = (state->clawSP < clawClosedPos) ? clawClosedPos : state->clawSP;
        state->clawCrossings = 0;

        motor[clawL] = 127;
        motor[clawR] = -127;
    } else {
        /* Steady-state control: */
        if(enableClawTBH) {
            state->clawErr = (state->clawErr - SensorValue[pot]);
            if(sgn(state->clawErr) != sgn(state->clawLErr)) {
                /* Sign change detected: */
                state->clawCrossings++;
            }

            float out = (clawKp*state->clawErr);
            if(state->clawCrossings > 0)
                out /= (float)pow(2.0, state->clawCrossings);

            motor[clawL] = -out;
            motor[clawR] = out;

            state->clawLErr = state->clawErr;
        } else {
            motor[clawL] = motor[clawR] = 0;
        }
    }
}

void driveControl(control_t* state) {
    motor[leftDrivea] = motor[leftDriveb] = ( (abs(state->left) > state->speedLimit) ? sgn(state->left) * state->speedLimit : state->left );
    motor[rightDrivea] = motor[rightDriveb] = ( (abs(state->right) > state->speedLimit) ? sgn(state->right) * state->speedLimit : state->right );
}

void armControl(control_t* state) {
    if(state->armUp) {
        motor[ArmRb] = -127;
        motor[ArmRt] = 127;
        motor[ArmLt] = -127;
        motor[ArmLa] = 127;
    } else if(state->armDown) {
        motor[ArmRb] = 127;
        motor[ArmRt] = -127;
        motor[ArmLt] = 127;
        motor[ArmLa] = -127;
    } else {
        motor[ArmRb] = 0;
        motor[ArmRt] = 0;
        motor[ArmLt] = 0;
        motor[ArmLa] = 0;
    }
}

void joystickToControl(control_t* state) {
    state->left = vexRT[Ch3];
    state->right = vexRT[Ch2];

    state->armUp = (bool)vexRT[Btn6U];
    state->armDown = (bool)vexRT[Btn6D];

    state->clawOpen = (bool)vexRT[Btn5U];
    state->clawClose = (bool)vexRT[Btn5D];
}

void replayToControl(control_t* state, replay_t* replay) {

}

#endif /* end of include guard: WARSPITE_C */
