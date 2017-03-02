/*
 * OMGWTFBBQ.c: or, the Most Configurable Robot Program in the World!
 * Intended for use by teams who cannot program or build robots on time.
 *
 * Implements:
 *  - Tank drive w/ speedlimit
 *  - 3 configurable togglable attachments
 *  - 3 configurable continuously-driven (forward/back) attachments
 *  - Simple (drive forward for 5 seconds) autonomous
 *
 * ...all in a rapidly-reconfigurable software package!
 * No need to actually write code, just plug in the port and button numbers,
 * and you're set!
 */

/* Drive config. */
const signed short driveLeft[4]  = {0, 0, 0, 0}; // Motor ports
const signed short driveRight[4] = {0, 0, 0, 0}; // Motor ports
const short speedLimit = 96;

/* Configurable continuous-drive apparatus. */
const signed short attachmentAlpha[2] = {0, 0}; // Motor ports
const short controlsAlpha[2] = {0, 0};          // Joystick buttons (fwd / back)
const signed short fwdSpeedAlpha = 127;
const signed short bwdSpeedAlpha = -127;

const signed short attachmentBeta[2] = {0, 0};
const short controlsBeta[2] = {0, 0};
const signed short fwdSpeedBeta = 127;
const signed short bwdSpeedBeta = -127;

const signed short attachmentGamma[2] = {0, 0};
const short controlsGamma[2] = {0, 0};
const signed short fwdSpeedGamma = 127;
const signed short bwdSpeedGamma = -127;

/* Configurable togglable apparatus. */
const signed short attachmentEins = {0, 0}; // Motor ports
const short toggleEins = 0;                 // Joystick button
const signed short fwdSpeedEins = 127;

const signed short attachmentZwei = {0, 0}; // Motor ports
const short toggleZwei = 0;                 // Joystick button
const signed short fwdSpeedZwei = 127;

const signed short attachmentDrei = {0, 0}; // Motor ports
const short toggleDrei = 0;                 // Joystick button
const signed short fwdSpeedDrei = 127;

// don't touch anything below this line:
/******************************************************************************/

// bookkeeping variables
bool stateEins = 0;
bool stateZwei = 0;
bool stateDrei = 0;

bool readJoystickButton(const short button) {
    if(button == 0) {
        return false;
    }

    return vexRT[button];
}

void setMotorGroup(const signed short* motorGroup, const int nMotors, signed short value) {
    for(int i=0;i<nMotors;i++) {
        if(motorGroup[i] != 0) {
            motors[abs(motorGroup[i])] = (motorGroup[i] > 0) ? value : -value;
        }
    }
}

void continuousControl(const signed short* motors, const short* controls, const signed short fwd, const signed short bwd) {
    if(readButton(controls[0])) {
        setMotorGroup(motors, 2, fwd);
    } else if(readButton(controls[1])) {
        setMotorGroup(motors, 2, bwd);
    } else {
        setMotorGroup(motors, 2, 0);
    }
}

void toggleControl(const signed short* motors, const short control, bool& state, const signed short fwd) {
    state = readButton(control) ? !state : state;

    if(state) {
        setMotorGroup(motors, fwd);
    } else {
        setMotorGroup(motors, 0);
    }
}

void driveControl() {
    signed short left = (abs(vexRT[Ch3]) > speedLimit) ? sgn(vexRT[Ch3])*speedLimit : vexRT[Ch3];
    signed short right = (abs(vexRT[Ch2]) > speedLimit) ? sgn(vexRT[Ch2])*speedLimit : vexRT[Ch2];

    setMotorGroup((const signed short*)driveLeft, 4, left);
    setMotorGroup((const signed short*)driveRight, 4, right);
}

task pre_auton() {}

task autonomous() {
        setMotorGroup((const signed short*)driveLeft, 4, 127);
        setMotorGroup((const signed short*)driveRight, 4, 127);
        sleep(5000);
        setMotorGroup((const signed short*)driveLeft, 4, 0);
        setMotorGroup((const signed short*)driveRight, 4, 0);
}

task usercontrol() {
    while(true) {
        driveControl();

        continuousControl((const signed short*)attachmentAlpha, (const short*)controlsAlpha, fwdSpeedAlpha, bwdSpeedAlpha);
        continuousControl((const signed short*)attachmentBeta, (const short*)controlsBeta, fwdSpeedBeta, bwdSpeedBeta);
        continuousControl((const signed short*)attachmentGamma, (const short*)controlsGamma, fwdSpeedGamma, bwdSpeedGamma);

        toggleControl((const signed short*)attachmentEins, toggleEins, fwdSpeedEins);
        toggleControl((const signed short*)attachmentZwei, toggleZwei, fwdSpeedZwei);
        toggleControl((const signed short*)attachmentDrei, toggleDrei, fwdSpeedDrei);

        sleep(20);
    }
}
