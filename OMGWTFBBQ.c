/*
 * OMGWTFBBQ.c -- Or, the Most Configurable Robot Software in the World!
 *
 * Implements:
 *  - Tank drive w/ speedlimit
 *  - 3 configurable togglable attachments
 *  - 3 configurable continuously-driven (forward/back) attachments
 *  - Simple (drive-forward) autonomous
 */

/* Drive config. */
const signed short driveLeft[4]  = {0, 0, 0, 0}; // Motor ports
const signed short driveRight[4] = {0, 0, 0, 0}; // Motor ports
const short speedLimit = 96;

/* Configurable continuous-drive apparatus. */
const signed short attachmentAlpha[2] = {0, 0}; // Motor ports
const short alphaControls[2] = {0, 0};          // Joystick buttons (fwd / back)

const signed short attachmentBeta[2] = {0, 0};
const short betaControls[2] = {0, 0};

const signed short attachmentGamma[2] = {0, 0};
const short gammaControls[2] = {0, 0};

/* Configurable togglable apparatus. */
const signed short attachmentEins = {0, 0}; // Motor ports
const short toggleEins = 0;                 // Joystick button

const signed short attachmentZwei = {0, 0}; // Motor ports
const short toggleZwei = 0;                 // Joystick button

const signed short attachmentDrei = {0, 0}; // Motor ports
const short toggleDrei = 0;                 // Joystick button

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

void continuousControl(const signed short* motors, const short* controls) {
    if(readButton(controls[0])) {
        setMotorGroup(motors, 2, 127);
    } else if(readButton(controls[1])) {
        setMotorGroup(motors, 2, -127);
    } else {
        setMotorGroup(motors, 2, 0);
    }
}

void toggleControl(const signed short* motors, const short control, bool& state) {
    state = readButton(control) ? !state : state;

    if(state) {
        setMotorGroup(motors, 127);
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

        continuousControl((const signed short*)attachmentAlpha, (const short*)alphaControls);
        continuousControl((const signed short*)attachmentBeta, (const short*)betaControls);
        continuousControl((const signed short*)attachmentGamma, (const short*)gammaControls);

        toggleControl((const signed short*)attachmentEins, toggleEins);
        toggleControl((const signed short*)attachmentZwei, toggleZwei);
        toggleControl((const signed short*)attachmentDrei, toggleDrei);

        sleep(20);
    }
}
