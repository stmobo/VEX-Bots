#ifndef __ISLAND_WIND__
#define __ISLAND_WIND__

/*
 * IslandWind.c -- PID control code.
 * Experimental, requires lots of fine tuning.
 */

struct pidControlState {
	/* Controller parameters */
	float kp;
	float ti;
	float td;

	/* Loop parameter */
	float setpoint;
	float output;

	/* Internal state */
	float accError;
	float errorDeriv;
	float lastError;
	int lastTime;
	int runtime;

	/* Termination criteria */
	float derivThreshold;
	float outpThreshold;
	int watchdogTimer;
};

const int PID_NO_TUNE = 0;
const int PID_ZIEGLER_NICHOLS = 1;
const int PID_PESSEN_RULE = 2;
const int PID_SOME_OVERSHOOT = 3;
const int PID_NO_OVERSHOOT = 4;

void pidInitController(pidControlState* controller, float ku, float tu, int tuningMethod) {
	controller->output = 0;
	controller->accError = 0;
	controller->lastError = 0;
	controller->errorDeriv = 0;
	controller->lastTime = 0;
	controller->runtime = 0;
	controller->setpoint = 0;	
	
	switch(tuningMethod) {
		case 1: /* Ziegler-Nichols */
			controller->kp = (0.6 * ku);
			controller->ti = (0.5 * tu);
			controller->td = (0.25 * tu);
			break;
		case 2: /* Pessen Integral Rule */
			controller->kp = (0.7 * ku);
			controller->ti = (0.4 * tu);
			controller->td = (0.15 * tu);
			break;
		case 3: /* "Some Overshoot" */
			controller->kp = (0.33 * ku);
			controller->ti = (0.5 * tu);
			controller->td = (0.33 * tu);
			break;
		case 4: /* "No Overshoot" */
			controller->kp = (0.2 * ku);
			controller->ti = (0.5 * tu);
			controller->td = (0.33 * tu);
			break;
		default: break;
	}
}

void pidBeginLoop(pidControlState* controller, float setpoint) {
	controller->setpoint = setpoint;

	controller->output = 0;
	controller->accError = 0;
	controller->lastError = 0;
	controller->errorDeriv = 0;
	controller->lastTime = 0;
	controller->runtime = 0;
}

/* u = Kp(e + (1/Ti)S(e dt) + Td(de/dt)) */
float pidControlCycle(pidControlState* controller, float sensorIn) {
	float error = controller->setpoint - sensorIn;

	/* Calculate and update times: */
	int deltaT = nPgmTime - controller->lastTime;
	controller->lastTime = nPgmTime;
	controller->runtime += deltaT;
	
	/* Integral controller: */
	controller->accError += (error * deltaT);
	
	/* Derivative controller: */
	float derivOut = (error - controller->lastError) / ((float)deltaT);
	controller->errorDeriv = derivOut;
	controller->lastError = error;
	
	float out = error + (controller->accError / controller->ti) + (controller->td * derivOut);
	out *= controller->kp;
	
	controller->output = out;
	
	return out;
}

bool pidCheckLoopTerminated(pidControlState* controller) {
	if(abs(controller->output) < controller->outpThreshold &&
		 abs(controller->errorDeriv) < controller->derivThreshold)
		{
		 	return true;
	}
	
	if(controller->runtime >= controller->watchdogTimer) {
		return true;
	}

	return false;
}

/* End of IslandWind.c */
#endif