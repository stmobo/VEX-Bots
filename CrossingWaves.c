#ifndef __CROSSING_WAVES__
#define __CROSSING_WAVES__

/* CrossingWaves.c -- Take-Back-Half control code.
 * Stable, though may be suboptimal.
 */

struct tbhData {
	/* Internal variables: */
	float err;		/* Control loop error. */
	int lastSign;		/* Sign of err on last cycle. */
	float lOutput;		/* Output value from last setpoint crossing. */
	bool firstCross;	/* Will be set to FALSE after the first setpoint crossing. */

	/* Input parameters: */
	float setpoint;
	float gain;
	float maxOutput;	/* this->output will be capped to +/-(this->maxOutput). */

	float output;		/* Control output. (e.g. motor power) */

	/* Output event flags. Clear these after handling their event. */
	bool crossed;		/* Will be set to TRUE on a setpoint crossing. */
	bool active;		/* Will be set to TRUE on every tbhControlCycle(). */
};


void resetTBHData(tbhData* st, float setpoint) {
	st->setpoint = setpoint;
	st->err = 0;
	st->output = 0;
	st->lOutput = 0;
	st->lastSign = 0;
	st->firstCross = true;
	st->crossed = false;
	st->active = false;
}

/*
 * TBH Control Loop: (for S = setpoint)
 *  o V <- sensor
 *  o E <- (S - V)
 *  If V has crossed S: Out <- ((E*gain) + (lastOut)) / 2
 *  Else: o Out <- E * gain
 */

 /*
 * How to use TBHData:
 *  - Call resetTBHData(&your_tbh_struct, setpoint);
 *  - Ensure your gain and maxOutput variables are set (set all input parameters)
 *  - In a loop:
 *   o Call tbhControlCycle(&your_tbh_struct, sensor_input). This does the important control loop calculations, etc.
 *   o Handle raised events as necessary.
 *   o Do everything else. For example, actually using the output value(!) or checking exit conditions(!).
 *  - When done, make sure to set your_tbh_struct.active to FALSE for the benefit of any listeners.
 */
 
void tbhControlCycle(tbhData *st, float sensorIn) {
	float err = (st->setpoint - sensorIn);

	st->err = err;
	st->active = true;

	float curOut = (err * st->gain);
	if(sgn((int)err) != 0 && sgn((int)err) != st->lastSign) {
		/* Sign change */
		st->lastSign = sgn((int)err);
		st->lOutput = st->output = (st->firstCross ? curOut : ((curOut+st->lOutput) / 2) );
		st->firstCross = false;
		st->crossed = true;
	} else {
		st->output = curOut;
	}

	if(abs(st->output) > st->maxOutput) {
		st->output = sgn((int)st->output)*st->maxOutput;
	}
}

/* End of CrossingWaves.c */
#endif
