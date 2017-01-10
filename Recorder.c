#include "./RedCastle.c"
/* Recorder control stub. */

void pre_auton() {}

task autonomous() {
	replay.streamIndex = 0;
	initState();

	while(replay.streamIndex < replay.streamSize) {
		replayToControlState();
		controlLoopIteration();
		sleep((int)deltaT);
	}
	
	stopAllMotors();
}

unsigned int currentTime = 0;

/* Max recording time in milliseconds.
 *
 *  For a regular match autonomous recording, this should be 15000 milliseconds (= 15 seconds).
 *  For an Auto Skills recording, this should be 120000 milliseconds (= 120 seconds = 2 minutes).
 */
unsigned int timelimit = 120000;

task usercontrol()
{
	initState();
	clearReplay();
	
	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDCenteredString(0, "Ready to record.");
	displayLCDCenteredString(1, "Press any button or move stick.");
	
	while (true)
	{
		if(abs(vexRT[Ch2]) > deadband || abs(vexRT[Ch1]) > deadband) {
			break;
		}
		
		if(vexRT[Btn6D] || vexRT[Btn6U] || vexRT[Btn7U] ||
		   vexRT[Btn5U] || vexRT[Btn5D] || vexRT[Btn8U] || vexRT[Btn8D])
		{
			break;
		}
		
		sleep(5);
	}
	
	while (true)
	{
		controllerToControlState();
		controlLoopIteration();
		
		controlStateToReplay();
		
		currentTime += (int)deltaT;
		
		if(currentTime > timelimit) {
			break;
		}
		
		sleep((int)deltaT);
	}
	
	replay.streamSize = replay.streamIndex+1;
	
	stopAllMotors();
	querySaveReplay(&replay);
}