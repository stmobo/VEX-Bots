#include "./RedCastle.c"
/* Skills Challenge control stub. */

void pre_auton()
{
	loadReplayFromFile(skillsFilename, &replay);
}

task autonomous() {
	initState();
	while(replay.streamIndex < replay.streamSize) {
		replayToControlState();
		controlLoopIteration();
		sleep((int)deltaT);
	}
	
	stopAllMotors();
}

task usercontrol()
{
	resetState();
	
	while (true)
	{
		controllerToControlState();
		controlLoopIteration();
		sleep((int)deltaT);
	}
}