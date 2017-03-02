
task main()
{
TARGET = <a number>
Kp = <another number>
maxOut = <a number that represents power>

while(true){
	error = TARGET - Sensor;
	output = Kp * error;

	if(abs(error) < maxOut){
		motor[arm] = output;
	}
	else{
		motor[arm] = output/abs(output) * maxOut


}
