
task main()
{
	SetSensorType(S1, sensorSONAR);

	//ClearSensorValue(S1);
	float test = 0;


	while(1){
		test = SensorValue(S1);
		nxtDisplayTextLine(5, " MotorA: %05.2f ", test);
		wait1Msec(10);
	}
}
