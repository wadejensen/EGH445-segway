
task main()
{
	SetSensorType(S1, sensorSONAR);

	//ClearSensorValue(S1);


	while(1){
		nxtDisplayTextLine(5, " MotorA: %05.2f ", SensorValue(S1));
		wait1Msec(10);
	}
}
