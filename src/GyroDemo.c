task main()
{
	//SetSensorType(S4, sensorRotation);

	//ClearSensorValue(S1);


	while(1){
		nxtDisplayTextLine(5, " GyroRaw: %05.2f ", (SensorValue(S4)-596));
		wait1Msec(10);
	}
}
