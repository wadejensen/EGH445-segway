task main()
{
	//SetSensorType(S4, sensorRotation);

	//ClearSensorValue(S1);

	float angleVelocityPrevious = 0;
	float angleVelocityCurrent = 0;

	float angleDisplacementPrevious = 0;	//MAY NOT BE NEEDED
	float angleDisplacementCurrent = 0;
	int gyromax = 0;

	while(1){
		angleVelocityCurrent = SensorValue(S4) - 596;
		angleDisplacementCurrent = angleDisplacementCurrent + (angleVelocityCurrent) * 0.010;

		if (((SensorValue(S4)-595) > (gyromax-595)) || ((SensorValue(S4)-595) < (-gyromax-595))){
			gyromax = SensorValue(S4)-595;
		}


		nxtDisplayTextLine(4, " GyroMax: %05.2f ", gyromax);
		nxtDisplayTextLine(5, " GyroRaw: %05.2f ", (SensorValue(S4)-596));
		nxtDisplayTextLine(6, " Angle: %05.2f ", (angleDisplacementCurrent));
		wait1Msec(10);
	}
}
