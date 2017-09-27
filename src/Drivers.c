// Driver functions for the NXT Mindstorm robot
// Note that all values are assumed motors A and B are connected, and A is the master motor


// Reset all values to desired defaults
void setupDefault (void) {
	// Reset encoders to default
	nMotorEncoder[motorA] = 0;
	nMotorEncoder[motorB] = 0;
	nMotorEncoder[motorC] = 0;

	// Synchronize motors
	nSyncedMotors = synchAB;

	//Turn PID speed control on
	nMotorPIDSpeedCtrl[motorA] = mtrSyncRegMaster;
	nMotorPIDSpeedCtrl[motorB] = mtrSyncRegSlave;
	nMotorPIDSpeedCtrl[motorC] = mtrSyncRegSlave;

	//Turn Sensors on (if needed)
	SetSensorType(S1, sensorSONAR);

}

// Converts a given torque to a desired speed
	// toque - given torque (in N.m)
int torqueToSpeed (int torque){


}


// Moves the motors by a specified regulated PID speed
	// motorSpeed - Percentage of the speed (0-100)
void moveSpeed (int motorSpeed){
	motor[motorA] = motorSpeed;
	return;
}


// Moves the motors to a specified angle
	// degreeMove - what angle to move to, in degrees
	// motorSpeed - what speed to set the motor (positive speeds only)
void moveToAngle (int degreeMove, int motorSpeed) {
	if (nMotorEncoder[motorA]<degreeMove){
		while(nMotorEncoder[motorA]<degreeMove){
			motor[motorA] = motorPower;
		}
	}
	else{
		while(nMotorEncoder[motorA]>degreeMove){
			motor[motorA] = motorPower * -1;
		}		
	}
	return;
}


// Moves the motors relative to the current angle
	// degreeMove - what relative angle to move
	// motorSpeed - what speed to set the motor (positive speeds only)
void moveToRelative (int relativeMove, int motorSpeed) {
	int desiredAngle = nMotorEncoder[motorA] + relativeMove;
	moveToAngle(desiredAngle, motorPower);
	return;
}



//-----------------------Sensors-----------------------

// Print the encoder value of the motors
int encoderValue(void){
	 nxtDisplayTextLine(5, " MotorA: %05.2f ", nMotorEncoder[motorA]);
	return nMotorEncoder[motorA];
}


// Returns the value of the specific sensor
int sensorValue(string sensorNumber){
	return SensorValue(sensorNumber);
}
