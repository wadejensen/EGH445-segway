
void setupDefault(void);
void moveSpeed (int motorSpeed);

task main()
{
	setupDefault();

	int encoderStart = nMotorEncoder[motorA];
	int encoderEnd = 0;


	moveSpeed(100);
	wait1Msec(1000);
	encoderEnd = nMotorEncoder[motorA];
	moveSpeed(0);		//Double check this does a hard break
	
	nxtDisplayTextLine(3, " MotorAStart: %05.2f ", encoderStart);
	nxtDisplayTextLine(6, " MotorAEnd: %05.2f ", enencoderEnd);
	wait1Msec(5000);

}



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

}

void moveSpeed (int motorSpeed){
	motor[motorA] = motorSpeed;
	return;
}
