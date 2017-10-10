
void setupDefault(void);
void moveSpeed (int motorSpeed);

task main()
{
	setupDefault();

	int encoderStart = nMotorEncoder[motorA];
	int encoderEnd = 0;


	moveSpeed(100);





	wait1Msec(2000);
	moveSpeed(0);		//Double check this does a hard break
	encoderEnd = nMotorEncoder[motorA];
	// Maybe actually do a calculation to return speed here? - or otherwise return encoder ticks per second
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
	//TODO: check if the speed can be a negative percentage, or if I need to set the direction to reverse, and if motors stay synched
	motor[motorA] = motorSpeed;
	return;
}
