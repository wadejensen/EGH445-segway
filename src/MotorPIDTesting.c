
void setupDefault(void);
void moveSpeed (int motorSpeed);

task main()
{
	setupDefault();
	moveSpeed(-50);
	wait1Msec(1000);

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
