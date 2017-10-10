// Driver functions for the NXT Mindstorm robot
// Note that all values are assumed motors A and B are connected, and A is the master motor

// Global Declarations & Defines
#DEFINE CONTROLLERFREQUENCY 290
#DEFINE MILLTOKG 10000000
#DEFINE G 981
#DEFINE MAXSPEED 0

#DEFINE SONAR S1
#DEFINE GYRO S4


// Function Prototypes
void setupDefault (void);
void moveSpeed (int motorSpeed);

int encoderValue(void);
int sensorValue(string sensorNumber);

float disDerivative (long x1, long x2, int deltaT);
float disIntegral (long x1, long x2, int deltaT);


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

// Computes the discrete derivative of a function using integer mathematics
	// x1 - The value x[n-1]
	// x2 - The value x[n]
	// deltaT - The period T in seconds
float disDerivative (float x1, float x2, int deltaT){
	return ((x2 - x1)/deltaT);
}

// Computes the discrete integral of a function using integer mathematics
	// x1 - The value x[n-1]
	// x2 - The value x[n]
	// deltaT - The period T in seconds
float disIntegral (float x1, float x2, int deltaT){
	return ((x2 - x1)*deltaT);
}


// Converts a given torque to a desired speed
	// toque - given torque (in N.m)
float torqueToSpeed (float torque){
	int pidPercentage = 0;
	float velocity = 0;		// IS THIS A LINEAR VELO OR ANGULAR VELO?


	// Convert torque to velocity and scale to a percentage of maximum motor speed
	velocity = 
	pidPercentage = (velocity * 100) / MAXSPEED

	// Limit motor speed to maximum
		if (pidPercentage > 100) {
			pidPercentage = 100;
		}
		else if (pidPercentage < -100){
			pidPercentage = -100;
		}

	return pidPercentage;
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
