/*
 *  ======== MainCode.c ========
 * EGH445 2017
 * Author: Gerard Rallos
 */

 // Global Declarations & Defines
#define  CONTROLLERFREQUENCY 290
#define  SAMPLEPERIOD 0.003448
// About 700-710 ticks per second at 100%
#define  MAXSPEED 700
#define GYROFFSET 595

#define  SONAR S1
#define  GYRO S4


/* ============== Includes ============== */
void setupDefault (void);
void moveSpeed (int motorSpeed);
int encoderValue(void);
float torqueToSpeed (float torque, float inertia);

/*
 *  ======== Main Code Loop ========
 */
//TODO: check about int vs long - might need to specify stuff as long and multiply everything by 10^6 so I have a constant division for everything
// Could probably do everything as floating point and if it becomes a problem do integer arithmetic?

 // GOOD K GAINS:
 	//   -1.2627   -0.0728   -0.3162   -0.3042 --> 1, 0.001, 0.1, 0.001
 // 	 -6.3502 	 -1.1975 	 -1.0000   -1.6101 --> motor torque 0.1 multiuplier
 //		 -0.9409   -0.0532   -0.2236   -0.2164 --> 0.5, 0.0005, 0.05, 0.0005 --> best, 1/5 of this weighting is smooth but unstable


// May need to run different tasks, depends if necessary or not - e.g. task that just reads sensor values - also can probably reduce memory size by removing a lot of uneeded variables and simplifying expressions - at the moment this emphasizes readability
task main()
{
	// Run Setup Script
	setupDefault();

	// Variable Declarations
	float inertiaWheel = 0.000014045;		// Intertia in kg.m^2 (x10^6, use kg.m^2)
	float wheelRadius = 0.028000;					// Radius of the wheel in um (x10^6, use m)

	// TODO: - gyro gives a velocity, so really just need to multiply the velocity by the time sample, may not actually need the previous sample - same maybe with motor encoders?
	// For displacement, angle = previous angle + velocity * time

	float angleVelocityPrevious = 0;
	float angleVelocityCurrent = 0;

	float angleDisplacementPrevious = 0;	//MAY NOT BE NEEDED
	float angleDisplacementCurrent = 0;

	int encoderValuePrevious = 0;
	int encoderValueCurrent = 0;
	int motorAngle = 0;
	float linearDisplacementPrevious = 0;
	float linearDisplacementCurrent = 0;

	float linearVelocityPrevious = 0;		//MAY NOT BE NEEDED
	float linearVelocityCurrent = 0;

	float motorTorque = 0;
	int pidPercentage = 0;

	//float L1 = 0;
	//float L2 = 0;
	//float L3 = 0;
	//float L4 = 0;

	//DEBUG:
	//int timerloop = 0;
	//clearTimer(T1);
	//timerloop = time1[T1];

	// Run Main Loop
	while(1){
		//DEBUG:


		// Compute state variables - TODO: theta is accumilating - due to drift - should I maybe average a number of samples to get a more accurate reading?
		// Are we running into the maximum gyroscope reading on either side - range seems to be between 0 and 975, centred at 565 - lower range appears larger than higher range?
		angleVelocityCurrent = SensorValue(S4) - GYROFFSET;
		angleDisplacementCurrent = angleDisplacementCurrent + (angleVelocityCurrent) * SAMPLEPERIOD;			// TODO: - may need to do integer mathematics here depending on if this derivative is a fraction with small number
																													// TODO: - is it more accurate to go current velocity - previous velocity = displacement? - or how do discrete integrals work again?

		encoderValuePrevious = encoderValueCurrent;
		encoderValueCurrent = encoderValue();
		linearDisplacementPrevious = linearDisplacementCurrent;
		linearDisplacementCurrent = linearDisplacementPrevious + ((encoderValueCurrent - encoderValuePrevious) * (PI / 180.0)) * wheelRadius;	// TODO: - pretty sure encoder is an accumilator, may need to double check

		linearVelocityPrevious = linearVelocityCurrent;
		linearVelocityCurrent = (linearDisplacementCurrent - linearDisplacementPrevious) * CONTROLLERFREQUENCY;

		// Estimate gains & compute control rule (L x s) . K
		//control1 = (0.0083 * angleDisplacementCurrent + 1.7460 * angleVelocityCurrent + 0.0001 * linearDisplacementCurrent + 0.0329 * linearVelocityCurrent) * 100000;
		//control2 = (0.0001 * angleDisplacementCurrent + 0.0466 * angleVelocityCurrent + 0.0083 * linearDisplacementCurrent + 1.7135 * linearVelocityCurrent) * 100000;
		//L1 = (831.8732 * angleDisplacementCurrent) + (11.2053 * linearDisplacementCurrent);
		//L2 = (176140 * angleDisplacementCurrent) + (4656.2 * linearDisplacementCurrent);
		//L3 = (8.2434 * angleDisplacementCurrent) + (828.1268 * linearDisplacementCurrent);
		//L4 = (3164.1 * angleDisplacementCurrent) + (171350 * linearDisplacementCurrent);

		//motorTorque = (L1 * -12.9644) + (L2 * -1.4068) + (L3 * -3.1623) + (L4 * -3.3708);
		//TODO: Tune gains below for stability and faster settling - overshoot doesn't matter so much
	 // motorTorque = motorTorque * 0.1; //TODO - temp fix
		//motorTorque = (angleDisplacementCurrent * (PI/180) * -1.2627) + (angleVelocityCurrent * (PI/180) * -0.0728) + (linearDisplacementCurrent * -0.3162) + (linearVelocityCurrent * -0.3042);
		motorTorque = (angleDisplacementCurrent * (PI/180) * -0.9409) + (angleVelocityCurrent * (PI/180) *  -0.0532) + (linearDisplacementCurrent * -0.2236) + (linearVelocityCurrent * -0.2164);

		// Compute velocity command
		pidPercentage = torqueToSpeed(-motorTorque, inertiaWheel);		//Motor torque should make robot move in the same direction as it is tilting
		moveSpeed(pidPercentage);


		//DEBUG: Display State Variables
		 //nxtDisplayTextLine(2, " TimeLoop: %d ", (timerloop));
		 nxtDisplayTextLine(1, " PID: %05.2f ", pidPercentage);
		 nxtDisplayTextLine(2, " Torque: %05.2f ", motorTorque);
		 nxtDisplayTextLine(3, " Theta: %05.2f ", angleDisplacementCurrent);
		 nxtDisplayTextLine(4, " Thetadot: %05.2f ", angleVelocityCurrent);
		 nxtDisplayTextLine(5, " X: %05.2f ", linearDisplacementCurrent);
		 nxtDisplayTextLine(6, " Xdot: %05.2f ", linearVelocityCurrent);

		wait1Msec(2);
	}
	//DEBUG:
	//nxtDisplayTextLine(2, " TimeLoop: %d ", (time1[T1] - timerloop));
	//wait1Msec(300000);
}

//-- DRIVERS--//
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
float torqueToSpeed (float torque, float inertia){
	float pidPercentage = 0;
	float velocity = 0;		// IS THIS A LINEAR VELO OR ANGULAR VELO?

	// Convert torque to velocity and scale to a percentage of maximum motor speed
	velocity = (torque/inertia) * SAMPLEPERIOD;						//In rad/s
	velocity = velocity * (180/PI);												//In ticks/s
	pidPercentage = (velocity * 100) / MAXSPEED;

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


//-----------------------Sensors-----------------------

// Return the encoder value of the motors
int encoderValue(void){
	return nMotorEncoder[motorA];
}
