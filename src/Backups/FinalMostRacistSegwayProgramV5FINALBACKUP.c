#pragma config(Sensor, S1,     HTGYRO,              sensorAnalogInactive)
/*
 *  ======== MainCode.c ========
 * EGH445 2017
 * Author: Gerard Rallos
 */

/* ============== Global Declarations & Defines ============== */
#define  CONTROLLERFREQUENCY 290
#define  SAMPLEPERIOD 0.003448
// About 700-710 ticks per second at 100%
#define  MAXSPEED 700
#define GYROFFSET 595

#define RANGEMAX 25.0
#define RANGEMAXABS 40.0
#define RANGEMIN 0.0
#define RANGEZEROHIGH 19.0
#define RANGEZEROLOW 16.0

#define nBar -0.0				// Try -0.1 normally
#define rangeScale 0.6	//-0.6 normally, 0 for testing

#define  SONAR S1
#define  GYRO S4

/* ============== Includes ============== */
#include "drivers/hitechnic-gyro.h"


/* ============== Function Prototypes ============== */
void setupDefault (void);
void moveSpeed (int motorSpeed);
int encoderValue(void);
float torqueToSpeed (float torque, float inertia);


/* ============== Good K gains (DEBUG) ============== */
 	//   -1.2627   -0.0728   -0.3162   -0.3042 --> 1, 0.001, 0.1, 0.001
 // 	 -6.3502 	 -1.1975 	 -1.0000   -1.6101 --> motor torque 0.1 multiuplier
 //		 -0.9409   -0.0532   -0.2236   -0.2164 --> 0.5, 0.0005, 0.05, 0.0005 --> best, 1/5 of this weighting is smooth but unstable
 //		 -1.0185   -0.0696   -0.5477   -0.3533 --> Drifts less, but doesn't quite seem to be as stable
 //		 -1.1573   -0.0495   -0.2280   -0.2449 --> New best values, very stable and lower drift
 //		 -1.1573   -0.0495   -0.4280   -0.2449 --> New best values, very stable and lower drift

/*
 *  ======== Main Code Loop ========
 */

task main()
{
	//======== Gyroscope Calibration ========
	nxtDisplayTextLine(2, "Offset: %f", HTGYROstartCal(HTGYRO));			//TODO: maybe fix this calibration later - add the actual offset value
	nxtDisplayTextLine(2, "CALIBRATION FINISHED");
 	wait1Msec(2500);

	//======== Main Code  ========
	// Run Setup Script
	setupDefault();

	// Variable Declarations
	float inertiaWheel = 0.000014045;		// Intertia in kg.m^2 (x10^6, use kg.m^2)
	float wheelRadius = 0.028000;					// Radius of the wheel in um (x10^6, use m)
	float angleVelocityPrevious = 0;
	float angleVelocityCurrent = 0;

	float angleDisplacementPrevious = 0;	//MAY NOT BE NEEDED
	float angleDisplacementCurrent = 0;

	int encoderValuePrevious = 0;
	int encoderValueCurrent = 0;

	float linearDisplacementPrevious = 0;
	float linearDisplacementCurrent = 0;

	float linearVelocityPrevious = 0;		//MAY NOT BE NEEDED
	float linearVelocityCurrent = 0;

	float motorTorque = 0;
	int pidPercentage = 0;

	//Extra Feature: Range sensor
	float rangeValue = 0;
	float overshootCorrection = 0;
	int checkSkip = 0;

	//DEBUG:
	//int timerloop = 0;
	//clearTimer(T1);
	//timerloop = time1[T1];

	// Run Main Loop
	while(1){
		//DEBUG:

		// Reset Loops
		checkSkip = 0;

		// Compute state variables
		angleVelocityCurrent = SensorValue(S4) - GYROFFSET;
		angleDisplacementCurrent = angleDisplacementCurrent + (angleVelocityCurrent) * SAMPLEPERIOD;

		encoderValuePrevious = encoderValueCurrent;
		encoderValueCurrent = encoderValue();
		linearDisplacementPrevious = linearDisplacementCurrent;
		linearDisplacementCurrent = linearDisplacementPrevious + ((encoderValueCurrent - encoderValuePrevious) * (PI / 180.0)) * wheelRadius;	// TODO: - pretty sure encoder is an accumilator, may need to double check

		linearVelocityPrevious = linearVelocityCurrent;
		linearVelocityCurrent = (linearDisplacementCurrent - linearDisplacementPrevious) * CONTROLLERFREQUENCY;

		// Extra Feature - Range sensor offset
		rangeValue = SensorValue(S1);															// Range reading in cm;
		if((rangeValue <= RANGEMIN) || (rangeValue >= RANGEMAXABS)) {
			rangeValue = RANGEMIN;
		}
		else if((rangeValue >= RANGEMAX) && (rangeValue < RANGEMAXABS)){
			rangeValue = 0.4/rangeValue;
			checkSkip = 1;
		}
		else if((rangeValue >= RANGEZEROLOW) && (rangeValue <= RANGEZEROHIGH)){
			rangeValue = 0;
			checkSkip = 1;
	  }

		// Determine if robot should move backwards or forwards to track object
		if((rangeValue > RANGEZEROHIGH) && (rangeValue <= RANGEMAX)){
			rangeValue = (rangeValue - RANGEZEROHIGH) / 100;											// Move forward reference in meters
		}
		else if ((rangeValue < RANGEZEROLOW) && (rangeValue > RANGEMIN) && (checkSkip == 0)){
			rangeValue = (rangeValue - RANGEZEROLOW) / 100;												// Move backward reference in meters
		}

		// Estimate gains & compute control rule K * s
		motorTorque = (angleDisplacementCurrent * (PI/180) * -1.1573) + (angleVelocityCurrent * (PI/180) *  -0.0495) + (linearDisplacementCurrent * -0.4280) + (linearVelocityCurrent * -0.2449);
		motorTorque = -motorTorque + (rangeScale * -rangeValue) + (linearDisplacementCurrent * nBar);



		// Compute velocity command
		pidPercentage = torqueToSpeed(motorTorque, inertiaWheel);		//Motor torque should make robot move in the same direction as it is tilting
		moveSpeed(pidPercentage);

		//DEBUG: Display State Variables
		 //nxtDisplayTextLine(2, " TimeLoop: %d ", (timerloop));
		 //nxtDisplayTextLine(1, " PID: %05.2f ", pidPercentage);
		 //nxtDisplayTextLine(2, " Torque: %05.2f ", motorTorque);
		 //nxtDisplayTextLine(3, " Theta: %05.2f ", angleDisplacementCurrent);
		 //nxtDisplayTextLine(4, " Thetadot: %05.2f ", angleVelocityCurrent);
		 //nxtDisplayTextLine(5, " X: %05.2f ", linearDisplacementCurrent);
		 //nxtDisplayTextLine(6, " Xdot: %05.2f ", linearVelocityCurrent);
		 //nxtDisplayTextLine(7, " Range: %04.3f ", rangeValue);

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
