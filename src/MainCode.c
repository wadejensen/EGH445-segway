/*
 *  ======== MainCode.c ========
 * EGH445 2017
 * Author: Gerard Rallos
 */


/* ============== Includes ============== */
#include <Drivers.h>


/*
 *  ======== Main Code Loop ========
 */
//TODO: check about int vs long - might need to specify stuff as long
// Could probably do everything as floating point and if it becomes a problem do integer arithmetic?

task main()
{
	// Run Setup Script
	setupDefault();

	// Variable Declarations (multiplied to use integer division) (Does long use more computation time? - all values currently multipled by 10^6)
	long massRobot = 543100;		// Mass in milligrams (x10^6, use kg)
	long massWheel = 35830;			// Mass in milligrams (x10^6, use kg)
	long inertiaWheel = 7022700;	// Intertia in kg.m^2 (x10^6, use kg.m^2)
	int massCentreDist = 80;		// Distance from wheel to centre of mass in mm (x10^3, use m)
	int wheelRadius = 28;			// Radius of the wheel in mm (x10^3, use m)

	// TODO: - do I need 3 samples for the gyo since I integrate it twice, or just two and do a double multiple?
	int gyoValuePrevious = 0;
	int gyroValueCurrent = 0;
	int angleVelocity = 0;			// TODO: - may need to make long

	int encoderValuePrevious = 0;
	int encoderValueCurrent = 0;	
	int motorAngle = 0;				// TODO: - may need to make long. Motor angle will be multiplied by 100 to remove floating point precision

	long motorTorque = 0;
	int pidPercentage = 0;

	// Run Main Loop
	while(1){

		// Read sensor values 

		encoderValuePrevious = encoderValueCurrent;
		encoderValueCurrent = nMotorEncoder[motorA];


		// Compute state variables

		// Estimate state vector & compute contro, rule

		// Compute velocity command

		moveSpeed(pidPercentage);
	}
}


