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
//TODO: check about int vs long - might need to specify stuff as long and multiply everything by 10^6 so I have a constant division for everything
// Could probably do everything as floating point and if it becomes a problem do integer arithmetic?


// May need to run different tasks, depends if necessary or not - e.g. task that just reads sensor values - also can probably reduce memory size by removing a lot of uneeded variables and simplifying expressions - at the moment this emphasizes readability
task main()
{
	// Run Setup Script
	setupDefault();

	// Variable Declarations (multiplied to use integer division) (Does long use more computation time? - all values currently multipled by 10^6) - SHOULD I MAYBE JUST MAKE EVERYTHING FLOAT CONSIDERING I'M NOT ACTUALLY DOING OPERATIONS FREQUENTLY? - ALSO MAYBE MULTIPLY EVERYTHING BY THE SAME AMOUNT
	// Try with float first, see if it makes a difference, otherwise use long
	float massRobot = 0.543100;		// Mass in milligrams (x10^6, use kg)
	float massWheel = 0.035830;			// Mass in milligrams (x10^6, use kg)
	float inertiaWheel = 7.022700;	// Intertia in kg.m^2 (x10^6, use kg.m^2)
	float massCentreDist = 0.080000;	// Distance from wheel to centre of mass in um (x10^6, use m)
	float wheelRadius = 0.028000;		// Radius of the wheel in um (x10^6, use m)

	// TODO: - gyro gives a velocity, so really just need to multiply the velocity by the time sample, may not actually need the previous sample - same maybe with motor encoders?
	// For displacement, angle = previous angle + velocity * time

	float angleVelocityPrevious = 0;			// TODO: - may need to make long - MAY NOT NEED?
	float angleVelocityCurrent = 0;			// TODO: - may need to make long

	float angleDisplacementPrevious = 0;	//MAY NOT BE NEEDED
	float angleDisplacementCurrent = 0;

	int encoderValuePrevious = 0;
	int encoderValueCurrent = 0;	
	int motorAngle = 0;				// TODO: - may need to make long. Motor angle will be multiplied by 100 to remove floating point precision - MAY NOT BE NEEDED
	float linearDisplacementPrevious = 0;
	float linearDisplacementCurrent = 0;

	float linearVelocityPrevious = 0;		//MAY NOT BE NEEDED
	float linearVelocityCurrent = 0;


	float motorTorque = 0;
	int pidPercentage = 0;


	float control1 = 0;
	float control2 = 0;

	// Run Main Loop
	while(1){

		// Read sensor values 
		encoderValuePrevious = encoderValueCurrent;
		encoderValueCurrent = nMotorEncoder[motorA];

		// Compute state variables
		angleVelocityCurrent = sensorValue(GYRO) + GYOROFFSET;
		angleDisplacementCurrent = angleDisplacementCurrent + ((angleVelocityCurrent + angleVelocityPrevious)/2) * CONTROLLERFREQUENCY;			// TODO: - may need to do integer mathematics here depending on if this derivative is a fraction with small number
																													// TODO: - is it more accurate to go current velocity - previous velocity = displacement? - or how do discrete integrals work again?
		encoderValuePrevious = encoderValueCurrent;
		encoderValueCurrent = encoderValue();
		linearDisplacementPrevious = linearDisplacementCurrent;
		linearDisplacementCurrent = linearDisplacementPrevious + ((encoderValueCurrent - encoderValuePrevious) * (PI / 180.0)) * wheelRadius;	// TODO: - pretty sure encoder is an accumilator, may need to double check

		linearVelocityPrevious = linearVelocityCurrent;
		linearVelocityCurrent = (linearDisplacementCurrent - linearDisplacementPrevious) * CONTROLLERFREQUENCY; 

		// Estimate gains & compute control rule (theta, theta_dot, x, x_dot)
		control1 = (0.0083 * angleDisplacementCurrent + 1.7460 * angleVelocityCurrent + 0.0001 * linearDisplacementCurrent + 0.0329 * linearVelocityCurrent) * 100000;
		control2 = (0.0001 * angleDisplacementCurrent + 0.0466 * angleVelocityCurrent + 0.0083 * linearDisplacementCurrent + 1.7135 * linearVelocityCurrent) * 100000;

		motorTorque = control1 + control2;

		// Compute velocity command
		pidPercentage = torqueToSpeed(motorTorque);
		moveSpeed(pidPercentage);
	}
}


