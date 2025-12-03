#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include <vector>

#include "api.h"
#include "hskylib.h"

// TODO: Clean up and document
struct AutonConstants {
	double wheelDiameterIn;
	double trackWidthIn;
	double ticksPerRev;	 // encoder ticks per revolution

	double kPDrive;
	double kIDrive;
	double kDDrive;

	double kPTurn;
	double kITurn;
	double kDTurn;

	double driveToleranceIn;
	double turnToleranceDeg;

	double maxDriveVelocityDutyCycle;  // velocity units
	double maxTurnVelocityDutyCycle;   // velocity units
};



extern pros::Controller ctrl;
extern TankDrive tankdrive;
extern Transport intake;
extern Transport scoring;
extern Pneumatics mogoClamp;

void robotInit();

#endif