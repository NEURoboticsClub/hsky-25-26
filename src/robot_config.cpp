#include "robot_config.hpp"

#include <hskylib/robot_specs.h>
#include <hskylib/subsystems/pneumatics.h>
#include <hskylib/utils/utils.h>

#include "pros/adi.hpp"
#include "pros/motors.h"

#define ROBOT_2

HskyController controller(pros::E_CONTROLLER_MASTER);

//---------------------------------------------------
// ##################### Robot 1 #####################
//---------------------------------------------------

#ifdef ROBOT_1

//===================== CONFIG =====================

robot_specs_t robotConfig{.driveWheelDiam = 0.0,
						  .trackWidth = 0.0,
						  .odomWheelDiam = 0.0,
						  .maxDrivePct = 0,
						  .maxTurnPct = 0,
						  .drivePID = nullptr,
						  .turnPID = nullptr};

//===================== DEVICES =====================

pros::MotorGroup leftDriveMotors({-12, 13, -14, 15});
pros::MotorGroup rightDriveMotors({17, -18, 19, -20});

pros::MotorGroup intakeMotors({6});
pros::MotorGroup lowerScoringMotors({-9});
pros::MotorGroup upperScoringMotors({4});

pros::adi::DigitalOut scraperCylinder('c');
pros::adi::DigitalOut hoodCylinder('a');
pros::adi::DigitalOut wingCylinder('b');

//==================== SUBSYSTEMS ====================
TankDrive driveBase(leftDriveMotors, rightDriveMotors, DriveStyle::ARCADE,

					pros::E_MOTOR_BRAKE_COAST, pros::E_MOTOR_GEAR_600, 1.0,

					robotConfig);

Transport intake(intakeMotors, 0.75, pros::E_MOTOR_BRAKE_COAST,

				 pros::E_MOTOR_GEAR_600);
Transport lowerScoring(lowerScoringMotors, 0.75, pros::E_MOTOR_BRAKE_COAST,

					   pros::E_MOTOR_GEAR_600);
Transport upperScoring(upperScoringMotors, 0.75, pros::E_MOTOR_BRAKE_COAST,

					   pros::E_MOTOR_GEAR_600);

Pneumatics scraper(scraperCylinder);
Pneumatics hood(hoodCylinder);
Pneumatics wing(wingCylinder);

//---------------------------------------------------
// ##################### Robot 2 #####################
//---------------------------------------------------

#elifdef ROBOT_2

//===================== CONFIG =====================

robot_specs_t robotConfig{.driveWheelDiam = 0.0,
						  .trackWidth = 0.0,
						  .odomWheelDiam = 0.0,
						  .maxDrivePct = 0,
						  .maxTurnPct = 0,
						  .drivePID = nullptr,
						  .turnPID = nullptr};

//===================== DEVICES =====================

pros::MotorGroup leftDriveMotors({11, -12, 13, -14});
pros::MotorGroup rightDriveMotors({17, -18, 19, -20});

pros::MotorGroup intakeMotors({9});
pros::MotorGroup lowerScoringMotors({-10});
pros::MotorGroup upperScoringMotors({1});

pros::adi::DigitalOut scraperCylinder('h');
pros::adi::DigitalOut hoodCylinder('g');
pros::adi::DigitalOut wingCylinder('a');

//==================== SUBSYSTEMS ====================

TankDrive driveBase(leftDriveMotors, rightDriveMotors, DriveStyle::ARCADE,
					pros::E_MOTOR_BRAKE_COAST, pros::E_MOTOR_GEAR_600, 1.0,
					robotConfig);

Transport intake(intakeMotors, 0.75, pros::E_MOTOR_BRAKE_COAST,
				 pros::E_MOTOR_GEAR_600);
Transport lowerScoring(lowerScoringMotors, 0.75, pros::E_MOTOR_BRAKE_COAST,
					   pros::E_MOTOR_GEAR_600);
Transport upperScoring(upperScoringMotors, 0.75, pros::E_MOTOR_BRAKE_COAST,
					   pros::E_MOTOR_GEAR_600);

Pneumatics scraper(scraperCylinder);
Pneumatics hood(hoodCylinder);
Pneumatics wing(wingCylinder);

#endif

//====================== UTILS ======================

void scoreLong() {
	hood.extendPiston();
	upperScoring.moveIn();
	lowerScoring.moveIn();
	intake.moveIn();
}

void scoreUpper() {
	upperScoring.moveOut(60);
	lowerScoring.moveIn();
	intake.moveIn();
}

void scoreLower() {
	upperScoring.moveOut();
	lowerScoring.moveOut();
	intake.moveOut(60);
}

void matchLoad() {
	hood.retractPiston();
	scraper.extendPiston();
	upperScoring.moveIn();
	lowerScoring.moveIn();
	intake.moveIn();
}

void intakeField() {
	hood.retractPiston();
	upperScoring.moveIn();
	lowerScoring.moveIn();
	intake.moveIn();
}

void intakeLoader() {
	hood.retractPiston();
	scraper.extendPiston();
	upperScoring.moveIn();
	lowerScoring.moveIn();
	intake.moveIn();
}

void stopAll() {
	upperScoring.stop();
	lowerScoring.stop();
	intake.stop();
}

void deviceInit() {}

void opcontrolInit() {
	controller.ButtonR1.onPressed([]() { scoreUpper(); });
	controller.ButtonR1.onReleased([]() { stopAll(); });

	controller.ButtonR2.onPressed([]() { scoreLong(); });
	controller.ButtonR2.onReleased([]() { stopAll(); });

	controller.ButtonL1.onPressed([]() { intakeField(); });
	controller.ButtonL1.onReleased([]() { stopAll(); });

	controller.ButtonL2.onPressed([]() { intakeLoader(); });
	controller.ButtonL2.onReleased([]() {
		stopAll();
		scraper.retractPiston();
	});

	controller.ButtonY.onPressed([]() { wing.extendPiston(); });
	controller.ButtonY.onReleased([]() { wing.retractPiston(); });

	controller.ButtonA.onPressed([]() { scoreLower(); });
	controller.ButtonA.onReleased([]() { stopAll(); });

	controller.ButtonB.onPressed([]() { hood.extendPiston(); });
	controller.ButtonB.onReleased([]() { hood.retractPiston(); });
}

void robotInit() { deviceInit(); }