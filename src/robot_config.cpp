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

PIDController<pose_t>::pid_config_t drivePIDConfig{.kP = 0.0,
												   .kI = 0.0,
												   .kD = 0.0,

												   .deadband = 0.0};
PIDController<double>::pid_config_t turnPIDConfig{.kP = 0.0,
												  .kI = 0.0,
												  .kD = 0.0,

												  .deadband = 0.0};

PIDController<pose_t> drivePID(drivePIDConfig);
PIDController<double> turnPID(turnPIDConfig);

robot_specs_t robotConfig{.driveWheelDiam = 0.0,
						  .trackWidth = 0.0,
						  .odomWheelDiam = 0.0,

						  .maxDrivePct = 0,
						  .maxTurnPct = 0,

						  .drivePID = &drivePID,
						  .turnPID = &turnPID};

//===================== DEVICES =====================

pros::MotorGroup leftDriveMotors({12, -13, 14, -15});
pros::MotorGroup rightDriveMotors({-16, 3, -4, 1});

pros::MotorGroup intakeMotors({9});
pros::MotorGroup lowerScoringMotors({-10});
pros::MotorGroup upperScoringMotors({2});

pros::adi::DigitalOut scraperCylinder('a');
pros::adi::DigitalOut hoodCylinder('b');
// pros::adi::DigitalOut wingCylinder('c');

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
// Pneumatics wing(wingCylinder);

//---------------------------------------------------
// ##################### Robot 2 #####################
//---------------------------------------------------

#elifdef ROBOT_2

//===================== CONFIG =====================

PIDController<pose_t>::pid_config_t drivePIDConfig{.kP = 0.0,
												   .kI = 0.0,
												   .kD = 0.0,

												   .deadband = 0.0};
PIDController<double>::pid_config_t turnPIDConfig{.kP = 0.0,
												  .kI = 0.0,
												  .kD = 0.0,

												  .deadband = 0.0};

PIDController<pose_t> drivePID(drivePIDConfig);
PIDController<double> turnPID(turnPIDConfig);

robot_specs_t robotConfig{.driveWheelDiam = 0.0,
						  .trackWidth = 0.0,
						  .odomWheelDiam = 0.0,

						  .maxDrivePct = 0,
						  .maxTurnPct = 0,

						  .drivePID = &drivePID,
						  .turnPID = &turnPID};

//===================== DEVICES =====================

pros::MotorGroup leftDriveMotors({11, -12, 13, -14});
pros::MotorGroup rightDriveMotors({17, -18, 19, -20});

pros::MotorGroup intakeMotors({9});
pros::MotorGroup lowerScoringMotors({-10});
pros::MotorGroup upperScoringMotors({1});

pros::adi::DigitalOut scraperCylinder('b');
pros::adi::DigitalOut hoodCylinder('a');

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

#endif

//====================== UTILS ======================

void deviceInit() {}

void opcontrolInit() {
	controller.ButtonR1.onHold([]() {
		intake.moveIn();
		lowerScoring.moveIn();
		upperScoring.moveIn(50);
	});
	controller.ButtonR1.onReleased([]() {
		intake.stop();
		lowerScoring.stop();
		upperScoring.stop();
	});

	controller.ButtonL1.onHold([]() {
		intake.moveOut();
		lowerScoring.moveOut();
	});
	controller.ButtonL1.onReleased([]() {
		intake.stop();
		lowerScoring.stop();
	});

	controller.ButtonR2.onHold([]() {
		intake.moveIn();
		lowerScoring.moveIn();
		upperScoring.moveIn();
	});
	controller.ButtonR2.onReleased([]() {
		intake.stop();
		lowerScoring.stop();
		upperScoring.stop();
	});

	controller.ButtonL2.onHold([]() { upperScoring.moveOut(); });
	controller.ButtonL2.onReleased([]() { upperScoring.stop(); });

	controller.ButtonA.onPressed([]() { scraper.retractPiston(); });

	controller.ButtonA.onReleased([]() { scraper.extendPiston(); });

	controller.ButtonUp.onPressed([]() { hood.extendPiston(); });
	controller.ButtonDown.onPressed([]() { hood.retractPiston(); });
}

void robotInit() { deviceInit(); }