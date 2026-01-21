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

// //===================== CONFIG =====================

// robot_specs_t robotConfig{.driveWheelDiam = 0.0,
// 						  .trackWidth = 0.0,
// 						  .odomWheelDiam = 0.0,
// 						  .maxDrivePct = 0,
// 						  .maxTurnPct = 0,
// 						  .drivePID = nullptr,
// 						  .turnPID = nullptr};

// //===================== DEVICES =====================

// pros::MotorGroup leftDriveMotors({12, -13, 14, -15});
// pros::MotorGroup rightDriveMotors({-16, 3, -4, 1});

// pros::MotorGroup intakeMotors({9});
// pros::MotorGroup lowerScoringMotors({-10});
// pros::MotorGroup upperScoringMotors({2});

// pros::adi::DigitalOut scraperCylinder('a');
// pros::adi::DigitalOut hoodCylinder('b');

// //==================== SUBSYSTEMS ====================
// TankDrive driveBase(leftDriveMotors, rightDriveMotors, DriveStyle::ARCADE,

// 					pros::E_MOTOR_BRAKE_COAST, pros::E_MOTOR_GEAR_600, 1.0,

// 					robotConfig);

// Transport intake(intakeMotors, 0.75, pros::E_MOTOR_BRAKE_COAST,

// 				 pros::E_MOTOR_GEAR_600);
// Transport lowerScoring(lowerScoringMotors, 0.75, pros::E_MOTOR_BRAKE_COAST,

// 					   pros::E_MOTOR_GEAR_600);
// Transport upperScoring(upperScoringMotors, 0.75, pros::E_MOTOR_BRAKE_COAST,

// 					   pros::E_MOTOR_GEAR_600);

// Pneumatics scraper(scraperCylinder);
// Pneumatics hood(hoodCylinder);

//---------------------------------------------------
// ##################### Robot 2 #####################
//---------------------------------------------------

#elifdef ROBOT_2

//===================== CONFIG =====================

PIDController drivePid(1.0, 0.0, 0.0, PIDController::ERROR_TYPE::LINEAR);
PIDController turnPid(0.0, 0.0, 0.0, PIDController::ERROR_TYPE::ANGULAR);


robot_specs_t robotConfig{.driveWheelDiam = 0.0,
						  .trackWidth = 11.0,
						  .odomWheelDiam = 0.0,
						  .maxDrivePct = 0,
						  .maxTurnPct = 0,
						  .drivePID = &drivePid,
						  .turnPID = &turnPid};

//===================== DEVICES =====================

pros::MotorGroup leftDriveMotors({11, -12, 13, -14});
pros::MotorGroup rightDriveMotors({17, -18, 19, -20});

pros::MotorGroup intakeMotors({9});
pros::MotorGroup lowerScoringMotors({-10});
pros::MotorGroup upperScoringMotors({1});

pros::adi::DigitalOut scraperCylinder('b');
pros::adi::DigitalOut hoodCylinder('a');

DrivebaseOdometry odom(&leftDriveMotors, &rightDriveMotors,
						pros::E_MOTOR_GEAR_600, robotConfig.trackWidth);

//==================== SUBSYSTEMS ====================

TankDrive driveBase(leftDriveMotors, rightDriveMotors, DriveStyle::ARCADE,
					pros::E_MOTOR_BRAKE_COAST, pros::E_MOTOR_GEAR_600, 1.0,
				robotConfig, &odom);
Transport lowerScoring(lowerScoringMotors, 0.75, pros::E_MOTOR_BRAKE_COAST,
					   pros::E_MOTOR_GEAR_600);
Transport upperScoring(upperScoringMotors, 0.75, pros::E_MOTOR_BRAKE_COAST,
					   pros::E_MOTOR_GEAR_600);

Pneumatics scraper(scraperCylinder);
Pneumatics hood(hoodCylinder);

#endif

//====================== UTILS ======================

void deviceInit() {	
	odom.reset();
	odom.init();

}

void opcontrolInit() {}

void robotInit() { deviceInit(); }