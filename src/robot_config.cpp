#include "robot_config.hpp"

#include <hskylib/robot_specs.h>
#include <hskylib/subsystems/pneumatics.h>
#include <hskylib/utils/utils.h>

#include "pros/adi.hpp"
#include "pros/motors.h"

HskyController controller(pros::E_CONTROLLER_MASTER);
std::queue<Command*> commandQueue;

// ---------------------------------------------------------
// ##################### Configuration #####################
// ---------------------------------------------------------
#define ROBOT_2
#define BLUE



//---------------------------------------------------
// ##################### Robot 1 #####################
//---------------------------------------------------

#ifdef ROBOT_1
bool isLeft = false;

// //===================== CONFIG =====================

// robot_specs_t robotConfig{.driveWheelDiam = 0.0,
// 						  .trackWidth = 0.0,
// 						  .odomWheelDiam = 0.0,
// 						  .maxDrivePct = 0,
// 						  .maxTurnPct = 0,
// 						  .drivePID = nullptr,
// 						  .turnPID = nullptr};

// //===================== DEVICES =====================

pros::MotorGroup leftDriveMotors({-11, 12, -13, 14});
pros::MotorGroup rightDriveMotors({17, -18, 19, -20});

pros::MotorGroup intakeMotors({6});
pros::MotorGroup lowerScoringMotors({-9});
pros::MotorGroup upperScoringMotors({4});

pros::adi::DigitalOut scraperCylinder('c');
pros::adi::DigitalOut hoodCylinder('a');
pros::adi::DigitalOut wingCylinder('b');

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

Pneumatics scraper(scraperCylinder);
Pneumatics hood(hoodCylinder);
Pneumatics wing(wingCylinder);

//---------------------------------------------------
// ##################### Robot 2 #####################
//---------------------------------------------------

#elifdef ROBOT_2
bool isLeft = false; // TODO: Should be true

//===================== CONFIG =====================

PIDController drivePid(0.1, 0.0, 0, PIDController::ERROR_TYPE::LINEAR);
// PIDController drivePid(0.15, 0.0, 0.05, PIDController::ERROR_TYPE::LINEAR);
PIDController turnPid(53.0, 0.67, 115.0, PIDController::ERROR_TYPE::ANGULAR);


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

pros::IMU imu(16);

pros::MotorGroup intakeMotors({9});
pros::MotorGroup lowerScoringMotors({-10});
pros::MotorGroup upperScoringMotors({1});

pros::adi::DigitalOut scraperCylinder('h');
pros::adi::DigitalOut hoodCylinder('g');
pros::adi::DigitalOut wingCylinder('a');

DrivebaseIMUOdometry odom(&leftDriveMotors, &rightDriveMotors,
						&imu, pros::E_MOTOR_GEAR_600, robotConfig.trackWidth);

//==================== SUBSYSTEMS ====================

TankDrive driveBase(leftDriveMotors, rightDriveMotors, DriveStyle::ARCADE,
					pros::E_MOTOR_BRAKE_COAST, pros::E_MOTOR_GEAR_600, 1.0,
				robotConfig, &odom);
Transport lowerScoring(lowerScoringMotors, 1, pros::E_MOTOR_BRAKE_COAST,
					   pros::E_MOTOR_GEAR_600);
Transport upperScoring(upperScoringMotors, 1, pros::E_MOTOR_BRAKE_COAST,
					   pros::E_MOTOR_GEAR_600);

Transport intake(intakeMotors, 1, pros::E_MOTOR_BRAKE_COAST,
				 pros::E_MOTOR_GEAR_600);

Pneumatics scraper(scraperCylinder);
Pneumatics hood(hoodCylinder);
Pneumatics wing(wingCylinder);
pros::Optical opticalSensor(15);

#endif

// Set red or blue
#ifdef RED
bool isRed = true;
#else
bool isRed = false;
#endif

//====================== UTILS ======================

void deviceInit() {	
	pros::delay(1000);  // Allow time for devices to initialize
	odom.reset();
	odom.init();

	imu.reset();
	while (imu.is_calibrating() || !std::isfinite(imu.get_heading())) {
		pros::delay(20);
	}

}
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
	scraper.retractPiston();
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

void constructAuton(bool isLeft, bool isRed) {
	// Drive to loader
	commandQueue.push(new DriveDistance(driveBase, odom, 40.0, 1500));
	commandQueue.push(new TurnToHeading(driveBase, odom, isLeft ? 90.0: 270.0));

	// Intake from loader
	commandQueue.push(new InstantCommand([&]() {
		intakeLoader();
	}));
	commandQueue.push(new DriveDistance(driveBase, odom, 18, 1400));
	commandQueue.push(new TimeoutCommand(2000));
	commandQueue.push(new InstantCommand([&]() {
		stopAll();
	}));
	
	// Back up and turn to corner
	commandQueue.push(new DriveDistance(driveBase, odom, -22, 1500));
	commandQueue.push(new TurnToHeading(driveBase, odom, isLeft ? 45.0 : 315.0));

	// Spit out wrong color
	commandQueue.push(new InstantCommand([&]() {
		scoreLower();
	}));
	commandQueue.push(new WaitUntilColorSensor(opticalSensor, isRed, 2000));
	commandQueue.push(new InstantCommand([&]() {
		stopAll();
	}));

	// // Spit out again to avoid getting stuck
	// commandQueue.push(new InstantCommand([&]() {
	// 	intakeField();
	// }));
	// commandQueue.push(new TimeoutCommand(250));
	// commandQueue.push(new InstantCommand([&]() {
	// 	scoreLower();
	// }));
	// commandQueue.push(new WaitUntilColorSensor(opticalSensor, isRed, 1000));
	// commandQueue.push(new InstantCommand([&]() {
	// 	stopAll();
	// }));

	// Drive to goal
	commandQueue.push(new TurnToHeading(driveBase, odom, isLeft ? 90.0 : 270.0));
	commandQueue.push(new DriveDistance(driveBase, odom, -18, 1500));

	// Score
	commandQueue.push(new InstantCommand([&]() {
		scoreLong();
	}));
	commandQueue.push(new TimeoutCommand(1000));
	commandQueue.push(new DriveDistance(driveBase, odom, -7, 1000));
	commandQueue.push(new TimeoutCommand(1000));
	commandQueue.push(new InstantCommand([&]() {
		stopAll();
	}));

	// Drive to loader again
	commandQueue.push(new InstantCommand([&]() {
		intakeLoader();
	}));
	commandQueue.push(new DriveDistance(driveBase, odom, 18, 1500));
	commandQueue.push(new TimeoutCommand(100));
	commandQueue.push(new DriveDistance(driveBase, odom, 16, 1500));

	// Intake
	commandQueue.push(new TimeoutCommand(2000));
	commandQueue.push(new InstantCommand([&]() {
		stopAll();
	}));

	// Go to goal
	commandQueue.push(new TurnToHeading(driveBase, odom, isLeft ? 90.0 : 270.0));
	commandQueue.push(new DriveDistance(driveBase, odom, -37, 1500));

	// Score
	commandQueue.push(new InstantCommand([&]() {
		scoreLong();
	}));
	commandQueue.push(new TimeoutCommand(5000));

	// Stop and flip hood
	commandQueue.push(new InstantCommand([&]() {
		hood.retractPiston();
		stopAll();
	}));
	commandQueue.push(new TimeoutCommand(1000));
	commandQueue.push(new InstantCommand([&]() {
		hood.extendPiston();
		stopAll();
	}));
	commandQueue.push(new TimeoutCommand(100000));
}

void opcontrolInit() {
	controller.ButtonR1.onHold([]() { scoreUpper(); });
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

void robotInit() { 
	deviceInit();
	constructAuton(isLeft, isRed);
}