#include "robot_config.hpp"

#include <hskylib/robot_specs.h>
#include <hskylib/subsystems/pneumatics.h>
#include <hskylib/utils/commands/drive_commands.h>
#include <hskylib/utils/utils.h>

#include "pros/adi.hpp"
#include "pros/motors.h"

HskyController controller(pros::E_CONTROLLER_MASTER);
std::queue<Command *> commandQueue;

// ---------------------------------------------------------
// ##################### Configuration #####################
// ---------------------------------------------------------
#define ROBOT_1
#define BLUE

//---------------------------------------------------
// ##################### Robot 1 #####################
//---------------------------------------------------

#ifdef ROBOT_1
bool isLeft = false;

// //===================== CONFIG =====================

PIDController drivePid(7.0, 0.0, 0, PIDController::ERROR_TYPE::LINEAR);
// PIDController drivePid(0.15, 0.0, 0.05, PIDController::ERROR_TYPE::LINEAR);
// PIDController turnPid(53.0, 0.67, 115.0, PIDController::ERROR_TYPE::ANGULAR);
// PIDController turnPid(130.0, 10, 90.0, PIDController::ERROR_TYPE::ANGULAR);
// PIDController turnPid(70.0, 0.0, 0.0, PIDController::ERROR_TYPE::ANGULAR);
PIDController turnPid(81.25, 0, 0, PIDController::ERROR_TYPE::ANGULAR);
PIDController headingPid(0.0, 0, 0.0, PIDController::ERROR_TYPE::ANGULAR);

robot_specs_t robotConfig{.driveWheelDiameter = 2.75,
						  .trackWidth = 11.0,
						  .odomPodDiameter = 0.0,
						  .maxDrivePct = 33,
						  .maxTurnPct = 100,
						  .drivePID = &drivePid,
						  .headingPID = &headingPid,
						  .turnPID = &turnPid};

// //===================== DEVICES =====================

pros::MotorGroup leftDriveMotors({-11, 12, -13, 14});
pros::MotorGroup rightDriveMotors({17, -18, 19, -20});

pros::IMU imu(16);

pros::MotorGroup intakeMotors({9});
pros::MotorGroup lowerScoringMotors({-6});
pros::MotorGroup upperScoringMotors({4});

pros::adi::DigitalOut scraperCylinder('b');
pros::adi::DigitalOut hoodCylinder('a');
pros::adi::DigitalOut wingCylinder('c');

pros::Optical opticalSensor(15);

// //==================== SUBSYSTEMS ====================
DrivebaseOdometry odom(&leftDriveMotors, &rightDriveMotors, robotConfig, &imu,
					   true);

TankDrive driveBase(leftDriveMotors, rightDriveMotors,
					pros::E_MOTOR_BRAKE_COAST, pros::E_MOTOR_GEAR_600, 1.0,
					0.75);
Transport lowerScoring(lowerScoringMotors, 1, pros::E_MOTOR_BRAKE_COAST,
					   pros::E_MOTOR_GEAR_600);
Transport upperScoring(upperScoringMotors, 1, pros::E_MOTOR_BRAKE_COAST,
					   pros::E_MOTOR_GEAR_600);

Transport intake(intakeMotors, 1, pros::E_MOTOR_BRAKE_COAST,
				 pros::E_MOTOR_GEAR_600);

Pneumatics scraper(scraperCylinder);
Pneumatics hood(hoodCylinder);
Pneumatics wing(wingCylinder);

//---------------------------------------------------
// ##################### Robot 2 #####################
//---------------------------------------------------

#elifdef ROBOT_2
bool isLeft = true;	 // TODO: Should be true

//===================== CONFIG =====================

PIDController drivePid(7.0, 0.0, 0, PIDController::ERROR_TYPE::LINEAR);
// PIDController drivePid(0.15, 0.0, 0.05, PIDController::ERROR_TYPE::LINEAR);
// PIDController turnPid(80.0, 0.0 , 0.0, PIDController::ERROR_TYPE::ANGULAR);
PIDController turnPid(80.0, 2.0, 750.0, PIDController::ERROR_TYPE::ANGULAR);
PIDController headingPid(0.0, 0, 0.0, PIDController::ERROR_TYPE::ANGULAR);
// PIDController headingPid(60.0, 0.0, 0.0, PIDController::ERROR_TYPE::ANGULAR);


robot_specs_t robotConfig{.driveWheelDiameter = 2.75,
						  .trackWidth = 11.0,
						  .odomPodDiameter = 0.0,
						  .maxDrivePct = 30,
						  .maxTurnPct = 80,
						  .drivePID = &drivePid,
						  .headingPID = &headingPid,
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
pros::adi::DigitalOut wingCylinder('f');

DrivebaseOdometry odom(&leftDriveMotors, &rightDriveMotors, robotConfig, &imu,
					   true);

//==================== SUBSYSTEMS ====================

TankDrive driveBase(leftDriveMotors, rightDriveMotors,
					pros::E_MOTOR_BRAKE_COAST, pros::E_MOTOR_GEAR_600, 1.0,
					0.75);
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
	pros::delay(1000);	// Allow time for devices to initialize
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
	lowerScoring.moveOut(0);
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

void constructMatchAuton(bool isLeft, bool isRed) {
	// Drive to loader
	commandQueue.push(new InstantCommand([&]() { imu.tare(); }));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 37.0, 1500));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, isLeft ? 90.0 : 270.0));

	// Intake from loader
	commandQueue.push(new InstantCommand([&]() { intakeLoader(); }));
	// commandQueue.push(
	// 	new DriveDistance(driveBase, odom, robotConfig, 18, 1400));

	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 40, 2000, 100));
	// commandQueue.push(
		// new DriveDeadReckon(driveBase, 100, 100, 1000, 100));
	commandQueue.push(new TimeoutCommand(2000));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));

	// Back up and turn to corner
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -22, 1500));
	// commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, isLeft
	// ? 45.0 : 315.0));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, isLeft ? 45.0 : 315.0));

	// Spit out wrong color
	commandQueue.push(new InstantCommand([&]() { scoreLower(); }));
	commandQueue.push(new WaitUntilColorSensor(opticalSensor, isRed, 2000));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));

	// // Spit out again to avoid getting stuck
	// commandQueue.push(new InstantCommand([&]() {
	// 	intakeField();
	// }));
	// commandQueue.push(new TimeoutCommand(250));
	// commandQueue.push(new InstantCommand([&]() {
	// 	scoreLower();
	// }));
	// commandQueue.push(new WaitUntilColorSensor(opticalSensor, isRed,
	// 1000));ePisto commandQueue.push(new InstantCommand([&]() { 	stopAll();
	// }));

	// Drive to goal
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, isLeft ? 90.0 : 270.0));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -18, 1500, 25));

	// Score
	commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	commandQueue.push(new TimeoutCommand(1000));
	commandQueue.push(
	// 	new DriveDistance(driveBase, odom, robotConfig, -7, 1000));
			new DriveDistance(driveBase, odom, robotConfig, -7, 2000, 25));
	commandQueue.push(new TimeoutCommand(1000));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));

	// Drive to loader again

	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 20, 1500, 50));
	commandQueue.push(new InstantCommand([&]() { intakeLoader(); }));
	commandQueue.push(new TimeoutCommand(100));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 16, 1500));

	// Intake
	// commandQueue.push(;
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig,
										isLeft ? 90.0 : 270.0, 100));
	commandQueue.push(new TimeoutCommand(2000));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));

	// Go to goal
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, isLeft ? 90.0 : 270.0));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -37, 1500, 25));
	commandQueue.push(new InstantCommand([&]() { lowerScoring.moveOut(); }));
	commandQueue.push(new TimeoutCommand(200));
	// Score
	commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -7, 2000, 25));
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

void constructSkillsAuton(bool isLeft, bool isRed) {
	commandQueue.push(new InstantCommand([&]() { imu.tare(); }));

	// Drive to loader
	commandQueue.push(new InstantCommand([&]() { imu.tare(); }));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 40, 1500));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, isLeft ? 90.0 : 270.0));

	// Intake from loader
	commandQueue.push(new InstantCommand([&]() { intakeLoader(); }));
	commandQueue.push(new TimeoutCommand(100));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 18, 1400));
	commandQueue.push(new TimeoutCommand(1500));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, -5,
	// 1400)); commandQueue.push(new DriveDistance(driveBase, odom, robotConfig,
	// 5, 1400)); commandQueue.push(new TimeoutCommand(1500));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));

	// Drive to goal
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, isLeft ? 90.0 : 270.0));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -40, 1500));

	// Score
	commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	commandQueue.push(new TimeoutCommand(2500));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));
	commandQueue.push(new TimeoutCommand(150));
	commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	// commandQueue.push(new TimeoutCommand(1500));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, -7,
	// 1000)); commandQueue.push(new TimeoutCommand(1000));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));

	// Back up

	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 10, 1000));
	commandQueue.push(new InstantCommand([&]() { scraper.retractPiston(); }));
	commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	// Drive towards center
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 180.0));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 18, 1500));

	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, isLeft ? 90.0 : 270.0));

	commandQueue.push(new InstantCommand([&]() { stopAll(); }));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -84, 2500));
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0.0));
	// commandQueue.push(new InstantCommand([&]() {
	// 	scraper.retractPiston();
	// }));

	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 53, 1500));
	commandQueue.push(new InstantCommand([&]() { intakeField(); }));
	commandQueue.push(new TimeoutCommand(100));
	commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, -5, 500));
	commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, 5, 500));
	commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, -5, 500));
	commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, 5, 1000));
	commandQueue.push(new TimeoutCommand(500));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -15, 1500));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, isLeft ? 270.0 : 90.0));
	commandQueue.push(new TimeoutCommand(1000));

	intakeField();
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -24, 1400));
	commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	commandQueue.push(new TimeoutCommand(1000));

	commandQueue.push(new InstantCommand([&]() { intakeLoader(); }));
	commandQueue.push(new TimeoutCommand(1000));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 40, 1500));
	commandQueue.push(new TimeoutCommand(1500));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -5, 1400));
	commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, 6, 1400));
	commandQueue.push(new TimeoutCommand(500));

	// Drive to goal

	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -44, 1500));
	// Score
	commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	commandQueue.push(new TimeoutCommand(2500));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));
	commandQueue.push(new TimeoutCommand(150));
	commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	commandQueue.push(new TimeoutCommand(500));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));

	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 18, 1000));
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0.0));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -24, 2000));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, isLeft ? 270.0 : 90.0));
	commandQueue.push(new TimeoutCommand(1000));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -120, 2500));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -10, 500));

	commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, 1, 1000));
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0.0));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -24, 2000));
	// // Drive to loader again

	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, 18,
	// 1500)); commandQueue.push(new InstantCommand([&]() { 	intakeLoader();
	// }));
	// commandQueue.push(new TimeoutCommand(100));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, 16,
	// 1500));

	// // Intake
	// // commandQueue.push(;
	// commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, isLeft
	// ? 90.0 : 270.0, 100)); commandQueue.push(new TimeoutCommand(2000));
	// commandQueue.push(new InstantCommand([&]() {
	// 	stopAll();
	// }));

	// // Go to goal
	// commandQueue.push(
	// 	new TurnToHeading(driveBase, odom, robotConfig, isLeft ? 90.0 : 270.0));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, -37,
	// 1500)); commandQueue.push(new InstantCommand([&]() {
	// 	lowerScoring.moveOut();
	// }));
	// commandQueue.push(new TimeoutCommand(200));
	// // Score
	// commandQueue.push(new InstantCommand([&]() {
	// 	scoreLong();
	// }));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, -7,
	// 1000)); commandQueue.push(new TimeoutCommand(5000));

	// // Stop and flip hood
	// commandQueue.push(new InstantCommand([&]() {
	// 	hood.retractPiston();
	// 	stopAll();
	// }));
	// commandQueue.push(new TimeoutCommand(1000));
	// commandQueue.push(new InstantCommand([&]() {
	// 	hood.extendPiston();
	// 	stopAll();
	// }));
	// commandQueue.push(new TimeoutCommand(100000));
}

void constructTuningAuton(bool isLeft, bool isRed) {
	// commandQueue.push(
	// 	new DriveDistance(driveBase, odom, robotConfig, 20.0, 1500));
	// commandQueue.push(new InstantCommand([&]() { imu.tare(); }));
	// commandQueue.push(new TimeoutCommand(100));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, 10.0, 99999));
	// commandQueue.push(new TimeoutCommand(1000));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, -10.0, 99999));
	// commandQueue.push(new TurnToHeading())
	for (int i = 0; i < 8; i++) {
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 90));
	commandQueue.push(new TimeoutCommand(1000));
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0));
	}
	// commandQueue.push(
	// 	new DriveDistance(driveBase, odom, robotConfig, 20.0, 1500));
}

#ifdef ROBOT_1
bool scraperToggle = false;
bool wingToggle = false;

void opcontrolInit() {
	// Intake
	controller.ButtonR1.onPressed([]() { intakeField(); });
	controller.ButtonR1.onReleased([]() { stopAll(); });

	// Score Long
	controller.ButtonR2.onPressed([]() { scoreLong(); });
	controller.ButtonR2.onReleased([]() { stopAll(); });

	// Hood
	controller.ButtonY.onPressed([]() { hood.retractPiston(); });
	controller.ButtonY.onReleased([]() { hood.extendPiston(); });

	// Score Lower
	controller.ButtonL1.onPressed([]() { scoreLower(); });
	controller.ButtonL1.onReleased([]() { stopAll(); });

	// Score Upper
	controller.ButtonL2.onPressed([]() { scoreUpper(); });
	controller.ButtonL2.onReleased([]() { stopAll(); });

	// Upper Scoring Backward
	controller.ButtonLeft.onPressed([]() { upperScoring.moveOut(); });
	controller.ButtonLeft.onReleased([]() { stopAll(); });

	// Scraper
	controller.ButtonA.onPressed([]() {
		scraperToggle = !scraperToggle;
		if (scraperToggle) {
			scraper.extendPiston();
		} else {
			scraper.retractPiston();
		}
	});

	// Wing
	controller.ButtonRight.onPressed([]() {
		wingToggle = !wingToggle;
		if (wingToggle) {
			wing.extendPiston();
		} else {
			wing.retractPiston();
		}
	});

	stopAll();
}

#elifdef ROBOT_2
bool wingToggle = false;

void opcontrolInit() {
	// Score Upper
	controller.ButtonR1.onHold([]() { scoreUpper(); });
	controller.ButtonR1.onReleased([]() { stopAll(); });

	// Score Long
	controller.ButtonR2.onPressed([]() { scoreLong(); });
	controller.ButtonR2.onReleased([]() { stopAll(); });

	// Intake
	controller.ButtonL1.onPressed([]() { intakeField(); });
	controller.ButtonL1.onReleased([]() { stopAll(); });

	// Intake
	controller.ButtonL2.onPressed([]() { intakeLoader(); });
	controller.ButtonL2.onReleased([]() {
		stopAll();
		scraper.retractPiston();
	});

	// Score Lower
	controller.ButtonY.onPressed([]() { scoreLower(); });
	controller.ButtonY.onReleased([]() { stopAll(); });

	// Hood
	controller.ButtonX.onPressed([]() { hood.extendPiston(); });
	controller.ButtonX.onReleased([]() { hood.retractPiston(); });

	// Wing
	controller.ButtonRight.onPressed([]() {
		wingToggle = !wingToggle;
		if (wingToggle) {
			wing.extendPiston();
		} else {
			wing.retractPiston();
		}
	});

	stopAll();
}

#endif

void robotInit() {
	deviceInit();
	//constructMatchAuton(false, true);
	// constructMatchAuton(true, true);
	// constructSkillsAuton(false, false);
	constructTuningAuton(true, false);
}