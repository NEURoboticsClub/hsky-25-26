#include "robot_config.hpp"

#include <hskylib/robot_specs.h>
#include <hskylib/subsystems/pneumatics.h>
#include <hskylib/utils/commands/follow_trajectory_command.h>
#include <hskylib/utils/controls/feedforward.h>
#include <hskylib/utils/trajectory.h>
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

PIDController drivePid(0.1, 0.0, 0, PIDController::ERROR_TYPE::LINEAR);
// PIDController drivePid(0.15, 0.0, 0.05, PIDController::ERROR_TYPE::LINEAR);
// PIDController turnPid(53.0, 0.67, 115.0, PIDController::ERROR_TYPE::ANGULAR);
PIDController turnPid(53.0, 0, 90.0, PIDController::ERROR_TYPE::ANGULAR);
PIDController headingPid(0.0, 0, 0.0, PIDController::ERROR_TYPE::ANGULAR);

PIDController velocityLeftPid(50, 0.0, 0, PIDController::ERROR_TYPE::LINEAR);
PIDController velocityRightPid(50, 0.0, 0,
							   PIDController::ERROR_TYPE::LINEAR);

FeedForward feedForwardLeft(5, 1, 0.1);
FeedForward feedForwardRight(5, 1, 0.1);

robot_specs_t robotConfig{.driveWheelDiameter = 2.75,
						  .trackWidth = 11.0,
						  .odomPodDiameter = 0.0,
						  .maxDrivePct = 100,
						  .maxTurnPct = 100,
						  .drivePID = &drivePid,
						  .headingPID = &headingPid,
						  .turnPID = &turnPid,
						  .velocityLeftPID = &velocityLeftPid,
						  .velocityRightPID = &velocityRightPid,
						  .feedForwardLeft = &feedForwardLeft,
						  .feedForwardRight = &feedForwardRight};

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
					0.75, *robotConfig.feedForwardLeft,
					*robotConfig.feedForwardRight, *robotConfig.velocityLeftPID,
					*robotConfig.velocityRightPID);
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

PIDController drivePid(5.0, 0.0, 0.0, PIDController::ERROR_TYPE::LINEAR);
// PIDController drivePid(0.15, 0.0, 0.05, PIDController::ERROR_TYPE::LINEAR);
PIDController turnPid(50.0, 0.0, 0.0, PIDController::ERROR_TYPE::ANGULAR);
PIDController headingPid(0.0, 0.0, 0.0, PIDController::ERROR_TYPE::ANGULAR);

PIDController velocityLeftPid(0.0, 0.0, 0.0, PIDController::ERROR_TYPE::LINEAR);
PIDController velocityRightPid(0.0, 0.0, 0.0,
							   PIDController::ERROR_TYPE::LINEAR);

FeedForward feedForwardLeft(0.0, 0.0, 0.0);
FeedForward feedForwardRight(0.0, 0.0, 0.0);

robot_specs_t robotConfig{.driveWheelDiameter = 2.75,
						  .trackWidth = 11.0,
						  .odomPodDiameter = 0.0,
						  .maxDrivePct = 100,
						  .maxTurnPct = 100,
						  .drivePID = &drivePid,
						  .headingPID = &headingPid,
						  .turnPID = &turnPid,
						  .velocityLeftPID = &velocityLeftPid,
						  .velocityRightPID = &velocityRightPid,
						  .feedForwardLeft = &feedForwardLeft,
						  .feedForwardRight = &feedForwardRight};

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
					pros::E_MOTOR_BRAKE_COAST, pros::E_MOTOR_GEAR_600, 1.0, 1.0,
					*robotConfig.feedForwardLeft, *robotConfig.feedForwardRight,
					*robotConfig.velocityLeftPID,
					*robotConfig.velocityRightPID);
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

void constructAuton(bool isLeft, bool isRed) {
	commandQueue.push(new InstantCommand([&]() { imu.tare(); }));

	static trajectoryState_t forwardStates[] = {
		{.time = 0.0, .pose = {0.0, 0.0, 0.0},  .v = 12.0, .omega = 0.0},
		{.time = 2.0, .pose = {24.0, 0.0, 0.0}, .v = 12.0, .omega = 0.0},
	};

	Trajectory forwardTrajectory(forwardStates, 2);

    commandQueue.push(new FollowTrajectoryCommand(
        driveBase, odom, robotConfig, forwardTrajectory,
        robotConfig.trackWidth, 5000
    ));
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
}

#endif

void robotInit() {
	deviceInit();
	constructAuton(isLeft, isRed);
}