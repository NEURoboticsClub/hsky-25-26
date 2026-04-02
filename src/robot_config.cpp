#include "robot_config.hpp"

#include <hskylib/robot_specs.h>
#include <hskylib/subsystems/pneumatics.h>
#include <hskylib/utils/commands/base_commands.h>
#include <hskylib/utils/commands/drive_commands.h>
#include <hskylib/utils/utils.h>

#include "hskylib/utils/color_reader.h"
#include "hskylib/subsystems/cycler.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

HskyController controller(pros::E_CONTROLLER_MASTER);

std::queue<Command *> commandQueue;

// ---------------------------------------------------------
// ##################### Configuration #####################
// ---------------------------------------------------------
// Red starts on the left, Purple starts on the right
#define PURPLE_ROBOT // RED_ROBOT, PURPLE_ROBOT
#define SKILLS // MATCH, SKILLS, AWP
#define RED // RED, BLUE

//---------------------------------------------------
// ##################### Robot 1 #####################
//---------------------------------------------------

#ifdef PURPLE_ROBOT
bool isPurpleRobot = true;

// //===================== CONFIG =====================

PIDController drivePid(7.0, 0.0, 0, PIDController::ERROR_TYPE::LINEAR);
// PIDController drivePid(0.15, 0.0, 0.05, PIDController::ERROR_TYPE::LINEAR);
// PIDController turnPid(53.0, 0.67, 115.0, PIDController::ERROR_TYPE::ANGULAR);
// PIDController turnPid(130.0, 10, 90.0, PIDController::ERROR_TYPE::ANGULAR);
// PIDController turnPid(70.0, 0.0, 0.0, PIDController::ERROR_TYPE::ANGULAR);
PIDController turnPid(120, 0, 10, PIDController::ERROR_TYPE::ANGULAR);
PIDController headingPid(50.0, 0, 0.0, PIDController::ERROR_TYPE::ANGULAR);

robot_specs_t robotConfig{.driveWheelDiameter = 2.75,
						  .trackWidth = 11.0,
						  .odomPodDiameter = 0.0,
						//   .maxDrivePct = 33,
						  .maxDrivePct = 45,
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
pros::adi::DigitalOut hoodCylinder('a', true);
pros::adi::DigitalOut wingCylinder('c');

pros::Optical opticalSensor(15);
HskyColorReader colorReader(15);  // Port 15 same as opticalSensor

// Cycler for color-sorted intake/outtake
Cycler cycler(colorReader);

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

#elifdef RED_ROBOT
bool isPurpleRobot = false;

//===================== CONFIG =====================

PIDController drivePid(5.0, 0.0, 0, PIDController::ERROR_TYPE::LINEAR);
// PIDController drivePid(0.15, 0.0, 0.05, PIDController::ERROR_TYPE::LINEAR);
// PIDController turnPid(80.0, 0.0 , 0.0, PIDController::ERROR_TYPE::ANGULAR);
PIDController turnPid(131, 0, 2, PIDController::ERROR_TYPE::ANGULAR);
PIDController headingPid(100.0, 0.0, 0.0, PIDController::ERROR_TYPE::ANGULAR);
// PIDController headingPid(60.0, 0.0, 0.0, PIDController::ERROR_TYPE::ANGULAR);

robot_specs_t robotConfig{.driveWheelDiameter = 2.75,
						  .trackWidth = 11.0,
						  .odomPodDiameter = 0.0,
						  .maxDrivePct = 33,
						  .maxTurnPct = 50,
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
pros::adi::DigitalOut wingCylinder('f'); //f

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
bool isRedTeam = true;
#else
bool isRedTeam = false;
#endif

// Set skils or match
#ifdef MATCH
int autonType = 0;
#elifdef SKILLS
int autonType = 1;
#else // AWP
int autonType = 2;
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


void queueWiggleIntake(double distance = 5.0, int times = 2) {
	for (int i = 0; i < times; i++) {
		commandQueue.push(
			new DriveDistance(driveBase, odom, robotConfig, -distance, 500));
		commandQueue.push(
			new DriveDistance(driveBase, odom, robotConfig, distance, 500));
	}
}

void queueLoaderCycle(int deadReckonTime, uint8_t speectPct = 100) {
	commandQueue.push(new InstantCommand([&]() { intakeLoader(); }));
	commandQueue.push(new TimeoutCommand(250));
	commandQueue.push(
		new DriveDeadReckon(driveBase, speectPct, speectPct, deadReckonTime));
	// queueWiggleIntake(1.0, 2);
	// commandQueue.push(
	// 	new DriveDeadReckon(driveBase, speectPct, speectPct, deadReckonTime/2 ));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));
}

void queueScoreLong() {
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -37, 1500));

	commandQueue.push(
		new DriveDeadReckon(driveBase, -50, -50, 250)
	);
	commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	commandQueue.push(
		new DriveDeadReckon(driveBase, -50, -50, 1000)
	);
	commandQueue.push(new TimeoutCommand(1200));
}

void queueScoreLongCut() {
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -37, 1500));

	commandQueue.push(
		new DriveDeadReckon(driveBase, -50, -50, 250)
	);
	commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	// commandQueue.push(
	// 	new DriveDeadReckon(driveBase, -50, -50, 1000)
	// );
	commandQueue.push(new TimeoutCommand(2200));
}

void constructRedMatchAuton(bool isRed) {
	const int LOADER_DEAD_RECKON_TIME = 3100;
	const int LOADER_DEAD_RECKON_SPEED = 40;

	commandQueue.push(new InstantCommand([&]() { imu.tare(); }));

	// First loader cycle
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 32.0, 2500));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 90.0, 1500));
	
	// Intake from loader
	queueLoaderCycle(LOADER_DEAD_RECKON_TIME, LOADER_DEAD_RECKON_SPEED);

	// Back up and turn to corner
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -22, 1500));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 45.0, 1000));

	// Spit out wrong color
	commandQueue.push(new InstantCommand([&]() { scraper.retractPiston(); }));
	commandQueue.push(new InstantCommand([&]() { scoreLower(); }));
	commandQueue.push(new WaitUntilColorSensor(opticalSensor, isRed, 2000));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 90.0, 1000));

	// Score long
	queueScoreLongCut();
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));

	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 20, 1500, 50));
	queueLoaderCycle(LOADER_DEAD_RECKON_TIME, LOADER_DEAD_RECKON_SPEED);

	// Score long
	queueScoreLongCut();

	// Stop
	commandQueue.push(new InstantCommand([&]() {
		hood.extendPiston();
		stopAll();
	}));
	commandQueue.push(new TimeoutCommand(100000));
}

void constructPurpleMatchAuton(bool isRed) {
	const int LOADER_DEAD_RECKON_TIME = 3100;
	const int LOADER_DEAD_RECKON_SPEED = 40;

	commandQueue.push(new InstantCommand([&]() { imu.tare(); }));

	// First loader cycle
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 30.5, 1500));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 270.0, 1500));

	// Intake from loader
	queueLoaderCycle(LOADER_DEAD_RECKON_TIME, LOADER_DEAD_RECKON_SPEED);

	// Back up and turn to corner
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -22, 1500));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 315.0, 1000));

	// Spit out wrong color
	commandQueue.push(new InstantCommand([&]() { scraper.retractPiston(); }));
	commandQueue.push(new InstantCommand([&]() { scoreLower(); }));
	commandQueue.push(new WaitUntilColorSensor(opticalSensor, isRed, 2000));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 270.0, 1000));

	// Score long
	queueScoreLong();
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));

	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 20, 1500, 50));
	queueLoaderCycle(LOADER_DEAD_RECKON_TIME, LOADER_DEAD_RECKON_SPEED);

	// Score long
	queueScoreLong();

	// Stop
	commandQueue.push(new InstantCommand([&]() {
		hood.extendPiston();
		stopAll();
	}));
	commandQueue.push(new TimeoutCommand(100000));
}

void constructRedAWPAuton(bool isRed) {}

void constructPurpleAWPAuton(bool isRed) {
	const int LOADER_DEAD_RECKON_TIME = 2800;
	const int LOADER_DEAD_RECKON_SPEED = 40;

	commandQueue.push(new InstantCommand([&]() { imu.tare(); }));

	// Go to center
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 45.0, 2000));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 45.0, 1000));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 7.5, 1500));

	// Score center
	commandQueue.push(new InstantCommand([&]() { scoreLower(); }));
	commandQueue.push(new TimeoutCommand(1500));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));

	// Drive to loader
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -46.5, 2000));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 180.0, 1500, 40));

	// Intake from loader
	queueLoaderCycle(LOADER_DEAD_RECKON_TIME, LOADER_DEAD_RECKON_SPEED);

	// Back up and turn to corner
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -22, 1500));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 225.0, 1000));

	// Spit out wrong color
	commandQueue.push(new InstantCommand([&]() { scoreLower(); }));
	commandQueue.push(new WaitUntilColorSensor(opticalSensor, isRed, 2000));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 180.0, 1000));

	// Score long
	queueScoreLong();
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));

	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 20, 1500, 50));
	queueLoaderCycle(LOADER_DEAD_RECKON_TIME, LOADER_DEAD_RECKON_SPEED);

	// Score long
	queueScoreLong();

	// Stop
	commandQueue.push(new InstantCommand([&]() {
		hood.extendPiston();
		stopAll();
	}));
	commandQueue.push(new TimeoutCommand(100000));
}

void constructPurpleSkillsAuton() {
	// const int LOADER_DEAD_RECKON_TIME = 2800;
	const int LOADER_DEAD_RECKON_TIME = 3100;
	const int LOADER_DEAD_RECKON_SPEED = 40;
	const int SCORE_LONG_TIME = 1500;

	commandQueue.push(new InstantCommand([&]() { imu.tare(); }));

	// First loader cycle
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 30.5, 1500));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 270.0, 1250));

	// Intake from loader
	queueLoaderCycle(LOADER_DEAD_RECKON_TIME, LOADER_DEAD_RECKON_SPEED);
	queueScoreLong();

	// Back up
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 10, 1000));
	commandQueue.push(new InstantCommand([&]() { scraper.retractPiston();
	})); 
	commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	
	// Drive towards center
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 180.0, 1250)); 
	commandQueue.push( 	new DriveDistance(driveBase, odom, robotConfig, 18, 1500)); 
	commandQueue.push( 	new TurnToHeading(driveBase, odom, robotConfig, 270.0, 1250));

	// Cross field to opposite side
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -88, 3000));
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0.0, 1250));

	// Collect field ball with jiggle motion
	commandQueue.push(new InstantCommand([&]() { intakeField(); }));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 67, 1500));
	commandQueue.push(
		new DriveDeadReckon(driveBase, 100, 100, 300));
	commandQueue.push(new InstantCommand([&]() { intakeField(); }));
	commandQueue.push(new TimeoutCommand(100));
	queueWiggleIntake(5.0, 2);
	commandQueue.push(new TimeoutCommand(500));

	// Retreat and turn to side goal
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -15, 1500));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 90.0, 1250));

	// Score field balls
	intakeField();
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -24, 1400));
	commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	commandQueue.push(new TimeoutCommand(1000));

	commandQueue.push(new InstantCommand([&]() { intakeLoader(); }));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 32, 1400));
	// Intake from loader
	
	queueLoaderCycle(LOADER_DEAD_RECKON_TIME, LOADER_DEAD_RECKON_SPEED);
	commandQueue.push(new TimeoutCommand(100));
	queueScoreLong();
	commandQueue.push(new InstantCommand([&]() { 
		stopAll();
		scraper.retractPiston();
	 }));

	commandQueue.push(new TimeoutCommand(100));


	//Back up and clear blue zone
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 10, 1400));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 0.0, 1250));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -46, 2000));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 90, 1250));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 12, 1400));
	commandQueue.push(new InstantCommand([&]() { wing.extendPiston(); }));
	commandQueue.push(new TimeoutCommand(500));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -10, 1400));
	commandQueue.push(new InstantCommand([&]() { wing.retractPiston(); }));

	// Park ?
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 180, 1250));	
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -24, 2000));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 80, 1500));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -108, 3500));
	commandQueue.push(
		new DriveDeadReckon(driveBase, -50, -50, 300));

	// commandQueue.push(
	// 	new DriveDistance(driveBase, odom, robotConfig, -108, 5000));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 10, 1000));	
		
	
	commandQueue.push(
		new DriveDeadReckon(driveBase, -50, -50, 5000));
}

void constructRedSkillsAuton() {
	const int LOADER_DEAD_RECKON_TIME = 3100;
	const int LOADER_DEAD_RECKON_SPEED = 40;
	const int SCORE_LONG_TIME = 1500;

	commandQueue.push(new InstantCommand([&]() { imu.tare(); }));
	
	// Clear park zone
	commandQueue.push(new InstantCommand([&]() { wing.extendPiston(); }));
	commandQueue.push(new TimeoutCommand(500));
	commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, -8.0, 1500));
	commandQueue.push(new InstantCommand([&]() { wing.retractPiston(); }));
	commandQueue.push(new TimeoutCommand(500));

	// Drive to loader
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 90.0, 1500));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -48, 3000));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 0.0, 1500));

	// Intake from loader
	queueLoaderCycle(LOADER_DEAD_RECKON_TIME + 500, LOADER_DEAD_RECKON_SPEED); // Longer drive
	queueScoreLong();

	// Back up
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 10, 1000));
	commandQueue.push(new InstantCommand([&]() { scraper.retractPiston();
	})); 
	commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	
	// Drive towards center
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 90.0, 1500));
	commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, 18, 1500)); 
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0.0, 1500));

	// Cross field to opposite side
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -88, 4500));
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 270.0, 1500));

	// Collect field ball with jiggle motion
	commandQueue.push(new InstantCommand([&]() { intakeField(); }));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 67, 1500));
	commandQueue.push(
		new DriveDeadReckon(driveBase, 50, 50, 300));
	commandQueue.push(new InstantCommand([&]() { intakeField(); }));
	commandQueue.push(new TimeoutCommand(100));
	queueWiggleIntake(5.0, 2);
	commandQueue.push(new TimeoutCommand(500));

	// Retreat and turn to side goal
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -15, 1500));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 180.0, 1500));
	commandQueue.push(new TimeoutCommand(1000));

	// Score field balls
	intakeField();
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -24, 1400));
	commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	commandQueue.push(new TimeoutCommand(1000));

	commandQueue.push(new InstantCommand([&]() { intakeLoader(); }));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 32, 1400));
	// Intake from loader
	
	queueLoaderCycle(LOADER_DEAD_RECKON_TIME, LOADER_DEAD_RECKON_SPEED);
	commandQueue.push(new TimeoutCommand(100));
	queueScoreLong();
	commandQueue.push(new InstantCommand([&]() { 
		stopAll();
		scraper.retractPiston();
	 }));

	commandQueue.push(new TimeoutCommand(100));

	// Drive to Center
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 10, 1400));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 90, 1500));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 24, 3000));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 0, 1500));
	
	// Park
	commandQueue.push(
		new DriveDeadReckon(driveBase, 40, 40, 4500));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, 260, 1500));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -30, 2500, 50));
}

void constructTuningAuton() {
	commandQueue.push(new InstantCommand([&]() { imu.tare(); }));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, 48.0,
	// 5500)); commandQueue.push(new TimeoutCommand(3000));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, -48.0,
	// 5500));

	// commandQueue.push(new InstantCommand([&]() { imu.tare(); }));
	// commandQueue.push(new TimeoutCommand(100));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, 10.0, 99999));
	// commandQueue.push(new TimeoutCommand(1000));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, -10.0, 99999));
	// commandQueue.push(new TurnToHeading())
	// for (int i = 0; i < 4; i++) {
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 45, 1250));
	commandQueue.push(new TimeoutCommand(1000));
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0, 1250));
	commandQueue.push(new TimeoutCommand(1000));
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 90, 1250));
	commandQueue.push(new TimeoutCommand(1000));
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0, 1250));
	commandQueue.push(new TimeoutCommand(1000));
	// }
	// commandQueue.push(
	// 	new DriveDistance(driveBase, odom, robotConfig, 20.0, 1500));
}

void setupCycler() {
	// Register intake transports
	cycler.addIntakeTransport(&intake);

	// Register outtake transports for INACTIVE mode
	cycler.addOuttakeTransport(&upperScoring);
	cycler.addOuttakeTransport(&lowerScoring);

	// Register color-sorted outtake transports
	cycler.addCorrectColorOuttakeTransport(&upperScoring);
	cycler.addIncorrectColorOuttakeTransport(&lowerScoring);

	// Initialize the color reader (starts background task)
	colorReader.initialize();
}

void colorSortAwareOuttake() {
	ColorSort sortMode = cycler.getColorSort();
	int sortVal = static_cast<int>(sortMode);

	if (sortVal == 0) {  // INACTIVE
		// Normal mode: use the old logic
		scoreLower();
	} else {
		// Color sort mode: check ball color and eject appropriately
		ColorType color = colorReader.getColor();
		int colorVal = static_cast<int>(color);
		int wrongColorVal = (sortVal == 1) ? 0 : 1;  // RED=1, BLUE=0

		if (colorVal == wrongColorVal) {
			lowerScoring.moveOut();  // Eject wrong color from bottom
		} else {
			upperScoring.moveOut();  // Let correct color out the top
		}
	}
}

#ifdef PURPLE_ROBOT
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

	// Color sort toggle
	controller.ButtonX.onPressed([]() {
		ColorSort current = cycler.getColorSort();
		int currentVal = static_cast<int>(current);
		ColorSort next;

		if (currentVal == 0) {  // INACTIVE
			next = static_cast<ColorSort>(1);  // RED
		} else if (currentVal == 1) {  // RED
			next = static_cast<ColorSort>(2);  // BLUE
		} else {
			next = static_cast<ColorSort>(0);  // INACTIVE
		}
		cycler.setColorSort(next);
	});

	// Score Lower - now color-sort aware
	controller.ButtonL1.onPressed([]() { colorSortAwareOuttake(); });
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

#elifdef RED_ROBOT
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

	// Color sort toggle
	controller.ButtonLeft.onPressed([]() {
		ColorSort current = cycler.getColorSort();
		int currentVal = static_cast<int>(current);
		ColorSort next;

		if (currentVal == 0) {  // INACTIVE
			next = static_cast<ColorSort>(1);  // RED
		} else if (currentVal == 1) {  // RED
			next = static_cast<ColorSort>(2);  // BLUE
		} else {
			next = static_cast<ColorSort>(0);  // INACTIVE
		}
		cycler.setColorSort(next);
	});

	// Score Lower - now color-sort aware
	controller.ButtonY.onPressed([]() { colorSortAwareOuttake(); });
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
	setupCycler();

	// constructTuningAuton();
	if (autonType == 0) {
		if (isPurpleRobot) {
			constructPurpleMatchAuton(isRedTeam);
		} else {
			constructRedMatchAuton(isRedTeam);
		}
	} else if (autonType == 1) {
		if (isPurpleRobot) {
			constructPurpleSkillsAuton();
		} else {
			constructRedSkillsAuton();
		}
	} else {
		if (isPurpleRobot) {
			constructPurpleAWPAuton(isRedTeam);
		} else {
			constructRedAWPAuton(isRedTeam);
		}
	}
}