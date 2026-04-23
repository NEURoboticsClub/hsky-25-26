#include "robot_config.hpp"

#include <cstdio>
#include <hskylib/robot_specs.h>
#include <hskylib/subsystems/pneumatics.h>
#include <hskylib/ui/auton_selector.h>
#include <hskylib/utils/commands/base_commands.h>
#include <hskylib/utils/commands/command_runner.h>
#include <hskylib/utils/commands/drive_commands.h>
#include <hskylib/utils/utils.h>

#include "hskylib/utils/color_reader.h"
#include "hskylib/subsystems/cycler.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

HskyController controller(pros::E_CONTROLLER_MASTER);

std::queue<Command *> commandQueue;
std::optional<pose_t> startPose;
bool isAutonomousRunning = false;
pros::Task* failsafeTask = nullptr;
CommandRunner* activeCommandRunner = nullptr;

// ---------------------------------------------------------
// ##################### Configuration #####################
// ---------------------------------------------------------
// Red starts on the left, Purple starts on the right
#define PURPLE_ROBOT // RED_ROBOT, PURPLE_ROBOT
#define MATCH // MATCH, SKILLS, AWP
#define RED_TEAM // RED, BLUE

//---------------------------------------------------
// ##################### Robot 1 #####################
//---------------------------------------------------

#ifdef PURPLE_ROBOT
bool isPurpleRobot = true;

// //===================== CONFIG =====================

PIDFController drivePid(7.0, 0.0, 0, 0, PIDFController::ERROR_TYPE::LINEAR);
PIDFController turnPid(40, 0, 450, 16, PIDFController::ERROR_TYPE::ANGULAR);
PIDFController headingPid(50.0, 0, 0.0, 0, PIDFController::ERROR_TYPE::ANGULAR);

robot_specs_t robotConfig{.driveWheelDiameter = 2.75,
						  .trackWidth = 11.0,
						  .odomPodDiameter = 0.0,
						  .maxDrivePct = 45,
						  .maxTurnPct = 50,
						  .drivePID = &drivePid,
						  .headingPID = &headingPid,
						  .turnPID = &turnPid};

// //===================== DEVICES =====================

pros::MotorGroup leftDriveMotors({10, -9, 8, -7, 6});
pros::MotorGroup rightDriveMotors({-1, 2, -3, 4, -5});

pros::IMU imu(15);

pros::MotorGroup intakeMotors({12, 13, -20}); // 12 doesn't work currently
pros::MotorGroup scraperIntakeMotors({-17});
pros::MotorGroup upperScoringMotors({-18, 19}); // 18 is fried

pros::adi::DigitalOut scraperCylinder('a');
pros::adi::DigitalOut hoodCylinder('b');
pros::adi::DigitalOut wingCylinder('c');
pros::adi::DigitalOut flapCylinder('d');

pros::Optical opticalSensor(21);
HskyColorReader colorReader(21);  // Port 21 same as opticalSensor

// Cycler for color-sorted intake/outtake
Cycler cycler(colorReader);

// //==================== SUBSYSTEMS ====================
DrivebaseOdometry odom(&leftDriveMotors, &rightDriveMotors, robotConfig, &imu,
					   true);

TankDrive driveBase(leftDriveMotors, rightDriveMotors,
					pros::E_MOTOR_BRAKE_COAST, pros::E_MOTOR_GEAR_600, 1.0,
					0.75);
Transport intake(intakeMotors, 1, pros::E_MOTOR_BRAKE_COAST,
					  pros::E_MOTOR_GEAR_600);
Transport scraperIntake(scraperIntakeMotors, 1, pros::E_MOTOR_BRAKE_COAST,
                      pros::E_MOTOR_GEAR_600);
Transport upperScoring(upperScoringMotors, 1, pros::E_MOTOR_BRAKE_COAST,
                      pros::E_MOTOR_GEAR_600);


Pneumatics scraper(scraperCylinder);
Pneumatics hood(hoodCylinder);
Pneumatics wing(wingCylinder);
Pneumatics flap(flapCylinder);

const uint32_t LOADER_EJECT_WINDOW_MS = 1000;
const uint32_t FIELD_EJECT_WINDOW_MS = 50;

//---------------------------------------------------
// ##################### Robot 2 #####################
//---------------------------------------------------

#elifdef RED_ROBOT
bool isPurpleRobot = false;

//===================== CONFIG =====================

PIDFController drivePid(5.0, 0.0, 0, 0, PIDFController::ERROR_TYPE::LINEAR);
PIDFController turnPid(131, 0, 2, 0, PIDFController::ERROR_TYPE::ANGULAR);
PIDFController headingPid(100.0, 0.0, 0.0, 0, PIDFController::ERROR_TYPE::ANGULAR);

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
pros::MotorGroup scraperIntakeMotors({-6});
pros::MotorGroup upperScoringMotors({4, 3});

pros::adi::DigitalOut scraperCylinder('b');
pros::adi::DigitalOut hoodCylinder('a');
pros::adi::DigitalOut wingCylinder('c');
pros::adi::DigitalOut flapCylinder('a');

pros::Optical opticalSensor(15);

// //==================== SUBSYSTEMS ====================
DrivebaseOdometry odom(&leftDriveMotors, &rightDriveMotors, robotConfig, &imu,
					   true);

TankDrive driveBase(leftDriveMotors, rightDriveMotors,
					pros::E_MOTOR_BRAKE_COAST, pros::E_MOTOR_GEAR_600, 1.0,
					0.75);
Transport intake(intakeMotors, 1, pros::E_MOTOR_BRAKE_COAST,
					  pros::E_MOTOR_GEAR_600);
Transport scraperIntake(scraperIntakeMotors, 1, pros::E_MOTOR_BRAKE_COAST,
                      pros::E_MOTOR_GEAR_600);
Transport upperScoring(upperScoringMotors, 1, pros::E_MOTOR_BRAKE_COAST,
                      pros::E_MOTOR_GEAR_600);


Pneumatics scraper(scraperCylinder);
Pneumatics hood(hoodCylinder);
Pneumatics wing(wingCylinder);
Pneumatics flap(flapCylinder);

const uint32_t LOADER_EJECT_WINDOW_MS = 1000;
const uint32_t FIELD_EJECT_WINDOW_MS = 300;

#endif

// Set red or blue
#ifdef RED_TEAM
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
		printf("Calibrating IMU");
		pros::delay(20);
	}
}

void scoreLong() {
   hood.extendPiston();
   flap.extendPiston();
   upperScoring.moveOut();
   intake.moveIn();
   scraperIntake.moveIn();
}


void scoreUpper() {
   hood.retractPiston();
   flap.extendPiston();
   upperScoring.moveOut();//60
   intake.moveIn();
   scraperIntake.moveIn();
}


void scoreLower() {
   scraper.retractPiston();
   upperScoring.moveIn();
   intake.moveOut();
//    scraperIntake.moveOut();
}

void intakeLoader() {
	flap.retractPiston();
	upperScoring.moveOut();
	scraper.extendPiston();
	scraperIntake.moveIn(50);

	ColorSort sortMode = cycler.getColorSort();
	int sortVal = static_cast<int>(sortMode);

	ColorType wrongColor = (sortVal == 1) ? ColorType::BLUE : ColorType::RED;
	uint32_t msSinceWrongColor = pros::millis() - colorReader.getLastDetectionTime(wrongColor);
	bool shouldEject = (sortVal != 0) && (msSinceWrongColor < LOADER_EJECT_WINDOW_MS);

	if (shouldEject) {
		intake.moveOut();
		printf("Ejecting wrong color ball\n");
	} else {
		intake.moveIn();
	}
}


void intakeField() {
	flap.retractPiston();
	intake.moveIn();

	ColorSort sortMode = cycler.getColorSort();
	int sortVal = static_cast<int>(sortMode);

	ColorType wrongColor = (sortVal == 1) ? ColorType::BLUE : ColorType::RED;
	uint32_t msSinceWrongColor = pros::millis() - colorReader.getLastDetectionTime(wrongColor);

	constexpr uint32_t CORRECT_COLOR_OVERRIDE_MS = 10;
	ColorType correctColor = (sortVal == 1) ? ColorType::RED : ColorType::BLUE;
	uint32_t lastWrongTime = colorReader.getLastDetectionTime(wrongColor);
	uint32_t lastCorrectTime = colorReader.getLastDetectionTime(correctColor);
	bool correctOverride = (lastCorrectTime + CORRECT_COLOR_OVERRIDE_MS >= lastWrongTime);
	bool shouldEject = (sortVal != 0) && (msSinceWrongColor < FIELD_EJECT_WINDOW_MS)
					   && !correctOverride;

	if (shouldEject) {
		scraperIntake.moveOut();
		upperScoring.moveOut(50);
		printf("Ejecting wrong color ball\n");
	} else {
		scraperIntake.moveIn(60);
		upperScoring.moveOut();
	}
}


void stopAll() {
   scraperIntake.stop();
   upperScoring.stop();
   scraperIntake.stop();
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
	startPose = pose_t(0, 0, 0 * std::numbers::pi / 180.0);
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
	startPose = pose_t(0, 0, 0 * std::numbers::pi / 180.0);
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
	startPose = pose_t(0, 0, 0 * std::numbers::pi / 180.0);
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
	startPose = pose_t(0, 0, 0 * std::numbers::pi / 180.0);
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
	startPose = pose_t(0, 0, 0 * std::numbers::pi / 180.0);
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
	startPose = pose_t(0, 8, 0 * std::numbers::pi / 180.0);
	commandQueue.push(new InstantCommand([&]() { imu.tare(); }));
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 90.0, 100000, 0, 1));
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0.0, 100000, 0, 1));
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 180.0, 100000, 0, 1));
	commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0.0, 100000, 0, 1));
	// commandQueue.push(new DriveDeadReckon(driveBase, 20, 20, 100000));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, 48.0, 5500)); 
	commandQueue.push(new TimeoutCommand(100000));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, -48.0,
	// 5500));

	// commandQueue.push(new InstantCommand([&]() { imu.tare(); }));
	// commandQueue.push(new TimeoutCommand(100));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, 10.0, 99999));
	// commandQueue.push(new TimeoutCommand(1000));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, -10.0, 99999));
	// commandQueue.push(new TurnToHeading())
	// for (int i = 0; i < 4; i++) {
	// commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 45, 1250));
	// commandQueue.push(new TimeoutCommand(1000));
	// commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0, 1250));
	// commandQueue.push(new TimeoutCommand(1000));
	// commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 90, 1250));
	// commandQueue.push(new TimeoutCommand(1000));
	// commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0, 1250));
	// commandQueue.push(new TimeoutCommand(1000));
	// }
	// commandQueue.push(
	// 	new DriveDistance(driveBase, odom, robotConfig, 20.0, 1500));
}

void setupCycler() {
	// Register intake transports
	cycler.addIntakeTransport(&intake);

	// Register outtake transports for INACTIVE mode
	cycler.addOuttakeTransport(&upperScoring);
	cycler.addOuttakeTransport(&scraperIntake);

	// Register color-sorted outtake transports
	cycler.addCorrectColorOuttakeTransport(&upperScoring);
	cycler.addIncorrectColorOuttakeTransport(&scraperIntake);

	// Initialize the color reader (starts background task)
	colorReader.initialize();
}

#ifdef PURPLE_ROBOT
bool wingToggle = false;

void opcontrolInit() {
	printf("Initializing operator control...\n");
	// Score Upper
	controller.ButtonR1.onHold([]() { scoreUpper(); });
	controller.ButtonR1.onReleased([]() { stopAll(); });

	// Score Long
	controller.ButtonR2.onPressed([]() { scoreLong(); });
	controller.ButtonR2.onReleased([]() { stopAll(); });

	// Intake
	controller.ButtonL1.onHold([]() { intakeField(); });
	controller.ButtonL1.onReleased([]() { stopAll(); });

	// Intake
	controller.ButtonL2.onHold([]() { intakeLoader(); });
	controller.ButtonL2.onReleased([]() {
		stopAll();
		scraper.retractPiston();
	});

	// Color sort toggle
	controller.ButtonLeft.onPressed([]() {
		ColorSort current = cycler.getColorSort();
		int currentVal = static_cast<int>(current);
		ColorSort next;
		const char* label;

		if (currentVal == 0) {  // INACTIVE
			next = static_cast<ColorSort>(1);  // RED
			label = "SORT: RED";
			printf("Switched to RED color sort mode\n");
		} else if (currentVal == 1) {  // RED
			next = static_cast<ColorSort>(2);  // BLUE
			label = "SORT: BLUE";
			printf("Switched to BLUE color sort mode\n");
		} else {
			next = static_cast<ColorSort>(0);  // INACTIVE
			label = "SORT: OFF";
			printf("Switched to INACTIVE color sort mode\n");
		}
		cycler.setColorSort(next);

		controller.print(0, 0, "%-19s", label);
		pros::delay(55);
   		controller.rumble(".");
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

#elifdef RED_ROBOT
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

#endif

AutonSelector selector;

void autonSelectorInit() {
	// Set default for quick testing - change this line as needed
	selector.setDefault(3, AllianceColor::RED_ALLIANCE, StartingSide::LEFT);

	selector.registerAuton(0, "Match", [](std::queue<Command *> &q,
										  AllianceColor color,
										  StartingSide side) {
		bool isRed = (color == AllianceColor::RED_ALLIANCE);
		if (side == StartingSide::RIGHT) {
			constructPurpleMatchAuton(isRed);
		} else {
			constructRedMatchAuton(isRed);
		}
	});

	selector.registerAuton(1, "Skills", [](std::queue<Command *> &q,
										   AllianceColor color,
										   StartingSide side) {
		if (side == StartingSide::RIGHT) {
			constructPurpleSkillsAuton();
		} else {
			constructRedSkillsAuton();
		}
	});

	selector.registerAuton(2, "AWP", [](std::queue<Command *> &q,
										AllianceColor color,
										StartingSide side) {
		bool isRed = (color == AllianceColor::RED_ALLIANCE);
		if (side == StartingSide::RIGHT) {
			constructPurpleAWPAuton(isRed);
		} else {
			constructRedAWPAuton(isRed);
		}
	});

	selector.registerAuton(3, "Test", [](std::queue<Command *> &q,
										 AllianceColor color,
										 StartingSide side) {
		startPose = pose_t(0, 0, 0);

		// Color picks direction, side picks speed.
		// RED  + LEFT  -> forward  25%
		// RED  + RIGHT -> forward  75%
		// BLUE + LEFT  -> reverse  25%
		// BLUE + RIGHT -> reverse  75%
		bool isRed = (color == AllianceColor::RED_ALLIANCE);
		bool isRight = (side == StartingSide::RIGHT);
		int direction = isRed ? 1 : -1;
		int speed = isRight ? 75 : 25;
		int pct = direction * speed;

		commandQueue.push(new InstantCommand([pct, isRed, isRight]() {
			printf("Test auton: color=%s side=%s pct=%d\n",
				   isRed ? "RED" : "BLUE",
				   isRight ? "RIGHT" : "LEFT",
				   pct);
		}));
		commandQueue.push(new DriveDeadReckon(driveBase, pct, pct, 2000));
		commandQueue.push(new TimeoutCommand(500));
	});

}

void autonSelectorRun() {
	selector.run();
}

void populateAutonQueue() {
	selector.populateCommandQueue(commandQueue);
}

void robotInit() {
	deviceInit();
	setupCycler();
}