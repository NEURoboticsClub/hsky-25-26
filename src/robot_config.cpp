#include "robot_config.hpp"

#include <hskylib/robot_specs.h>
#include <hskylib/subsystems/pneumatics.h>
#include <hskylib/ui/auton_selector.h>
#include <hskylib/utils/commands/base_commands.h>
#include <hskylib/utils/commands/command_runner.h>
#include <hskylib/utils/commands/drive_commands.h>
#include <hskylib/utils/utils.h>

#include <cmath>
#include <cstdio>

#include "hskylib/subsystems/cycler.h"
#include "hskylib/utils/color_reader.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

HskyController controller(pros::E_CONTROLLER_MASTER);

std::queue<Command *> commandQueue;
std::optional<pose_t> startPose;
bool isAutonomousRunning = false;
pros::Task *failsafeTask = nullptr;
CommandRunner *activeCommandRunner = nullptr;

// ---------------------------------------------------------
// ##################### Configuration #####################
// ---------------------------------------------------------
// Red starts on the left, Purple starts on the right
#define PURPLE_ROBOT  // RED_ROBOT, PURPLE_ROBOT
#define MATCH		  // MATCH, SKILLS, AWP
#define RED_TEAM	  // RED, BLUE

//---------------------------------------------------
// ##################### Robot 1 #####################
//---------------------------------------------------

#ifdef PURPLE_ROBOT
bool isPurpleRobot = true;

// //===================== CONFIG =====================

PIDFController drivePid(7.0, 0.0, 0, 0, PIDFController::ERROR_TYPE::LINEAR);
PIDFController turnPid(40, 0, 450, 16, PIDFController::ERROR_TYPE::ANGULAR);
PIDFController headingPid(50.0, 0, 0.0, 0, PIDFController::ERROR_TYPE::ANGULAR);

robot_specs_t robotConfig{.driveWheelDiameter = 2.58,
						  .trackWidth = 11.0,
						  .odomGearRatio = 1.0,
						  .maxDrivePct = 45,
						  .maxTurnPct = 50,
						  .drivePID = &drivePid,
						  .headingPID = &headingPid,
						  .turnPID = &turnPid};

// //===================== DEVICES =====================

pros::MotorGroup leftDriveMotors({10, -9, 8, -7, 6});
pros::MotorGroup rightDriveMotors({-1, 2, -3, 4, -5});

pros::IMU imu(13);

pros::MotorGroup intakeMotors({12, -20});
pros::MotorGroup scraperIntakeMotors({-17});
pros::MotorGroup upperScoringMotors({-18, 19});	 // 18 is fried

pros::adi::DigitalOut scraperCylinder('c');
pros::adi::DigitalOut hoodCylinder('b');
pros::adi::DigitalOut wingCylinder('a');
pros::adi::DigitalOut flapCylinder('h');

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
StartingSide defaultSide = StartingSide::RIGHT;
//===================== CONFIG =====================

PIDFController drivePid(7.0, 0.0, 0, 0, PIDFController::ERROR_TYPE::LINEAR);
PIDFController turnPid(40, 0, 450, 16, PIDFController::ERROR_TYPE::ANGULAR);
PIDFController headingPid(50.0, 0, 0.0, 0, PIDFController::ERROR_TYPE::ANGULAR);

robot_specs_t robotConfig{.driveWheelDiameter = 2.58,
						  .trackWidth = 11.0,
						  .odomGearRatio = 1.0,
						  .maxDrivePct = 45,
						  .maxTurnPct = 50,
						  .drivePID = &drivePid,
						  .headingPID = &headingPid,
						  .turnPID = &turnPid};
//===================== DEVICES =====================

pros::MotorGroup leftDriveMotors({17, -11, 7, -2, 10});
pros::MotorGroup rightDriveMotors({-21, 8, -9, 18, -19});

pros::IMU imu(15);

pros::MotorGroup intakeMotors({-5, -3});
pros::MotorGroup scraperIntakeMotors({-6});
pros::MotorGroup upperScoringMotors({-4, 1});

pros::adi::DigitalOut scraperCylinder('d');
pros::adi::DigitalOut hoodCylinder('c');
pros::adi::DigitalOut wingCylinder('b');
pros::adi::DigitalOut flapCylinder('a');

pros::Optical opticalSensor(12);
HskyColorReader colorReader(12);  // Port 21 same as opticalSensor

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
#else  // AWP
int autonType = 2;
#endif

//====================== UTILS ======================

void deviceInit() {
	pros::delay(1000);	// Allow time for devices to initialize
	odom.reset();
	odom.init();

	imu.reset();
	double startTime = pros::millis();
	while (imu.is_calibrating() || !std::isfinite(imu.get_heading()) &&
									   pros::millis() - startTime < 2000) {
		printf("Calibrating IMU");
		pros::delay(20);
	}
}

void intakeLoader() {
	flap.retractPiston();
	upperScoring.moveOut();
	scraper.extendPiston();
	scraperIntake.moveIn();

	ColorSort sortMode = cycler.getColorSort();
	int sortVal = static_cast<int>(sortMode);

	ColorType wrongColor = (sortVal == 1) ? ColorType::BLUE : ColorType::RED;
	ColorType correctColor = (sortVal == 1) ? ColorType::RED : ColorType::BLUE;
	uint32_t msSinceWrongColor =
		pros::millis() - colorReader.getLastDetectionTime(wrongColor);
	bool shouldEject =
		(sortVal != 0) && (msSinceWrongColor < LOADER_EJECT_WINDOW_MS);
	
	bool inRange = opticalSensor.get_proximity() > 40;
	bool correctOverride =
		(sortVal != 0) && (colorReader.getLastDetectionTime(correctColor) >
							colorReader.getLastDetectionTime(wrongColor)) &&
		(pros::millis() - colorReader.getLastDetectionTime(correctColor) <
		 LOADER_EJECT_WINDOW_MS);

	if (shouldEject && !correctOverride) {
		intake.moveOut();
		scraperIntake.moveIn();
		upperScoring.stop();
		printf("Ejecting wrong color ball from loader\n");
	} else {
		intake.moveIn();
		scraperIntake.moveIn();
		upperScoring.moveOut(70);
	}
}

void sort() {
	ColorSort sortMode = cycler.getColorSort();
	int sortVal = static_cast<int>(sortMode);

	ColorType wrongColor = (sortVal == 1) ? ColorType::BLUE : ColorType::RED;
	uint32_t msSinceWrongColor =
		pros::millis() - colorReader.getLastDetectionTime(wrongColor);

	constexpr uint32_t CORRECT_COLOR_OVERRIDE_MS = 10;
	constexpr uint32_t STOP_DELAY_MS = 50;
	constexpr double INTAKE_SPEED_THRESOLD_RPM = 10;
	ColorType correctColor = (sortVal == 1) ? ColorType::RED : ColorType::BLUE;
	uint32_t lastWrongTime = colorReader.getLastDetectionTime(wrongColor);
	uint32_t lastCorrectTime = colorReader.getLastDetectionTime(correctColor);
	// bool correctOverride = (lastCorrectTime + CORRECT_COLOR_OVERRIDE_MS >=
	// lastWrongTime);
	bool shouldEject =
		(sortVal != 0) && (msSinceWrongColor < FIELD_EJECT_WINDOW_MS);
	bool correctOverride =
		(sortVal != 0) && (lastCorrectTime > lastWrongTime) &&
		(pros::millis() - lastCorrectTime < CORRECT_COLOR_OVERRIDE_MS);

	if (sortVal == 0) {
		intake.moveIn();
		scraperIntake.moveIn();
		upperScoring.moveOut();
		return;
	}

	// Optional unstuck
	// TODO should check if there's the wrong color or nothing in the color
	// sensor area before trying)
	bool inRange = opticalSensor.get_proximity() > 35;
	bool isStopped = false;
	for (int i = 0; i < intakeMotors.size(); i++) {
		double intake_motor_speed = abs(intakeMotors.get_actual_velocity(i));
		if (intake_motor_speed < INTAKE_SPEED_THRESOLD_RPM) {
			isStopped = true;
			break;
		}
	}
	if (isStopped || (!inRange && shouldEject)) {
		intake.moveOut(35);
		pros::delay(10);
		return;
	}
	if (shouldEject) {
		intakeMotors.get_actual_velocity();
		upperScoring.stop();
		intake.stop();
		scraperIntake.moveOut();
		printf("Ejecting wrong color ball\n");
		pros::delay(STOP_DELAY_MS);
		intake.moveIn(80);
		upperScoring.moveIn();
	} else if (correctOverride) {
		intake.moveIn();
		scraperIntake.moveIn();
		upperScoring.moveOut();
		// printf("Overriding eject due to correct color detection\n");
	} else {
		intake.moveIn(80);
		scraperIntake.moveIn(60);
		upperScoring.moveOut();
	}
}

void intakeField() {
	flap.retractPiston();
	sort();
}

void ejectLower() {
	ColorSort sortMode = cycler.getColorSort();
	int sortVal = static_cast<int>(sortMode);

	ColorType wrongColor = (sortVal == 1) ? ColorType::BLUE : ColorType::RED;
	uint32_t msSinceWrongColor =
		pros::millis() - colorReader.getLastDetectionTime(wrongColor);

	constexpr uint32_t CORRECT_COLOR_OVERRIDE_MS = 10;
	constexpr uint32_t STOP_DELAY_MS = 50;
	constexpr double INTAKE_SPEED_THRESOLD_RPM = 10;
	ColorType correctColor = (sortVal == 1) ? ColorType::RED : ColorType::BLUE;
	uint32_t lastWrongTime = colorReader.getLastDetectionTime(wrongColor);
	uint32_t lastCorrectTime = colorReader.getLastDetectionTime(correctColor);
	// bool correctOverride = (lastCorrectTime + CORRECT_COLOR_OVERRIDE_MS >=
	// lastWrongTime);
	bool shouldEject =
		(sortVal != 0) && (msSinceWrongColor < FIELD_EJECT_WINDOW_MS);
	bool correctOverride =
		(sortVal != 0) && (lastCorrectTime > lastWrongTime) &&
		(pros::millis() - lastCorrectTime < CORRECT_COLOR_OVERRIDE_MS);

	if (sortVal == 0) {
		intake.moveOut();
		scraperIntake.stop();
		upperScoring.moveOut();
		return;
	}

	// Optional unstuck
	// TODO should check if there's the wrong color or nothing in the color
	// sensor area before trying)
	bool inRange = opticalSensor.get_proximity() > 35;
	
	// if (shouldEject) {
	// 	intake.moveIn();
	// 	scraperIntake.moveOut();
	// 	upperScoring.moveIn();
	// } else if (correctOverride) {
	// 	intake.moveOut(70);
	// 	scraperIntake.moveOut(40);
	// 	upperScoring.moveIn();
	// 	// printf("Overriding eject due to correct color detection\n");
	// } 
	// else {
	// 	intake.moveOut(70);
	// 	scraperIntake.moveOut(40);
	// 	upperScoring.moveIn();
	// }
		if (shouldEject) {
		intake.moveIn();
		scraperIntake.moveOut();
		upperScoring.moveIn();
	} else {
		intake.stop();
		scraperIntake.stop();
		upperScoring.stop();
	}
}

void scoreLong() {
	flap.extendPiston();
	sort();
}

void scoreLower() {
	scraper.retractPiston();
	upperScoring.moveIn();
	intake.moveOut();
	scraperIntake.moveOut(60);
}

void stopAll() {
	scraperIntake.stop();
	upperScoring.stop();
	scraperIntake.stop();
	intake.stop();
}

void constructTuningAuton() {
	startPose = pose_t(0, 0, 0 * std::numbers::pi / 180.0);
	commandQueue.push(new InstantCommand([&]() { imu.tare(); }));
	// commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 90.0,
	// 100000, 0, 1)); commandQueue.push(new TurnToHeading(driveBase, odom,
	// robotConfig, 0.0, 100000, 0, 1)); commandQueue.push(new
	// TurnToHeading(driveBase, odom, robotConfig, 180.0, 100000, 0, 1));
	// commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0.0,
	// 100000, 0, 1)); commandQueue.push(new DriveDeadReckon(driveBase, 20, 20,
	// 100000));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 48.0, 90.0, 100000));
	commandQueue.push(new TimeoutCommand(100000));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, -48.0,
	// 5500));

	// commandQueue.push(new InstantCommand([&]() { imu.tare(); }));
	// commandQueue.push(new TimeoutCommand(100));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, 10.0,
	// 99999)); commandQueue.push(new TimeoutCommand(1000));
	// commandQueue.push(new DriveDistance(driveBase, odom, robotConfig, -10.0,
	// 99999)); commandQueue.push(new TurnToHeading()) for (int i = 0; i < 4;
	// i++) { commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig,
	// 45, 1250)); commandQueue.push(new TimeoutCommand(1000));
	// commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0,
	// 1250)); commandQueue.push(new TimeoutCommand(1000));
	// commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 90,
	// 1250)); commandQueue.push(new TimeoutCommand(1000));
	// commandQueue.push(new TurnToHeading(driveBase, odom, robotConfig, 0,
	// 1250)); commandQueue.push(new TimeoutCommand(1000));
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
bool flapToggle = false;
bool hoodToggle = false;

StartingSide defaultSide = StartingSide::LEFT;
void opcontrolInit() {
	// Flap
	controller.ButtonX.onPressed([]() {
		flapToggle = !flapToggle;
		if (flapToggle) {
			flap.extendPiston();
		} else {
			flap.retractPiston();
		}
	});

	// Hood
	controller.ButtonY.onPressed([]() {
		hoodToggle = !hoodToggle;
		if (hoodToggle) {
			hood.extendPiston();
		} else {
			hood.retractPiston();
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

	// Score High
	controller.ButtonL2.onHold([]() { scoreLong(); });
	controller.ButtonL2.onReleased([]() { stopAll(); });

	// Score Low
	controller.ButtonR2.onPressed([]() { scoreLower(); });
	controller.ButtonR2.onReleased([]() { stopAll(); });

	// Intake Field
	controller.ButtonR1.onHold([]() { intakeField(); });
	controller.ButtonR1.onReleased([]() { stopAll(); });

	// Intake Loader
	controller.ButtonL1.onHold([]() { intakeLoader(); });
	controller.ButtonL1.onReleased([]() {
		stopAll();
		scraper.retractPiston();
	});

	// Color sort toggle
	controller.ButtonA.onPressed([]() {
		ColorSort current = cycler.getColorSort();
		int currentVal = static_cast<int>(current);
		ColorSort next;
		const char *label;

		// NO RED
		if (currentVal == 0) {				   // INACTIVE
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

	stopAll();
}

#elifdef RED_ROBOT
bool wingToggle = false;
bool hoodToggle = false;
bool flapToggle = false;

void opcontrolInit() {
	// Flap
	controller.ButtonX.onPressed([]() {
		flapToggle = !flapToggle;
		if (flapToggle) {
			flap.extendPiston();
		} else {
			flap.retractPiston();
		}
	});

	// Hood
	controller.ButtonR1.onPressed([]() {
		hoodToggle = !hoodToggle;
		if (hoodToggle) {
			hood.extendPiston();
		} else {
			hood.retractPiston();
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

	// Score High
	controller.ButtonR2.onHold([]() { scoreLong(); });
	controller.ButtonR2.onReleased([]() { stopAll(); });

	// Score Low
	controller.ButtonY.onPressed([]() { scoreLower(); });
	controller.ButtonY.onReleased([]() { stopAll(); });

	// Intake Field
	controller.ButtonL1.onHold([]() { intakeField(); });
	controller.ButtonL1.onReleased([]() { stopAll(); });

	// Intake Loader
	controller.ButtonL2.onHold([]() { intakeLoader(); });
	controller.ButtonL2.onReleased([]() {
		stopAll();
		scraper.retractPiston();
	});

	// Color sort toggle
	controller.ButtonA.onPressed([]() {
		ColorSort current = cycler.getColorSort();
		int currentVal = static_cast<int>(current);
		ColorSort next;
		const char *label;

		// NO RED
		if (currentVal == 0) {				   // INACTIVE
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

	stopAll();
}

#endif

void awp(AllianceColor color, StartingSide side) {
	double starting_offset = 9.435;
	double robot_length = 15.0;
	double robot_width = 13.0;
	double scraper_extension_in = 5.0;
	double load_time = 2000;

	double angle = 0;

	if (side == StartingSide::RIGHT) {
		angle = 180;
		startPose = pose_t(72 + robot_length / 2 + starting_offset, 17.5,
						   angle * std::numbers::pi / 180.0);
	} else if (side == StartingSide::LEFT) {
		angle = 0;
		startPose = pose_t(72 - robot_length / 2 - starting_offset, 17.5,
						   angle * std::numbers::pi / 180.0);
	}


	// Drive towards goal, turn, intake from loaer
	commandQueue.push(new InstantCommand([&]() {
		imu.tare();
		hood.extendPiston();
	}));
	commandQueue.push(new DriveDistance(
		driveBase, odom, robotConfig,
		-(48 - starting_offset - 0.5 * robot_length - 1), angle, 2000));
	commandQueue.push(new InstantCommand([&]() { hood.retractPiston(); }));
	commandQueue.push(new InstantCommand([&]() { scraper.extendPiston(); }));
	angle = 90;
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, angle, 1000));
	
	// Intake Loader
	commandQueue.push(new InstantCommand([&]() {
		cycler.setColorSort((color == AllianceColor::RED_ALLIANCE) ? ColorSort::RED: ColorSort::BLUE);
	}));
	commandQueue.push(new RepeatForTimeCommand([&]() { intakeLoader(); }, 100));
	commandQueue.push(new DriveDistance(
		driveBase, odom, robotConfig,
		-(24 - scraper_extension_in - 0.5 * robot_width), angle, 1000));
	commandQueue.push(new RepeatForTimeCommand([&]() { intakeLoader(); }, 1500));

	commandQueue.push(new DriveDistance(
		driveBase, odom, robotConfig, 3, angle, 500));
	commandQueue.push(new DriveDistance(
		driveBase, odom, robotConfig, -5, angle, 500));
	commandQueue.push(new RepeatForTimeCommand([&]() { intakeLoader(); }, 1000));
	commandQueue.push(new InstantCommand([&]() {cycler.setColorSort(ColorSort::INACTIVE);}));


	if (side == StartingSide::LEFT) {
		commandQueue.push(new DriveDistance(
			driveBase, odom, robotConfig, 12 - scraper_extension_in, angle, 2000));
	} else {
		commandQueue.push(new DriveDistance(
			driveBase, odom, robotConfig, 13 - scraper_extension_in, angle, 2000));
	}

	commandQueue.push(new InstantCommand([&]() {
		scraper.retractPiston();
		stopAll();
	}));

	// Turn to center goal, drive and sort out blocks
	angle = (side == StartingSide::LEFT) ? 45 : 135;
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, angle, 1000));

	if (side == StartingSide::LEFT) {
		commandQueue.push(new InstantCommand([&]() {flap.extendPiston();}));
	}

	// Drive to center
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 53.75, angle, 3000));

	// Score in center
	if (side == StartingSide::LEFT) {
		commandQueue.push(
			new DriveDistance(driveBase, odom, robotConfig, -1.5, angle, 1000));
		commandQueue.push(new RepeatForTimeCommand([&]() { scoreLong(); }, 3000));
	} else {
		commandQueue.push(new RepeatForTimeCommand([&]() { scoreLower(); }, 3000));
	}

	// Drive towards loader
	if (side == StartingSide::LEFT) {
		commandQueue.push(
			new DriveDistance(driveBase, odom, robotConfig, -49, angle, 3000));
	} else {
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -54, angle, 3000));
	}
	angle = 90;
	commandQueue.push(new InstantCommand([&]() { scraper.extendPiston(); }));
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, angle, 1000));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));

	// Intake Loader
	commandQueue.push(new InstantCommand([&]() {
		cycler.setColorSort((color == AllianceColor::RED_ALLIANCE) ? ColorSort::BLUE: ColorSort::RED);
	}));
	commandQueue.push(new RepeatForTimeCommand([&]() { intakeLoader(); }, 100));
	commandQueue.push(new DriveDistance(
		driveBase, odom, robotConfig,
		-(24 - scraper_extension_in - 0.5 * robot_width), angle, 1000));
	commandQueue.push(new RepeatForTimeCommand([&]() { intakeLoader(); }, 1500));

	commandQueue.push(new DriveDistance(
		driveBase, odom, robotConfig, 3, angle, 500));
	commandQueue.push(new DriveDistance(
		driveBase, odom, robotConfig, -5, angle, 500));
	commandQueue.push(new RepeatForTimeCommand([&]() { intakeLoader(); }, 1000));
	commandQueue.push(new InstantCommand([&]() {cycler.setColorSort(ColorSort::INACTIVE);}));

	// Drive and score on long goal
	commandQueue.push(new InstantCommand([&]() { hood.extendPiston(); }));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 50, angle, 2000));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -2.5, angle, 1000));
	commandQueue.push(new RepeatForTimeCommand([&]() { scoreLong(); }, 2000));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 5, angle, 1000));
	commandQueue.push(new RepeatForTimeCommand([&]() { scoreLong(); }, 2000));
	commandQueue.push(new TimeoutCommand(3000));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));
	commandQueue.push(new TimeoutCommand(100000));
}

void old_school(AllianceColor color, StartingSide side) {
	double starting_offset = 9.435;
	double robot_length = 15.0;
	double robot_width = 13.0;
	double scraper_extension_in = 5.0;
	double load_time = 2000;

	double angle = 0;

	if (side == StartingSide::RIGHT) {
		angle = 180;
		startPose = pose_t(72 + robot_length / 2 + starting_offset, 17.5,
						   angle * std::numbers::pi / 180.0);
	} else if (side == StartingSide::LEFT) {
		angle = 0;
		startPose = pose_t(72 - robot_length / 2 - starting_offset, 17.5,
						   angle * std::numbers::pi / 180.0);
	}

	// Drive towards goal, turn, intake from loaer
	commandQueue.push(new InstantCommand([&]() {
		imu.tare();
		hood.extendPiston();
	}));
	commandQueue.push(new DriveDistance(
		driveBase, odom, robotConfig,
		-(48 - starting_offset - 0.5 * robot_length - 2), angle, 2000));
	commandQueue.push(new InstantCommand([&]() { hood.retractPiston(); }));
	commandQueue.push(new InstantCommand([&]() { scraper.extendPiston(); }));
	angle = 90;
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, angle, 1000));
	
	// Intake Loader
	commandQueue.push(new InstantCommand([&]() {
		cycler.setColorSort((color == AllianceColor::RED_ALLIANCE) ? ColorSort::RED: ColorSort::BLUE);
	}));
	commandQueue.push(new RepeatForTimeCommand([&]() { intakeLoader(); }, 100));
	commandQueue.push(new DriveDistance(
		driveBase, odom, robotConfig,
		-(24 - scraper_extension_in - 0.5 * robot_width), angle, 1000));
	commandQueue.push(new RepeatForTimeCommand([&]() { intakeLoader(); }, 1500));

	commandQueue.push(new DriveDistance(
		driveBase, odom, robotConfig, 3, angle, 500));
	commandQueue.push(new DriveDistance(
		driveBase, odom, robotConfig, -5, angle, 500));
	commandQueue.push(new RepeatForTimeCommand([&]() { intakeLoader(); }, 1000));
	commandQueue.push(new InstantCommand([&]() {cycler.setColorSort(ColorSort::INACTIVE);}));

	// Drive and score on long goal
	commandQueue.push(new InstantCommand([&]() { hood.extendPiston(); }));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 50, angle, 2000));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -2.5, angle, 1000));
	commandQueue.push(new RepeatForTimeCommand([&]() { scoreLong(); }, 2000));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 5, angle, 500));
	commandQueue.push(new RepeatForTimeCommand([&]() { scoreLong(); }, 2000));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));

	// Intake Loader 2
	commandQueue.push(new InstantCommand([&]() {
		cycler.setColorSort((color == AllianceColor::RED_ALLIANCE) ? ColorSort::RED: ColorSort::BLUE);
	}));
	commandQueue.push(new RepeatForTimeCommand([&]() { intakeLoader(); }, 100));
	commandQueue.push(new DriveDistance(
		driveBase, odom, robotConfig,
		-50, angle, 1000));
	commandQueue.push(new RepeatForTimeCommand([&]() { intakeLoader(); }, 1500));

	commandQueue.push(new DriveDistance(
		driveBase, odom, robotConfig, 3, angle, 500));
	commandQueue.push(new DriveDistance(
		driveBase, odom, robotConfig, -5, angle, 500));
	commandQueue.push(new RepeatForTimeCommand([&]() { intakeLoader(); }, 1000));
	commandQueue.push(new InstantCommand([&]() {cycler.setColorSort(ColorSort::INACTIVE);}));

	// Drive and score on long goal
	commandQueue.push(new InstantCommand([&]() { hood.extendPiston(); }));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 50, angle, 2000));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, -2.5, angle, 1000));
	commandQueue.push(new RepeatForTimeCommand([&]() { scoreLong(); }, 2000));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 5, angle, 500));
	commandQueue.push(new RepeatForTimeCommand([&]() { scoreLong(); }, 2000));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));
	commandQueue.push(new TimeoutCommand(100000));
}

void ucf(AllianceColor color, StartingSide side) {
	double angle = 90;
	if (side == StartingSide::RIGHT) {
		startPose = pose_t(94, 8, angle * std::numbers::pi / 180.0);
	} else if (side == StartingSide::LEFT) {
		startPose = pose_t(94, 8, angle * std::numbers::pi / 180.0);
	}

	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 80, angle, 4000));
	angle = (side == StartingSide::LEFT) ? 315 : 225;
	commandQueue.push(
		new TurnToHeading(driveBase, odom, robotConfig, angle, 2000));
	commandQueue.push(
		new DriveDistance(driveBase, odom, robotConfig, 5, angle, 1000));

	// Score in center
	if (side == StartingSide::LEFT) {
		commandQueue.push(new InstantCommand([&]() { scoreLower(); }));
	} else {
		commandQueue.push(new InstantCommand([&]() { scoreLong(); }));
	}
	commandQueue.push(new TimeoutCommand(5000));
	commandQueue.push(new InstantCommand([&]() { stopAll(); }));
	commandQueue.push(new TimeoutCommand(100000));

}

AutonSelector selector;

void autonSelectorInit() {
	// Set default for quick testing - change this line as needed
	if (isPurpleRobot) {
		selector.setDefault(1, AllianceColor::BLUE_ALLIANCE, StartingSide::LEFT);
	} else {
		selector.setDefault(0, AllianceColor::BLUE_ALLIANCE, StartingSide::RIGHT);
	}

	selector.registerAuton(0, "AWP",
						   [](std::queue<Command *> &q, AllianceColor color,
							  StartingSide side) { awp(color, side); });

	selector.registerAuton(1, "Old",
						   [](std::queue<Command *> &q, AllianceColor color,
							  StartingSide side) { old_school(color, side); });

	selector.registerAuton(2, "UCF",
						   [](std::queue<Command *> &q, AllianceColor color,
							  StartingSide side) { ucf(color, side); });
}

void autonSelectorRun() { selector.run(); }

void populateAutonQueue() { selector.populateCommandQueue(commandQueue); }

void robotInit() {
	deviceInit();
	setupCycler();
}