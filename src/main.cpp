#include "main.h"

#include "robot_config.hpp"
#include <hskylib/utils/commands/base_commands.h>
#include <hskylib/utils/commands/command_runner.h>
#include <hskylib/utils/commands/drive_commands.h>
#include <queue>

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() { robotInit(); }

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	isAutonomousRunning = false;

	if (failsafeTask != nullptr) {
		failsafeTask->remove();
		delete failsafeTask;
		failsafeTask = nullptr;
	}
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	CommandRunner commandRunner(commandQueue);

	if (!(autonType == 1)) { // NOT SKILLS
		isAutonomousRunning = true;

		failsafeTask = new pros::Task([&commandRunner]() {
			pose_t* pose = new pose_t();
			while (isAutonomousRunning) {
				odom.getPose(pose);
				if (pose->y >= 50.0) {
					commandRunner.stop();
					printf("Failsafe triggered: y = %f\n", pose->y);
					break;
				}
				pros::delay(20);
			}
			delete pose;
		});
	}

	commandRunner.run();
	isAutonomousRunning = false;
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	opcontrolInit();

	controller.initialize();


	while (true) {
		driveBase.arcadeDrive(controller.AxisLeftY.position(), controller.AxisRightX.position());
		pros::delay(20);
	}
}