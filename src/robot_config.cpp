#include "robot_config.hpp"

#define ROBOT_1
// #define ROBOT_2

HskyController controller(pros::E_CONTROLLER_MASTER);

//----------------------------------------------------
// ##################### Robot 1 #####################
//----------------------------------------------------

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

pros::MotorGroup intakeMotorGroup({5});

//==================== SUBSYSTEMS ====================

Transport intake(intakeMotorGroup, 0.05, pros::E_MOTOR_BRAKE_COAST,
				 pros::E_MOTOR_GEAR_600);

//----------------------------------------------------
// ##################### Robot 2 #####################
//----------------------------------------------------

#elifdef ROBOT_2

//===================== CONFIG =====================

//===================== DEVICES =====================

//==================== SUBSYSTEMS ====================

#endif

//====================== UTILS ======================

void deviceInit() {}

void robotInit() { deviceInit(); }

void opcontrolInit() {
	controller.ButtonB.onHold([]() { intake.moveOut(); });
	controller.ButtonB.onReleased([]() { intake.stop(); });

	controller.initialize();
}