#include "robot_config.hpp"

#define ROBOT_1

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

pros::MotorGroup intakeMotorGroup({5});

//==================== SUBSYSTEMS ====================

Transport intake(intakeMotorGroup, 1.0, pros::E_MOTOR_BRAKE_COAST,
				 pros::E_MOTOR_GEAR_600);

//---------------------------------------------------
// ##################### Robot 2 #####################
//---------------------------------------------------

#elifdef ROBOT_2

//===================== CONFIG =====================

PIDController<Pose>::pid_config_t drivePIDConfig{.kP = 0.0,
												 .kI = 0.0,
												 .kD = 0.0,

												 .deadband = 0.0};
PIDController<double>::pid_config_t turnPIDConfig{.kP = 0.0,
												  .kI = 0.0,
												  .kD = 0.0,

												  .deadband = 0.0};

PIDController<Pose> drivePID(drivePIDConfig);
PIDController<double> turnPID(turnPIDConfig);

robot_specs_t robotConfig{.driveWheelDiam = 0.0,
						  .trackWidth = 0.0,
						  .odomWheelDiam = 0.0,

						  .maxDrivePct = 0,
						  .maxTurnPct = 0,

						  .drivePID = &drivePID,
						  .turnPID = &turnPID};

//===================== DEVICES =====================

pros::MotorGroup leftMotorGroup({-10, -1, 18});
pros::MotorGroup rightMotorGroup({4, 5, -2});

pros::MotorGroup intakeMotorGroup({-16, 6});

pros::MotorGroup scoringMotorGroup({9, -14});

pros::adi::DigitalOut mogoClampCylinder('a');

pros::Rotation xEncoder(5);
pros::Rotation yEncoder(-10);

pros::IMU imu1(3);
pros::IMU imu2(7);

//==================== SUBSYSTEMS ====================

PIDController<Pose> drivePIDMove(1.0, 0.0, 0.1);
PIDController<double> drivePIDAngular(2, 0.05, 0.1);

OdometryPerpendicularIMU odom(&xEncoder, &yEncoder, &imu1, &imu2);

TankDrive tankdrive(ctrl, leftMotorGroup, rightMotorGroup, DriveStyle::ARCADE,
					pros::E_MOTOR_BRAKE_COAST, pros::E_MOTOR_GEAR_600, 0.75,
					&odom, &drivePIDMove, &drivePIDAngular);

Transport intake(ctrl, intakeMotorGroup, 1.0, pros::E_CONTROLLER_DIGITAL_L1,
				 pros::E_CONTROLLER_DIGITAL_L2, pros::E_MOTOR_BRAKE_COAST,
				 pros::E_MOTOR_GEAR_600);

Transport scoring(ctrl, scoringMotorGroup, 0.25, pros::E_CONTROLLER_DIGITAL_R1,
				  pros::E_CONTROLLER_DIGITAL_R2, pros::E_MOTOR_BRAKE_COAST,
				  pros::E_MOTOR_GEAR_600);

Pneumatics mogoClamp(ctrl, mogoClampCylinder, pros::E_CONTROLLER_DIGITAL_B,
					 pros::E_CONTROLLER_DIGITAL_X);

#endif

//====================== UTILS ======================

void deviceInit() {}

void opcontrolInit() {

	controller.ButtonB.onPressed([] {
		intake.moveOut();
	});
}

void robotInit() {
	deviceInit();
}