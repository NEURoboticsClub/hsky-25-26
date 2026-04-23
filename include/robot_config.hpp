#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include <queue>
#include <vector>
#include <optional>

#include <hskylib/utils/commands/command_runner.h>
#include <hskylib/utils/pose.h>

#include "api.h"
#include "hskylib.h"

extern HskyController controller;
extern TankDrive driveBase;
extern Transport intake;
extern Transport lowerScoring;
extern Transport upperScoring;
extern Pneumatics scraper;
extern Pneumatics hood;
extern Pneumatics wing;
extern DrivebaseOdometry odom;
extern pros::IMU imu;
extern robot_specs_t robotConfig;
extern std::queue<Command *> commandQueue;
extern std::optional<pose_t> startPose;
extern bool isAutonomousRunning;
extern pros::Task* failsafeTask;
extern CommandRunner* activeCommandRunner;
extern int autonType;

void robotInit();
void opcontrolInit();
void autonSelectorInit();
void autonSelectorRun();
void populateAutonQueue();

#endif
