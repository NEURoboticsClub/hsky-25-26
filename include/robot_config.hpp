#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include <hskylib/utils/pose.h>
#include <vector>
#include <optional>

#include "api.h"
#include "hskylib.h"

extern HskyController controller;
extern TankDrive driveBase;
extern Transport intake;
extern Transport lowerScoring;
extern Transport upperScoring;
extern Pneumatics scraper;
extern Pneumatics hood;
extern DrivebaseOdometry odom;
extern pros::IMU imu;
extern std::queue<Command *> commandQueue;
extern std::optional<pose_t> startPose;

void opcontrolInit();
void robotInit();

#endif