#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include <queue>
#include <vector>

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

void opcontrolInit();
void robotInit();
void autonSelectorInit();
void autonSelectorRun();
void populateAutonQueue();

#endif