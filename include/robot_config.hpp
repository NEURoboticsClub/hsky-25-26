#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include <hskylib/subsystems/odometry/odometry_drivebase.h>
#include <vector>

#include "api.h"
#include "hskylib.h"
#include "hskylib/subsystems/odometry/odometry.h"

extern HskyController controller;
extern TankDrive driveBase;
extern Transport intake;
extern Transport lowerScoring;
extern Transport upperScoring;
extern Pneumatics scraper;
extern Pneumatics hood;
extern DrivebaseOdometry odom;

void opcontrolInit();
void robotInit();

#endif