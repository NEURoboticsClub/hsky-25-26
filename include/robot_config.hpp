#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include <pid_controller.h>
#include <vector>

#include "api.h"
#include "hskylib.h"
#include "robot_specs.h"

extern pros::Controller ctrl;
extern TankDrive tankdrive;
extern Transport intake;
extern Transport scoring;
extern Pneumatics mogoClamp;

void robotInit();

#endif