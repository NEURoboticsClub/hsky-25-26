#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include <vector>

#include "api.h"
#include "hskylib.h"

extern HskyController controller;
extern TankDrive tankdrive;
extern Transport intake;
extern Transport scoring;
extern Pneumatics mogoClamp;

void robotInit();

#endif