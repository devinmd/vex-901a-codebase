#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include "main.h"
#include "motors.h"

// Autonomous functions
void autonomousSkills();
void autonomousSkillsPark();
void autonomousRight(); // Right side
void autonomousLeft(); // Left side -- mid, match load, long goal, wing
void autonomousLeft2(); // Left side -- match load, long goal, wing, mid
void autonomousLeft3(); // Left side -- match load, long goal, mid, wing
void tuneAngularPID();
void tuneLateralPID();
void autonomousSoloAWP();

#endif // AUTONOMOUS_H