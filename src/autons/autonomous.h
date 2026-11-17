#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include "main.h"
#include "../motors.h"
#include "../lift.h"

// Autonomous functions
void autonomousMain();
void autonomousLeft();
void autonomousSkills();
void autoTuneAngularPID();
void autoTuneLateralPID();


#endif // AUTONOMOUS_H