// clang-format off
#include "autonomous.h"
#include "../motors.h"
#include "pros/rtos.hpp"

void autonomousSkills() {  

  chassis.setPose(-48.5, 16.8,0);
  chassis.moveToPoint(-47, 47, 1800, {.maxSpeed = 90}, false);
  chassis.turnToPoint(-60, 47, 700, {.maxSpeed = 80}, false);
}