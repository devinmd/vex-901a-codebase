
// clang-format off

#include "autonomous.h"
#include "../motors.h"
#include "pros/rtos.hpp"


void autonomousMain() {
  // set starting position on right side
  chassis.setPose(-48.9, -17, 102);
  chassis.moveToPoint(-22.5, -22.5, 2000, {.maxSpeed = 90}, true);
  pros::delay(400);
}
