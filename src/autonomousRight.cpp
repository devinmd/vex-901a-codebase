
// clang-format off

#include "autonomous.h"
#include "motors.h"
#include "pros/rtos.hpp"


void autonomousRight() {
  // set starting position on right side
  chassis.setPose(-48.9, -17, 102);
  tonguePiston.set_value(true);    // lifts tongue
  conveyorPiston.set_value(false); // raise conveyor

  // collect 3 middle balls (storage)
  bottomIntake.move(127);
  chassis.moveToPoint(-22.5, -22.5, 2000, {.maxSpeed = 90}, true);
  pros::delay(400);
  tonguePiston.set_value(false); // drop tongue to catch last ball

  const int yloc = -48;

  // move to goal & align, then run intake & score
  chassis.turnToPoint(-47, yloc, 500, {.maxSpeed=80}, false);
  chassis.moveToPoint(-47, yloc, 1600, {.maxSpeed=90}, false);
  chassis.turnToPoint(-58, yloc, 800, {.maxSpeed=80}, false);

  // move to loader, intake
  bottomIntake.move(127);
  pros::delay(300);
  chassis.moveToPoint(-58, yloc, 1300, {.maxSpeed = 80}, false);

  // move to long goal & score
  chassis.moveToPoint(-24, yloc, 4000, {.forwards = false, .maxSpeed = 80}, true);
  pros::delay(1500); // at long goal
  
  fullIntake.move(-127);
  pros::delay(150); // at long goal run bcackward

  fullIntake.move(127);
  pros::delay(2500); // intake for 2500
  fullIntake.move(0);

  // wing
  const int yloc2 = -59;
  chassis.moveToPoint(-39, yloc, 600, {.maxSpeed = 80}, false);
  chassis.turnToPoint(-32, yloc2, 300, {.forwards = false, .maxSpeed = 80}, false);
  chassis.moveToPoint(-32, yloc2, 800, {.forwards = false, .maxSpeed = 80}, false);
  chassis.turnToPoint(-7, yloc2, 300, {.forwards = false, .maxSpeed = 80}, false);
  chassis.moveToPoint(-7, yloc2, 4000, {.forwards = false, .maxSpeed = 70}, false);
  
}
