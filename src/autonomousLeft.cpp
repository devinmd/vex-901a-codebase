// clang-format off
#include "autonomous.h"
#include "motors.h"
#include "pros/rtos.hpp"


void autonomousLeft() {

  // set starting position on left side
  chassis.setPose(-48.36, 16.2, 77.89);
  tonguePiston.set_value(true);    // lifts tongue
  conveyorPiston.set_value(false); // raise conveyor

  // collect 3 middle balls (storage)
  // run intake
  bottomIntake.move(127);
  // move to 3 middle balls
  chassis.moveToPoint(-22.5, 22.5, 1800, {.maxSpeed = 90}, true);
  pros::delay(400);
  // drop tongue (catch last ball)
  tonguePiston.set_value(false);

 
  // turn to middle goal, move to it, & score
  chassis.turnToPoint(-6, 6, 500, { .forwards = false, .maxSpeed = 80, }, false);
  bottomIntake.move(0);
  chassis.moveToPoint(-6, 6, 1000, { .forwards = false, .maxSpeed = 70, }, false);
  conveyorPiston.set_value(true);
  bottomIntake.move(-127);
  pros::delay(150);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(1900);
  fullIntake.move(0);

  conveyorPiston.set_value(false); // raise

  // move to goal & align, then run intake & score
  chassis.moveToPoint(-47, 47, 1800, {.maxSpeed = 90}, false);
  chassis.turnToPoint(-58, 47, 500, {.maxSpeed = 80}, false);

  // move to loader, intake
  bottomIntake.move(127);
  pros::delay(300);
  chassis.moveToPoint(-58, 47, 1200, {.maxSpeed = 85}, false);

  // move to long goal & score
  chassis.moveToPoint(-24, 47, 3000, {.forwards = false, .maxSpeed = 80}, true);
  pros::delay(1500);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(1500);
  fullIntake.move(0);

  // wing
  chassis.moveToPoint(-39, 47, 600, {.maxSpeed = 70}, false);
  chassis.turnToPoint(-32, 36.75, 200, {.forwards = false, .maxSpeed = 80}, false);
  chassis.moveToPoint(-32, 36.75, 800, {.forwards = false, .maxSpeed = 80}, false);
  chassis.turnToPoint(-7, 36.75, 200, {.forwards = false, .maxSpeed = 80}, false);
  chassis.moveToPoint(-7, 36.75, 3000, {.forwards = false, .maxSpeed = 70}, false);
}
