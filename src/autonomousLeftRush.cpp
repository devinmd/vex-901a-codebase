// clang-format off
#include "autonomous.h"
#include "motors.h"
#include "pros/rtos.hpp"


void autonomousLeftRush() {

  // set starting position on left side
  // chassis.setPose(-48.9,17, 78);
  chassis.setPose(-48.5, 16.8,0);
  tonguePiston.set_value(true);    // lifts tongue
  conveyorPiston.set_value(false); // raise conveyor

  const int ycoordinate = 48;

  // move to goal & align for loader
  chassis.moveToPoint(-48.5, ycoordinate, 1800, {.maxSpeed = 90}, false);
  chassis.turnToPoint(-58, ycoordinate, 500, {.maxSpeed = 80}, false);

  // move to loader, intake
  bottomIntake.move(127);
  pros::delay(300);
  chassis.moveToPoint(-60, ycoordinate, 950, {.maxSpeed = 85}, false);

  // move to long goal & score
  chassis.moveToPoint(-20, ycoordinate, 3000, {.forwards = false, .maxSpeed = 80}, true);
  pros::delay(1500);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(1500);
  fullIntake.move(0);

  const int ycooord = 38;

  // wing
  chassis.moveToPoint(-41, ycoordinate, 600, {.maxSpeed = 70}, false);
  chassis.turnToPoint(-32, ycooord, 200, {.forwards = false, .maxSpeed = 80}, false);
  chassis.moveToPoint(-32, ycooord, 800, {.forwards = false, .maxSpeed = 80}, false);
  chassis.turnToPoint(-7, ycooord, 200, {.forwards = false, .maxSpeed = 80}, false);
  chassis.moveToPoint(-7, ycooord, 3000, {.forwards = false, .maxSpeed = 70}, false);
}
