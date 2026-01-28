// clang-format off
#include "autonomous.h"
#include "motors.h"
#include "pros/rtos.hpp"


void autonomousLeft() {

  // set starting position on left side
  chassis.setPose(-48.9,17, 0);
  tonguePiston.set_value(true);    // lifts tongue
  conveyorPiston.set_value(false); // raise conveyor

  const int ycoordinate = 47;

  // move to goal & align for loader
  chassis.moveToPoint(-47, ycoordinate, 1800, {.maxSpeed = 90}, false);
  chassis.turnToPoint(-58, ycoordinate, 500, {.maxSpeed = 80}, false);

  // move to loader, intake
  bottomIntake.move(127);
  pros::delay(300);
  chassis.moveToPoint(-60, ycoordinate, 1000, {.maxSpeed = 85}, false);

  // move to long goal & score
  chassis.moveToPoint(-20, ycoordinate, 3000, {.forwards = false, .maxSpeed = 80}, true);
  pros::delay(1500);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(1500);
  fullIntake.move(0);

  // wing
  chassis.moveToPoint(-41, ycoordinate, 600, {.maxSpeed = 70}, false);
  chassis.turnToPoint(-32, 36.75, 200, {.forwards = false, .maxSpeed = 80}, false);
  chassis.moveToPoint(-32, 36.75, 800, {.forwards = false, .maxSpeed = 80}, false);
  chassis.turnToPoint(-7, 36.75, 200, {.forwards = false, .maxSpeed = 80}, false);
  chassis.moveToPoint(-7, 36.75, 3000, {.forwards = false, .maxSpeed = 70}, false);

  wingPiston.set_value(true); // lift wing

  // go to mid 3 balls
  chassis.turnToPoint(-22.5, 22.5, 400, {.maxSpeed = 90}, false);
  chassis.moveToPoint(-22.5, 22.5, 1800, {.maxSpeed = 90}, true);
  pros::delay(200);
  tonguePiston.set_value(false); // drop tongue (catch last ball)

  // turn to middle goal, move to it, & score
  chassis.turnToPoint(-7, 7, 700, { .forwards = false, .maxSpeed = 90, }, false);
  bottomIntake.move(0);
  chassis.moveToPoint(-7, 7, 1000, { .forwards = false, .maxSpeed = 70, }, false);
  conveyorPiston.set_value(true);
  bottomIntake.move(-127);
  pros::delay(150);
  bottomIntake.move(0);
  fullIntake.move(100);  // run a little slower
  pros::delay(1900);
  fullIntake.move(0);

  conveyorPiston.set_value(false); // raise
}
