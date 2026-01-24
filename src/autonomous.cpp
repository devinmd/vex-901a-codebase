// clang-format off
#include "autonomous.h"
#include "motors.h"
#include "pros/rtos.hpp"

/*
forward is the tongue side
*/

/*

TODO

figure out good speed for the long distance moves
figure out good speed for short distance moves

figure out good speed for turns
figure out good timeout for turns

*/

void tuneLateralPID() {
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 48, 10000, {.forwards = true});
}

void tuneAngularPID() {
  chassis.setPose(0, 0, 0);
  chassis.turnToHeading(90, 10000);
}


// SKILLS AUTON (JUST PARK AND CLEAR)
void autonomousSkillsPark() {

  chassis.setPose(-61.7, -17, 0);
  tonguePiston.set_value(true);    // lifts tongue
  conveyorPiston.set_value(false); // raise conveyor

  bottomIntake.move(127); // run intake
  // move forward and clear park zone
  chassis.moveToPoint(-61.7, 1, 5000, {.maxSpeed = 70}, false);
}
