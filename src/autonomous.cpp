#include "autonomous.h"
#include "motors.h"

/*
forward is the tongue side
*/

void autonomousSkills() {
  int leftLongGoalY = 49;

  // starting position
  chassis.setPose(-48, 15, 0);
  // drive to align with matchloader
  chassis.moveToPose(-48, leftLongGoalY, 270, 1000,
                     {.forwards = true, .maxSpeed = 90}, false);
  // drop tongue
  tonguePiston.set_value(false);
  bottomIntake.move(127);
  pros::delay(200);
  // move into the match load
  chassis.moveToPoint(-62, leftLongGoalY, 2000,
                      {.forwards = true, .maxSpeed = 60}, false);
  // back out
  chassis.moveToPose(-48, leftLongGoalY, 1000, 270,
                     {.forwards = false, .maxSpeed = 90}, false);

  int leftOutsideLongGoalY = 60;

  // raise tongue
  tonguePiston.set_value(true);
  // drive to other side
  chassis.moveToPose(-36, leftOutsideLongGoalY, 90, 1000,
                     {.forwards = false, .maxSpeed = 90}, false);
  chassis.moveToPose(36, leftOutsideLongGoalY, 135, 1000,
                     {.forwards = false, .maxSpeed = 90}, false);
  chassis.moveToPose(48, leftLongGoalY, 270, 1000,
                     {.forwards = false, .maxSpeed = 90}, false);
  // drive into long goal & score

  // move to match load
  // drop tongue
  tonguePiston.set_value(false);
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

// RIGHT SIDE AUTON
void autonomousRight() {
  // set starting position on right side
  chassis.setPose(-48.36, -16.2, 102.3);
  tonguePiston.set_value(true);    // lifts tongue
  conveyorPiston.set_value(false); // raise conveyor

  // collect 3 middle balls (storage)
  bottomIntake.move(127);
  chassis.moveToPoint(-23.5, -21.6, 2000, {.maxSpeed = 60}, false);
  bottomIntake.move(0);

  int ylocation = -47;

  // move to goal & align, then run intake & score
  chassis.turnToPoint(-42, ylocation, 500, {.maxSpeed = 60}, false);
  chassis.moveToPoint(-42, ylocation, 2000, {.maxSpeed = 80}, false);
  chassis.turnToPoint(-60, ylocation, 300, {.maxSpeed = 40}, false);

  // drop tongue, move to loader, intake
  tonguePiston.set_value(false);
  bottomIntake.move(127);
  pros::delay(200);
  // move into the match load
  chassis.moveToPoint(-62, ylocation, 2000, {.forwards = true, .maxSpeed = 50},
                      false);
  // drive into long goal
  chassis.moveToPoint(-22, ylocation, 5000, {.forwards = false, .maxSpeed = 70},
                      true);
  pros::delay(1000);
  // start scoring
  fullIntake.move(127);
  pros::delay(3000);
  fullIntake.move(0);

  tonguePiston.set_value(true);

  // use wing

  // move backward then punch
  chassis.moveToPoint(-39, ylocation, 1000, {.maxSpeed = 60}, false);

  chassis.moveToPoint(-22, ylocation, 500, {.forwards = false, .maxSpeed = 100},
                      true);

  /*
    chassis.turnToPoint(-26.5, -58, 500, {.forwards = false, .maxSpeed = 70},
                        false);

    chassis.moveToPoint(-26.53, -58, 2000, {.forwards = true, .maxSpeed = 70},
                        false);

    chassis.turnToPoint(-10.2, -58, 500, {.forwards = true, .maxSpeed = 60},
                        false);
    chassis.moveToPoint(-10.2, -58, 2000, {.forwards = true, .maxSpeed = 60},
                        false);*/
}

// LEFT SIDE -- THIS WORKS
void autonomousLeft() {

  // set starting position on left side
  chassis.setPose(-48.36, 16.2, 77.89);
  tonguePiston.set_value(true);    // lifts tongue
  conveyorPiston.set_value(false); // raise conveyor

  // collect 3 middle balls (storage)
  // run intake
  bottomIntake.move(127);
  // move to 3 middle balls
  chassis.moveToPoint(-23.5, 21.6, 2000, {.maxSpeed = 60}, false);
  // stop intake -- DO WE NEED THIS? BETTER TO CYCLE BALLS?
  bottomIntake.move(0);

  // go to mid-top goal
  // drop conveyor -- MAYBE DROP AFTER WE ARE AT GOAL?
  conveyorPiston.set_value(true);
  // turn to middle goal
  chassis.turnToPoint(-6.6, 8.3, 500,
                      {
                          .forwards = false,
                          .maxSpeed = 60,
                      },
                      false);
  // move to middle goal
  chassis.moveToPoint(-6.6, 8.3, 1000,
                      {
                          .forwards = false,
                          .maxSpeed = 60,
                      },
                      false);
  // drop conveyor -- DROP HERE????? -- TEST THIS
  conveyorPiston.set_value(true);
  // run intake to score balls to middle goal
  fullIntake.move(127);
  pros::delay(2000);

  // raise conveyor
  conveyorPiston.set_value(false);
  // stop intake
  fullIntake.move(0);

  int ylocation = 50;

  // move to mid position
  chassis.turnToPoint(-42, ylocation, 500, {.maxSpeed = 100}, false);
  chassis.moveToPoint(-42, ylocation, 2000, {.maxSpeed = 70},
                      false); // 60 speed works
  chassis.turnToPoint(-60, ylocation, 300, {.maxSpeed = 40}, false);

  // drop tongue
  tonguePiston.set_value(false);
  // run in take
  bottomIntake.move(127);
  // give time for tongue to drop
  pros::delay(200);
  // move into the match load and get balls
  chassis.moveToPoint(-62, ylocation, 2000, {.forwards = true, .maxSpeed = 60},
                      false);

  // move to long goal
  chassis.moveToPoint(-22, ylocation, 3000, {.forwards = false, .maxSpeed = 70},
                      true);
  // wait until bot gets to long goal
  pros::delay(1000);
  // score long goal
  fullIntake.move(127);
  pros::delay(2000);
  fullIntake.move(0);

  // move back then come back and punch
  chassis.moveToPoint(-39, ylocation, 500, {.maxSpeed = 60}, false);
  chassis.moveToPoint(-22, ylocation, 500, {.forwards = false, .maxSpeed = 100},
                      true);
}