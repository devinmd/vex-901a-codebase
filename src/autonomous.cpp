// clang-format off

#include "autonomous.h"
#include "motors.h"

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

void testSpeed(){

  //
  const int moveSpeed = 127;
  const int rotateSpeed = 127;

  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 48, 2000, {.forwards = true, .maxSpeed=moveSpeed});
  
  pros::delay(1000);
  
  chassis.setPose(0, 0, 0);
  chassis.turnToHeading(90, 500, {.maxSpeed=rotateSpeed});

  chassis.setPose(0, 0, 0);
  chassis.turnToHeading(90, 500, {.maxSpeed=rotateSpeed});
}

void tuneLateralPID() {
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 48, 10000, {.forwards = true});
}

void tuneAngularPID() {
  chassis.setPose(0, 0, 0);
  chassis.turnToHeading(90, 10000);
}

void autonomousSkills() {

  // starting position
  chassis.setPose(-48, 15, 0);

  const int leftLongGoalY = 48;

  // drive to align with matchloader (midpoint)
  chassis.moveToPoint(-48, 48, 1000, {.forwards=true, .maxSpeed=70}, false);

  // drop tongue & start storage intake
  tonguePiston.set_value(false);
  bottomIntake.move(127);
  pros::delay(200);

  // move into the match load
  chassis.turnToPoint(-62, 48, 500, {.forwards=true, .maxSpeed=70}, false);
  chassis.moveToPoint(-62, 48, 1000, {.forwards=true, .maxSpeed=70}, false);

  // back out
  chassis.moveToPoint(-48, 48, 1000, {.forwards=false, .maxSpeed=70}, false);

  // raise tongue
  tonguePiston.set_value(true);

  const int leftOutsideLongGoalY = 62;
 
  // drive to other side
  chassis.turnToPoint(-36, 62, 500, {.forwards=false, .maxSpeed=70}, false);
  chassis.moveToPoint(-36, 62, 1000, {.forwards=false, .maxSpeed=70}, false);
  chassis.turnToPoint(36, 62, 500, {.forwards=false, .maxSpeed=70}, false);
  chassis.moveToPoint(36, 62, 1000, {.forwards=false, .maxSpeed=70}, false);
  chassis.turnToPoint(36, 48, 500, {.forwards=false, .maxSpeed=70}, false);
  chassis.moveToPoint(36, 48, 1000, {.forwards=false, .maxSpeed=70}, false);

  // drive into long goal & score
  chassis.turnToPoint(20, 48, 500, {.forwards=false, .maxSpeed=70}, false);
  chassis.moveToPoint(20, 48, 1000, {.forwards=false, .maxSpeed=70}, false);
  // TODO add scoring here

  // TODO reset position in long goal
  chassis.setPose(30, 48, 90);

  // drop tongue
  tonguePiston.set_value(false);

  // back out to match loader
  chassis.turnToPoint(62, 48, 500, {.forwards=true, .maxSpeed=70}, false);
  chassis.moveToPoint(62, 48, 1000, {.forwards=true, .maxSpeed=70}, false);

  // move back into goal and score
  chassis.turnToPoint(20, 48, 500, {.forwards=false, .maxSpeed=70}, false);
  chassis.moveToPoint(20, 48, 1000, {.forwards=false, .maxSpeed=70}, false);

  // raise tongue
  tonguePiston.set_value(true);

  // back out of long goal
  chassis.turnToPoint(48, 48, 500, {.forwards=true, .maxSpeed=70}, false);
  chassis.moveToPoint(48, 48, 1000, {.forwards=true, .maxSpeed=70}, false);

  // move to align with park
  chassis.turnToPoint(64, 24, 500, {.forwards=true, .maxSpeed=70}, false);
  chassis.moveToPoint(64, 24, 1000, {.forwards=true, .maxSpeed=70}, false);
  // TODO: RUN INTAKE

  // go through park
  chassis.turnToPoint(64, -17.4, 500, {.forwards=true, .maxSpeed=70}, false);
  chassis.moveToPoint(64, -17.4, 1000, {.forwards=true, .maxSpeed=70}, false);

  // go toward mid goal
  chassis.turnToPoint(24, -24, 500, {.forwards=true, .maxSpeed=70}, false);
  chassis.moveToPoint(24, -24, 1000, {.forwards=true, .maxSpeed=70}, false);

  // go into mid goal
  chassis.turnToPoint(10, -10, 500, {.forwards=false, .maxSpeed=70}, false);
  chassis.moveToPoint(10, -10, 1000, {.forwards=false, .maxSpeed=70}, false);
  // TODO: score mid goal

  // move out
  chassis.turnToPoint(48, -48, 500, {.forwards=true, .maxSpeed=70}, false);
  chassis.moveToPoint(48, -48, 1000, {.forwards=true, .maxSpeed=70}, false);

  // go into match loader
  chassis.turnToPoint(62, -48, 500, {.forwards=true, .maxSpeed=70}, false);
  chassis.moveToPoint(62, -48, 1000, {.forwards=true, .maxSpeed=70}, false);
  // TOOD: tongue & intake

  // back out
  chassis.turnToPoint(48, -48, 500, {.forwards=false, .maxSpeed=70}, false);
  chassis.moveToPoint(48, -48, 1000, {.forwards=false, .maxSpeed=70}, false);

  // drive to other side
  chassis.turnToPoint(36, -62, 500, {.forwards=false, .maxSpeed=70}, false);
  chassis.moveToPoint(36, -62, 1000, {.forwards=false, .maxSpeed=70}, false);
  chassis.turnToPoint(-36, -62, 500, {.forwards=false, .maxSpeed=70}, false);
  chassis.moveToPoint(-36, -62, 1000, {.forwards=false, .maxSpeed=70}, false);
  chassis.turnToPoint(-36, -48, 500, {.forwards=false, .maxSpeed=70}, false);
  chassis.moveToPoint(-36, -48, 1000, {.forwards=false, .maxSpeed=70}, false);

  // drive into long goal
  chassis.turnToPoint(-20, -48, 500, {.forwards=false, .maxSpeed=70}, false);
  chassis.moveToPoint(-20, -48, 1000, {.forwards=false, .maxSpeed=70}, false);
  // TODO: score
  // TODO: rest position
  // TODO: tongue

  // drive into match loader
  chassis.turnToPoint(-62, -48, 500, {.forwards=true, .maxSpeed=70}, false);
  chassis.moveToPoint(-62, -48, 1000, {.forwards=true, .maxSpeed=70}, false);
  // TODO: intake

  // drive back to long goal
  chassis.turnToPoint(-20, -48, 500, {.forwards=false, .maxSpeed=70}, false);
  chassis.moveToPoint(-20, -48, 1000, {.forwards=false, .maxSpeed=70}, false);

  // back out
  chassis.turnToPoint(-48, -48, 500, {.forwards=true, .maxSpeed=70}, false);
  chassis.moveToPoint(-48, -48, 1000, {.forwards=true, .maxSpeed=70}, false);

  // align with park
  chassis.turnToPoint(-64, -24, 500, {.forwards=true, .maxSpeed=70}, false);
  chassis.moveToPoint(-64, -24, 1000, {.forwards=true, .maxSpeed=70}, false);

  // park 
  chassis.turnToPoint(-64, 2, 500, {.forwards=true, .maxSpeed=70}, false);
  chassis.moveToPoint(-64, 2, 1000, {.forwards=true, .maxSpeed=70}, false);

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
  chassis.setPose(-48.36, -16.2, 103.692);
  tonguePiston.set_value(true);    // lifts tongue
  conveyorPiston.set_value(false); // raise conveyor

  // collect 3 middle balls (storage)
  bottomIntake.move(127);
  chassis.moveToPoint(-22.5, -22.5, 2000, {.maxSpeed = 50}, false);
  bottomIntake.move(0);

  // move to goal & align, then run intake & score
  chassis.turnToPoint(-47, -47, 500, {.maxSpeed=80}, false);
  chassis.moveToPoint(-47, -47, 1600, {.maxSpeed=90}, false);
  chassis.turnToPoint(-58, -47, 800, {.maxSpeed=80}, false);

  // drop tongue, move to loader, intake
  bottomIntake.move(127);
  tonguePiston.set_value(false);
  pros::delay(400);
  chassis.moveToPoint(-58, -47, 1500, {.maxSpeed = 75}, false);

  // move to long goal & score
  chassis.moveToPoint(-24, -47, 4000, {.forwards = false, .maxSpeed = 80}, true);
  pros::delay(1500);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(2500);
  fullIntake.move(0);

  // wing
  chassis.moveToPoint(-39, -47, 1000, {.maxSpeed = 70}, false);
  chassis.turnToPoint(-32, -56.75, 600, {.forwards = false, .maxSpeed = 80}, false);
  chassis.moveToPoint(-32, -56.75, 1500, {.forwards = false, .maxSpeed = 70}, false);
  chassis.turnToPoint(-7, -56.75, 600, {.forwards = false, .maxSpeed = 80}, false);
  chassis.moveToPoint(-7, -56.75, 3000, {.forwards = false, .maxSpeed = 80}, false);
  
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
  // drop tongue (catch the last ball)
  tonguePiston.set_value(false);

  // go to mid-top goal
  // drop conveyor -- MAYBE DROP AFTER WE ARE AT GOAL?
  conveyorPiston.set_value(true);
  // turn to middle goal
  chassis.turnToPoint(-6.6, 8.3, 500, { .forwards = false, .maxSpeed = 60, }, false);
  // move to middle goal
  chassis.moveToPoint(-6.6, 8.3, 1000, { .forwards = false, .maxSpeed = 60, }, false);

  // drop conveyor -- DROP HERE????? -- TEST THIS
  conveyorPiston.set_value(true);
  // run intake to score balls to middle goal
  fullIntake.move(127);
  pros::delay(2000);

  // raise conveyor
  conveyorPiston.set_value(false);
  // stop intake
  fullIntake.move(0);

  const int ylocation = 50;

  // move to mid position
  chassis.turnToPoint(-42, ylocation, 500, {.maxSpeed = 100}, false);
  chassis.moveToPoint(-42, ylocation, 2000, {.maxSpeed = 70}, false); // 60 speed works
  chassis.turnToPoint(-60, ylocation, 300, {.maxSpeed = 40}, false);

  // drop tongue
  tonguePiston.set_value(false);
  // run in take
  bottomIntake.move(127);
  // give time for tongue to drop
  pros::delay(200);
  // move into the match load and get balls
  chassis.moveToPoint(-62, ylocation, 2000, {.forwards = true, .maxSpeed = 60}, false);

  // move to long goal
  chassis.moveToPoint(-22, ylocation, 3000, {.forwards = false, .maxSpeed = 70}, true);
  // wait until bot gets to long goal
  pros::delay(1000);
  // score long goal
  fullIntake.move(127);
  pros::delay(2000);
  fullIntake.move(0);

  // move back then come back and punch
  chassis.moveToPoint(-39, ylocation, 500, {.maxSpeed = 60}, false);
  chassis.moveToPoint(-22, ylocation, 500, {.forwards = false, .maxSpeed = 100}, true);
}

void autonomousSoloAWP(){

  chassis.setPose(-47, 0, 180);
  tonguePiston.set_value(true); // lifts tongue
  conveyorPiston.set_value(false); // raise conveyor

  // push teammate off park zone barrier
  chassis.moveToPoint(-47, 6, 1000, {.forwards=false, .maxSpeed=60}, false);
  // drive to align with matchloader (midpoint)
  chassis.moveToPoint(-47, -47, 2000, {.maxSpeed = 90}, false);

  // move into match loader & drop tongue & run intake
  chassis.turnToPoint(-62, -47, 500, {.forwards=true, .maxSpeed = 100}, false);
  tonguePiston.set_value(false);
  bottomIntake.move(127);
  pros::delay(200);
  chassis.moveToPoint(-62, -47, 1100, {.forwards=true, .maxSpeed = 70}, false);

  // move into long goal
  chassis.moveToPoint(-24, -47, 2000, {.forwards = false, .maxSpeed = 90}, true);
  pros::delay(1000); // allow time to drive into goal
  fullIntake.move(127); // start scoring
  pros::delay(800); // time to score
  fullIntake.move(0); // stop scoring
  tonguePiston.set_value(true); // raise tongue

  // back out
  chassis.moveToPoint(-36, -47, 400, {.forwards=true, .maxSpeed = 90}, false);

  // go to mid 3 balls
  bottomIntake.move(127);
  chassis.turnToPoint(-22.5, -22.5, 400, {.forwards=true, .maxSpeed=100}, false);
  chassis.moveToPoint(-22.5, -22.5, 1000, {.forwards=true, .maxSpeed=100}, true);
  pros::delay(100);
  tonguePiston.set_value(false); // drop tongue to grab last ball

  // go to left side mid 3 balls (longer traverse)
  chassis.turnToPoint(-22.5, 22.5, 300, {.forwards=true, .maxSpeed=100}, false);
  tonguePiston.set_value(true); // raise tongue 
  chassis.moveToPoint(-22.5, 22.5, 1600, {.maxSpeed = 100}, true);
  pros::delay(850); // wait until at the balls
  tonguePiston.set_value(false); // drop tongue to grab last ball
  
  // go to mid goal
  chassis.turnToPoint(-7, 7, 400, {.forwards=false, .maxSpeed=100}, false);
  chassis.moveToPoint(-7, 7, 700, {.forwards=false, .maxSpeed=70}, false);

  // score mid goal
  conveyorPiston.set_value(true);
  // bottomIntake.move(-127);
  // pros::delay(50);
  // bottomIntake.moxve(0);
  fullIntake.move(127);
  pros::delay(500);
  conveyorPiston.set_value(false);
  fullIntake.move(0);

  // go to left match loader align midpoint
  chassis.moveToPoint(-46, 47, 1400, {.forwards=true, .maxSpeed = 90}, false);
  
  // drop tongue & start intake
  bottomIntake.move(127);
  
  // go into left match loader
  chassis.turnToPoint(-57, 47, 400, {.forwards=true, .maxSpeed=100}, false);
  chassis.moveToPoint(-57, 47, 1100, {.forwards=true, .maxSpeed=80}, false);

  // score left long goal
  chassis.moveToPoint(-24, 47, 3500, {.forwards=false, .maxSpeed=80}, true);
  pros::delay(1200);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(2300);
  fullIntake.move(0);
}