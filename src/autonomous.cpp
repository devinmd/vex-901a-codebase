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
  chassis.moveToPoint(-22.5, -22.5, 2000, {.maxSpeed = 90}, false);
  pros::delay(400);
  tonguePiston.set_value(false); // drop tongue to catch last ball

  // move to goal & align, then run intake & score
  chassis.turnToPoint(-47, -47, 500, {.maxSpeed=80}, false);
  chassis.moveToPoint(-47, -47, 1600, {.maxSpeed=90}, false);
  chassis.turnToPoint(-58, -47, 800, {.maxSpeed=80}, false);

  // move to loader, intake
  bottomIntake.move(127);
  pros::delay(400);
  chassis.moveToPoint(-58, -47, 1500, {.maxSpeed = 80}, false);

  // move to long goal & score
  chassis.moveToPoint(-24, -47, 4000, {.forwards = false, .maxSpeed = 80}, true);
  pros::delay(1500);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(2500);
  fullIntake.move(0);

  // wing
  chassis.moveToPoint(-39, -47, 600, {.maxSpeed = 80}, false);
  chassis.turnToPoint(-32, -56.75, 300, {.forwards = false, .maxSpeed = 80}, false);
  chassis.moveToPoint(-32, -56.75, 800, {.forwards = false, .maxSpeed = 80}, false);
  chassis.turnToPoint(-7, -56.75, 300, {.forwards = false, .maxSpeed = 80}, false);
  chassis.moveToPoint(-7, -56.75, 3000, {.forwards = false, .maxSpeed = 70}, false);
  
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




// SOLO AWP -- WORKING & CAN SCORE 12 BALLS IF IT WORKS (MATCHLOAD IS DROPPED)
void autonomousSoloAWP() {

  // set starting position for middle
  chassis.setPose(-47, -15, 180);
  tonguePiston.set_value(true); // lifts tongue

  // move to tube
  chassis.moveToPoint(-47, -47, 1200, {.maxSpeed = 90}, false);
  chassis.turnToPoint(-57, -47, 500, {.maxSpeed = 80}, false);
  tonguePiston.set_value(false);
  pros::delay(300);
  bottomIntake.move(127);
  chassis.moveToPoint(-57, -47, 900, {.maxSpeed = 70}, false);

  // move to long goal and score
  chassis.moveToPoint(-24, -47, 2000, {.forwards = false, .maxSpeed = 75}, true);
  pros::delay(1200);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(900);
  fullIntake.move(0);
  tonguePiston.set_value(true);

  // move back and get 6 balls
  chassis.moveToPoint(-36, -47, 500, { .maxSpeed = 90}, true);
  chassis.turnToPoint(-22.5, -22.5, 400, {.maxSpeed = 80}, false);
  bottomIntake.move(127);
  chassis.moveToPoint(-22.5, -22.5, 900, {.maxSpeed = 100}, true);
  pros::delay(350);
  tonguePiston.set_value(false);
  chassis.turnToPoint(-22.5, 22.5, 200, {.maxSpeed = 80}, false);
  tonguePiston.set_value(true);
  chassis.moveToPoint(-22.5, 22.5, 1600, {.maxSpeed = 100}, true);
  pros::delay(850);
  tonguePiston.set_value(false);
  
  // score middle top
  chassis.turnToPoint(-7, 7, 400, {.forwards = false, .maxSpeed = 80}, false);
  chassis.moveToPoint(-7, 7, 700, {.forwards = false, .maxSpeed = 70}, false);
  conveyorPiston.set_value(true);
  bottomIntake.move(-127);
  pros::delay(50);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(500);
  fullIntake.move(0);

  //move to other tube
  conveyorPiston.set_value(false);
  chassis.moveToPoint(-46, 47, 1400, {.maxSpeed = 90}, false);
  chassis.turnToPoint(-57, 47, 400, {.maxSpeed = 80}, false);
  bottomIntake.move(127);
  chassis.moveToPoint(-57, 47, 1100, {.maxSpeed = 80}, false);

  // move to long goal and score
  chassis.moveToPoint(-23, 47, 3500, {.forwards = false, .maxSpeed = 70}, true);
  pros::delay(1200);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(2300);
  fullIntake.move(0);

}