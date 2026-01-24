// clang-format off
#include "autonomous.h"
#include "motors.h"
#include "pros/rtos.hpp"


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