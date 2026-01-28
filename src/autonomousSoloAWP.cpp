// clang-format off
#include "autonomous.h"
#include "motors.h"
#include "pros/rtos.hpp"

void autonomousSoloAWP() {

  // set starting position for middle
  chassis.setPose(-47, 0, 0);
  bottomIntake.move(127); // run intake to grab teammates preload
  tonguePiston.set_value(true); // lifts tongue

  // push teammate off park zone barrier
  chassis.moveToPoint(-47, 8, 500, {.forwards=true, .maxSpeed = 90}, false);

  // move to match loader 
  chassis.moveToPoint(-47, -47, 1700, {.forwards=false, .maxSpeed = 100}, false);
  chassis.turnToPoint(-57, -47, 500, {.maxSpeed = 110}, false); 
  tonguePiston.set_value(false);
  pros::delay(200); 
  bottomIntake.move(127);
  chassis.moveToPoint(-62, -47, 1100, {.maxSpeed = 90}, false); 

  // move to long goal and score
  chassis.moveToPoint(-24, -47, 2400, {.forwards = false, .maxSpeed = 110}, true); 
  pros::delay(850); // 850ms works
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(1000);
  fullIntake.move(0);
  tonguePiston.set_value(true);

  // move back
  chassis.moveToPoint(-36, -47, 500, { .maxSpeed = 127}, true); 
  // go for 3 balls
  chassis.turnToPoint(-22.5, -22.5, 600, {.maxSpeed = 100}, false); 
  bottomIntake.move(127);
  chassis.moveToPoint(-22.5, -22.5, 800, {.maxSpeed = 100}, true); 
  pros::delay(400);
  tonguePiston.set_value(false);
  // go for other 3 balls
  chassis.turnToPoint(-22.5, 22.5, 300, {.maxSpeed = 80}, false);
  tonguePiston.set_value(true);
  chassis.moveToPoint(-22.5, 22.5, 1600, {.maxSpeed = 100}, true);
  pros::delay(750);
  tonguePiston.set_value(false);
  
  // score middle top
  chassis.turnToPoint(-6, 6, 400, {.forwards = false, .maxSpeed = 80}, false);
  chassis.moveToPoint(-6, 6, 800, {.forwards = false, .maxSpeed = 80}, false);

  bottomIntake.move(-127);
  pros::delay(100);
  conveyorPiston.set_value(true);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(1200);
  fullIntake.move(0);

  // move to other tube
  conveyorPiston.set_value(false);
  chassis.moveToPoint(-47, 47, 1400, {.maxSpeed = 110}, false); 
  chassis.turnToPoint(-57, 47, 400, {.maxSpeed = 80}, false);
  bottomIntake.move(127);
  chassis.moveToPoint(-62, 47, 1100, {.maxSpeed = 80}, false); 

  // move to long goal and score
  chassis.moveToPoint(-23, 47, 3500, {.forwards = false, .maxSpeed = 90}, true);
  pros::delay(850);
  // pros::delay(350);
  // bottomIntake.move(-127);
  // pros::delay(100);
  // bottomIntake.move(127);
  // pros::delay(400);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(2300);
  fullIntake.move(0);

}