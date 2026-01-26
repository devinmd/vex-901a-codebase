// clang-format off
#include "autonomous.h"
#include "motors.h"
#include "pros/rtos.hpp"

void fromthefront() {

  // set starting position for middle
  chassis.setPose(-56, 0, 270);
  tonguePiston.set_value(true); // lifts tongue

  // move to ball
  bottomIntake.move(127);
  chassis.moveToPoint(-65, 0, 700, {.maxSpeed = 70}, false);
  chassis.moveToPoint(-54, 0, 600, {.forwards = false, .maxSpeed = 90}, true);
  // pros::delay(200);
  // bottomIntake.move(-127);
  // pros::delay(80);
  // bottomIntake.move(127);
  chassis.moveToPoint(-65, 0, 1000, {.maxSpeed = 100}, false);
  chassis.moveToPoint(-54, 0, 700, {.forwards = false, .maxSpeed = 90}, true);
  // pros::delay(200);
  // bottomIntake.move(-127);
  // pros::delay(80);
  // bottomIntake.move(127);
  chassis.moveToPoint(-65, 1, 1000, {.maxSpeed = 100}, false);
  chassis.moveToPoint(-54, 0, 700, {.forwards = false, .maxSpeed = 90}, true);
  chassis.moveToPoint(-65, -1, 1000, {.maxSpeed = 100}, false);
  chassis.moveToPoint(-54, 0, 700, {.forwards = false, .maxSpeed = 90}, true);

  chassis.moveToPoint(-43, 0, 2000, {.forwards = false, .maxSpeed = 90}, false);
  bottomIntake.move(0);
  chassis.moveToPoint(-50, 0, 1500, {.maxSpeed = 60}, false);
  chassis.setPose(-45.5, 0, 270);

  chassis.moveToPoint(-22.5, 0, 1000, {.forwards = false, .maxSpeed = 80}, false);
  chassis.turnToPoint(-19.5, 19, 500, {.maxSpeed = 80}, false);
  bottomIntake.move(127);
  chassis.moveToPoint(-19.5, 19, 1500, {.maxSpeed = 80}, true);
  pros::delay(430);
  tonguePiston.set_value(false);

  chassis.turnToPoint(-6.5, 6.5, 500, { .forwards = false, .maxSpeed = 80, }, false);
  chassis.moveToPoint(-6.5, 6.5, 800, { .forwards = false, .maxSpeed = 70, }, false);
  conveyorPiston.set_value(true);
  bottomIntake.move(-127);
  pros::delay(80);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(3000);
  fullIntake.move(0);
  conveyorPiston.set_value(false);
}

void middle() {

  // set starting position for middle
  chassis.setPose(-43.586, 7.176, 51.961);
  tonguePiston.set_value(true); // lifts tongue
  wingPiston.set_value(true);

  bottomIntake.move(127);
  chassis.moveToPoint(-31.512, 16.986, 800, {.maxSpeed = 90}, false);
  chassis.moveToPose(-6.5, 6.5, 315, 1600, {.forwards = false, .horizontalDrift = 15, .lead = 0.45, .maxSpeed = 80}, false);
  conveyorPiston.set_value(true);
  pros::delay(100);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(1000);
  fullIntake.move(0);
  conveyorPiston.set_value(false);


}

void autonomousSkills() {  // need the odometry to not get stuck

  // fromthefront();
  
  // score the 1 red ball in the middle goal
  // middle();

  //
  chassis.setPose(-48.5, 16.8,0);
  wingPiston.set_value(true);

  // move to goal & align
  bottomIntake.move(127);
  chassis.moveToPoint(-48.5, 47, 1800, {.maxSpeed = 90}, false);
  chassis.turnToPoint(-62, 47, 800, {.maxSpeed = 80}, false);

  // drop tongue, move to loader, intake
  tonguePiston.set_value(false);
  pros::delay(400);
  chassis.moveToPoint(-62, 47, 800, {.maxSpeed = 70}, false);
  pros::delay(2200);
  
  // move to other side
  chassis.moveToPoint(-47, 47, 800, {.forwards = false, .maxSpeed = 90}, false);
  tonguePiston.set_value(true);
  chassis.turnToPoint(-37, 59, 500, {.maxSpeed = 80}, false);
  chassis.moveToPoint(-37, 59, 1700, {.maxSpeed = 90}, false);
  chassis.turnToPoint(32, 59, 500, {.maxSpeed = 80}, false);
  chassis.moveToPoint(32, 59, 3600, {.maxSpeed = 100}, false);
  chassis.turnToPoint(39, 47, 500, {.maxSpeed = 80}, false);
  chassis.moveToPoint(39, 47, 1300, {.maxSpeed = 90}, false);
  chassis.turnToPoint(24, 47, 500, {.forwards = false, .maxSpeed = 80}, false);

  // move to long goal & score
  chassis.moveToPoint(24, 47, 700, {.forwards = false, .maxSpeed = 80}, false);
  tonguePiston.set_value(false);
  bottomIntake.move(-127);
  pros::delay(100);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(4000);
  fullIntake.move(0);

  // move to loader, intake
  bottomIntake.move(127);
  chassis.moveToPoint(62, 47, 800, {.maxSpeed = 70}, false);
  pros::delay(2200);
  

  // move to long goal & score
  chassis.moveToPoint(24, 47, 700, {.forwards = false, .maxSpeed = 80}, false);
  bottomIntake.move(-127);
  pros::delay(100);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(4000);
  tonguePiston.set_value(true);
  fullIntake.move(0);
  chassis.moveToPoint(36, 47, 500, {.maxSpeed = 80}, false);
  chassis.turnToPoint(36, -48, 500);

/*

  // clear zone and pick up
  chassis.moveToPoint(55, 47, 500, {.maxSpeed = 80}, false);

  chassis.moveToPose(63, -32, 180, 9500, {.horizontalDrift = 15, .lead = 0.6, .maxSpeed = 80}, true);
  pros::delay(1200);
  bottomIntake.move(127);
  // chassis.turnToPoint(63, 17, 400, {.maxSpeed = 80}, false);
  // chassis.moveToPoint(63, 17, 1500, {.maxSpeed = 90}, false);
  // bottomIntake.move(127);
  // chassis.moveToPoint(63, 17, 400, {.maxSpeed = 55}, false);
  // chassis.moveToPoint(63, -32, 7000, {.maxSpeed = 70}, false);
  chassis.moveToPoint(63, -12, 2000, {.maxSpeed = 50}, false);
  chassis.setPose(63, -17.25, 180);

  // go to mid goal and score
  chassis.moveToPose(6.5, -6.5, 135, 3000, {.forwards = false, .horizontalDrift = 15, .lead = 0.45, .maxSpeed = 80}, false);
  conveyorPiston.set_value(true);
  bottomIntake.move(-127);
  pros::delay(100);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(6000);
  fullIntake.move(0);
*/

  //move to other tube
  conveyorPiston.set_value(false);
  bottomIntake.move(127);
  chassis.moveToPoint(36, -47, 2500, {.maxSpeed = 90}, false);
  chassis.turnToPoint(57, -47, 400, {.maxSpeed = 80}, false);
  bottomIntake.move(127);
  chassis.moveToPoint(62, -47, 3000, {.maxSpeed = 80}, false);
  
  // move to to other side
  chassis.moveToPoint(47, -47, 700, {.forwards = false, .maxSpeed = 90}, false);
  bottomIntake.move(0);
  tonguePiston.set_value(true);
  chassis.turnToPoint(37, -59, 700, {.maxSpeed = 80}, false);
  chassis.moveToPoint(37, -59, 2000, {.maxSpeed = 90}, false);
  chassis.turnToPoint(-32, -59, 500, {.maxSpeed = 80}, false);
  chassis.moveToPoint(-32, -59, 4000, {.maxSpeed = 100}, false);
  chassis.turnToPoint(-39, -47, 400, {.maxSpeed = 80}, false);
  chassis.moveToPoint(-39, -47, 2000, {.maxSpeed = 90}, false);
  chassis.turnToPoint(-24, -47, 500, {.forwards = false, .maxSpeed = 80}, false);








  // // go out and get tube
  // chassis.moveToPoint(19, 19, 1000, {.forwards = false, .maxSpeed = 70}, false);
  // chassis.turnToPoint(42, -47, 500, {.maxSpeed = 60}, false);
  // bottomIntake.move(127);
  // chassis.moveToPoint(42, -47, 5000, {.maxSpeed = 80}, false);
  // chassis.turnToPoint(58, -47, 500, {.maxSpeed = 60}, false);

  // // drop tongue, move to loader, intake
  // tonguePiston.set_value(true);
  // pros::delay(200);
  // chassis.moveToPoint(58, -47, 2500, {.maxSpeed = 60}, false);

  // // move to long goal & score
  // chassis.moveToPoint(30, -47, 2000, {.forwards = false, .maxSpeed = 80}, false);
  // bottomIntake.move(0);
  // fullIntake.move(127);
  // pros::delay(6000);
  // fullIntake.move(0);
  // tonguePiston.set_value(false);
  
  // // move to other side
  // // chassis.moveToPoint(37, -47, 2000, {.maxSpeed = 70}, false);
  // // chassis.turnToPoint(28, -59, 500, {.maxSpeed = 60}, false);
  // // chassis.moveToPoint(28, -59, 2000, {.maxSpeed = 70}, false);
  // // chassis.turnToPoint(-28, -59, 500, {.maxSpeed = 60}, false);
  // // chassis.moveToPoint(-28, -59, 4000, {.maxSpeed = 80}, false);
  // // chassis.turnToPoint(-37, -47, 500, {.maxSpeed = 60}, false);
  // // chassis.moveToPoint(-37, -47, 2000, {.maxSpeed = 70}, false);
  // // chassis.turnToPoint(-30, -47, 500, {.maxSpeed = 60}, false);

  //  // drop tongue, move to loader, intake
  // tonguePiston.set_value(false);
  // bottomIntake.move(127);
  // pros::delay(200);
  // chassis.moveToPoint(-58, -47, 2500, {.maxSpeed = 70}, false);
  
  // // move to long goal & scorese
  // chassis.moveToPoint(-30, -47, 2000, {.forwards = false, .maxSpeed = 80}, false);
  // bottomIntake.move(0);
  // fullIntake.move(127);
  // pros::delay(6000);
  // fullIntake.move(0);
  // tonguePiston.set_value(false);

  // // move back and park
  // chassis.moveToPoint(-40, -47, 1800, {.forwards = false, .maxSpeed = 70}, false);
  // chassis.turnToPoint(-40, 0, 500, {.maxSpeed = 60}, false);
  // chassis.moveToPoint(-40, 0, 3000, {.maxSpeed = 80}, false);
  // chassis.turnToPoint(-62, 0, 500, {.maxSpeed = 60}, false);
  // chassis.moveToPoint(-62, 0, 4000, {.maxSpeed = 70}, false);

}