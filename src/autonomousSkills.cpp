// clang-format off
#include "autonomous.h"
#include "motors.h"
#include "pros/rtos.hpp"

void autonomousSkills() {  

  // fromthefront();
  // middle();

  //
  chassis.setPose(-48.5, 16.8,0);
  wingPiston.set_value(true);

  // move to match loader
  bottomIntake.move(127);
  chassis.moveToPoint(-47, 47, 1800, {.maxSpeed = 90}, false);
  chassis.turnToPoint(-60, 47, 700, {.maxSpeed = 80}, false);

  // drop tongue, move to 1st loader, intake
  tonguePiston.set_value(false);
  pros::delay(400);
  chassis.moveToPoint(-60, 47, 3000, {.maxSpeed = 70}, false);
  
  const int ylocation1 = 60;

  // move to other side
  chassis.moveToPoint(-50, 47, 800, {.forwards = false, .maxSpeed = 90}, false);
  tonguePiston.set_value(true);
  chassis.turnToPoint(-37, ylocation1, 500, {.maxSpeed = 80}, false);
  chassis.moveToPoint(-37, ylocation1, 1700, {.maxSpeed = 90}, false);
  chassis.turnToPoint(32, ylocation1, 500, {.maxSpeed = 80}, false);
  chassis.moveToPoint(32, ylocation1, 3600, {.maxSpeed = 100}, false);
  chassis.turnToPoint(39, 47, 500, {.maxSpeed = 60}, false);
  chassis.moveToPoint(39, 47, 1300, {.maxSpeed = 90}, false);
  chassis.turnToPoint(24, 47, 500, {.forwards = false, .maxSpeed = 80}, false);

  // move to 1st long goal & score
  chassis.moveToPoint(24, 47, 700, {.forwards = false, .maxSpeed = 80}, false);
  tonguePiston.set_value(false);
  bottomIntake.move(-127);
  pros::delay(400);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(4000);
  fullIntake.move(0);

  // move to 2nd loader, intake
  bottomIntake.move(127);
  chassis.moveToPoint(60, 47, 3000, {.maxSpeed = 70}, false);
  
  // move to 2nd long goal & score
  chassis.moveToPoint(22, 47, 700, {.forwards = false, .maxSpeed = 80}, false);
  bottomIntake.move(-127);
  pros::delay(100);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(4000);
  fullIntake.move(0);
  // chassis.setPose(30,47,90);

  // move to 3rd loader (leave tongue down ?)
  const int y3 = -48;
  chassis.moveToPoint(44, 47, 1200, {.maxSpeed = 80}, false);
  chassis.turnToPoint(44, y3, 600, {.maxSpeed = 80}, false);
  chassis.moveToPoint(44, y3, 3000, {.maxSpeed = 100}, false);
  chassis.turnToPoint(59, y3, 400, {.maxSpeed = 80}, false);
  bottomIntake.move(127);
  chassis.moveToPoint(60, y3, 3000, {.maxSpeed = 70}, false);  
  
  // move to to other side (3)
  chassis.moveToPoint(47, -47, 800, {.forwards = false, .maxSpeed = 90}, false);
  tonguePiston.set_value(true);
  // move to alley
  const int ylocation = -60;
  chassis.turnToPoint(37, ylocation, 500, {.maxSpeed = 80}, false);
  chassis.moveToPoint(37, ylocation, 1700, {.maxSpeed = 90}, false);
  chassis.turnToPoint(-32, ylocation, 500, {.maxSpeed = 80}, false);
  chassis.moveToPoint(-32, ylocation, 3600, {.maxSpeed = 100}, false);
  chassis.turnToPoint(-39, -47, 500, {.maxSpeed = 80}, false);
  chassis.moveToPoint(-39, -47, 1300, {.maxSpeed = 90}, false);
  chassis.turnToPoint(-24, -47, 500, {.forwards = false, .maxSpeed = 80}, false);

  // move to long goal & score
  chassis.moveToPoint(-20, -47, 700, {.forwards = false, .maxSpeed = 80}, false);
  tonguePiston.set_value(false);
  bottomIntake.move(-127);
  pros::delay(100);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(4000);
  fullIntake.move(0);

  // move to loader, intake
  bottomIntake.move(127);
  chassis.moveToPoint(-62, -47, 3000, {.maxSpeed = 70}, false);

  // move to long goal & score
  chassis.moveToPoint(-24, -47, 700, {.forwards = false, .maxSpeed = 80}, false);
  bottomIntake.move(-127);
  pros::delay(100);
  bottomIntake.move(0);
  fullIntake.move(127);
  pros::delay(4000);
  tonguePiston.set_value(true);
  fullIntake.move(0);

  chassis.moveToPoint(-50, -47, 500, {.maxSpeed = 80}, false);
  chassis.moveToPoint(-64, -34, 800, {.maxSpeed = 80}, false);

  // parking 
  fullIntake.move(127);
  // chassis.moveToPose(-64, -22, 0, 2000, {.horizontalDrift = 15, .lead = 0.7, .maxSpeed = 70}, false);

  chassis.turnToPoint(-67, -22, 1000, {.maxSpeed = 80}, false);
  // chassis.moveToPose(-67, -22,0,2000, {.maxSpeed = 80}, false);
  // chassis.turnToPoint(-67, 2, 200, {.maxSpeed = 80}, false);
  chassis.moveToPose(-66, -38, 0,2000, {.forwards=false,.maxSpeed = 80}, false);
  chassis.moveToPose(-66, 6, 0, 8000, {.maxSpeed = 127}, false);
  // bottomIntake.move(127);

}