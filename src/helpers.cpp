#include "helpers.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "motors.h"
#include "pros/motors.hpp"

#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

bool isMotorJammed(pros::Motor &m) {
  bool isLowEff = m.get_efficiency() < 5;           // motor efficiency 0-100
  bool isHighCurrent = m.get_current_draw() > 2100; // current draw 0-2500
  bool isStalled = std::abs(m.get_actual_velocity()) < 5; // velocity in RPM

  return (isStalled && isHighCurrent && isLowEff);
}
/*
bool isMotorJammed(pros::Motor &m) {
  // Use voltage check so it works regardless of move() or move_velocity()
  bool isTryingToMove = std::abs(m.get_voltage()) > 2000;
  bool isStalled = std::abs(m.get_actual_velocity()) < 5;
  bool isHighCurrent = m.get_current_draw() > 2100;
  bool isLowEff = m.get_efficiency() < 5;

  return (isTryingToMove && isStalled && isHighCurrent && isLowEff);
}*/

void wingMacro() {
  chassis.setPose(-28, 48, 270);

  const int yCoord = 38;

  // move back
  chassis.moveToPoint(-38, 47, 400, {.maxSpeed = 70}, false);
  // move to side
  chassis.turnToPoint(-32, yCoord, 200, {.forwards = false, .maxSpeed = 80},
                      false);
  chassis.moveToPoint(-32, yCoord, 500, {.forwards = false, .maxSpeed = 80},
                      false);
  // move in
  chassis.turnToPoint(-2, yCoord, 200, {.forwards = false, .maxSpeed = 80},
                      false);
  chassis.moveToPoint(-2, yCoord, 800, {.forwards = false, .maxSpeed = 127},
                      false);
}

void testMotors() {
  const int delay = 2000;

  pros::c::motor_move(3, 127); // good
  pros::delay(delay);
  pros::c::motor_move(3, 0);
  pros::delay(delay);

  pros::c::motor_move(2, 127); // good
  pros::delay(delay);
  pros::c::motor_move(2, 0);
  pros::delay(delay);

  pros::c::motor_move(1, 127); // slow
  pros::delay(delay);
  pros::c::motor_move(1, 0);
  pros::delay(delay);

  pros::c::motor_move(15, 127); // slow
  pros::delay(delay);
  pros::c::motor_move(15, 0);
  pros::delay(delay);

  pros::c::motor_move(14, 127); // popped out
  pros::delay(delay);
  pros::c::motor_move(14, 0);
  pros::delay(delay);

  pros::c::motor_move(13, 127); // good
  pros::delay(delay);
  pros::c::motor_move(13, 0);
}

void lockMacro() {
  chassis.setPose(0, 0, 0);
  chassis.moveToPose(0, 0, 0, 10000, {}, true);

  // TODO: TEST
  while (chassis.isInMotion()) {
    if (controller.get_digital(DIGITAL_LEFT)) { // If LEFT is pressed
      chassis.cancelMotion();                   // Exit the movement
      break;
    }
    pros::delay(20);
  }
}