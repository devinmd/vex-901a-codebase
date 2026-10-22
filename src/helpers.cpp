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