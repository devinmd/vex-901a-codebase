#include "lift.h"
#include "motors.h"
#include <algorithm>
#include <cmath>

namespace lift {

namespace {
double target_height_in = 0;
}

void initialize() {
  // the lift must start at the bottom, which becomes its 0 position
  lift_motors.tare_position();

  // hold position when not moving, in both auton and driver control
  lift_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void setHeight(double height_in, int timeout_ms, bool async, int velocity_rpm) {
  target_height_in = std::max(height_in, 0.0); // can't go below the bottom

  // the motor's built-in position PID slows down before the target instead of
  // coasting past it
  lift_motors.move_absolute(target_height_in / LIFT_IN_PER_DEG, velocity_rpm);

  if (!async) {
    waitUntilDone(timeout_ms);
  }
}

double getHeight() {
  return lift_motors.get_position() * LIFT_IN_PER_DEG;
}

bool isDone() {
  return std::abs(getHeight() - target_height_in) < LIFT_SETTLED_RANGE_IN;
}

void waitUntilDone(int timeout_ms) {
  const std::uint32_t start = pros::millis();
  while (!isDone() && pros::millis() - start < static_cast<std::uint32_t>(timeout_ms)) {
    pros::delay(10);
  }
}

} // namespace lift
