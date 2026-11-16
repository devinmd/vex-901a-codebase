#include "../motors.h"
#include "autonomous.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <new>

/*

Auto-tunes kP and kD by trying gains, scoring how well each move went, and
keeping whichever scores best (the "twiddle" method)

each test is an out-and-back move so the robot ends up where it started:
  - angular: turn to 90 deg then back to 0 (needs room to spin)
  - lateral: drive forward 24 in then back (needs ~30 in of clear space in front)

score (lower is better) = time to settle
                        + penalty for overshooting
                        + penalty for not reaching the target

results print to the brain screen and the PROS terminal, copy the best gains
into motors.cpp when it finishes

tune angular first since lateral moves also use the angular PID to drive straight
press B on the controller to stop early (keeps the best gains found so far)

*/

namespace {

const int SAMPLE_MS = 10;
const int MAX_ITERATIONS = 15;
const float MIN_KP = 0.1;

// how hard each kind of miss is penalized, in seconds per unit
struct ScoreSettings {
  int test_time_ms;          // how long each leg of the test gets
  float settled_range;       // within this of the target counts as settled
  float overshoot_weight;    // per degree/inch of overshoot
  float final_error_weight;  // per degree/inch of error at the end
  float not_settled_penalty; // extra if it never settles
};

const ScoreSettings ANGULAR_SCORE = {1500, 1.0, 0.1, 0.2, 1.0}; // degrees
const ScoreSettings LATERAL_SCORE = {2000, 0.5, 0.5, 1.0, 1.0}; // inches

const float TURN_ANGLE = 90;     // degrees
const float DRIVE_DISTANCE = 24; // inches

bool aborted = false;

bool abort_pressed() {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    aborted = true;
  }
  return aborted;
}

// LemLib's PID gains can't be changed after it's made, so rebuild it in place
// with the new gains (the other settings stay the same as motors.cpp)
void set_gains(lemlib::PID &pid, const lemlib::ControllerSettings &settings, float kP, float kD) {
  pid.~PID();
  new (&pid) lemlib::PID(kP, settings.kI, kD, settings.windupRange, true);
}

// watches a move that has already started and scores it
// read_position returns the current heading (angular) or y position (lateral)
float score_move(float start, float target, const ScoreSettings &score, float (*read_position)()) {
  const float direction = target > start ? 1 : -1;

  float overshoot = 0;
  float remaining = 0; // distance left to the target, negative once past it
  int last_unsettled_ms = 0;

  for (int t = 0; t < score.test_time_ms; t += SAMPLE_MS) {
    if (abort_pressed()) {
      chassis.cancelMotion();
      return 0;
    }

    remaining = (target - read_position()) * direction;
    overshoot = std::max(overshoot, -remaining);
    if (std::abs(remaining) > score.settled_range) {
      last_unsettled_ms = t + SAMPLE_MS;
    }

    pros::delay(SAMPLE_MS);
  }

  chassis.waitUntilDone();

  float cost = last_unsettled_ms / 1000.0;
  cost += score.overshoot_weight * overshoot;
  cost += score.final_error_weight * std::abs(remaining);
  if (std::abs(remaining) > score.settled_range) {
    cost += score.not_settled_penalty;
  }

  pros::delay(300); // let the robot come to a full stop before the next move
  return cost;
}

float read_heading() { return chassis.getPose().theta; }
float read_y() { return chassis.getPose().y; }

float test_angular(float kP, float kD) {
  set_gains(chassis.angularPID, angular_controller, kP, kD);
  chassis.setPose(0, 0, 0);

  chassis.turnToHeading(TURN_ANGLE, ANGULAR_SCORE.test_time_ms, {}, true);
  float cost = score_move(0, TURN_ANGLE, ANGULAR_SCORE, read_heading);

  chassis.turnToHeading(0, ANGULAR_SCORE.test_time_ms, {}, true);
  cost += score_move(TURN_ANGLE, 0, ANGULAR_SCORE, read_heading);

  return cost;
}

float test_lateral(float kP, float kD) {
  set_gains(chassis.lateralPID, lateral_controller, kP, kD);
  chassis.setPose(0, 0, 0);

  chassis.moveToPoint(0, DRIVE_DISTANCE, LATERAL_SCORE.test_time_ms, {.forwards = true}, true);
  float cost = score_move(0, DRIVE_DISTANCE, LATERAL_SCORE, read_y);

  chassis.moveToPoint(0, 0, LATERAL_SCORE.test_time_ms, {.forwards = false}, true);
  cost += score_move(DRIVE_DISTANCE, 0, LATERAL_SCORE, read_y);

  return cost;
}

void print_result(const char *name, int trial, const float gains[2], float cost, const float best[2], float best_cost) {
  printf("%s trial %d: kP=%.3f kD=%.3f cost=%.3f | best kP=%.3f kD=%.3f cost=%.3f\n",
         name, trial, gains[0], gains[1], cost, best[0], best[1], best_cost);
  pros::lcd::print(0, "%s tuning (B to stop)", name);
  pros::lcd::print(1, "trial %d: kP %.3f kD %.3f", trial, gains[0], gains[1]);
  pros::lcd::print(2, "cost %.3f", cost);
  pros::lcd::print(3, "BEST kP %.3f kD %.3f", best[0], best[1]);
  pros::lcd::print(4, "BEST cost %.3f", best_cost);
}

// twiddle: nudge each gain up, then down, keep whichever helps, and shrink
// the nudge when neither does
void twiddle(const char *name, lemlib::PID &pid, const lemlib::ControllerSettings &settings, float (*test)(float, float)) {
  aborted = false;

  float best[2] = {settings.kP, settings.kD};
  float steps[2] = {settings.kP * 0.5f, std::max(settings.kD * 0.5f, 1.0f)};
  const float min_gains[2] = {MIN_KP, 0};

  int trial = 0;
  float best_cost = test(best[0], best[1]);
  print_result(name, trial, best, best_cost, best, best_cost);

  // tries the gains, and keeps them if they beat the best so far
  auto try_gains = [&](const float gains[2]) {
    const float cost = test(gains[0], gains[1]);
    if (aborted) {
      return false;
    }
    const bool better = cost < best_cost;
    if (better) {
      best_cost = cost;
      best[0] = gains[0];
      best[1] = gains[1];
    }
    print_result(name, ++trial, gains, cost, best, best_cost);
    return better;
  };

  for (int iteration = 0; iteration < MAX_ITERATIONS && !aborted; iteration++) {
    for (int i = 0; i < 2 && !aborted; i++) {
      float gains[2] = {best[0], best[1]};

      // try a bigger gain
      gains[i] = best[i] + steps[i];
      if (try_gains(gains)) {
        steps[i] *= 1.1;
        continue;
      }

      // try a smaller gain
      gains[i] = std::max(best[i] - steps[i], min_gains[i]);
      if (gains[i] != best[i] && try_gains(gains)) {
        steps[i] *= 1.1;
        continue;
      }

      // neither helped, try smaller nudges next time
      steps[i] *= 0.5;
    }
  }

  // leave the chassis running the best gains found
  set_gains(pid, settings, best[0], best[1]);

  printf("%s DONE%s: kP=%.3f kD=%.3f cost=%.3f\n", name, aborted ? " (stopped early)" : "", best[0], best[1], best_cost);
  pros::lcd::print(0, "%s DONE%s", name, aborted ? " (stopped)" : "");
  pros::lcd::print(1, "kP %.3f kD %.3f", best[0], best[1]);
  pros::lcd::print(2, "cost %.3f", best_cost);
  pros::lcd::print(3, "copy these into motors.cpp");
  pros::lcd::print(4, "");
}

} // namespace

void autoTuneAngularPID() {
  twiddle("ANGULAR", chassis.angularPID, angular_controller, test_angular);
}

void autoTuneLateralPID() {
  twiddle("LATERAL", chassis.lateralPID, lateral_controller, test_lateral);
}
