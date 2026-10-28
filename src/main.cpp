
//
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>

//
#include "./autons/autonomous.h"
#include "motors.h"

namespace {

constexpr int BUTTON_LEFT = 145;
constexpr int BUTTON_TOP = 120;
constexpr int BUTTON_RIGHT = 335;
constexpr int BUTTON_BOTTOM = 180;

void draw_test_button(bool pressed) {
  pros::screen::set_pen(pressed ? pros::c::COLOR_DARK_BLUE
                                : pros::c::COLOR_BLUE);
  pros::screen::fill_rect(BUTTON_LEFT, BUTTON_TOP, BUTTON_RIGHT, BUTTON_BOTTOM);

  pros::screen::set_pen(pros::c::COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, 205, 140, "TEST");
}

void draw_brain_ui() {
  pros::screen::set_pen(pros::c::COLOR_BLACK);
  pros::screen::fill_rect(0, 0, 479, 239);

  pros::screen::set_pen(pros::c::COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_LARGE, 145, 35, "Hello World");

  draw_test_button(false);

  pros::screen::set_pen(pros::c::COLOR_LIGHT_GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 155, 205,
                      "Tap TEST to check the touchscreen");
}

void brain_ui_touch() {
  const pros::screen_touch_status_s_t touch = pros::screen::touch_status();

  if (touch.x >= BUTTON_LEFT && touch.x <= BUTTON_RIGHT &&
      touch.y >= BUTTON_TOP && touch.y <= BUTTON_BOTTOM) {
    draw_test_button(true);

    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::fill_rect(155, 205, 325, 220);
    pros::screen::set_pen(pros::c::COLOR_GREEN);
    pros::screen::print(pros::E_TEXT_SMALL, 205, 205, "Button pressed!");
  }
}

} // namespace

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
// select autonomous here
void autonomous() {

  // autonomousSkills();
  // autonomousMain();
  // tuneAngularPID();
  // tuneLateralPID();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  draw_brain_ui();
  pros::screen::touch_callback(brain_ui_touch, pros::E_TOUCH_PRESSED);
  chassis.calibrate();

  // pros::Task intake_motor_watcher(intake_motor_monitor_task);

  // tonguePiston.set_value(true);    // lifts tongue
  // conveyorPiston.set_value(false); // raise conveyor

  /*
  pros::Task screenTask([&]() {
    while (true) {
      pros::lcd::print(0, "X: %f", chassis.getPose().x);
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
      lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
      pros::delay(50);
    }
  });
  */
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

// forward is the tongue side

// drive
const double TURN_MULTIPLIER = 0.95;

// anti-tip
const double TIP_THRESHOLD = 8.0; // degrees of tilt before anti-tip engages
const double ANTI_TIP_GAIN = 5.0; // how aggressively it drives under itself

// wrist
const int WRIST_MAX_POSITION = 900; // max wrist angle (90 deg) to prevent overextension

// lift
const double LIFT_INTAKE_HEIGHT = 560;  // lift position for grabbing a pin and cup
const double LIFT_BOTTOM_TOLERANCE = 50; // within this of 0 counts as "at the bottom"
const int LIFT_INTAKE_VELOCITY = 150;    // rpm (green cartridge max is 200)

// motor power limits
const int FULL_POWER = 127;

void opcontrol() {
  int direction = 1; // 1 = normal throttle, -1 = inverted
  bool last_x = false;
  bool last_intaking = false;
  bool lift_auto_moving = false; // lift is moving itself to LIFT_INTAKE_HEIGHT

  // treat the starting positions of the lift and wrist as zero
  lift_motors.set_zero_position_all(0.0);
  wrist_motor.set_zero_position(0.0);

  // hold motor position when not moving
  lift_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  wrist_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  while (true) {
    double lift_height = lift_motors.get_position();
    double current_roll = imu.get_roll();

    //
    // DRIVETRAIN (with anti-tip)
    if (std::abs(current_roll) > TIP_THRESHOLD) {
      // drive under the tilt with power proportional to how severe the tip is
      int correction_power = current_roll * -ANTI_TIP_GAIN;
      correction_power = std::clamp(correction_power, -FULL_POWER, FULL_POWER);
      left_mg.move(correction_power);
      right_mg.move(correction_power);
    } else {
      // curvature drive
      const int left_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
      const int right_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
      chassis.curvature(left_y * direction, TURN_MULTIPLIER * right_x);
    }

    //
    // INTAKE (L2, also closes the claw to grab the pin)
    const bool intaking = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    intake_motor.move(intaking ? FULL_POWER : 0);

    // starting to intake with the lift at the bottom raises it to intake
    // height; if the lift is already up we're scoring, so leave it alone
    if (intaking && !last_intaking && lift_height < LIFT_BOTTOM_TOLERANCE) {
      lift_auto_moving = true;
      // the motor's built-in position PID slows down before the target
      // instead of coasting past it
      lift_motors.move_absolute(LIFT_INTAKE_HEIGHT, LIFT_INTAKE_VELOCITY);
    }
    last_intaking = intaking;

    //
    // LIFT (R1 up, L1 down, can't go below 0)
    const bool lift_up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    const bool lift_down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    if (lift_up || lift_down) {
      lift_auto_moving = false; // driver takes over from the auto move
    }

    if (lift_up) {
      lift_motors.move(FULL_POWER);
    } else if (lift_down && lift_height > 0) {
      lift_motors.move(-FULL_POWER);
    } else if (!lift_auto_moving) {
      lift_motors.move(0);
    }

    //
    // CLAW (RIGHT open, Y close)
    const bool claw_open = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    const bool claw_close =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) || intaking;

    if (claw_open) {
      claw_motor.move(FULL_POWER);
    } else if (claw_close) {
      claw_motor.move(-FULL_POWER);
    } else {
      claw_motor.move(0);
    }

    //
    // WRIST (R2 up, down currently disabled)
    const bool wrist_up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    const bool wrist_down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    const double wrist_position = wrist_motor.get_position();

    if (wrist_up && wrist_position < WRIST_MAX_POSITION) {
      wrist_motor.move(FULL_POWER);
    } else if (wrist_down && wrist_position > 0) {
      wrist_motor.move(-FULL_POWER);
    } else {
      wrist_motor.move(0);
    }

    //
    // INVERT THROTTLE (X, toggles on press)
    const bool x_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    if (x_pressed && !last_x) {
      direction = -direction;
    }
    last_x = x_pressed;

    //
    // DEBUG
    pros::lcd::print(0, "Wrist pos: %f", wrist_position);
    pros::lcd::print(2, "Lift pos: %f", lift_height);

    pros::delay(20);
  }
}
