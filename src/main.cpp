
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

//
#include "autonomous.h"
#include "helpers.h"
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

// select autonomous here
void autonomous() {

  // autonomousSoloAWP();
  // autonomousRight();
  // autonomousLeft();
  // autonomousLeft2();
  // autonomousLeft3();
  // autonomousSkills();
  // autonomousLeftRush();
  // autonomousSkillsPark()
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

/*
forward is the tongue side
*/

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

const double TURN_MULTIPLIER = 0.95;
const double TIP_THRESHOLD = 8.0; // degrees of tilt before anti-tip engages
const double ANTI_TIP_GAIN = 5.0; // how aggressively it drives under itself

void opcontrol() {

  int direction = 1;
  bool last_x = false;
  bool x_pressed = false;

  const int zero = lift_motors.set_zero_position_all(0.0);

  const int wristZero = wrist_motor.set_zero_position(0.0);
  const int wristMax = 720;

  lift_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  wrist_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  while (true) {

    int liftHeight = lift_motors.get_position();

    // Read angle from IMU
    double current_roll = imu.get_roll();

    if (std::abs(current_roll) > TIP_THRESHOLD) {
      // Force motors to COAST/BRAKE so they react instantly without fighting a
      // HOLD lock? Calculate corrective power proportional to how severe the
      // tip is
      int correction_power = current_roll * -ANTI_TIP_GAIN;

      // Cap the power to V5 voltage limits (-127 to 127)
      if (correction_power > 127)
        correction_power = 127;
      if (correction_power < -127)
        correction_power = -127;

      // Drive directly under the tilt direction
      left_mg.move(correction_power);
      right_mg.move(correction_power);
    } else {
      // curvature drive
      int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
      int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
      chassis.curvature(leftY * direction, 0.95 * rightX);
    }

    //
    // LIFT
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      lift_motors.move(127);
    } else if (controller.get_digital((pros::E_CONTROLLER_DIGITAL_L1)) &&
               lift_motors.get_position() > 0) {
      // stop lift from going below 0
      lift_motors.move(-127);
    } else {
      lift_motors.move(0);
    }

    //
    // CLAW & WRIST
    const bool wristUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    const bool wristDown = false;
    // controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    const bool clawOpenPressed =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    bool clawClosedPressed =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

    //
    // INTAKE
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake_motor.move(127);
      clawClosedPressed = true; // close claw when intaking
      // } else if (controller.get_digital((pros::E_CONTROLLER_DIGITAL_L1)) &&
      // lift_motors.get_position() > 0) { stop lift from going below 0
      // lift_motors.move(-127);
    } else {
      intake_motor.move(0);
    }

    claw_motor.move(clawOpenPressed ? 127 : clawClosedPressed ? -127 : 0);

    pros::lcd::print(0, "Wrist pos: %f", wrist_motor.get_position());
    pros::lcd::print(2, "Lift pos: %f", liftHeight);

    if (wrist_motor.get_position() < wristMax && wristUp) {
      wrist_motor.move(127);
    } else if (wrist_motor.get_position() > 0 && wristDown) {
      wrist_motor.move(-127);
    } else {
      wrist_motor.move(0);
    }

    //
    // DRIVETRAIN MOTOR TEST
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      // testMotors();
    }

    // invert throttle button
    x_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    if (x_pressed && !last_x) {
      direction = -direction;
    }
    last_x = x_pressed;

    pros::delay(20);
  }
}