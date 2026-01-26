
//
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

//
#include "autonomous.h"
#include "motors.h"

// select autonomous here

void autonomous() {
  // autonomousSoloAWP();
  // autonomousRight();
  // autonomousLeft();
  autonomousSkills();
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
  chassis.calibrate();

  // tonguePiston.set_value(true);    // lifts tongue
  // conveyorPiston.set_value(false); // raise conveyor

  pros::delay(500);

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

void opcontrol() {

  int direction = 1;
  bool lastX = false;
  bool xPressed = false;
  bool tonguePistonState = false;
  bool conveyorPistonState = false;
  bool wingPistonState = false;

  while (true) {

    // curvature drive
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    chassis.curvature(leftY * direction, rightX);

    // intake motors
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      // storage
      bottomIntake.move(127);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      // both
      topIntake.move(127);
      bottomIntake.move(127);
    } else if (controller.get_digital((pros::E_CONTROLLER_DIGITAL_R2))) {
      // reverse
      topIntake.move(-127);
      bottomIntake.move(-127);
    } else {
      // none
      topIntake.move(0);
      bottomIntake.move(0);
    }

    // middle goal
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      conveyorPistonState = !conveyorPistonState;
      conveyorPiston.set_value(conveyorPistonState);
    }

    // tongue
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      tonguePistonState = !tonguePistonState;
      tonguePiston.set_value(tonguePistonState);
    }

    // wing
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      wingPistonState = !wingPistonState;
      wingPiston.set_value(wingPistonState);
    }

    // invert throttle button
    xPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    if (xPressed && !lastX) {
      direction = -direction;
    }
    lastX = xPressed;

    pros::delay(25);
  }
}