
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
#include "helpers.h"
#include "motors.h"

//
// constantly checks if motors are jammed in intake
void intake_motor_monitor_task(void *param) {

  int jam_timer = 0;           // Tracks how long we've been "jammed"
  const int JAM_THRESHOLD = 5; // 15 loops * 20ms = 300ms of sustained jamming

  int bottomTargetVelocity = bottomIntake.get_target_velocity();
  int topTargetVelocity = topIntake.get_target_velocity();

  while (true) {
    if (isMotorJammed(bottomIntake))
      jam_timer++;
    else
      jam_timer = 0;

    if (jam_timer >= JAM_THRESHOLD) {
      bottomIntake.move(-127);
      topIntake.move(-127);
      pros::delay(50);

      // then revert motor back to its original power
      bottomIntake.move_velocity(bottomTargetVelocity);
      topIntake.move_velocity(topTargetVelocity);

      jam_timer = 0;
    }
    // let other tasks run
    pros::delay(20);
  }
}

// select autonomous here
void autonomous() {

  // autonomousSoloAWP();
  // autonomousRight();
  autonomousLeft();
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
    chassis.curvature(leftY * direction, 0.95 * rightX);
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      // storage motor
      if (!isMotorJammed(bottomIntake)) {
      topIntake.move(0);
      bottomIntake.move(127);
      }
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      // both motors
      if (conveyorPistonState) {
        // slow it down when scoring on middle goal
        if (!isMotorJammed(bottomIntake)) {
        topIntake.move(80);
        bottomIntake.move(80);
        }
      } else {
        // full speed
        if (!isMotorJammed(bottomIntake)) {
        topIntake.move(127);
        bottomIntake.move(127);
        }
      }
    } else if (controller.get_digital((pros::E_CONTROLLER_DIGITAL_R2))) {
      // reverse intake
      topIntake.move(-127);
      bottomIntake.move(-127);
    } else {
      // none
      topIntake.move(0);
      bottomIntake.move(0);
    }

    // wing macro
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      wingMacro();
    }

    // motor test and LOCK
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {

      testMotors();
      // lockMacro();

      /*chassis.setPose(-47, 15, 0);
      tonguePiston.set_value(false); // drop tongue

      // move to match loader
      chassis.moveToPoint(-47, 47, 1700, {.forwards = false, .maxSpeed = 100},
                          false); // 90 speed works
      chassis.turnToPoint(-57, 47, 500, {.maxSpeed = 110},
                          false); // 80 speed works
      // tonguePiston.set_value(false);
      // pros::delay(200); // 300ms works
      bottomIntake.move(127);
      chassis.moveToPoint(-62, 47, 1100, {.maxSpeed = 90},
                          false); // 80 speed works, 800ms works*/
    }

    // middle goal piston
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      conveyorPistonState = !conveyorPistonState;
      conveyorPiston.set_value(conveyorPistonState);
    }

    // tongue piston
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      tonguePistonState = !tonguePistonState;
      tonguePiston.set_value(tonguePistonState);
    }

    // wing piston
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

    pros::delay(20);
  }
}