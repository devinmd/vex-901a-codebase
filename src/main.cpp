
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
#include <numbers>

//
#include "./autons/autonomous.h"
#include "motors.h"

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
  autonomousLeft();
  // autonomousSkills();
  // autonomousMain();
  // tuneAngularPID();
  // tuneLateralPID();
  // autoTuneAngularPID(); // tune angular first
  // autoTuneLateralPID();
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

  // the lift must start at the bottom, which becomes its 0 position
  lift_motors.tare_position();

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

// lift
const int LIFT_STAGES = 3;
const double WINCH_RADIUS = 0.4;                                                        // inches
const double LIFT_MULTIPLIER = 2 * std::numbers::pi * WINCH_RADIUS / 360 * LIFT_STAGES; // inches lifted per degree the lift motor rotates
const double LIFT_BOTTOM_HEIGHT_IN = 5.75;                                              // height of the center of the claw from the floor at the lift's lowest position
const double GRAB_CUP_CONE_LIFT_HEIGHT = 6.55 - LIFT_BOTTOM_HEIGHT_IN;                  // height of the center of the claw to pick up a cup and cone standing on the floor
const double LIFT_INTAKE_HEIGHT = 2.6;                                                  // lift position for grabbing from intake in inches if 0 is the absolute bottom lift position
const double LIFT_BOTTOM_TOLERANCE = 50;                                                // within this of 0 counts as "at the bottom"

// motor power limits
const int FULL_POWER = 127;

void opcontrol() {
  int direction = 1; // 1 = normal throttle, -1 = inverted
  bool last_x = false;
  bool last_intaking = false;
  bool lift_auto_moving = false; // lift is moving itself to LIFT_INTAKE_HEIGHT

  // wrist & claw
  bool wrist_up = true;
  bool claw_closed = true;

  // hold lift motor position when not moving
  lift_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  while (true) {
    double current_imu_roll = imu.get_roll();

    const double lift_motor_position = lift_motors.get_position();

    const double LIFT_HEIGHT = LIFT_MULTIPLIER * lift_motor_position;

    //
    // DRIVETRAIN (with anti-tip)
    if (std::abs(current_imu_roll) > TIP_THRESHOLD) {
      // drive under the tilt with power proportional to how severe the tip is
      int correction_power = current_imu_roll * -ANTI_TIP_GAIN;
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
    // INTAKE (L2)
    const bool intaking = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    intake_motor.move(intaking ? FULL_POWER : 0);

    // starting to intake with the lift at the bottom raises it to intake height
    // if the lift is already up then leave it
    if (intaking && !last_intaking && lift_motor_position < LIFT_BOTTOM_TOLERANCE) {
      lift_auto_moving = true;
      lift_motors.move_absolute(LIFT_INTAKE_HEIGHT / LIFT_MULTIPLIER, 150);
    }
    last_intaking = intaking;

    //
    // LIFT (R1 up, L1 down)
    const bool lift_up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    const bool lift_down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    if (lift_up || lift_down) {
      lift_auto_moving = false; // driver takes over from the auto move
    }

    if (lift_up) {
      lift_motors.move(FULL_POWER);
    } else if (lift_down && lift_motor_position > 0) {
      lift_motors.move(-FULL_POWER);
    } else if (!lift_auto_moving) {
      lift_motors.move(0);
    }

    //
    // TOGGLE CLAW (Y)
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      claw_closed = !claw_closed;
      clawPiston.set_value(claw_closed);
    }

    //
    // TOGGLE WRIST (R2)
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      wrist_up = !wrist_up;
      wristPiston.set_value(wrist_up);
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
    pros::lcd::print(2, "Lift pos: %f", lift_motor_position);

    pros::delay(20);
  }
}
