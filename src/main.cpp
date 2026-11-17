
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
#include <cerrno>

//
#include "./autons/autonomous.h"
#include "lift.h"
#include "motors.h"

// claw auto close state, here so the brain screen can show it
bool claw_auto_close_armed = true; // the lift starts at the bottom, so it's ready for the first grab
bool rearm_when_clear = false;     // driver opened the claw, re-arm once the object is gone

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
  // autonomousLeft();
  // autonomousSkills();
  // autonomousMain();
  // tuneAngularPID();
  // tuneLateralPID();
  autoTuneAngularPID(); // tune angular first
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

  lift::initialize(); // the lift must start at the bottom

  // pros::Task intake_motor_watcher(intake_motor_monitor_task);

  // tonguePiston.set_value(true);    // lifts tongue
  // conveyorPiston.set_value(false); // raise conveyor

  // brain screen, always on (lines 5-7 are left for the PID tuner)
  pros::Task screen_task([]() {
    while (true) {
      // print odometry pose
      const lemlib::Pose pose = chassis.getPose();
      pros::lcd::print(0, "X %.2f  Y %.2f  Theta %.2f", pose.x, pose.y, pose.theta);
      pros::lcd::print(1, "Lift: %.1f deg  %.2f in", lift_motors.get_position(), lift::getHeight());
      const std::int32_t claw_proximity = claw_optical_sensor.get_proximity();
      const char *auto_close_state = claw_auto_close_armed ? "ARMED" : rearm_when_clear ? "waiting for clear" : "off";
      pros::lcd::print(2, "Claw proximity: %d  %s", claw_proximity, auto_close_state);

      // the gains the chassis is using, these update after running the tuner
      pros::lcd::print(3, "Angular kP %.3f kD %.3f", angular_controller.kP, angular_controller.kD);
      pros::lcd::print(4, "Lateral kP %.3f kD %.3f", lateral_controller.kP, lateral_controller.kD);

      pros::delay(50);
    }
  });
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
const double TIP_THRESHOLD_DEG = 8.0; // degrees of tilt before anti-tip engages
const double ANTI_TIP_GAIN = 5.0;     // how aggressively it drives under itself

// lift (heights and geometry are in lift.h)
const double LIFT_BOTTOM_TOLERANCE_DEG = 50; // within this of 0 counts as "at the bottom"

// claw
const int CLAW_AUTO_CLOSE_PROXIMITY = 255;       // optical sensor proximity (0 far to 255 touching), the claw closes at this
const double CLAW_AUTO_CLOSE_MAX_LIFT_DEG = 100; // lift motor degrees, only auto close below this (objects standing on the ground)
const double CLAW_AUTO_CLOSE_REARM_HEIGHT_IN = 7; // claw height from the floor, auto close re-arms when the lift comes back down below this

// motor power limits
const int FULL_POWER = 127;

void opcontrol() {
  int direction = 1; // 1 = normal throttle, -1 = inverted
  bool last_x = false;
  bool last_intaking = false;
  bool lift_auto_moving = false; // lift is moving itself (intake height or A lowering to the bottom)
  bool lowering_to_bottom = false; // A is lowering the lift to the bottom

  // wrist & claw
  bool wrist_up = true;
  bool claw_closed = true;
  bool last_lift_low = true;

  while (true) {
    double current_imu_roll = imu.get_roll();

    const double lift_motor_position = lift_motors.get_position();

    const double LIFT_HEIGHT_IN = lift::getHeight();

    //
    // DRIVETRAIN (with anti-tip)
    if (std::abs(current_imu_roll) > TIP_THRESHOLD_DEG) {
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
    if (intaking && !last_intaking && lift_motor_position < LIFT_BOTTOM_TOLERANCE_DEG) {
      lift_auto_moving = true;
      lowering_to_bottom = false;
      lift::setHeight(LIFT_INTAKE_HEIGHT_IN);
    }
    last_intaking = intaking;

    //
    // LIFT (R1 up, L1 down)
    const bool lift_up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    const bool lift_down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    if (lift_up || lift_down) {
      // driver takes over from any auto move
      lift_auto_moving = false;
      lowering_to_bottom = false;
    }

    //
    // LOWER TO BOTTOM (A), the driver opens the claw (Y) at the right time on the way down
    // pressing A again (or R1/L1 above) cancels it and holds the lift where it is
    if (lowering_to_bottom && lift::isDone()) {
      lowering_to_bottom = false; // reached the bottom, so the next A press starts a new one
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      if (lowering_to_bottom) {
        lift_auto_moving = false; // the manual lift code below stops and holds it
        lowering_to_bottom = false;
      } else {
        lift::setHeight(0);
        lift_auto_moving = true;
        lowering_to_bottom = true;
      }
    }

    if (lift_up) {
      lift_motors.move(FULL_POWER);
    } else if (lift_down && lift_motor_position > 0) {
      lift_motors.move(-FULL_POWER);
    } else if (!lift_auto_moving) {
      lift_motors.move(0);
    }

    //
    // TOGGLE CLAW (Y, once per press)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      claw_closed = !claw_closed;
      clawPiston.set_value(!claw_closed); // piston out opens the claw
      // opening needs the object to leave before auto close works again,
      // otherwise it would grab right back onto what was just let go
      if (!claw_closed) {
        claw_auto_close_armed = false;
      }
      rearm_when_clear = !claw_closed;

      // the cup is let go, so finish lowering at full speed
      if (!claw_closed && lowering_to_bottom) {
        lift::setHeight(0, LIFT_TIMEOUT_MS, true, LIFT_MAX_VELOCITY_RPM);
      }
    }

    // auto close the claw when an object on the ground reaches it
    // it fires once, then re-arms when either
    //  - the lift comes back down below CLAW_AUTO_CLOSE_REARM_HEIGHT_IN, or
    //  - the driver opens the claw (Y) and the object is gone from the sensor
    // so the driver can open the claw to drop something without it closing again
    const bool lift_low = lift::getHeight() + LIFT_BOTTOM_HEIGHT_IN < CLAW_AUTO_CLOSE_REARM_HEIGHT_IN;
    if (lift_low && !last_lift_low) {
      claw_auto_close_armed = true;
    }
    last_lift_low = lift_low;

    const std::int32_t claw_proximity = claw_optical_sensor.get_proximity();
    const bool object_in_claw = claw_proximity != PROS_ERR && claw_proximity >= CLAW_AUTO_CLOSE_PROXIMITY;
    if (rearm_when_clear && !object_in_claw) {
      claw_auto_close_armed = true;
      rearm_when_clear = false;
    }

    // only when the claw is open, so holding an object doesn't use up the arm
    if (claw_auto_close_armed && !claw_closed && object_in_claw && lift_motor_position < CLAW_AUTO_CLOSE_MAX_LIFT_DEG) {
      claw_closed = true;
      clawPiston.set_value(!claw_closed); // piston out opens the claw
      claw_auto_close_armed = false;
    }

    //
    // TOGGLE WRIST (R2, once per press)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
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
    // PID AUTO TUNER (LEFT angular, RIGHT lateral)
    // only works when not plugged into a field or competition switch, so it
    // can't start during a match. B stops it early
    if (!pros::competition::is_connected()) {
      if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        autoTuneAngularPID();
      } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        autoTuneLateralPID();
      }
    }

    pros::delay(20);
  }
}
