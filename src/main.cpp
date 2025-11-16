#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

//
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::adi::DigitalOut piston('A');
pros::MotorGroup left_mg({-2, 3, -4}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({8, 9, -10}, pros::MotorGearset::blue);
pros::Motor bottomIntake(-6);
pros::Motor topIntake(-7);
pros::MotorGroup fullIntake({-6, -7});
pros::Imu imu(5);
pros::Rotation horizontal_rotation_sensor(19); 
pros::Rotation vertical_rotation_sensor(20);   

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 14.5,
                              lemlib::Omniwheel::NEW_325, 450, 2);

// tracking wheels
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotation_sensor,
                                                lemlib::Omniwheel::NEW_2,
                                                0); // TODO: set offset distance
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation_sensor,
                                              lemlib::Omniwheel::NEW_2,
                                              0); // TODO: set offset distance

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(13,  //
                       0,   //
                       3,   //
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(2,  // 2
                       0,  // skipped
                       12, // 11 or 12
                       0,  // skipped
                       0,  // small error range, in inches
                       0,  // small error range timeout, in milliseconds
                       0,  // large error range, in inches
                       0,  // large error range timeout, in milliseconds
                       0   // maximum acceleration (slew)
    );

// odometry settings
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr,                    // horizontal tracking wheel 2, set to nullptr
    &imu                        // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
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

  // chassis.setPose(-48.36, 16.2, 77.89); // start position
  // chassis.setPose(0, 0, 0); // start position

  // thread to for brain screen and position logging
  pros::Task screenTask([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // log position telemetry
      lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
      // delay to save resources
      pros::delay(50);
    }
  });

  pros::lcd::register_btn1_cb(on_center_button);
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

/// RIGHT SIDE
void autonomousRight() {

  chassis.setPose(-48.36, -16.2, 102.11); // start position
  chassis.turnToPoint(-23.85, -21.6, 80, {.maxSpeed = 120},
                      false); // middle balls
  fullIntake.move(127);       // run intake
  chassis.moveToPoint(-17.647, -23.811, 2000, {.maxSpeed = 60}, false);
  // pros::delay(1000);
  fullIntake.move(0); // end intake

  // move to match loader alignment spot
  chassis.turnToPoint(-42, -51, 500, {.maxSpeed = 60},
                      false); // turn to midpoint
  chassis.moveToPoint(-42, -51, 2000, {.maxSpeed = 60},
                      false); // go to midpoint

  chassis.turnToPoint(-60, -51, 500, {.maxSpeed = 60},
                      false); // face match loader

  chassis.moveToPoint(-10, -51, 10000, {.forwards = false, .maxSpeed = 60},
                      true); // back up into goal to align
}

// LEFT SIDE -- THIS WORKS & CAN SCORE 3 BALLS (MATCHLOAD IS DROPPED -- NEED TO FIX THIS)
void autonomous() {

  // set starting position on left side
  chassis.setPose(-48.36, 16.2, 77.89);
  // collect 3 middle balls
  chassis.turnToPoint(-23.85, 21.6, 100, {.maxSpeed = 120}, false);
  fullIntake.move(127);
  chassis.moveToPoint(-17.647, 23.811, 2000, {.maxSpeed = 60}, false);
  fullIntake.move(0);

  // move to goal & align, then run intake & score
  chassis.turnToPoint(-42, 51, 500, {.maxSpeed = 60}, false);
  chassis.moveToPoint(-42, 51, 2000, {.maxSpeed = 60}, false);
  chassis.turnToPoint(-60, 51, 500, {.maxSpeed = 60}, false);
  chassis.moveToPoint(-10, 51, 10000, {.forwards = false, .maxSpeed = 60},
                      true);
  fullIntake.move(127);
  pros::delay(10000);
  fullIntake.move(0);
}
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

  int direction = -1;
  bool lastX = false;
  bool xPressed = false;
  bool piston_state = false;

  while (true) {

    // curvature drive
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    chassis.curvature(leftY * direction, rightX);

    // intake motors
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ||
        controller.get_digital((pros::E_CONTROLLER_DIGITAL_A))) {
      // forward
      topIntake.move(127);
      bottomIntake.move(127);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) ||
               controller.get_digital((pros::E_CONTROLLER_DIGITAL_B))) {
      // backward
      topIntake.move(-127);
      bottomIntake.move(-127);
    } else {
      topIntake.move(0);
      bottomIntake.move(0);
    }

    // piston
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      piston_state = !piston_state;
      piston.set_value(piston_state);
    }

    // trigger auton for testing
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      // auton left
      autonomous();
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