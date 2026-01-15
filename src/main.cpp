#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

// constants
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::adi::DigitalOut tonguePiston('B', true);
pros::adi::DigitalOut conveyorPiston('C', false);
pros::adi::DigitalOut wingPiston('A', true);
pros::MotorGroup left_mg({15, -14, -13}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({3, -2, 1}, pros::MotorGearset::blue);
pros::Motor bottomIntake(-12);
pros::Motor topIntake(-11);
pros::MotorGroup fullIntake({-12, -11});
pros::Imu imu(19);
// pros::Rotation horizontal_rotation_sensor(16);
pros::Rotation vertical_rotation_sensor(17);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 14.5,
                              lemlib::Omniwheel::NEW_325, 450, 2);

// tracking wheels
/*
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotation_sensor,
                                               lemlib::Omniwheel::NEW_2,
                                               0); // TODO: set offset distance
*/

lemlib::TrackingWheel vertical_tracking_wheel(
    &vertical_rotation_sensor, lemlib::Omniwheel::NEW_275,
    0); // TODO: set offset distance // lateral PID controller

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
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr
    &imu     // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  chassis.calibrate();

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

void autonomousskills()

{
  int leftLongGoalY = 49;

  // starting position
  chassis.setPose(-48, 15, 0);
  // drive to align with matchloader
  chassis.moveToPose(-48, leftLongGoalY, 270, 1000,
                     {.forwards = true, .maxSpeed = 90}, false);
  // drop tongue
  tonguePiston.set_value(false);
  bottomIntake.move(127);
  pros::delay(200);
  // move into the match load
  chassis.moveToPoint(-62, leftLongGoalY, 2000,
                      {.forwards = true, .maxSpeed = 60}, false);
  // back out
  chassis.moveToPose(-48, leftLongGoalY, 1000, 270,
                     {.forwards = false, .maxSpeed = 90}, false);

  int leftOutsideLongGoalY = 60;

  // raise tongue
  tonguePiston.set_value(true);
  // drive to other side
  chassis.moveToPose(-36, leftOutsideLongGoalY, 90, 1000,
                     {.forwards = false, .maxSpeed = 90}, false);
  chassis.moveToPose(36, leftOutsideLongGoalY, 135, 1000,
                     {.forwards = false, .maxSpeed = 90}, false);
  chassis.moveToPose(48, leftLongGoalY, 270, 1000,
                     {.forwards = false, .maxSpeed = 90}, false);
  // drive into long goal & score

  // move to match load
  // drop tongue
  tonguePiston.set_value(false);
}

// SKILLS AUTON (JUST PARK)
void autonomousskillspark() {

  chassis.setPose(-61.7, -17, 0);
  tonguePiston.set_value(true);    // lifts tongue
  conveyorPiston.set_value(false); // raise conveyor

  // move backward
  chassis.moveToPoint(-61.7, -27, 5000, {.forwards = false, .maxSpeed = 60},
                      false);
  // move forward and clear park zone
  chassis.moveToPoint(-61.7, 0, 5000, {.maxSpeed = 60}, false);
}

// RIGHT SIDE AUTON
void autonomous() {
  // set starting position on right side
  chassis.setPose(-48.36, -16.2, 102.3);
  tonguePiston.set_value(true);    // lifts tongue
  conveyorPiston.set_value(false); // raise conveyor

  // collect 3 middle balls (storage)
  bottomIntake.move(127);
  chassis.moveToPoint(-23.5, -21.6, 2000, {.maxSpeed = 60}, false);
  bottomIntake.move(0);

  int ylocation = -47;

  // move to goal & align, then run intake & score
  chassis.turnToPoint(-42, ylocation, 500, {.maxSpeed = 60}, false);
  chassis.moveToPoint(-42, ylocation, 2000, {.maxSpeed = 80}, false);
  chassis.turnToPoint(-60, ylocation, 300, {.maxSpeed = 40}, false);

  // drop tongue, move to loader, intake
  tonguePiston.set_value(false);
  bottomIntake.move(127);
  pros::delay(200);
  // move into the match load
  chassis.moveToPoint(-62, ylocation, 2000, {.forwards = true, .maxSpeed = 50},
                      false);
  // drive into long goal
  chassis.moveToPoint(-22, ylocation, 5000, {.forwards = false, .maxSpeed = 70},
                      true);
  pros::delay(1000);
  // start scoring
  fullIntake.move(127);
  pros::delay(3000);
  fullIntake.move(0);

  tonguePiston.set_value(true);

  // use wing

  // move backward then punch
  chassis.moveToPoint(-39, ylocation, 1000, {.maxSpeed = 60}, false);

  chassis.moveToPoint(-22, ylocation, 500, {.forwards = false, .maxSpeed = 100},
                      true);

  /*
    chassis.turnToPoint(-26.5, -58, 500, {.forwards = false, .maxSpeed = 70},
                        false);

    chassis.moveToPoint(-26.53, -58, 2000, {.forwards = true, .maxSpeed = 70},
                        false);

    chassis.turnToPoint(-10.2, -58, 500, {.forwards = true, .maxSpeed = 60},
                        false);
    chassis.moveToPoint(-10.2, -58, 2000, {.forwards = true, .maxSpeed = 60},
                        false);*/
}

// LEFT SIDE -- THIS WORKS
void autonomousl() {

  // set starting position on left side
  chassis.setPose(-48.36, 16.2, 77.89);
  tonguePiston.set_value(true);    // lifts tongue
  conveyorPiston.set_value(false); // raise conveyor

  // collect 3 middle balls (storage)
  // run intake
  bottomIntake.move(127);
  // move to 3 middle balls
  chassis.moveToPoint(-23.5, 21.6, 2000, {.maxSpeed = 60}, false);
  // stop intake -- DO WE NEED THIS? BETTER TO CYCLE BALLS?
  bottomIntake.move(0);

  // go to mid-top goal
  // drop conveyor -- MAYBE DROP AFTER WE ARE AT GOAL?
  conveyorPiston.set_value(true);
  // turn to middle goal
  chassis.turnToPoint(-6.6, 8.3, 500,
                      {
                          .forwards = false,
                          .maxSpeed = 60,
                      },
                      false);
  // move to middle goal
  chassis.moveToPoint(-6.6, 8.3, 1000,
                      {
                          .forwards = false,
                          .maxSpeed = 60,
                      },
                      false);
  // drop conveyor -- DROP HERE????? -- TEST THIS
  conveyorPiston.set_value(true);
  // run intake to score balls to middle goal
  fullIntake.move(127);
  pros::delay(2000);

  // raise conveyor
  conveyorPiston.set_value(false);
  // stop intake
  fullIntake.move(0);

  int ylocation = 50;

  // move to mid position
  chassis.turnToPoint(-42, ylocation, 500, {.maxSpeed = 100}, false);
  chassis.moveToPoint(-42, ylocation, 2000, {.maxSpeed = 70},
                      false); // 60 speed works
  chassis.turnToPoint(-60, ylocation, 300, {.maxSpeed = 40}, false);

  // drop tongue
  tonguePiston.set_value(false);
  // run in take
  bottomIntake.move(127);
  // give time for tongue to drop
  pros::delay(200);
  // move into the match load and get balls
  chassis.moveToPoint(-62, ylocation, 2000, {.forwards = true, .maxSpeed = 60},
                      false);

  // move to long goal
  chassis.moveToPoint(-22, ylocation, 3000, {.forwards = false, .maxSpeed = 70},
                      true);
  // wait until bot gets to long goal
  pros::delay(1000);
  // score long goal
  fullIntake.move(127);
  pros::delay(2000);
  fullIntake.move(0);

  // move back then come back and punch
  chassis.moveToPoint(-39, ylocation, 500, {.maxSpeed = 60}, false);
  chassis.moveToPoint(-22, ylocation, 500, {.forwards = false, .maxSpeed = 100},
                      true);
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