#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// piston
pros::adi::DigitalOut piston('A');

// drivetrain motors
pros::MotorGroup left_mg({2, -3, -4}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({8, 9, -10}, pros::MotorGearset::blue);
// intake motors
pros::Motor bottomIntake(6);
pros::Motor topIntake(7);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg,                   // left motor group
                              &right_mg,                  // right motor group
                              10,                         // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // wheel type
                              450, // drivetrain rpm is 360
                              2    // horizontal drift is 2 (for now)
);

// imu
// pros::Imu imu(10);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(19);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(20);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_275,
                                                -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder,
                                              lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr,                    // horizontal tracking wheel 2, set to nullptr
    nullptr                        // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(10,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       3,   // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(2,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       10,  // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
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
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors
  // print position to brain screen
  /*pros::Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // delay to save resources
      pros::delay(20);
    }
  });*/

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

 ASSET(path_txt); // '.' replaced with "_" to make c++ happy


void autonomous() {

  chassis.setPose(0, 0, 0);
  // chassis.moveToPoint(10, 10, 5000);
  // chassis.follow(path_txt, 10, 5000, true);
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
    pros::lcd::print(0, "%d %d %d",
                     (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                     (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                     (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >>
                         0); // Prints status of the emulated screen LCDs

    // arcade drive
    // get left y and right x positions
    // int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    // int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

    // move the robot
    // prioritize steering slightly
    // chassis.arcade(leftY * direction, leftX, false, 0.75);

    // curvature drive
    // get left y and right x positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot
    chassis.curvature(leftY * direction, rightX);

    // arcade control
    // int dir = direction *
    //           controller.get_analog(ANALOG_LEFT_Y);   // invert
    //           forward/backward
    // int turn = controller.get_analog(ANALOG_RIGHT_X); // keep turning the
    // same

    // left_mg.move(dir + turn);
    // right_mg.move(dir - turn);

    // intake motors
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      // forward
      topIntake.move(-127);
      bottomIntake.move(-127);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      // backward
      topIntake.move(127);
      bottomIntake.move(127);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      // top backward
      topIntake.move(127);
      bottomIntake.move(-127);
    } else {
      topIntake.move(0);
      bottomIntake.move(0);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      piston_state = !piston_state;   // toggle state
      piston.set_value(piston_state); // activate or deactivate piston
    }

    
      if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        // autonomous();
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