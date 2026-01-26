#include "motors.h"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Pistons
pros::adi::DigitalOut tonguePiston('B', true);
pros::adi::DigitalOut conveyorPiston('C', false);
pros::adi::DigitalOut wingPiston('A', false);

// Motor Groups
pros::MotorGroup left_mg({15, -14, -13}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({3, -2, 1}, pros::MotorGearset::blue);
pros::Motor bottomIntake(-12);
pros::Motor topIntake(-11);
pros::MotorGroup fullIntake({-12, -11});

// Sensors
pros::Imu imu(19);
pros::Rotation vertical_rotation_sensor(-17);

// Drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 14.5,
                              lemlib::Omniwheel::NEW_325, 450, 2);

// Tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation_sensor,
                                              lemlib::Omniwheel::NEW_275, 0);

/*

Tune kP until small oscillations, kD to dampen those. Only use kI for little
boosts for ranges of input where you have consistent miss at the end

angular with have higher kD because it has less friction to stop it

*/

/*

increase kP until it overshoots and moves back and forth a few times

increase kD until it stops smoothly and perfectly at 24 inches with no bouncing
or shaking
    - kD is the deceleration

  */

/*

increase kP until it turns quickly and overshoots a bit
stop increasing when it slightly overshoots
if it spins or vibrates crazy then its too high

increase kD until it stops perfectly at 90 degrees and stops smoothly without
bouncing angular kD usually higher than lateral

if it consistently stops at less than 90 degrees, increase kI a little to help
it get there


*/


// TUNED AS OF JANUARY 16 2026
int lateral_kP = 10; // 10-14 good // was 12
int lateral_kI = 0;
int lateral_kD = 90; // was 70

int angular_kP = 3; // 2 is no oscillation, 4 is slight, 3 is slight
int angular_kI = 0;
int angular_kD = 18; // 30 too much, 20 good, 16 too little

// Lateral PID controller
// set all other to 0 when tuning
// TUNED AS OF JANUARY 16 2026
lemlib::ControllerSettings
    lateral_controller(lateral_kP, lateral_kI, lateral_kD,
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

// Angular PID controller
// set all other to 0 when tuning
// TUNED AS OF JANUARY 16 2026
lemlib::ControllerSettings
    angular_controller(angular_kP, angular_kI, angular_kD,
                       0, // anti windup
                       0, // small error range, in inches
                       0, // small error range timeout, in milliseconds
                       0, // large error range, in inches
                       0, // large error range timeout, in milliseconds
                       0  // maximum acceleration (slew)
    );

// Odometry settings
lemlib::OdomSensors odometrySensors(&vertical_tracking_wheel, nullptr, nullptr,
                                    nullptr, &imu);

// Create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        odometrySensors);