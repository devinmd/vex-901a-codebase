#include "motors.h"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor Groups
pros::MotorGroup left_mg({-10, -3}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({1, 2}, pros::MotorGearset::blue);
pros::MotorGroup lift_motors({11, -4}, pros::MotorGearset::green);
pros::Motor intake_motor(7, pros::MotorGearset::blue);

// Pistons
pros::adi::DigitalOut clawPiston('A', false); // true opens the claw, false closes it
pros::adi::DigitalOut wristPiston('B', false);


// Sensors
pros::Imu imu(6);
// pros::Rotation vertical_rotation_sensor(-17);
pros::Optical claw_optical_sensor(9); // sits right behind the claw

// Drivetrain configuration
lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 12.5, lemlib::Omniwheel::NEW_325, 360, 2);

// Vertical tracking wheel
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation_sensor, lemlib::Omniwheel::NEW_2, 0);

/*
Tune kP until small oscillations, kD to dampen those. Only use kI for little
boosts for ranges of input where you have consistent miss at the end
angular with have higher kD because it has less friction to stop it

increase kP until it overshoots and moves back and forth a few times
increase kD until it stops smoothly and perfectly at 24 inches with no bouncing
or shaking.
kD is the deceleration

increase kP until it turns quickly and overshoots a bit
stop increasing when it slightly overshoots
if it spins or vibrates crazy then its too high
increase kD until it stops perfectly at 90 degrees and stops smoothly without
bouncing angular kD usually higher than lateral
if it consistently stops at less than 90 degrees, increase kI a little to help
it get there
*/

// PID controller constants
int lateral_kP = 10;
int lateral_kI = 0;
int lateral_kD = 90;

int angular_kP = 3;
int angular_kI = 0;
int angular_kD = 18;

// Lateral PID controller
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
lemlib::ControllerSettings
    angular_controller(angular_kP, angular_kI, angular_kD,
                       0, // anti windup
                       0, // small error range, in inches
                       0, // small error range timeout, in milliseconds
                       0, // large error range, in inches
                       0, // large error range timeout, in milliseconds
                       0  // maximum acceleration (slew)
    );

// Odometry sensor settings
lemlib::OdomSensors odometrySensors(nullptr, nullptr, nullptr, nullptr, &imu);

// Create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, odometrySensors);
