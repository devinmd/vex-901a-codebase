#include "motors.h"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Pistons
pros::adi::DigitalOut tonguePiston('B', true);
pros::adi::DigitalOut conveyorPiston('C', false);
pros::adi::DigitalOut wingPiston('A', true);

// Motor Groups
pros::MotorGroup left_mg({15, -14, -13}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({3, -2, 1}, pros::MotorGearset::blue);
pros::Motor bottomIntake(-12);
pros::Motor topIntake(-11);
pros::MotorGroup fullIntake({-12, -11});

// Sensors
pros::Imu imu(19);
pros::Rotation vertical_rotation_sensor(17);

// Drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 14.5,
                              lemlib::Omniwheel::NEW_325, 450, 2);

// Tracking wheels
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation_sensor,
                                              lemlib::Omniwheel::NEW_275,
                                              0); // TODO: set offset distance

// Lateral PID controller
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

// Angular PID controller
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

// Odometry settings
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr
    &imu     // inertial sensor
);

// Create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);