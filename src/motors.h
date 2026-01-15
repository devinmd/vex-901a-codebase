#ifndef MOTORS_H
#define MOTORS_H

#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

// Controller
extern pros::Controller controller;

// Pistons
extern pros::adi::DigitalOut tonguePiston;
extern pros::adi::DigitalOut conveyorPiston;
extern pros::adi::DigitalOut wingPiston;

// Motor Groups
extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;
extern pros::Motor bottomIntake;
extern pros::Motor topIntake;
extern pros::MotorGroup fullIntake;

// Sensors
extern pros::Imu imu;
extern pros::Rotation vertical_rotation_sensor;

// Drivetrain
extern lemlib::Drivetrain drivetrain;

// Tracking Wheels
extern lemlib::TrackingWheel vertical_tracking_wheel;

// Controllers
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

// Odometry Sensors
extern lemlib::OdomSensors sensors;

// Chassis
extern lemlib::Chassis chassis;

#endif // MOTORS_H