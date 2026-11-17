#ifndef MOTORS_H
#define MOTORS_H

#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

// Controller
extern pros::Controller controller;

// Motor Groups
extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;
extern pros::MotorGroup lift_motors;
extern pros::Motor intake_motor;

extern pros::adi::DigitalOut clawPiston;
extern pros::adi::DigitalOut wristPiston;



// Sensors
extern pros::Imu imu;
extern pros::Optical claw_optical_sensor;
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
