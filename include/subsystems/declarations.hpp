// HEADER GUARDS

#ifndef DECLARATIONS_H
#define DECLARATIONS_H

// OTHER LIBRARIES

#include "main.h"

#include "subsystems\declarations.hpp"
#include "subsystems\autonomous.hpp"

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

#include <iomanip>
#include <math.h>
#include <sstream>

// PREPROCESSOR MACROS

// Driving

#define DRIVING_RB_PORT 11
#define DRIVING_RM_PORT 2
#define DRIVING_RF_PORT 13

#define DRIVING_LB_PORT -3
#define DRIVING_LM_PORT -14
#define DRIVING_LF_PORT -15

// Game mechanisms

#define MOTOR_UPTAKE_PORT 4
#define MOTOR_INTAKE_PORT 1

#define SCRAPER_PORT 'C'
#define OUTTAKE_PORT 'A'
#define DESCORE_PORT 'B'
#define PARK_PORT 'D'
#define BLOCKER_PORT 'E'

// More sensors

#define FRONT_DISTANCE_SENSOR_PORT 16
#define BACK_DISTANCE_SENSOR_PORT 6
#define BALL_FACING_DISTANCE_SENSOR_PORT 6
#define OPTICAL_SENSOR_PORT 10

#define OUR_COLOR BlockColor::Blue // or BlockColor::Red

// Odometry

#define H_ENC_PORT 12
#define V_ENC_PORT 17
#define IMU_PORT 20

// FORWARD DECLARATIONS

// Driving

extern pros::Controller master;

extern pros::MotorGroup right_mg;
extern pros::MotorGroup left_mg;

// Game mechanisms

extern pros::Motor uptake;
extern pros::Motor intake;

extern pros::MotorGroup intake_mg;

// Pneumatics game mechanisms

extern pros::adi::DigitalOut park;
extern bool park_value;

extern pros::adi::DigitalOut scraper;
extern bool scraper_value;

extern pros::adi::DigitalOut descore;
extern bool descore_value;

extern pros::adi::DigitalOut outtake_pneumatics;
extern bool outtake_value;

extern pros::adi::DigitalOut blocker;
extern bool blocker_value;

// Odometry

extern pros::Rotation horizontalEnc;
extern pros::Rotation verticalEnc;
extern pros::IMU imu;

// Sensors

extern pros::Distance front_dist;
extern pros::Distance ball_facing_dist;
extern pros::Optical optical_sensor;

// LEMLIB

extern lemlib::Drivetrain drivetrain;

extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::TrackingWheel vertical_tracking_wheel;

extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

extern lemlib::Chassis chassis;

#endif // DECLARATIONS_H