#include "main.h"

#include "subsystems\declarations.hpp"
#include "subsystems\autonomous.hpp"

#include "pros/ai_vision.hpp"
#include "pros/vision.hpp"

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

#include <math.h>

// Driving

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup right_mg({DRIVING_RB_PORT, DRIVING_RM_PORT, DRIVING_RF_PORT},
                          pros::MotorGearset::blue);
pros::MotorGroup left_mg({DRIVING_LB_PORT, DRIVING_LM_PORT, DRIVING_LF_PORT},
                         pros::MotorGearset::blue);

// Game mechanisms

pros::MotorGroup intake_mg({MOTOR_UPTAKE_PORT, MOTOR_INTAKE_PORT});
pros::Vision vision_sensor(VISION_SENSOR_PORT);

// Pneumatics mechanisms

pros::adi::DigitalOut park(PARK_PORT);
bool park_value = false;

pros::adi::DigitalOut scraper(SCRAPER_PORT);
bool scraper_value = false;

pros::adi::DigitalOut descore(DESCORE_PORT);
bool descore_value = false;

pros::adi::DigitalOut outtake_pneumatics(OUTTAKE_PORT);
bool outtake_value = true;

pros::adi::DigitalOut blocker(BLOCKER_PORT);
bool blocker_value = false;

// Odometry

pros::Rotation horizontalEnc(H_ENC_PORT);
pros::Rotation verticalEnc(V_ENC_PORT);
pros::IMU imu(IMU_PORT);

// More sensors

pros::Distance front_dist(FRONT_DISTANCE_SENSOR_PORT);

// LEMLIB

lemlib::Drivetrain drivetrain(&left_mg,                   // left motor group
                              &right_mg,                  // right motor group
                              11.3,                       // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 2" omnis
                              480, // drivetrain rpm is 360
                              8    // horizontal drift is 2 (for now)
);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, lemlib::Omniwheel::NEW_2, -0.25);
lemlib::TrackingWheel vertical_tracking_wheel(&verticalEnc, lemlib::Omniwheel::NEW_2, -1);

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);

lemlib::ControllerSettings
    lateral_controller(10,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       12,  // derivative gain (kD)
                       2,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

lemlib::ControllerSettings
    angular_controller(5.7, // proportional gain (kP)
                       0,   // integral gain (kI)
                       40,  // derivative gain (kD)
                       5,   // anti windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);