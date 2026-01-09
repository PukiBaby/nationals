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
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors

  // print position to brain screen (task must outlive initialize)
  static pros::Task screen_task([]() {
    while (true) {
      // get pose once per loop
      auto p = chassis.getPose();

      // format doubles safely (some embedded printf's don't support %f)
      std::ostringstream xs, ys, ts;
      xs << std::fixed << std::setprecision(2) << p.x;
      ys << std::fixed << std::setprecision(2) << p.y;
      ts << std::fixed << std::setprecision(2) << p.theta;

      pros::lcd::set_text(0, ("X: " + xs.str()).c_str());
        // .str() gets the string's content
        // .c_str gets the pointer to the string
      pros::lcd::set_text(1, ("Y: " + ys.str()).c_str());
      pros::lcd::set_text(2, ("Theta: " + ts.str()).c_str());

      // update every 100ms to save resources
      pros::delay(100);
    }
  });

  // watch for autonomous abort (task must outlive initialize)
  static pros::Task autonomous_abort_task([]() {
    while (true) {
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        // autonomous_task.suspend();
      }
      pros::delay(100);
    }
  });
}

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

void autonomous() {
  // set position to x:0, y:0, heading:0
  chassis.setPose(0, 0, 0);
  outtake_value = false;
  outtake_pneumatics.set_value(outtake_value);
  blocker_value = true;
  blocker.set_value(blocker_value);
  intake_mg.move(127);
  // high goal right side
  /*
  intake_mg.move(100);
  chassis.moveToPose(0, 23, 0, 1150, {.minSpeed = 115, .earlyExitRange = 4});
  chassis.moveToPose(7.5, 40.5, -10, 5000, {.maxSpeed = 46});
  chassis.moveToPose(-1.6, 47.2, 315, 500, {.maxSpeed = 127});
  chassis.moveToPose(33.8, 15.8, 180, 2500,
                     {.forwards = false, .maxSpeed = 127});
  pros::Task([]() {
    pros::delay(2000); // wait 2s
    scraper_value = true;
    scraper.set_value(scraper_value);
  });
  // Start a task to monitor the front distance sensor and cancel the motion if
  // an object is closer than 135 mm
  pros::Task([]() {
    while (chassis.isInMotion()) {
      int d = front_dist.get(); // distance in mm
      if (d > 0 && d < 130) {   // 135 mm threshold
        chassis.cancelMotion();
        break;
      }
      pros::delay(10);
    }
  });
  chassis.moveToPose(32.5, 4.5, 182, 1600, {.maxSpeed = 127});
  pros::delay(1900);
  pros::Task([]() {
    outtake_value = false;
    outtake_pneumatics.set_value(outtake_value);
    blocker_value = true;
    blocker.set_value(blocker_value);
    intake_mg.move(127);
  });
  chassis.moveToPose(33.4, 33.9, 180, 1000,
                     {.forwards = false, .maxSpeed = 127});
  pros::Task([]() {
    pros::delay(1100);
    outtake_value = false;
    outtake_pneumatics.set_value(outtake_value);
    blocker_value = false;
    blocker.set_value(blocker_value);
    intake_mg.move(127);
    scraper_value = false;
    scraper.set_value(scraper_value);
  });
  pros::delay(3000);
  intake_mg.brake();
  chassis.setPose(0, 0, 180);
  chassis.moveToPoint(1, -16, 3000, {.maxSpeed = 127});
  chassis.turnToHeading(210, 500);
  chassis.moveToPoint(10.9,  -2.9, 3000, {.forwards = false, .maxSpeed = 127});
  chassis.turnToHeading(179, 500);
  descore_value = true;
  descore.set_value(descore_value);
  chassis.moveToPoint(9.8, 14.5, 3000, {.forwards = false, .minSpeed = 120});
  pros::Task([]() {
    pros::delay(990); // wait 2s
    descore_value = false;
    descore.set_value(descore_value);
  });
  */

  // solo awp option #2
  chassis.setPose(0, 0, 90);
  chassis.moveToPoint(32.5, 0, 2000, {.maxSpeed = 127});
  chassis.turnToHeading(180, 500);
  pros::Task([]() {
    pros::delay(500);
    scraper_value = true;
    scraper.set_value(scraper_value);
  });
  pros::Task([]() {
    int consecutive = 0;
    const int needed = 3; // require 3 consistent reads
    const int threshold = 135;
    while (chassis.isInMotion()) {
      int d = front_dist.get(); // mm
      // optional: log to LCD so you can see the values
      pros::lcd::set_text(3, ("dist: " + std::to_string(d)).c_str());

      if (d > 10 && d < threshold) { // ignore 0/invalid readings
        consecutive++;
        if (consecutive >= needed) {
          chassis.cancelMotion();
          break;
        }
      } else {
        consecutive = 0;
      }
      pros::delay(20); // sampling period
    }
  });
  chassis.moveToPose(32.5, -12, 182, 1600, {.maxSpeed = 127});
  pros::delay(1900); 
  pros::Task([]() { // Storage
    outtake_value = false;
    outtake_pneumatics.set_value(outtake_value);
    blocker_value = true;
    blocker.set_value(blocker_value);
    intake_mg.move(127);
    pros::delay(1000); // Tune this
    intake_mg.move(0);
  });
  chassis.moveToPose(32.5, -6, 182, 1600, {.forwards = false, .maxSpeed = 127});
  pros::Task([]() { // Outtake into top
    intake_mg.move(127);
    scraper_value = false;
    scraper.set_value(scraper_value);
  });

  /*
  pros::Task([]() { // Watch for opponent color (need to have both options and a variable for our team)
    int counter = 0;
    while (true) {
      // if opponent color
        intake_mg.move(-127); // reverse
        counter = 0;
      // if not opponent color
        counter += 100;
      // if counter = 500
        intake_mg.move(0);
      pros::delay(100);
    }
  });
  */
  
  /*
  pros::delay(3000);
  blocker_value = true;
  blocker.set_value(blocker_value);
  chassis.moveToPose(32.5, 7.5, 180, 1000, {.maxSpeed = 127});
  pros::Task([]() {
    pros::delay(1100);
    outtake_value = false;
    outtake_pneumatics.set_value(outtake_value);
    blocker_value = false;
    blocker.set_value(blocker_value);
    intake_mg.move(127);
    scraper_value = false;
    scraper.set_value(scraper_value);
  });
  pros::delay(3000);
  blocker_value = true;
  blocker.set_value(blocker_value);
  chassis.moveToPose(32.5, 7.5, 180, 1000, {.maxSpeed = 127});
  chassis.moveToPoint(15.1, 17.2, 1000, {.minSpeed = 110, .earlyExitRange = 3});
  chassis.moveToPose(6.2, 27.6, 312, 3000, {.maxSpeed = 50});
  chassis.moveToPose(-0.5, 32.1, 315, 2000, {.maxSpeed = 127});
  pros::Task([]() {
    pros::delay(800);
    intake_mg.move(-115); // wait 0.5s
  });
  pros::delay(2200);
  intake_mg.move(127);
  chassis.moveToPose(6.3, 23.7, 275, 1000,
                     {.forwards = false, .minSpeed = 110});
  chassis.moveToPose(-33, 23.7, 270, 5000,
                     {.minSpeed = 70, .earlyExitRange = 4});
  chassis.moveToPose(-28.2, 35.2, 225, 5000,
                     {.forwards = false, .maxSpeed = 127});
  pros::Task([]() {
    pros::delay(1000);
    outtake_value = true;
    outtake_pneumatics.set_value(outtake_value);
    blocker_value = false; // switched, open
    blocker.set_value(blocker_value);
    intake_mg.move(127);
  });*/

  /*
    //high goal left side
  intake_mg.move(100);
  chassis.moveToPose(0, 23, 0, 1150, {.minSpeed = 115, .earlyExitRange = 4});
  chassis.moveToPose(-7.7, 40.5, -10, 5000, {.maxSpeed = 46});
  chassis.moveToPose(1.6, 47.2, 315, 2000, {.maxSpeed = 127});
  chassis.moveToPose(-34.7, 14.5, 180, 2500,
                     {.forwards = false, .maxSpeed = 127});
  pros::Task([]() {
    pros::delay(1800); // wait 2s
    scraper_value = true;
    scraper.set_value(scraper_value);
  });
  chassis.moveToPose(-34.7, 4.3,182,1600, {.maxSpeed = 100});
  pros::delay(2000);
  pros::Task([]() {
    outtake_value = false;
    outtake_pneumatics.set_value(outtake_value);
    blocker_value = true;
    blocker.set_value(blocker_value);
    intake_mg.move(127);
  });
  chassis.moveToPose(-34.9, 34.7, 180, 2000,
                     {.forwards = false, .maxSpeed = 127});
  pros::Task([]() {
    pros::delay(1100);
    outtake_value = false;
    outtake_pneumatics.set_value(outtake_value);
    blocker_value = false;
    blocker.set_value(blocker_value);
    intake_mg.move(127);
    scraper_value = false;
    scraper.set_value(scraper_value);
  });
  /*
  // partner awp right side
  intake_mg.move(100);
  chassis.moveToPose(0, 23, 0, 1150, {.minSpeed = 115, .earlyExitRange = 4});
  chassis.moveToPose(7.7, 40.5, -10, 5000, {.maxSpeed = 46});
  chassis.moveToPose(-1.6, 47.2, 315, 2000, {.maxSpeed = 127});
  pros::Task([]() {
    pros::delay(800);
    intake_mg.move(-115); // wait 0.5s
  });
  pros::delay(2200);
  intake_mg.move(127);
  chassis.moveToPose(34.7, 14.5, 180, 2500,
                     {.forwards = false, .maxSpeed = 127});
  pros::Task([]() {
    pros::delay(1800); // wait 2s
    scraper_value = true;
    scraper.set_value(scraper_value);
  });
  chassis.moveToPose(34.7, 4.3,182,1600, {.maxSpeed = 100});
  pros::delay(2000);
  pros::Task([]() {
    outtake_value = false;
    outtake_pneumatics.set_value(outtake_value);
    blocker_value = true;
    blocker.set_value(blocker_value);
    intake_mg.move(127);
  });
  chassis.moveToPose(34.9, 34.7, 180, 2000,
                     {.forwards = false, .maxSpeed = 127});
  pros::Task([]() {
    pros::delay(1100);
    outtake_value = false;
    outtake_pneumatics.set_value(outtake_value);
    blocker_value = false;
    blocker.set_value(blocker_value);
    intake_mg.move(127);
    scraper_value = false;
    scraper.set_value(scraper_value);
  });
  // solo awp
  /*
   chassis.moveToPose(8.2, 42, 14, 5000, {.minSpeed = 70, .earlyExitRange =
  4});
   /*pros::Task([]() {
     pros::delay(750);    // wait 0.5s
     scraper_value = true;
     scraper.set_value(scraper_value); // start action
     pros::delay(800);   // run for 1.5s
     scraper_value = false;
     scraper.set_value(scraper_value);  // stop
   });

   chassis.moveToPose(-3.6, 48.7, 314, 5000, {.maxSpeed = 127});
   pros::Task([]() {
     pros::delay(800);
     intake_mg.move(-100); // wait 0.5s
   });
   pros::delay(1800);
   intake_mg.move(127);
   chassis.moveToPose(4.5, 40.5, 280, 5000,
                      {.forwards = false, .minSpeed = 110});
   chassis.moveToPose(-38, 41.7, 280, 5000,
                      {.minSpeed = 70, .earlyExitRange = 4});
   chassis.moveToPose(-31, 51.5, 225, 5000,
                      {.forwards = false, .maxSpeed = 127});
   pros::Task([]() {
     pros::delay(1000);
     outtake_value = true;
     outtake_pneumatics.set_value(outtake_value);
     blocker_value = false; // switched, open
     blocker.set_value(blocker_value);
     intake_mg.move(127);
     pros::delay(1000);
     intake_mg.move(127);
     outtake_value = false;
     outtake_pneumatics.set_value(outtake_value);
     blocker_value = true;
     blocker.set_value(blocker_value);
   });
   chassis.moveToPose(-47, 22.5, 220, 5000,
                      {.minSpeed = 90, .earlyExitRange = 4});
   chassis.moveToPose(-68, 17.418, 180, 2000, {.maxSpeed = 127});
   pros::Task([]() { // wait 0.5s
     scraper_value = true;
     scraper.set_value(scraper_value);
   });
   chassis.moveToPose(-68, 4, 180, 1500, {.maxSpeed = 127});
   pros::delay(1700);
   pros::Task([]() {
     outtake_value = false;
     outtake_pneumatics.set_value(outtake_value);
     blocker_value = true;
     blocker.set_value(blocker_value);
     intake_mg.move(127);
   });
   chassis.moveToPose(-68, 34.7, 180, 2000,
                      {.forwards = false, .maxSpeed = 127});
   pros::Task([]() {
     pros::delay(1000);
     outtake_value = false;
     outtake_pneumatics.set_value(outtake_value);
     blocker_value = false;
     blocker.set_value(blocker_value);
     intake_mg.move(127);
     scraper_value = false;
     scraper.set_value(scraper_value);
   });*/
}

void opcontrol() {
  autonomous_is_running = true;
  pros::lcd::set_text(6, "Autonomous mode.");
  if (autonomous_is_running == true) {
    autonomous_selection_variable = autonomous_routine_class::test;
    pros::Task autonomous_task([](void* param) {
        auto selection_ptr = static_cast<autonomous_routine_class*>(param);
        run_autonomous(*selection_ptr, 0); // replace 0 with whatever int you need
    }, &autonomous_selection_variable, "Autonomous Task");
    while (autonomous_is_running) {
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        autonomous_task.suspend();
        autonomous_is_running = false;
      }
      pros::delay(100);
    }
  }

  // Defaults
  pros::lcd::set_text(6, "Driving mode.");
  park.set_value(park_value);
  scraper.set_value(scraper_value);
  descore_value = false;
  descore.set_value(descore_value);
  blocker.set_value(blocker_value);
  
  while (true) {
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    chassis.curvature(leftY, rightX);

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      park_value = !park_value;
      park.set_value(park_value);
      pros::delay(170); // Toggle
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      scraper_value = !scraper_value;
      scraper.set_value(scraper_value);
      pros::delay(170); // Toggle
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) &&
            !master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      blocker_value = false;
      blocker.set_value(blocker_value);
      outtake_value = false;
      outtake_pneumatics.set_value(outtake_value);
      intake_mg.move(127);
      pros::delay(10); // Hold, need to check
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
            !master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // Descore macro

      /*
      chassis.setPose(0, 0, 180);
      chassis.moveToPose(0, -15, 0, 3000, {.maxSpeed = 127});
      chassis.turnToHeading(-31, 500);

      chassis.moveToPoint(-11, -4.2, 3000, {.forwards = false, .maxSpeed = 127}); 
      chassis.turnToHeading(180, 500); 
      descore_value = true;
      descore.set_value(descore_value);
      chassis.moveToPoint(-10.5,-20, 3000, {.forwards = false, .maxSpeed = 127});
      */

      chassis.setPose(0, 0, 180);
      chassis.moveToPoint(1, -16, 3000, {.maxSpeed = 127});
      chassis.turnToHeading(210, 500);
      chassis.moveToPoint(10.9, -2.3, 3000,
                          {.forwards = false, .maxSpeed = 127});
      chassis.turnToHeading(179, 500);
      descore_value = true;
      descore.set_value(descore_value);
      chassis.moveToPoint(9.8, 18.5, 3000,
                          {.forwards = false, .minSpeed = 120});
      pros::Task([]() {
        pros::delay(990); // wait 2s
        descore_value = false;
        descore.set_value(descore_value);
      });
      pros::delay(200); // Toggle
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      outtake_value = true;
      outtake_pneumatics.set_value(outtake_value);
      blocker_value = false; // switched, open
      blocker.set_value(blocker_value);
      intake_mg.move(127);
      pros::delay(10); // Hold, need to check
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      outtake_value = false;
      outtake_pneumatics.set_value(outtake_value);
      blocker_value = true;
      blocker.set_value(blocker_value);
      intake_mg.move(127);
      pros::delay(10); // Hold, need to check
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      scraper_value = false;
      scraper.set_value(scraper_value);
      intake_mg.move(-127);
      pros::delay(10); // Hold
    }

    else {
      intake_mg.move(0);
    }

    // delay to save resources
    pros::delay(25);
  }
}  
