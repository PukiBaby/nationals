#include "main.h"

#include "subsystems\declarations.hpp"
#include "subsystems\autonomous.hpp"
#include "subsystems\helper_functions.hpp"

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

#include <iomanip>
#include <math.h>
#include <sstream>

// Needed for cooperative scheduling, which is NOT implemented right now
#include <functional>
#include <vector> // Use to store the steps
#include <atomic> // Types that multiple threads can simultaneously operate on without causing undefined behavior.

bool autonomous_is_running = false;
bool run_autonomous_straight = false;
autonomous_routine_class autonomous_selection_variable;
int limit;
void run_autonomous (autonomous_routine_class selection)
{
    int stage = 0;
    while (stage < limit) {
        // Run the selected stage
        switch (selection) {
            case autonomous_routine_class::test:
                switch (stage) {
                    case 0:
                        break;
                    case 1:
                        break;
                    // Placeholder
                }
                break;
            case autonomous_routine_class::solo_awp_2:
                switch (stage) {
                    case 0:
                        chassis.setPose(0, 0, 90);
                        chassis.moveToPoint(38.562, 0, 2000, {.maxSpeed = 127});
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
                        chassis.moveToPose(36.562, -10.058, 180, 1600, {.maxSpeed = 127});
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
                        break;
                    // Placeholder
                }
                break;
            case autonomous_routine_class::solo_awp_1:
                switch (stage) {
                    case 0:
                        break;
                    case 1:
                        break;
                    // Placeholder; add more soon
                }
                break;
        }

        // Delay until user input says to move to the next stage
        while (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && !pros::competition::is_autonomous()) { // Wait for user input to move on when running autonomous in driver control mode
            pros::lcd::set_text(7, "Press B to move to the next stage.");
            pros::delay(100);
        }

        // Increment
        stage += 1;
    }
}

