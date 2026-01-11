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
                        chassis.setPose(0, 0, 0);
                        oscillateHeading_cycles(10, 10.0);
                        break;
                    case 1:
                        chassis.moveToPoint(0, 10, 1000);
                        break;
                }
                break;
            case autonomous_routine_class::solo_awp_2:
                switch (stage) {
                    case 0:
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
                    // Placeholder
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
    autonomous_is_running = false;
}

