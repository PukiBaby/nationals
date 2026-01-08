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

// Needed for cooperative scheduling, which is NOT implemented right now
#include <functional>
#include <vector> // Use to store the steps
#include <atomic> // Types that multiple threads can simultaneously operate on without causing undefined behavior.

bool autonomous_is_running = false;
bool run_autonomous_straight = false;
autonomous_routine_class autonomous_selection_variable;
void run_autonomous (autonomous_routine_class selection, int stage = 0)
{
    // Determine the number of stages
    int limit;
    switch (selection) {
        case autonomous_routine_class::test:
            limit = 2; // Placeholder, 2 stages
            break;
        case autonomous_routine_class::solo_awp_2:
            limit = 1; // Placeholder, 1 stage
            break;
        case autonomous_routine_class::solo_awp_1:
            limit = 2; // Placeholder, 2 stages
            break;
        default:
            pros::lcd::set_text(7, "Invalid autonomous selection.");
            return; // Exit function
    }

    // Do not permit any stages past the final stage to be run
    if (stage >= limit) { // Run stages 0, 1, 2, ..., limit - 1, i.e. the number of stages run is the variable limit.
        pros::lcd::set_text(7, "The autonomous routine has finished. You can drive now.");
        autonomous_is_running = false;
        return;
    }

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

    // Recursive logic to call the next stage
    while (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && !(run_autonomous_straight == true)) { // Wait for user input to move on
        pros::lcd::set_text(7, "Press B to move to the next stage.");
        pros::delay(100);
    }
    run_autonomous (selection, stage + 1); // Call the next stage
}

