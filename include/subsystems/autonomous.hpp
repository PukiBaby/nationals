#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include "main.h"

#include "subsystems\declarations.hpp"

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

enum class autonomous_routine_class {test, solo_awp_1, solo_awp_2};

extern int run_autonomous (autonomous_routine_class selection, int stage); // function declaration

extern bool autonomous_is_running;
extern autonomous_routine_class autonomous_selection_variable;

// std::atomic<bool> pause_autonomous;
// std::atomic<bool> abort_autonomous;
// Instantiate like this: std::atomic<bool> abort_autonomous{false};
// extern bool interruption_gate();

#endif // AUTONOMOUS_H