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

#include "pros\colors.hpp"

#include <iomanip>
#include <math.h>
#include <sstream>

// see https://www.hslpicker.com/
// tune with actual blocks
std::string determine_color_of_block(double hue, double saturation) {
    if (saturation <= 20) {
        return "Gray";
    }
    else if (hue < 50 || hue > 280) {
        return "Red";
    }
    else if (200 < hue && hue < 265) {
        return "Blue";
    }
    else {
        return "N/A";
    }
}