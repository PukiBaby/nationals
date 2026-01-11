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
BlockColor determine_color_of_block(double hue, double saturation) {
    if (saturation <= 20) {
        return BlockColor::Gray;
    }
    else if (hue < 50 || hue > 280) {
        return BlockColor::Red;
    }
    else if (200 < hue && hue < 265) {
        return BlockColor::Blue;
    }
    else {
        return BlockColor::Red;
    }
}

int number_of_blocks_collected = 0;
void collect_blocks_emptiness(int desired, int timeout = 3000) {
    number_of_blocks_collected = 0;
    int start_time = pros::millis();
    bool block_present = false; // flag to avoid overcounting the same block by relying on the ASSUMPTION that there is emptiness between when a block leaves
    intake_mg.move(127);
    while (number_of_blocks_collected < desired && pros::millis() - start_time < timeout) {
        bool valid_block = ball_facing_dist.get_distance() <= 14.5/2 // half of bot width, need to tune
                            && (determine_color_of_block(optical_sensor.get_hue(), optical_sensor.get_saturation()) == OUR_COLOR);
        if (valid_block && !block_present) { 
            number_of_blocks_collected += 1;
            block_present = true;
        }
        if (!valid_block) {
            block_present = false;
        }
        pros::delay(100);
    }
    intake_mg.move(0);
}

void collect_blocks_encoder(int desired, int timeout = 3000) {
    number_of_blocks_collected = 0;
    int start_time = pros::millis();
    double last_pos = intake_mg.get_position();
    intake_mg.move(127);
    while (number_of_blocks_collected < desired && pros::millis() - start_time < timeout) {
        bool valid_block = ball_facing_dist.get_distance() <= 14.5/2 // half of bot width, need to tune
                            && (determine_color_of_block(optical_sensor.get_hue(), optical_sensor.get_saturation()) == OUR_COLOR);
        double pos = intake_mg.get_position();
        double delta = fabs(pos - last_pos);
        
        if (delta > DEGREES_PER_BLOCK && valid_block) { 
            number_of_blocks_collected += 1;
            last_pos = pos;
        }
        pros::delay(10);
    }
    intake_mg.move(0);
}

void oscillateHeading_cycles (int cycles, double amplitude) {
    double base_heading = chassis.getPose().theta;
    for (int i = 0; i < cycles; i++) {
        chassis.turnToHeading(base_heading + amplitude, 500, {}, false);
        chassis.turnToHeading(base_heading - amplitude, 500, {}, false);
    }
    chassis.turnToHeading(base_heading, 500);
}

void oscillateHeading_while (bool condition, double amplitude) {
    double base_heading = chassis.getPose().theta;
    while (condition) {
        chassis.turnToHeading(base_heading + amplitude, 500, {}, false);
        chassis.turnToHeading(base_heading - amplitude, 500, {}, false);
    }
    chassis.turnToHeading(base_heading, 500);
}