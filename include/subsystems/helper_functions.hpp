#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#define DEGREES_PER_BLOCK 180

enum class BlockColor {Red, Blue, Gray, NotApplicable};
BlockColor determine_color_of_block(double hue, double saturation);
extern int number_of_blocks_collected;
void collect_blocks_emptiness(int desired, int timeout);
void collect_blocks_encoder(int desired, int timeout);
void oscillateHeading_cycles (int cycles, double amplitude);
void oscillateHeading_while (bool condition, double amplitude);

#endif // HELPER_FUNCTIONS_H