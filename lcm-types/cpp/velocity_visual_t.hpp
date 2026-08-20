// Minimal POD stand-in for the lcm-gen output (STM32MP1 port).
#ifndef __velocity_visual_t_hpp__
#define __velocity_visual_t_hpp__
#include <stdint.h>

struct velocity_visual_t {
  double vel_cmd[3];
  double base_position[3];
};

#endif
