// Minimal POD stand-in for the lcm-gen output (STM32MP1 port).
#ifndef __obstacle_visual_t_hpp__
#define __obstacle_visual_t_hpp__
#include <stdint.h>

struct obstacle_visual_t {
  int32_t num_obs;
  double location[100][3];
  double sigma;
  double height;
  double mesh_center_pos[3];
};

#endif
