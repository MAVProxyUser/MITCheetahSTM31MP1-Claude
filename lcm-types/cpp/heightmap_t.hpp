// Minimal POD stand-in for the lcm-gen output (STM32MP1 port).
#ifndef __heightmap_t_hpp__
#define __heightmap_t_hpp__
#include <stdint.h>

struct heightmap_t {
  double map[100][100];
  double robot_loc[3];
};

#endif
