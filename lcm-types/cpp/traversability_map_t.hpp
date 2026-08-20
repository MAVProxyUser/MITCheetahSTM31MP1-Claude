// Minimal POD stand-in for the lcm-gen output (STM32MP1 port).
#ifndef __traversability_map_t_hpp__
#define __traversability_map_t_hpp__
#include <stdint.h>

struct traversability_map_t {
  int32_t map[100][100];
};

#endif
