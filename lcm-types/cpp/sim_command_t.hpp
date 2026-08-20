// Minimal POD stand-in for the lcm-gen output (STM32MP1 port).
#ifndef __sim_command_t_hpp__
#define __sim_command_t_hpp__
#include <stdint.h>
#include <vector>

struct sim_command_t {
  int32_t command_number;
  int32_t data_size;
  std::vector<double> data;   // lcm variable-length array
};

#endif
