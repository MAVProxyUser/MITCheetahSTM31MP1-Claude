// Minimal POD stand-in for the lcm-gen output (STM32MP1 port).
#ifndef __localization_lcmt_hpp__
#define __localization_lcmt_hpp__
#include <stdint.h>

struct localization_lcmt {
  float xyz[3];
  float rpy[3];
};

#endif
