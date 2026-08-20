// Minimal POD stand-in for the lcm-gen output (STM32MP1 port; core/driver use fields only).
#ifndef __cheetah_visualization_lcmt_hpp__
#define __cheetah_visualization_lcmt_hpp__

struct cheetah_visualization_lcmt {
  float q[12];
  float x[3];
  float quat[4];
  float rgba[4];
};

#endif
