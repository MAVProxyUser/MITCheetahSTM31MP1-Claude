// Minimal POD stand-in for the lcm-gen output (compute-derisk build; no encode/decode used by core).
#ifndef __leg_control_data_lcmt_hpp__
#define __leg_control_data_lcmt_hpp__

struct leg_control_data_lcmt {
  float q[12];
  float qd[12];
  float p[12];
  float v[12];
  float tau_est[12];
};

#endif
