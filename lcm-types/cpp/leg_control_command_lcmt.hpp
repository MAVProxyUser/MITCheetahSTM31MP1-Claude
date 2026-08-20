// Minimal POD stand-in for the lcm-gen output (compute-derisk build; no encode/decode used by core).
#ifndef __leg_control_command_lcmt_hpp__
#define __leg_control_command_lcmt_hpp__

struct leg_control_command_lcmt {
  float tau_ff[12];
  float f_ff[12];
  float q_des[12];
  float qd_des[12];
  float p_des[12];
  float v_des[12];
  float kp_cartesian[12];
  float kd_cartesian[12];
  float kp_joint[12];
  float kd_joint[12];
};

#endif
