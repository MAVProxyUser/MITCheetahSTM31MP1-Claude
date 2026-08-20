// Minimal POD stand-in for the lcm-gen output (compute-derisk build; no encode/decode used by core).
#ifndef __state_estimator_lcmt_hpp__
#define __state_estimator_lcmt_hpp__

struct state_estimator_lcmt {
  float p[3];
  float vWorld[3];
  float vBody[3];
  float rpy[3];
  float omegaBody[3];
  float omegaWorld[3];
  float quat[4];
};

#endif
