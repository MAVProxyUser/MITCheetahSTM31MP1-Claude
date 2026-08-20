// Minimal POD stand-in for the lcm-gen output (compute-derisk build; no encode/decode used by core).
#ifndef __gamepad_lcmt_hpp__
#define __gamepad_lcmt_hpp__

struct gamepad_lcmt {
  int32_t leftBumper, rightBumper, leftTriggerButton, rightTriggerButton, back,
      start, a, b, x, y, leftStickButton, rightStickButton;
  float leftTriggerAnalog, rightTriggerAnalog;
  float leftStickAnalog[2];
  float rightStickAnalog[2];
};

#endif
