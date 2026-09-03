#include <cmath>
#include "StandController.hpp"

void StandController::runController() {
  static int iter = 0;
  ++iter;
  // Ramp stiffness up over ~1 s so the legs settle into the stance gently.
  float s = (iter < 500) ? (iter / 500.f) : 1.f;

  Mat3<float> kp = Mat3<float>::Zero();
  kp(0, 0) = 60.f * s; kp(1, 1) = 60.f * s; kp(2, 2) = 60.f * s;
  Mat3<float> kd = Mat3<float>::Zero();
  kd(0, 0) = 2.f; kd(1, 1) = 2.f; kd(2, 2) = 2.f;

  _legController->_maxTorque = 35;
  _legController->_legsEnabled = true;

  // Go1 stance (identity bridge map): abad ~0, hip ~0.8, knee ~-1.5 rad.
  float hip = 0.8f, knee = -1.5f;
  // After settling, a slow body squat (0.25 Hz) - demonstrates dynamic control
  // through the full SITL loop, and that it stays stable while moving.
  if (iter > 2000) {
    float t = (iter - 2000) * 0.002f;                 // seconds (500 Hz loop)
    float bob = 0.30f * std::sin(2.f * (float)M_PI * 0.25f * t);
    hip += bob; knee -= 2.f * bob;                    // extend/retract legs together
  }
  const float qStand[3] = {0.0f, hip, knee};
  for (int leg = 0; leg < 4; ++leg) {
    for (int j = 0; j < 3; ++j) {
      _legController->commands[leg].qDes[j] = qStand[j];
      _legController->commands[leg].qdDes[j] = 0.f;
      _legController->commands[leg].tauFeedForward[j] = 0.f;
    }
    _legController->commands[leg].kpJoint = kp;
    _legController->commands[leg].kdJoint = kd;
  }
}
