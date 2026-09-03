#include <cmath>
#include <cstdlib>
#include "WalkController.hpp"

// Go1 stand pose (identity bridge mapping): abad 0, hip 0.8, knee -1.5.
static const float hipMid = 0.8f, kneeMid = -1.5f, standTime = 3.0f;

void WalkController::runController() {
  static int iter = 0;
  ++iter;
  float t = iter * 0.002f;                 // 500 Hz loop

  Mat3<float> kp = Mat3<float>::Zero();
  kp(0, 0) = kp(1, 1) = kp(2, 2) = 60.f;
  Mat3<float> kd = Mat3<float>::Zero();
  kd(0, 0) = kd(1, 1) = kd(2, 2) = 2.f;
  _legController->_maxTorque = 35;
  _legController->_legsEnabled = true;

  auto set = [&](int leg, float hip, float knee, float kpScale) {
    _legController->commands[leg].qDes = Vec3<float>(0.f, hip, knee);
    _legController->commands[leg].qdDes = Vec3<float>::Zero();
    _legController->commands[leg].tauFeedForward = Vec3<float>::Zero();
    _legController->commands[leg].kpJoint = kp * kpScale;
    _legController->commands[leg].kdJoint = kd;
  };

  // Phase 1: stand (ramp gains), settle onto the legs.
  if (t < standTime) {
    float s = fminf(t / 2.0f, 1.f);
    for (int leg = 0; leg < 4; ++leg) set(leg, hipMid, kneeMid, s);
    return;
  }

  // Phase 2: statically-stable creep. One leg swings at a time (FR,RR,FL,RL);
  // stance legs sweep their thighs back to drive the body forward.
  float tg = t - standTime;
  float T        = getenv("WALK_T")    ? atof(getenv("WALK_T"))    : 2.0f;   // cycle period
  float hipAmp   = getenv("WALK_AMP")  ? atof(getenv("WALK_AMP"))  : 0.12f;  // stance sweep
  float liftAmp  = getenv("WALK_LIFT") ? atof(getenv("WALK_LIFT")) : 0.4f;   // swing knee lift
  float turn     = getenv("WALK_TURN") ? atof(getenv("WALK_TURN")) : 0.0f;   // +L/-R bias
  const float offset[4] = {0.0f, 0.5f, 0.75f, 0.25f};   // FR, FL, RR, RL

  for (int leg = 0; leg < 4; ++leg) {
    float ph = fmodf(tg / T + offset[leg], 1.0f);
    // turning: right legs (FR=0,RR=2) shorter stride, left legs (FL=1,RL=3) longer
    float amp = hipAmp * (1.0f + ((leg == 1 || leg == 3) ? turn : -turn));
    float hip, knee;
    const float swingFrac = 0.25f;
    if (ph < swingFrac) {                       // swing: lift + reset foot forward
      float s = ph / swingFrac;
      hip = hipMid - amp + 2 * amp * s;
      knee = kneeMid - liftAmp * sinf((float)M_PI * s);
    } else {                                    // stance: sweep back -> body forward
      float s = (ph - swingFrac) / (1.f - swingFrac);
      hip = hipMid + amp - 2 * amp * s;
      knee = kneeMid;
    }
    set(leg, hip, knee, 1.f);
  }
}
