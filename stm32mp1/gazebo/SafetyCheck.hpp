/*!
 * @file SafetyCheck.hpp
 * @brief Fall detection / E-stop for the STM32MP1 gaits.
 *
 * A legged robot is NOT a multirotor: hard vertical hits are normal - every
 * footfall is an impact spike on the IMU, and a fast trot slams harder still.
 * So acceleration is the wrong discriminator. What is never normal is ATTITUDE:
 * a working quadruped is never on its side or its face. This latches on
 * sustained roll/pitch (and on the body collapsing to the deck), which
 * separates "walking hard" from "fallen over".
 *
 * On trip the legs go limp (zero gains, zero torque) and STAY limp - a fallen
 * robot that keeps running its gait grinds its legs against the ground, which
 * on real hardware means stripped gears or a burnt motor. Recovery is a
 * deliberate restart, not something the controller decides for itself.
 *
 * This mirrors what MIT's stack already does and these gaits bypassed: their
 * ControlFSM::runFSM() calls safetyPreCheck() (SafetyChecker::checkSafeOrientation,
 * |roll| or |pitch| >= 0.5 rad) and on failure forces operatingMode = ESTOP and
 * switches to the PASSIVE state. Controllers here subclass RobotController
 * directly and never go through ControlFSM, so they inherited none of it.
 * Default trip is 35 deg - tighter than a fall, looser than MIT's 28.6 deg,
 * which a hard trot corner can touch transiently (hence the hold time).
 */
#ifndef STM32MP1_SAFETY_CHECK_H
#define STM32MP1_SAFETY_CHECK_H

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <Controllers/LegController.h>
#include <Controllers/StateEstimatorContainer.h>

class SafetyCheck {
 public:
  //! @return true if the robot is FAULTED (caller should stop commanding a gait)
  bool update(const StateEstimate<float>* est, float dt) {
    if (_faulted) return true;
    if (!est) return false;

    const float rollLim  = _envf("SAFE_ROLL_DEG",  35.f) * (float)M_PI / 180.f;
    const float pitchLim = _envf("SAFE_PITCH_DEG", 35.f) * (float)M_PI / 180.f;
    const float holdS    = _envf("SAFE_HOLD_S",    0.25f);

    float roll  = est->rpy[0];
    float pitch = est->rpy[1];
    bool bad = (fabsf(roll) > rollLim) || (fabsf(pitch) > pitchLim);

    if (bad) {
      _badFor += dt;
      if (_badFor >= holdS) {
        _faulted = true;
        printf("\n*** SAFETY STOP: attitude out of range "
               "(roll %.0f deg, pitch %.0f deg held %.2f s). Legs limp. ***\n"
               "*** A ground robot takes hard hits, but it is never on its side. "
               "Restart deliberately after checking the machine. ***\n",
               roll * 57.2958f, pitch * 57.2958f, _badFor);
        fflush(stdout);
      }
    } else {
      _badFor = 0.f;
    }
    return _faulted;
  }

  //! Make the legs safe: no stiffness, no torque. Call every tick while faulted.
  static void goLimp(LegController<float>* legs) {
    for (int leg = 0; leg < 4; ++leg) {
      legs->commands[leg].qDes = legs->datas[leg].q;
      legs->commands[leg].qdDes = Vec3<float>::Zero();
      legs->commands[leg].tauFeedForward = Vec3<float>::Zero();
      legs->commands[leg].forceFeedForward = Vec3<float>::Zero();
      legs->commands[leg].kpJoint = Mat3<float>::Zero();
      legs->commands[leg].kdJoint = Mat3<float>::Identity() * 0.4f;  // gentle damping only
      legs->commands[leg].kpCartesian = Mat3<float>::Zero();
      legs->commands[leg].kdCartesian = Mat3<float>::Zero();
    }
  }

  bool faulted() const { return _faulted; }

 private:
  static float _envf(const char* k, float d) {
    const char* v = getenv(k);
    return v ? (float)atof(v) : d;
  }
  bool  _faulted = false;
  float _badFor = 0.f;
};

#endif
