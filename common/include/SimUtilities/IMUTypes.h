/*! @file IMUTypes.h
 *  @brief Data from IMUs
 */

#ifndef PROJECT_IMUTYPES_H
#define PROJECT_IMUTYPES_H

#include "cppTypes.h"

/*!
 * Mini Cheetah's IMU
 */
struct VectorNavData {
  // DEFAULT-INITIALISED (OPEN-6, 2026-08-28). Eigen does NOT zero its
  // types, and nothing memset this struct, so between process start and
  // the first sensor packet the estimator read raw stack garbage: measured
  // quat = (0, 5.6e28, 0, 0) with a NaN position on control iteration 0 -
  // the "STATE ESTIMATE WENT NON-FINITE" blip printed by ~2/3 of all runs.
  // The NaN guard caught it, but only by luck: that garbage quaternion is
  // FINITE, and VectorNavOrientationEstimator captures its heading datum
  // (_ori_ini_inv) on its own first visit - so a garbage-but-finite boot
  // quaternion could silently fix the whole run's heading reference to
  // nonsense without tripping any guard at all. Identity quat + zero
  // rates is the honest pre-sensor state. Same defect class as the
  // already-documented uninitialised GamepadCommand (CLAUDE.md).
  Vec3<float> accelerometer = Vec3<float>::Zero();
  Vec3<float> gyro = Vec3<float>::Zero();
  Quat<float> quat = (Quat<float>() << 1.f, 0.f, 0.f, 0.f).finished();
  // TRUE once a real sensor packet has landed. The orientation
  // estimator captures its heading DATUM on first visit and keeps it
  // for the whole run, so capturing it from the default pose above
  // would fix the run's heading reference to the world frame instead
  // of the spawn pose. Before the default-init went in, the boot
  // garbage happened to trip RobotRunner's non-finite guard, which
  // re-created the estimator and so re-armed the capture until real
  // data arrived - the datum was correct BY ACCIDENT. This flag makes
  // that dependency explicit instead of relying on a guard firing.
  bool valid = false;
  // todo is there status for the vectornav?
};

/*!
 * "Cheater" state sent to the robot from simulator
 */
template <typename T>
struct CheaterState {
  Quat<T> orientation;
  Vec3<T> position;
  Vec3<T> omegaBody;
  Vec3<T> vBody;
  Vec3<T> acceleration;
};

#endif  // PROJECT_IMUTYPES_H
