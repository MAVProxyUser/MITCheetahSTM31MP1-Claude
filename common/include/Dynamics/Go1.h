/*! @file Go1.h
 *  @brief Utility function to build a Unitree Go1 Quadruped object.
 *
 * Kinematics + inertials from unitree_ros go1_description (go1.urdf). Same
 * structure as MiniCheetah.h; robotType stays MINI_CHEETAH so the existing
 * mini-cheetah code paths (spiData/spiCommand, leg conventions) apply, but the
 * dynamics/kinematics match the Go1 so MPC/WBC foot targets are correct.
 */
#ifndef PROJECT_GO1_H
#define PROJECT_GO1_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

template <typename T>
Quadruped<T> buildGo1() {
  Quadruped<T> go1;
  go1._robotType = RobotType::MINI_CHEETAH;

  go1._bodyMass = 5.204;              // trunk
  go1._bodyLength = 0.1881 * 2;       // 0.3762  (FR_hip_joint x)
  go1._bodyWidth = 0.04675 * 2;       // 0.0935  (FR_hip_joint y)
  go1._bodyHeight = 0.114;
  go1._abadGearRatio = 6.33;
  go1._hipGearRatio = 6.33;
  go1._kneeGearRatio = 6.33;
  go1._abadLinkLength = 0.08;         // hip -> thigh lateral offset
  go1._hipLinkLength = 0.213;         // thigh
  go1._kneeLinkY_offset = 0.0;
  go1._kneeLinkLength = 0.213;        // calf
  go1._maxLegLength = 0.40;

  go1._motorTauMax = 23.7f;           // Go1 joint motor
  go1._batteryV = 21.6;
  go1._motorKT = .05;
  go1._motorR = 0.173;
  go1._jointDamping = .01;
  go1._jointDryFriction = .2;

  // rotor inertia (approx; Go1 rotors are small, reuse mini-cheetah magnitudes)
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;
  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX = RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY = RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias (FR leg, from URDF inertials; matrices are I*1e6, symmetric)
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 334, 11, 1, 11, 619, -2, 1, -2, 401;
  abadRotationalInertia *= 1e-6;
  Vec3<T> abadCOM(-0.005657, 0.008752, -0.000102);
  SpatialInertia<T> abadInertia(0.591, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 4432, -57, -218, -57, 4486, -572, -218, -572, 740;
  hipRotationalInertia *= 1e-6;
  Vec3<T> hipCOM(-0.003342, 0.018054, -0.033451);
  SpatialInertia<T> hipInertia(0.92, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia;
  kneeRotationalInertia << 1089, 0, 7, 0, 1100, 2, 7, 2, 25;
  kneeRotationalInertia *= 1e-6;
  Vec3<T> kneeCOM(0.006197, 0.001408, -0.116695);
  SpatialInertia<T> kneeInertia(0.135862, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 16813, -230, -295, -230, 63010, -42, -295, -42, 71655;
  bodyRotationalInertia *= 1e-6;
  Vec3<T> bodyCOM(0.0223, 0.002, -0.0005);
  SpatialInertia<T> bodyInertia(go1._bodyMass, bodyCOM, bodyRotationalInertia);

  go1._abadInertia = abadInertia;
  go1._hipInertia = hipInertia;
  go1._kneeInertia = kneeInertia;
  go1._abadRotorInertia = rotorInertiaX;
  go1._hipRotorInertia = rotorInertiaY;
  go1._kneeRotorInertia = rotorInertiaY;
  go1._bodyInertia = bodyInertia;

  // locations (from URDF joint origins)
  go1._abadRotorLocation = Vec3<T>(0.1881, 0.04675, 0);
  go1._abadLocation = Vec3<T>(go1._bodyLength, go1._bodyWidth, 0) * 0.5;
  go1._hipLocation = Vec3<T>(0, go1._abadLinkLength, 0);
  go1._hipRotorLocation = Vec3<T>(0, 0.08, 0);
  go1._kneeLocation = Vec3<T>(0, 0, -go1._hipLinkLength);
  go1._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return go1;
}

#endif  // PROJECT_GO1_H
