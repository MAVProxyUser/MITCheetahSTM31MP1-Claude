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
  // GEAR RATIOS, recovered from Unitree's own Legged_sport binary (the constant
  // pool of its buildMiniCheetah<float>(), at .rodata 0x2fdc20):
  //     6.333   6.333   -9.4995
  // The knee is geared 9.4995, NOT 6.33. This port previously assumed one ratio
  // for all three joints and then had to invent a second torque cap to explain
  // the knee's 35.55 Nm; see _motorTauMax below. Unitree's sign on the knee is
  // negative (joint direction convention); the magnitude is what the actuator
  // model needs. Mini-cheetah does the same thing with 6/6/9.33.
  go1._abadGearRatio = 6.333;
  go1._hipGearRatio = 6.333;
  go1._kneeGearRatio = 9.4995;
  go1._abadLinkLength = 0.08;         // hip -> thigh lateral offset
  go1._hipLinkLength = 0.213;         // thigh
  go1._kneeLinkY_offset = 0.0;
  go1._kneeLinkLength = 0.213;        // calf
  // Unitree's binary carries 0.430 here (.rodata 0x2fdc40). This port had
  // derived 0.385 from the URDF knee limit (-51 deg), reasoning that the leg can
  // never straighten so the reach is sqrt(l2^2+l3^2+2*l2*l3*cos(0.888)). That
  // is geometrically true but it is NOT what the factory controller uses, and
  // this value feeds the swing planner and SafetyChecker's
  // maxPDes = L*sin(60deg) - i.e. it BOUNDS how far the controller may place a
  // foot. Running 0.385 against a machine whose own firmware allows 0.430 is a
  // self-imposed 10% shorter stride, on the axis that sets speed.
  go1._maxLegLength = 0.430;

  // MOTOR-side torque cap. ActuatorModel computes
  //     tau_joint = gearRatio * clamp(tau_motor, +-_tauMax)
  // ONE motor type drives all three joints - the Go1's stronger knee comes from
  // GEARING, not from a bigger actuator:
  //     23.70 Nm / 6.333  = 3.7424 Nm at the motor
  //     35.55 Nm / 9.4995 = 3.7423 Nm at the motor
  // Identical to four decimal places, which is the proof. This port used to set
  // one gear ratio (6.33) for all three joints and then added a second cap,
  // _kneeMotorTauMax = 5.616, to recover the knee's torque - a workaround for a
  // wrong premise ("the Go1 knee is 1.5x at the SAME gear"). MIT's single-value
  // struct expressed this correctly all along, exactly as it does for
  // mini-cheetah's 6/6/9.33. The hack is gone; the gear ratio carries it.
  go1._motorTauMax = 3.744f;
  go1._batteryV = 24.0;   // Unitree binary .rodata 0x2fdc40 (was 21.6 here)
  go1._motorKT = .05;
  go1._motorR = 0.173;
  go1._jointDamping = .01;
  go1._jointDryFriction = .2;

  // Rotor mass/inertia, from the URDF's hip_rotor/thigh_rotor/calf_rotor links
  // (all three identical: mass 0.089, spin-axis I=111.842e-6, radial I=59.647e-6
  // kg m^2). This block was previously an unmodified copy of MiniCheetah.h's
  // rotor (mass 0.055, spin 63e-6, radial 33e-6) - never updated for the Go1,
  // wrong by 62-81% on every field. Independently confirmed: these exact URDF
  // values (59.646999, 111.842003) appear as immediates in Unitree's own
  // buildMiniCheetah<float>() in Legged_sport - found during the initial
  // reversing pass and not connected to this bug until now.
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 59.647, 0, 0, 0, 59.647, 0, 0, 0, 111.842;
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
  SpatialInertia<T> rotorInertiaX(0.089, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.089, rotorCOM, rotorRotationalInertiaY);

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

  // Locations, verified against the URDF's joint origins with the ACTUAL
  // consuming convention checked in Quadruped.cpp - not assumed. `withLegSigns`
  // (leg 0 = FR) maps stored (x,y,z) -> physical (x,-y,z), confirmed against
  // _abadLocation/_hipLocation/_kneeLocation which all already matched the
  // URDF exactly. The three ROTOR locations did not: _abadRotorLocation was a
  // literal copy of _abadLocation (impossible - different joints), and
  // _hipRotorLocation/_kneeRotorLocation were mini-cheetah-pattern guesses
  // (rotor co-located with the FAR joint) rather than the Go1's actual
  // near-zero motor-housing offsets.
  //
  //   joint              URDF FR xyz            wrapped in withLegSigns?
  //   FR_hip_rotor_joint  (0.11215,-0.04675,0)   yes (like abad)
  //   FR_thigh_rotor_joint(0.0, 0.00015, 0)      yes (like hip)
  //   FR_calf_rotor_joint (0.0, 0.03235, 0)      NO  (like knee - raw, no flip;
  //                                              Quadruped.cpp uses _kneeLocation
  //                                              and _kneeRotorLocation both
  //                                              unwrapped)
  go1._abadRotorLocation = Vec3<T>(0.11215, 0.04675, 0);
  go1._abadLocation = Vec3<T>(go1._bodyLength, go1._bodyWidth, 0) * 0.5;
  go1._hipLocation = Vec3<T>(0, go1._abadLinkLength, 0);
  go1._hipRotorLocation = Vec3<T>(0, -0.00015, 0);
  go1._kneeLocation = Vec3<T>(0, 0, -go1._hipLinkLength);
  go1._kneeRotorLocation = Vec3<T>(0, 0.03235, 0);

  return go1;
}

#endif  // PROJECT_GO1_H
