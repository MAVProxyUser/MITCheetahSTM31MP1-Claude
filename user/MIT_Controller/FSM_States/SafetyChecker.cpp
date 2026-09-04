/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 */

#include "SafetyChecker.h"
#include <cstdlib>
#include "../../../gazebo/ShmTrace.h"   // per-tick/text SHM tracing - see that file's own header
#include "Utilities/CtrlTuning.h"

/**
 * @return safePDesFoot true if safe desired foot placements
 *
 * DEBOUNCE ($CTRL_ORIENT_HOLD_MS, default 60 ms; 0 restores stock MIT).
 *
 * Upstream this is a ZERO-debounce trip: ONE tick past 0.5 rad (28.6 deg)
 * force-ESTOPs to PASSIVE and cuts the motors. That is right for a
 * mini-cheetah pottering about and wrong for this port at campaign speeds,
 * where the measured yaw envelope already puts cornering roll at 19.5 deg
 * at vx=1.5/wz=1.5 - so a transient at the lateral-acceleration budget
 * brushes 28.6 deg routinely. Every fall recorded on 2026-08-24, on all
 * three courses, is this line firing FIRST and the "collapse" following as
 * its consequence: the giveaway is a robot reported perfectly level
 * (roll=-1, pitch=-0) at z=-0.302 m, i.e. limp and sinking, not tipped.
 *
 * A brief hold distinguishes "clipped the limit through a hard corner" from
 * "actually going over" - it is the same argument this port already made
 * for its own fall detector (SIM_FALL_DEG 50 deg held SIM_FALL_HOLD_S
 * 0.5 s), which stays armed underneath as the genuine-fall arbiter. The
 * excursion is also PRINTED with its peak so the decision is measurable
 * rather than assumed.
 */
template <typename T>
bool SafetyChecker<T>::checkSafeOrientation() {
  static const int hold_ticks =
      (ctrl_tuning::integer("CTRL_ORIENT_HOLD_MS", 60)) / 2;   // 2 ms ticks
  // THE LIMIT IS NOW TUNABLE, AND THAT IS THE POINT (2026-09-04).
  //
  // 0.5 rad = 28.65 deg was hardcoded. Measured that day: 19 of 20 falls
  // E-STOP here first, at a peak pitch of 33.3 deg, and the E-stop precedes
  // the body's descent by 0.41 s - while 0 of 14 passing runs ever trip it.
  // Every "capability ceiling" in this project's record is therefore the
  // speed at which the robot first pitches past THIS NUMBER, not a limit of
  // its dynamics. Whether it could recover from 33 deg has never been
  // tested, because nothing has ever been allowed to try.
  static const float limit_rad =
      (float)ctrl_tuning::num("CTRL_ORIENT_LIMIT_DEG", 28.65) / 57.2958f;
  static bool announced = false;
  if (!announced) {
    announced = true;
    shmtrace::logf(0.0, "[orient] limit %.1f deg, hold %d ms",
                   limit_rad * 57.2958f, hold_ticks * 2);
  }
  static int over_ticks = 0;
  static float peak = 0.f;

  const float roll  = std::fabs((float)data->_stateEstimator->getResult().rpy(0));
  const float pitch = std::fabs((float)data->_stateEstimator->getResult().rpy(1));
  const float worst = std::max(roll, pitch);

  if (worst >= limit_rad) {
    ++over_ticks;
    if (worst > peak) peak = worst;
    if (over_ticks > hold_ticks) {
      shmtrace::logf(0.0, "Orientation safety check failed! (roll=%.1f pitch=%.1f deg, "
             "peak %.1f, held %d ms)",
             roll * 57.2958f, pitch * 57.2958f, peak * 57.2958f,
             over_ticks * 2);
      over_ticks = 0; peak = 0.f;
      return false;
    }
    return true;               // inside the hold - still recoverable
  }
  if (over_ticks) {            // recovered: report what it survived
    shmtrace::logf(0.0, "[orient] transient %.1f deg for %d ms - RECOVERED",
           peak * 57.2958f, over_ticks * 2);
  }
  over_ticks = 0; peak = 0.f;
  return true;
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::checkPDesFoot() {
  // Assumed safe to start
  bool safePDesFoot = true;

  // Safety parameters
  T maxAngle = 1.0472;  // 60 degrees (should be changed)
  T maxPDes = data->_quadruped->_maxLegLength * sin(maxAngle);

  // Check all of the legs
  for (int leg = 0; leg < 4; leg++) {
    // Keep the foot from going too far from the body in +x
    if (data->_legController->commands[leg].pDes(0) > maxPDes) {
      shmtrace::logf(0.0, "[CONTROL FSM] Safety: PDes leg: %d | coordinate: 0 | "
             "commanded: %f | modified: %f", leg,
             (double)data->_legController->commands[leg].pDes(0), (double)maxPDes);
      data->_legController->commands[leg].pDes(0) = maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in -x
    if (data->_legController->commands[leg].pDes(0) < -maxPDes) {
      shmtrace::logf(0.0, "[CONTROL FSM] Safety: PDes leg: %d | coordinate: 0 | "
             "commanded: %f | modified: %f", leg,
             (double)data->_legController->commands[leg].pDes(0), (double)-maxPDes);
      data->_legController->commands[leg].pDes(0) = -maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in +y
    if (data->_legController->commands[leg].pDes(1) > maxPDes) {
      shmtrace::logf(0.0, "[CONTROL FSM] Safety: PDes leg: %d | coordinate: 1 | "
             "commanded: %f | modified: %f", leg,
             (double)data->_legController->commands[leg].pDes(1), (double)maxPDes);
      data->_legController->commands[leg].pDes(1) = maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in -y
    if (data->_legController->commands[leg].pDes(1) < -maxPDes) {
      shmtrace::logf(0.0, "[CONTROL FSM] Safety: PDes leg: %d | coordinate: 1 | "
             "commanded: %f | modified: %f", leg,
             (double)data->_legController->commands[leg].pDes(1), (double)-maxPDes);
      data->_legController->commands[leg].pDes(1) = -maxPDes;
      safePDesFoot = false;
    }

    // Keep the leg under the motor module (don't raise above body or crash into
    // module)
    if (data->_legController->commands[leg].pDes(2) >
        -data->_quadruped->_maxLegLength / 4) {
      shmtrace::logf(0.0, "[CONTROL FSM] Safety: PDes leg: %d | coordinate: 2 | "
             "commanded: %f | modified: %f", leg,
             (double)data->_legController->commands[leg].pDes(2),
             (double)(-data->_quadruped->_maxLegLength / 4));
      data->_legController->commands[leg].pDes(2) =
          -data->_quadruped->_maxLegLength / 4;
      safePDesFoot = false;
    }

    // Keep the foot within the kinematic limits
    if (data->_legController->commands[leg].pDes(2) <
        -data->_quadruped->_maxLegLength) {
      shmtrace::logf(0.0, "[CONTROL FSM] Safety: PDes leg: %d | coordinate: 2 | "
             "commanded: %f | modified: %f", leg,
             (double)data->_legController->commands[leg].pDes(2),
             (double)(-data->_quadruped->_maxLegLength));
      data->_legController->commands[leg].pDes(2) =
          -data->_quadruped->_maxLegLength;
      safePDesFoot = false;
    }
  }

  // Return true if all desired positions are safe
  return safePDesFoot;
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::checkForceFeedForward() {
  // Assumed safe to start
  bool safeForceFeedForward = true;

  // Initialize maximum vertical and lateral forces
  T maxLateralForce = 0;
  T maxVerticalForce = 0;

  // Maximum force limits for each robot
  if (data->_quadruped->_robotType == RobotType::CHEETAH_3) {
    maxLateralForce = 1800;
    maxVerticalForce = 1800;

  } else if (data->_quadruped->_robotType == RobotType::MINI_CHEETAH) {
    maxLateralForce = 350;
    maxVerticalForce = 350;
  }

  // Check all of the legs
  for (int leg = 0; leg < 4; leg++) {
    // Limit the lateral forces in +x body frame
    if (data->_legController->commands[leg].forceFeedForward(0) >
        maxLateralForce) {
      shmtrace::logf(0.0, "[CONTROL FSM] Safety: Force leg: %d | coordinate: 0 | "
             "commanded: %f | modified: %f", leg,
             (double)data->_legController->commands[leg].forceFeedForward(0),
             (double)maxLateralForce);
      data->_legController->commands[leg].forceFeedForward(0) = maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in -x body frame
    if (data->_legController->commands[leg].forceFeedForward(0) <
        -maxLateralForce) {
      shmtrace::logf(0.0, "[CONTROL FSM] Safety: Force leg: %d | coordinate: 0 | "
             "commanded: %f | modified: %f", leg,
             (double)data->_legController->commands[leg].forceFeedForward(0),
             (double)-maxLateralForce);
      data->_legController->commands[leg].forceFeedForward(0) =
          -maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in +y body frame
    if (data->_legController->commands[leg].forceFeedForward(1) >
        maxLateralForce) {
      shmtrace::logf(0.0, "[CONTROL FSM] Safety: Force leg: %d | coordinate: 1 | "
             "commanded: %f | modified: %f", leg,
             (double)data->_legController->commands[leg].forceFeedForward(1),
             (double)maxLateralForce);
      data->_legController->commands[leg].forceFeedForward(1) = maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in -y body frame
    if (data->_legController->commands[leg].forceFeedForward(1) <
        -maxLateralForce) {
      shmtrace::logf(0.0, "[CONTROL FSM] Safety: Force leg: %d | coordinate: 1 | "
             "commanded: %f | modified: %f", leg,
             (double)data->_legController->commands[leg].forceFeedForward(1),
             (double)-maxLateralForce);
      data->_legController->commands[leg].forceFeedForward(1) =
          -maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the vertical forces in +z body frame
    if (data->_legController->commands[leg].forceFeedForward(2) >
        maxVerticalForce) {
      shmtrace::logf(0.0, "[CONTROL FSM] Safety: Force leg: %d | coordinate: 2 | "
             "commanded: %f | modified: %f", leg,
             (double)data->_legController->commands[leg].forceFeedForward(2),
             (double)-maxVerticalForce);
      data->_legController->commands[leg].forceFeedForward(2) =
          maxVerticalForce;
      safeForceFeedForward = false;
    }

    // Limit the vertical forces in -z body frame
    if (data->_legController->commands[leg].forceFeedForward(2) <
        -maxVerticalForce) {
      shmtrace::logf(0.0, "[CONTROL FSM] Safety: Force leg: %d | coordinate: 2 | "
             "commanded: %f | modified: %f", leg,
             (double)data->_legController->commands[leg].forceFeedForward(2),
             (double)maxVerticalForce);
      data->_legController->commands[leg].forceFeedForward(2) =
          -maxVerticalForce;
      safeForceFeedForward = false;
    }
  }

  // Return true if all feed forward forces are safe
  return safeForceFeedForward;
}

// template class SafetyChecker<double>; This should be fixed... need to make
// RobotRunner a template
template class SafetyChecker<float>;
