/*!
 * @file TrotController.hpp
 * @brief Dynamic trot for the Go1 SITL (STM32MP1 port) - the speed gait.
 *
 * Diagonal pairs (FR+RL, then FL+RR) alternate stance/swing. Unlike the
 * statically-stable crawl (StaticGaitController), the robot is never in
 * static equilibrium: balance comes from WHERE the swing foot is placed
 * (Raibert heuristic) plus body attitude feedback pushing on stance legs.
 *
 * Runs entirely in the Cheetah ABSTRACT leg convention (bridge BRIDGE_CONV=mit)
 * on the port's proven joint-PD path - no MPC, no WBC, no QP. Everything is
 * analytic, so the whole gait costs ~30 us per tick on the A7.
 *
 * Drive: gamepad leftStickAnalog[1] = forward m/s, rightStickAnalog[0] = yaw
 * rate rad/s (set by the waypoint follower), else the TR_* defaults.
 *
 * Env knobs:
 *   TR_V       target forward speed m/s      (default 0.6)
 *   TR_T       gait cycle seconds            (default 0.40)
 *   TR_H       body height m                 (default 0.28)
 *   TR_LIFT    swing foot lift m             (default 0.08)
 *   TR_KV      Raibert velocity gain         (default 0.08)
 *   TR_KP_ROLL / TR_KD_ROLL                  (default 0.09 / 0.012)
 *   TR_KP_PITCH / TR_KD_PITCH                (default 0.09 / 0.012)
 *   TR_KP_J    joint P gain                  (default 120)
 *   TR_KD_J    joint D gain                  (default 3)
 *   TR_YAW     yaw rate rad/s                (default 0)
 *   TR_STAND_S seconds of stand before gait  (default 3)
 */
#ifndef TROT_CONTROLLER_H
#define TROT_CONTROLLER_H

#include <RobotController.h>
#include "SafetyCheck.hpp"

class TrotController : public RobotController {
 public:
  TrotController() : RobotController() {}
  void initializeController() override {}
  void runController() override;
  void updateVisualization() override {}
  ControlParameters* getUserControlParameters() override { return nullptr; }

 private:
  SafetyCheck _safety;
  void legIK(int leg, float x, float y, float z, float* q);
  // per-leg foot position at the moment of liftoff (start of its swing arc)
  float _liftoffX[4] = {0, 0, 0, 0};
  bool  _wasStance[4] = {true, true, true, true};
  float _qPrev[4][3] = {{0}};
  bool  _qPrevValid[4] = {false, false, false, false};
  float _pPrev[4][3] = {{0}};
  bool  _pPrevValid[4] = {false, false, false, false};
  float _hddot = 0.f;
  float _vzPrev = 0.f;
  float _trkErr = 0.f;
  int   _trkN = 0;
};

#endif
