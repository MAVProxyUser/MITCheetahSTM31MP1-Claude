/*!
 * @file StaticGaitController.hpp
 * @brief Statically-stable crawl gait for the Go1 SITL (STM32MP1 port).
 *
 * Pure joint-space PD (the port's proven-stable control path - no MPC, no WBC,
 * no force feedforward): a four-beat crawl where the body's CoM is shifted
 * over the support triangle before each leg swings. Runs in the Cheetah
 * ABSTRACT leg convention (bridge BRIDGE_CONV=mit converts to Go1 URDF signs).
 *
 * Env knobs:
 *   SG_VX     forward speed m/s          (default 0.06)
 *   SG_T      full gait cycle seconds    (default 6.0)
 *   SG_SHIFT  lateral CoM shift m        (default 0.055)
 *   SG_LIFT   swing foot lift m          (default 0.06)
 *   SG_H      body height m              (default 0.23)
 *   SG_TURN   yaw bias (+left, rad of differential stride) (default 0)
 */
#ifndef STATIC_GAIT_CONTROLLER_H
#define STATIC_GAIT_CONTROLLER_H

#include <RobotController.h>
#include "SafetyCheck.hpp"

class StaticGaitController : public RobotController {
 public:
  StaticGaitController() : RobotController() {}
  void initializeController() override {}
  void runController() override;
  void updateVisualization() override {}
  ControlParameters* getUserControlParameters() override { return nullptr; }

 private:
  SafetyCheck _safety;
  // per-leg terrain memory: where this foot last found the ground (body frame)
  float _groundZ[4] = {0.f, 0.f, 0.f, 0.f};
  float _qLast[4] = {0.f, 0.f, 0.f, 0.f};
  bool  _touched[4] = {false, false, false, false};
  bool  _standContact[4] = {false, false, false, false};
  float _standZ[4] = {0.f, 0.f, 0.f, 0.f};
  float _zStandCmd[4] = {0.f, 0.f, 0.f, 0.f};
  bool  _terrainSeeded = false;
  float _standStall[4] = {0.f, 0.f, 0.f, 0.f};
  float _zStandPrevAct[4] = {0.f, 0.f, 0.f, 0.f};
  float _swPrevAct[4] = {0.f, 0.f, 0.f, 0.f};
  // ILC memory: feed-forward torque per (leg, gait-phase bin, joint)
  static const int ILC_BINS = 40;
  float _ilc[4][ILC_BINS][3] = {};
  // abstract-convention IK for one leg (x fwd, y left, z down<0, rel abad pivot)
  void legIK(int leg, float x, float y, float z, float* q);
};

#endif
